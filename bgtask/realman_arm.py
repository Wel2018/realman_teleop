# 重构版：RealmanArm client + task
# 依赖：Robotic_Arm, numpy, rich
# 官方文档：https://develop.realman-robotics.com/robot/apipython/classes/roboticArm/
# pip install Robotic_Arm (当前使用版本：1.1.1)

import threading
import time
from Robotic_Arm.rm_robot_interface import *  # type: ignore
import numpy as np
from rich import print
from typing import List, Tuple, Dict, Any, Optional

from toolbox.core.log import printc
from .dexhand import DexterousHand
from toolbox.qt.common.debug import enable_debugpy
from toolbox.qt import qtbase
from .. import APPCFG, IS_HAND_MODE


# ----------------------------
# numpy 旋转矩阵（更高效）
# ----------------------------
def rotx(theta: float) -> np.ndarray:
    """绕 X 轴旋转矩阵 3x3"""
    c = np.cos(theta); s = np.sin(theta)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]], dtype=float)

def roty(theta: float) -> np.ndarray:
    """绕 Y 轴旋转矩阵 3x3"""
    c = np.cos(theta); s = np.sin(theta)
    return np.array([[c, 0, -s],
                     [0, 1, 0],
                     [s, 0, c]], dtype=float)

def rotz(theta: float) -> np.ndarray:
    """绕 Z 轴旋转矩阵 3x3"""
    c = np.cos(theta); s = np.sin(theta)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]], dtype=float)

_ROT_MAP = {"x": rotx, "y": roty, "z": rotz}

# ----------------------------
# 将 3x3 rotation -> pose（使用 Algo 的转换函数）
# ----------------------------
def _matrix3_to_pose(algo: "Algo", R3: np.ndarray, position: List[float]) -> List[float]:
    """
    将 3x3 旋转矩阵和位置组合成 4x4 rm_matrix_t 后调用算法库转换为 pose。
    algo: 复用的 Algo 实例（必须存在）
    R3: 3x3 numpy array
    position: [x,y,z]（保留原值）
    返回 [x,y,z,roll,pitch,yaw]
    """
    # 构造 4x4 数据列表（row-major）
    data = [
        [float(R3[0, 0]), float(R3[0, 1]), float(R3[0, 2]), 0.0],
        [float(R3[1, 0]), float(R3[1, 1]), float(R3[1, 2]), 0.0],
        [float(R3[2, 0]), float(R3[2, 1]), float(R3[2, 2]), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    mat = rm_matrix_t(4, 4, data)  # type: ignore
    pose = algo.rm_algo_matrix2pos(mat)  # [x,y,z,roll,pitch,yaw]
    # 保持原 position（x,y,z）
    pose[0:3] = [float(position[0]), float(position[1]), float(position[2])]
    return pose

def pose_by_rot_cached(
    algo: "Algo",
    pose: List[float],
    rot_axis: str = "x",
    theta: float = 0.01,
) -> List[float]:
    """
    基于传入的 Algo（避免重复构建）对 pose 做绕轴小旋转并返回新的 pose。
    pose: [x,y,z, r,p,y] 其中 r,p,y 单位与 Algo 期望一致（保持原来的行为）
    rot_axis: 'x'/'y'/'z'
    theta: 小角度（弧度）
    """
    if rot_axis not in _ROT_MAP:
        raise ValueError("rot_axis must be 'x', 'y' or 'z'")

    # 提取位置与 RPY（注意：原代码中有 prec，但为 1；如果你的 RPY 是度请在这里转换）
    pos = [float(pose[0]), float(pose[1]), float(pose[2])]
    rpy = [float(pose[3]), float(pose[4]), float(pose[5])]

    # 1) 当前 R0（4x4 matrix） -> 提取 3x3
    R0 = algo.rm_algo_euler2matrix(rpy)  # 返回 rm_matrix_t
    R_np = np.array(R0.data, dtype=float).reshape(4, 4)
    R3 = R_np[:3, :3]

    # 2) 计算 dR 并更新 R
    dR = _ROT_MAP[rot_axis](theta)
    R_new = dR @ R3  # 矩阵乘法（更快）

    # 3) 转回 pose
    return _matrix3_to_pose(algo, R_new, pos)


# ----------------------------
# Realman 客户端（封装机器人/算法实例）
# ----------------------------
class RealmanArmClient(qtbase.QObject):
    sig_move_finger = qtbase.Signal(int, int) # index: int, position: int
    sig_connect_finished = qtbase.Signal()

    def __init__(self, arm_model: rm_robot_arm_model_e = rm_robot_arm_model_e.RM_MODEL_RM_63_III_E):
        super().__init__()
        # 缓存 Algo 实例，避免频繁创建
        force_type = rm_force_type_e.RM_MODEL_RM_B_E  # 参考原代码
        self.algo = Algo(arm_model, force_type)  # type: ignore
        self.is_connected = 0
    
    def connect(self, ip, port=8080):
        """在子线程中连接机械臂，避免阻塞主进程"""
        self.t_connect = threading.Thread(target=self._connect, args=(ip, port), daemon=True)
        self.t_connect.start()

    def _connect(self, ip, port=8080):
        from toolbox.core.log import printc
        self.ip = ip
        self.port = port
        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)  # type: ignore
        self.handle = self.robot.rm_create_robot_arm(ip, port)
        arm_id = getattr(self.handle, 'id', -1)
        printc(f"RobotArm ID: {arm_id}")

        if arm_id == -1:
            printc(f"机械臂连接失败！当前信息：ip={ip} port={port}", 'err')
            self.sig_connect_finished.emit()
            return

        # 打印并设置 tool frame（保留原行为）
        total_tool = self.robot.rm_get_total_tool_frame()
        printc(f"rm_get_total_tool_frame={total_tool}")
        # 切换到指定 tool frame（如果需要，可改为参数化）
        # self.robot.rm_change_tool_frame("hhltest2")
        
        # 打印软件信息（保持原逻辑）
        software_info = self.robot.rm_get_arm_software_info()
        if software_info[0] == 0:
            info = software_info[1]
            product_version = info.get('product_version')
            algorithm_info = info.get('algorithm_info', {}).get('version')
            ctrl_info = info.get('ctrl_info', {}).get('version')
            dynamic_info = info.get('dynamic_info', {}).get('version')
            plan_info = info.get('plan_info', {}).get('version')

            printc("================== 机械臂软件信息 ==================")
            printc(f"机械臂模型： {product_version}")
            printc(f"算法库版本: {algorithm_info}")
            printc(f"控制层软件版本: {ctrl_info}")
            printc(f"Dynamics: {dynamic_info}")
            printc(f"路径规划层软件版本: {plan_info}")
            printc("==================================================")
            
        # 初始化回零位置（保留原行为）
        # self.gozero()
        self.is_connected = 1
        self.is_hand_opened = 1   # 夹爪/灵巧手打开状态

        if IS_HAND_MODE:
            self.hand_init()
        else:
            self.gripper_init()
        self.sig_connect_finished.emit()
        
    def hand_init(self):
        printc("正在初始化灵巧手...")
        self.hand = DexterousHand(self.robot)
        self.hand.sig_move_finger.connect(self.sig_move_finger.emit)
        self.hand_open()

    def gripper_init(self):
        printc("正在初始化夹爪...")
        self.gripper_open()

    def gozero(self):
        # 使用较短的参数调用，保留原意
        try:
            self.robot.rm_movej(APPCFG['arm_zero_joint'], APPCFG['arm_speed'], 0, 0, 1)
        except Exception as e:
            printc(f"[error] gozero failed: {e}", 'err')

    def move_p_canfd(self, pose: List[float]):
        # 包装 movep 调用并打印（原来函数名保留）
        try:
            ret = self.robot.rm_movep_canfd(pose, False, 1, 60)
            printc(f"MOVE_P: {pose}, ret={ret}")
            return ret
        except Exception as e:
            printc(f"[error] move_p_canfd failed: {e}", 'err')
            return None

    def hand_open(self):
        self.hand.open_all()
        self.is_hand_opened = 1

    def hand_close(self):
        self.hand.close_all()
        self.is_hand_opened = 0

    def gripper_open(self):
        try:
            ret = self.robot.rm_set_gripper_release(APPCFG['arm_gripper_speed'], True, 10)
            printc(f"gripper_open: {ret}")
            self.is_hand_opened = 1
            return ret
        except Exception as e:
            printc(f"[warn] gripper_open failed: {e}", 'err')

    def gripper_close(self):
        try:
            ret = self.robot.rm_set_gripper_pick(APPCFG['arm_gripper_speed'], APPCFG['arm_gripper_force'], True, 10)
            printc(f"gripper_close: {ret}")
            self.is_hand_opened = 0
            return ret
        except Exception as e:
            printc(f"[warn] gripper_close failed: {e}", 'err')

    def get_pose(self) -> Tuple[int, Dict[str, Any]]:
        """返回 rm_get_current_arm_state 的原始结果 (ret, data)"""
        return self.robot.rm_get_current_arm_state()

    def matrix_pose_rotate(self, pose: List[float], axis: str, theta: float) -> List[float]:
        """基于缓存的 algo 对 pose 做绕轴旋转并返回新 pose"""
        return pose_by_rot_cached(self.algo, pose, rot_axis=axis, theta=theta)

    def __del__(self):
        self.disconnect()

    def disconnect(self):
        try:
            self.robot.rm_delete_robot_arm()
        except Exception:
            pass


# ----------------------------
# RealmanArmTask：QAsyncTask 子类（保留结构）
# ----------------------------
from toolbox.qt import qtbase
from .spacemouse import SpaceMouseListener


class RealmanArmTask(qtbase.QAsyncTask):
    def __init__(self, arm: RealmanArmClient, spacemouse_th: SpaceMouseListener, conf: dict = {}):
        super().__init__(conf)
        self.spacemouse_th = spacemouse_th
        self.spacemouse_th.sig_data.connect(self.on_spacemouse_btn_clicked)
        self.arm = arm
        # self.is_gripper_open = True  # True = open
        self.is_run = False
        self.joystick_enable = bool(1)   # 是否启用手柄
        
        # 控制参数（可按需调整）
        self.SCALE_XYZ = 0.2
        self.SCALE_RPY = 0.3
        self.ZERO_THRESHOLD = 0.001  # 认为输入为零的阈值
        self.LOOP_SLEEP_MS = 10  # 空闲时休眠（ms）
        self.is_opened = 1

    # @enable_debugpy(1)
    def on_spacemouse_btn_clicked(self, payload: dict):
        """处理 spacemouse 按钮事件，`payload` 格式: 
        ```
        'event': 'single' / 'double',
        'btn': 'btn1' / 'btn2' ...,
        'state': {...}
        ```
        """
        printc(f"on_spacemouse_btn_clicked: {payload}")
        self.set_joystick_enable(0)
        
        event_type = payload.get('event')  # 'single' or 'double'
        btn_name = payload.get('btn')  # 'gripper' or 'gozero'
        
        # 左键单击 -> 执行 gripper
        if event_type == 'single' and btn_name == 'btn1':
            printc("左键单击")
            self.single_click_front()
        
        # 左键双击 -> 执行 func
        elif event_type == 'double' and btn_name == 'btn1':
            printc("左键双击")
            self.double_click_front()
        
        # 右键单击 -> 执行 gozero
        elif event_type == 'single' and btn_name == 'btn2':
            printc("右键单击")
            self.single_click_back()
        
        # 右键双击 -> 执行 func
        elif event_type == 'double' and btn_name == 'btn2':
            printc("右键双击")
            self.double_click_back()

        self.set_joystick_enable(1)
        self.msleep(500)

    def set_joystick_enable(self, enable: int):
        self.joystick_enable = bool(enable)
        if enable:
            printc("启用摇杆")
        else:
            printc("禁用摇杆")

    def check_arm(self):
        if not self.arm.is_connected:
            printc("请先连接机械臂!!") 
        assert self.arm.is_connected == 1, "请先连接机械臂"

    def set_gripper(self):
        if not self.arm.is_hand_opened:
            self.arm.gripper_open()
        else:
            self.arm.gripper_close()

    def set_hand(self):
        self.check_arm()
        if not self.arm.is_hand_opened:
            self.arm.hand_open()
        else:
            self.arm.hand_close()

    def set_gripper_or_hand(self):
        if IS_HAND_MODE:
            self.set_hand()
        else:
            self.set_gripper()


    def single_click_front(self):
        """自定义功能函数，单击左键时执行"""
        if IS_HAND_MODE:
            printc("执行自定义功能 single_click_front: 设置灵巧手抓握")
            if self.is_opened:
                self._4finge_pick()
                self.is_opened = 0
            else:
                self._4finge_release()
                self.is_opened = 1
        else:
            printc("执行自定义功能 single_click_front: 设置夹爪抓握")
            self.set_gripper()

    def single_click_back(self):
        """自定义功能函数，单击左键时执行"""
        printc("执行自定义功能 single_click_back: 回零位")
        self.arm.gozero()

    def double_click_back(self):
        """自定义功能函数，双击左键时执行"""
        printc("执行自定义功能 double_click_back")

        # 预放置位置，小桌板上面一段距离
        approach_pose = APPCFG['approach_pose']
        _arm = self.arm.robot
        ret = _arm.rm_movej_p(approach_pose, 15, 0, 0, 1)

        ret = _arm.rm_set_teach_frame(1)
        # 在小桌板上面下移15cm
        ret = _arm.rm_set_pos_step(rm_pos_teach_type_e.RM_X_DIR_E, APPCFG['x_dir_e'], 15, 1)
        ret = _arm.rm_set_teach_frame(0)

        # 打开夹爪
        self.arm.gripper_open()

        ret = _arm.rm_set_teach_frame(1)
        # # 松开夹爪收，后撤12cm
        ret = _arm.rm_set_pos_step(rm_pos_teach_type_e.RM_Z_DIR_E, APPCFG['z_dir_e'], 15, 1)
        ret = _arm.rm_set_teach_frame(0)

        # 回到默认位置
        # ret = _arm.rm_movej_p([-0.3, 0, 0.6, 3.142, -1.047, 3.142], 10, 0, 0, 1)
        ret = _arm.rm_movec(
            APPCFG['go_zero_pose_via'], 
            APPCFG['go_zero_pose_to'],
             20, 0, 0, 0, 1)
        printc(f"movec ret={ret}")

    def double_click_front(self):
        """自定义功能函数，双击左键时执行"""
        if IS_HAND_MODE:
            printc(f"执行自定义功能 double_click_front: 设置灵巧手捏住")
            if self.is_opened:
                self._2finge_pick()
                self.is_opened = 0
            else:
                self._2finge_release()
                self.is_opened = 1
        else:
            printc(f"执行自定义功能 double_click_front: 设置夹爪捏住")
            self.set_gripper()

    def _2finge_release(self):
        self.arm.hand.open_2finger()
        self._hand_sleep()
        self.arm.hand.rotate_thumb(0)

    def _2finge_pick(self):
        self.arm.hand.rotate_thumb(90)
        self._hand_sleep()
        self.arm.hand.close_2finger()

    def _4finge_release(self):
        self.arm.hand.open_finger(0)
        self._hand_sleep()
        self.arm.hand.open_4finger()
        self.arm.hand.rotate_thumb(100)

    def _4finge_pick(self):
        self.arm.hand.close_4finger()
        self.arm.hand.rotate_thumb(100)
        self._hand_sleep()
        self.arm.hand.close_finger(0)
    
    def _hand_sleep(self):
        self.msleep(300)

    def _all_zero(self, cur_state: Dict[str, float]) -> bool:
        # 快速判断是否全零（只关心 motion 相关键）
        for k in ('x','y','z','R','P','Y'):
            if abs(cur_state.get(k, 0.0)) >= self.ZERO_THRESHOLD:
                return False
        return True

    def run(self):
        self.is_run = True
        while self.is_run:
            self.msleep(30)
            cur_state = self.spacemouse_th.cur_state

            if not self.joystick_enable:
                self.msleep(self.LOOP_SLEEP_MS)
                # printc("当前摇杆被禁用，等待摇杆启用...")
                continue

            # 获取当前机械臂状态（ret, data）
            ret, data = self.arm.get_pose()
            if ret != 0 or not data or 'pose' not in data:
                # 获取失败时短暂等待并跳过
                printc(f"[warn] get_pose failed or missing data, ret={ret}")
                self.msleep(self.LOOP_SLEEP_MS)
                continue

            pose = data['pose']  # [x,y,z,R,P,Y]
            # 构造便于处理的 dict
            cur_pose = {
                "x": float(pose[0]),
                "y": float(pose[1]),
                "z": float(pose[2]),
                "R": float(pose[3]),
                "P": float(pose[4]),
                "Y": float(pose[5]),
            }

            # 选择变化最大的轴（只取 motion 轴）
            axis_choice = max(('x','y','z','R','P','Y'), key=lambda k: abs(cur_state.get(k, 0.0)))
            max_val = cur_state.get(axis_choice, 0.0)

            # 如果最大值很小则视为无动作
            if abs(max_val) < self.ZERO_THRESHOLD:
                self.msleep(self.LOOP_SLEEP_MS)
                continue

            # 处理线性平移
            printc(f"SCALE_XYZ={self.SCALE_XYZ}")
            if axis_choice in ('x','y','z'):
                # 更新位置并下发 movep
                new_pose = [
                    cur_pose['x'] + (cur_state.get('x',0.0) * self.SCALE_XYZ),
                    cur_pose['y'] + (cur_state.get('y',0.0) * self.SCALE_XYZ),
                    cur_pose['z'] + (cur_state.get('z',0.0) * self.SCALE_XYZ),
                    cur_pose['R'],
                    cur_pose['P'],
                    cur_pose['Y'],
                ]
                try:
                    ret = self.arm.robot.rm_movep_canfd(new_pose, False, 1, 60)
                    printc(f"[movep xyz] ret={ret} new_pose={new_pose}")
                except Exception as e:
                    printc(f"[error] movep_xyz exception: {e}", 'err')
                continue

            # 处理姿态旋转（耦合）
            if axis_choice in ('R','P','Y'):
                theta = max_val * self.SCALE_RPY
                # roll, pitch, yaw : 绕相机 X, Y, Z 轴的旋转角度
                # roll 滚动角
                # pitch 俯仰角
                # yaw 偏航角
                axis_map = {'R':'y','P':'x','Y':'z'}
                rot_axis = axis_map[axis_choice]
                if rot_axis == "x":
                    theta = -theta
                if rot_axis == "y":
                    theta = -theta

                try:
                    # 使用缓存的算法实例做旋转并得到新的 pose
                    new_pose = self.arm.matrix_pose_rotate(
                        [cur_pose['x'], 
                         cur_pose['y'], 
                         cur_pose['z'],
                         cur_pose['R'], 
                         cur_pose['P'], 
                         cur_pose['Y']],
                        rot_axis, theta
                    )
                    # 下发 movep 命令，保持平滑（保持 current pos 平移参数）
                    ret = self.arm.robot.rm_movep_canfd(new_pose, False, 0, 60)
                    printc(f"[movep rpy] ret={ret} axis={rot_axis} theta={theta}")
                except Exception as e:
                    printc(f"[error] movep_rpy exception: {e}", 'err')
                continue

    def stop(self):
        self.is_run = False


######################################################
# 测试
# python projects/realman_teleop/bgtask/realman_arm.py
######################################################


def test_rm_client():
    arm = RealmanArmClient()

    while 1:
        try:
            state = arm.get_pose()
            printc(f"state={state}")
            time.sleep(1/1000)
        except KeyboardInterrupt:
            break


def test_rm_arm_task():
    from toolbox.qt import qtbase
    import sys

    class TestApp(qtbase.QWidget):
        def __init__(self):
            super().__init__()
            # 添加一个label
            self.label = qtbase.QLabel("SpaceMouse Data:")
            self.msg = qtbase.QLabel("")
            self._layout = qtbase.QVBoxLayout()
            self._layout.addWidget(self.label)
            self._layout.addWidget(self.msg)
            self.setLayout(self._layout)

            self.spacemouse_listener = SpaceMouseListener(devtype="SpaceMouse Compact", ui_label=self.msg)
            self.spacemouse_listener.sig_data.connect(self.on_spacemouse_data)
            self.spacemouse_listener.start()

        def on_spacemouse_data(self, data: dict):
            printc(f"SpaceMouse Data: {data}")
            self.msg.setText(str(data))

        def __del__(self):
            self.spacemouse_listener.stop()


    app = qtbase.QApplication(sys.argv)
    mapp = TestApp()
    mapp.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    test_rm_client()
