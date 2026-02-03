# 重构版：RealmanArm client + task
# 依赖：Robotic_Arm, numpy, rich
# 官方文档：https://develop.realman-robotics.com/robot/apipython/classes/roboticArm/
# pip install Robotic_Arm (当前使用版本：1.1.1)

import threading
import time

from toolbox.core.log import printc
from toolbox.qt.common.debug import enable_debugpy
from toolbox.qt import qtbase
from realman_teleop import APPCFG, IS_HAND_MODE
# from .dexhand import DexterousHand
from .calc import *  # noqa: F403


class RMArm(qtbase.QObject):
    """Realman 客户端（封装机器人/算法实例）"""
    sig_move_finger = qtbase.Signal(int, int) # index: int, position: int
    sig_connect_finished = qtbase.Signal()

    def __init__(self, arm_model: rm_robot_arm_model_e = rm_robot_arm_model_e.RM_MODEL_RM_63_III_E):
        super().__init__()
        # 缓存 Algo 实例，避免频繁创建
        force_type = rm_force_type_e.RM_MODEL_RM_B_E  # 参考原代码
        self.algo = Algo(arm_model, force_type)
        self.is_connected = 0   # 连接状态
        self.is_going_to_init_pos = 0   # 正在前往零位
    
    def myconnect(self, ip, port=8080):
        """在子线程中连接机械臂，避免阻塞主进程"""
        self.t_connect = threading.Thread(target=self._connect, args=(ip, port), daemon=True)
        self.t_connect.start()
    
    def wait_connect(self):
        printc("等待机械臂启动完成...")
        self.t_connect.join()
        time.sleep(2)
        printc("机械臂启动完成！")

    def _connect(self, ip, port=8080):
        from toolbox.core.log import printc
        self.ip = ip
        self.port = port
        
        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
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
            
        self.is_connected = 1
        self.sig_connect_finished.emit()
        
    def check(self):
        """检查连接状态"""
        err_msg = "请先连接机械臂"
        if not self.is_connected:
            printc(err_msg, 'error')
        assert self.is_connected == 1, err_msg

    def gozero(self):
        # 使用较短的参数调用，保留原意
        try:
            self.is_going_to_init_pos = 1
            self.robot.rm_movej(APPCFG['arm_zero_joint'], APPCFG['arm_speed'], 0, 0, 1)
            self.is_going_to_init_pos = 0
        except Exception as e:
            printc(f"[error] gozero failed: {e}", 'err')

    def move_p_canfd(self, pose: list[float]):
        # 包装 movep 调用并打印（原来函数名保留）
        try:
            ret = self.robot.rm_movep_canfd(pose, False, 1, 60)
            printc(f"MOVE_P: {pose}, ret={ret}")
            return ret
        except Exception as e:
            printc(f"[error] move_p_canfd failed: {e}", 'err')
            return None


    def _get_pose(self) -> tuple[int, dict[str, str]]:
        """返回 rm_get_current_arm_state 的原始结果 (ret, data)"""
        return self.robot.rm_get_current_arm_state()

    def matrix_pose_rotate(self, pose: list[float], axis: str, theta: float) -> list[float]:
        """基于缓存的 algo 对 pose 做绕轴旋转并返回新 pose"""
        return pose_by_rot_cached(self.algo, pose, rot_axis=axis, theta=theta)

    def __del__(self):
        self.mydisconnect()

    def mydisconnect(self):
        try:
            self.robot.rm_delete_robot_arm()
            self.is_connected = 0
        except Exception as e:
            printc(f"机械臂断开连接失败 e={e}", 'error')

    def get_collision(self):
        _, stage = self.robot.rm_get_collision_stage()
        return stage


    def get_empty_incr(self) -> dict:
        """获取当前增量"""
        return {
            'x': .0,
            'y': .0,
            'z': .0,
            'R': .0,
            'P': .0,
            'Y': .0,
        }

    def check_arm(self):
        msg = "请先连接机械臂"
        if not self.is_connected:
            printc(msg, 'err')
        assert self.is_connected == 1, msg

    def get_pose(self) -> dict:
        """获取机械臂当前位置"""
        self.check_arm()
        _ret, data = self._get_pose()
        pose: list = data['pose']  # ty:ignore[invalid-assignment]
        return {
            "x": pose[0],
            "y": pose[1],
            "z": pose[2],
            "R": pose[3],
            "P": pose[4],
            "Y": pose[5],
        }
    
    def get_incr(self, key: str, step_xyz: float, step_RPY: float) -> dict:
        """获取当前增量"""
        incr = self.get_empty_incr()
        # step_xyz = self.pos_vel
        # step_RPY = self.rot_vel

        if key == "A":
                incr['x'] = step_xyz
        elif key == "D":
                incr['x'] = -step_xyz
        elif key == "W":
                incr['y'] = step_xyz
        elif key == "S":
                incr['y'] = -step_xyz
        elif key == "Q":
                incr['z'] = step_xyz
        elif key == "Z":
                incr['z'] = -step_xyz

        elif key == "U":
                incr['R'] = step_RPY
        elif key == "J":
                incr['R'] = -step_RPY
        elif key == "I":
                incr['P'] = step_RPY
        elif key == "K":
                incr['P'] = -step_RPY
        elif key == "O":
                incr['Y'] = step_RPY
        elif key == "L":
                incr['Y'] = -step_RPY
        return incr

    def incr_has_xyz(self, incr: dict) -> bool:
        """判断增量是否包含 xyz 轴"""
        return incr['x'] or incr['y'] or incr['z']

    def incr_has_RPY(self, incr: dict) -> bool:
        """判断增量是否包含 RPY 轴"""
        return incr['R'] or incr['P'] or incr['Y']

    def get_new_pose(self, pose: dict, incr: dict) -> dict:
        """根据增量获取新位置"""
        pose_ = pose.copy()
        if self.incr_has_xyz(incr):
            for i, k in enumerate(['x','y','z']):
                pose_[k] += incr[k]
        if self.incr_has_RPY(incr):
            for i, k in enumerate(['R','P','Y']):
                pose_[k] += incr[k]
        return pose_

    def pose_to_list(self, pose: dict) -> list:
        """将 pose 转换为列表"""
        return [pose['x'], pose['y'], pose['z'], pose['R'], pose['P'], pose['Y']]


    def kb(self, key, pos_vel, rot_vel):
        pose = self.get_pose()
        incr = self.get_incr(key, pos_vel, rot_vel)
        printc(f"incr={incr}")

        # if key == "G":
        #     self.arm.gozero()
        #     self.add_log("回到零位")
        #     return
        
        # pose_bak = pose.copy()

        # 处理 xyz 轴 --------------------------
        if self.incr_has_xyz(incr):
            pose_ = self.get_new_pose(pose, incr)
            ret = self.robot.rm_movep_canfd(self.pose_to_list(pose_), False, 1, 60)
            
            # if self.VERBOSE:
            #     printc("xyz" + "-"*50)
            #     printc(f"ret={ret}, incr={incr}")
            #     printc(f"new_pose={pose_}")
            #     return
        
        # 处理 RPY 轴 --------------------------
        if self.incr_has_RPY(incr):
            pose_ = self.get_new_pose(pose, incr)
            theta = incr['R'] if incr['R'] else incr['P'] if incr['P'] else incr['Y']

            if incr['R']:
                new_pose = self.matrix_pose_rotate(self.pose_to_list(pose_), 'x', theta)
            elif incr['P']:
                new_pose = self.matrix_pose_rotate(self.pose_to_list(pose_), 'y', theta)
            elif incr['Y']:
                new_pose = self.matrix_pose_rotate(self.pose_to_list(pose_), 'z', theta)
            else:
                raise ValueError
            
            ret = self.robot.rm_movep_canfd(new_pose, False, 0, 60)

            # if self.VERBOSE:
            #     printc("RPY" + "-"*50)
            #     printc(f"ret={ret}, incr={incr}")
            #     printc(f"ret={ret}, new_pose={new_pose}")
