from __future__ import annotations
from typing import TYPE_CHECKING
import time
from toolbox.core.log import printc
from toolbox.qt import qtbase
from realman_teleop import IS_HAND_MODE, APPCFG
from realman_teleop.core.shared import Shared
from Robotic_Arm.rm_robot_interface import *  # noqa: F403


if TYPE_CHECKING:
    from realman_teleop.core.spacemouse import SpaceMouseListener
    from realman_teleop.core.end_effector.dexhand import DexterousHand
    from realman_teleop.core.end_effector.gripper import Gripper
    from realman_teleop.core.arm.realman import RMArm


class TeleopTask(qtbase.QAsyncTask):
    def __init__(
        self, 
        arm: RMArm, 
        spacemouse_th: SpaceMouseListener, 
        gripper: Gripper|None=None,
        dexhand: DexterousHand|None=None,
        conf: dict = {}, 
    ):
        super().__init__(conf)
        self.spacemouse_th = spacemouse_th
        self.spacemouse_th.sig_data.connect(self.on_spacemouse_btn_clicked)
        self.arm = arm
        self.gripper = gripper
        self.dexhand = dexhand

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
        Shared.is_joystick_enable = bool(enable)
        if enable:
            printc("启用摇杆")
        else:
            printc("禁用摇杆")

    def single_click_front(self):
        """自定义功能函数，单击左键时执行"""
        # 灵巧手模式
        if Shared.smx.is_dexhand:
            printc("执行自定义功能 single_click_front: 设置灵巧手抓握")
            assert self.dexhand
            if Shared.is_ee_opened:
                self.dexhand.five_finge_pick()
            else:
                self.dexhand.five_finge_release()
        # 夹爪模式
        else:
            printc("执行自定义功能 single_click_front: 设置夹爪抓握")
            assert self.gripper
            if Shared.is_ee_opened:
                self.gripper.pick()
            else:
                self.gripper.release()

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
        ret = _arm.rm_set_pos_step(rm_pos_teach_type_e.RM_X_DIR_E, APPCFG['x_dir_e'], 15, 1)  # noqa: F405
        ret = _arm.rm_set_teach_frame(0)

        # 打开夹爪
        self.arm.gripper_open()

        ret = _arm.rm_set_teach_frame(1)
        # # 松开夹爪收，后撤12cm
        ret = _arm.rm_set_pos_step(rm_pos_teach_type_e.RM_Z_DIR_E, APPCFG['z_dir_e'], 15, 1)  # noqa: F405
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
        # 灵巧手模式
        if Shared.smx.is_dexhand:
            assert self.dexhand
            printc(f"执行自定义功能 double_click_front: 设置灵巧手捏住")
            if Shared.is_ee_opened:
                self.dexhand.two_finge_pick()
            else:
                self.dexhand.two_finge_release()
        # 夹爪模式
        else:
            assert self.gripper
            printc(f"执行自定义功能 double_click_front: 设置夹爪捏住")
            if Shared.is_ee_opened:
                self.gripper.pick()
            else:
                self.gripper.release()

    def _all_zero(self, cur_state: dict[str, float]) -> bool:
        # 快速判断是否全零（只关心 motion 相关键）
        for k in ('x','y','z','R','P','Y'):
            if abs(cur_state.get(k, 0.0)) >= Shared.ZERO_THRESHOLD:
                return False
        return True

    def run(self):  # noqa: C901
        self.is_run = True
        while self.is_run:
            self.msleep(30)
            cur_state = self.spacemouse_th.cur_state

            if not Shared.is_joystick_enable:
                self.msleep(Shared.LOOP_SLEEP_MS)
                # printc("当前摇杆被禁用，等待摇杆启用...")
                continue

            # 获取当前机械臂状态（ret, data）
            ret, data = self.arm.get_pose()
            if ret != 0 or not data or 'pose' not in data:
                # 获取失败时短暂等待并跳过
                printc(f"get_pose failed or missing data, ret={ret}", 'error')
                self.msleep(Shared.LOOP_SLEEP_MS)
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
            if abs(max_val) < Shared.ZERO_THRESHOLD:
                self.msleep(Shared.LOOP_SLEEP_MS)
                continue

            # 处理线性平移
            printc(f"SCALE_XYZ={Shared.SCALE_XYZ}")
            if axis_choice in ('x','y','z'):
                # 更新位置并下发 movep
                new_pose = [
                    cur_pose['x'] + (cur_state.get('x',0.0) * Shared.SCALE_XYZ),
                    cur_pose['y'] + (cur_state.get('y',0.0) * Shared.SCALE_XYZ),
                    cur_pose['z'] + (cur_state.get('z',0.0) * Shared.SCALE_XYZ),
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
                theta = max_val * Shared.SCALE_RPY
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
