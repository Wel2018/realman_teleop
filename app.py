import sys
import threading
import time
import numpy as np
from rich import print
from toolbox.core.time_op import get_time_str
from toolbox.qt import qtbase
from .ui.ui_form import Ui_DemoWindow
from . import q_appcfg, logger
from .bgtask.spacemouse import SpaceMouseListener
from .bgtask.realman_arm import RealmanArmClient


USE_SPACEMOUSE = 1
USE_ARM = 1


# import str
import jurigged
jurigged.watch("./")


class SharedData:
    """共享数据"""
    incr = {
        "x": .0,
        "y": .0, 
        "z": .0,
        "R": .0,
        "P": .0,
        "Y": .0,
        "gripper": 0, # 0 表示没有动作，1 表示打开夹爪，-1 表示关闭夹爪
    }
    incr_bak = {}


class MainWindow(qtbase.QApp):
    """应用具体实现"""
    # 定时器和线程名称
    TH_CTL_MODE = "teleop_ctl_mode"
    TIMER_ROBOT_STATE = "robot_state"

    is_quit_confirm = 0  # 程序退出确认
    is_keyboard_ctrl = 1  # 键盘控制开关
    # is_collect_data = 0  # 数据集收集开关
    is_gripper_open = 1  # 当前夹爪状态
    is_mirror = 0  # 是否镜像操控 
    is_debug = 0
    is_going_to_init_pos = 0
    VERBOSE = q_appcfg.VERBOSE

    def pre_init(self):
        return super().pre_init()
    
    def post_init(self):
        ui = self.ui
        # 绑定点击事件
        self.bind_clicked(ui.btn_clear, self.clean_log)
        # self.bind_clicked(ui.btn_gripper, self.set_gripper)
        
        # 遥操作控制步长改变
        # 线速度、角速度
        self.pos_vel = ui.step_posi_vel.value()
        self.rot_vel = ui.step_angle_vel.value()
        self.max_duration = 1000*60
        
        # # 在 lambda 表达式中不能使用赋值语句
        self.bind_val_changed(
            ui.step_posi_vel, 
            lambda val: \
                setattr(self, 'pos_vel', round(val,3))
        )
        
        self.bind_val_changed(
            ui.step_angle_vel, 
            lambda val: \
                setattr(self, 'rot_vel', round(val,3))
        )

    def load_device(self):
        # 机械臂 --------------------
        if USE_ARM:
            self.arm = RealmanArmClient()
            pose = self.arm.get_pose()
            self.add_log(f"pose={pose}")
        self.add_log("程序初始化完成", color="green")
        # self.arm.robot.rm_set_tool_do_state

        # 遥操作 -----------------------
        if USE_SPACEMOUSE:
            self.add_log("SpaceMouse 控制")
            self.spacemouse_th = SpaceMouseListener(devtype="SpaceMouse Compact")
            # self.spacemouse_th.bind(on_data=self.spacemouse_cb, on_msg=self.add_log)
            self.add_th(self.TH_CTL_MODE, self.spacemouse_th, 1)
        
        from .bgtask.realman_arm import RealmanArmTask
        self.arm_task = RealmanArmTask(self.arm, self.spacemouse_th)
        self.add_th("arm_task", self.arm_task, 1)

        return super().post_init()

    def __init__(self, parent = None):
        ui = self.ui = Ui_DemoWindow()
        super().__init__(ui, parent, q_appcfg)
        self.pre_init()
        self.init(ui_logger=ui.txt_log, logger=logger)
        self.post_init()
        self.load_device()

    def play(self):
        """执行任务理解逻辑"""
        self.add_log("play")

    def get_empty_incr(self):
        incr = {
            "x": .0,
            "y": .0, 
            "z": .0,
            "R": .0,
            "P": .0,
            "Y": .0,
            "gripper": 0, # 0 表示没有动作，1 表示打开夹爪，-1 表示关闭夹爪
        }
        return incr

    def keyPressEvent(self, event: qtbase.QKeyEvent):
        """按下按键：键盘打开 caps lock 模式，可以实现长按模式
        （即按住 A，只会触发一次 keyPressEvent，不会连续触发，松开也是只触发一次）
        - 键盘长按会在第一次 isAutoRepeat=False, 之后是 True
        """
        key = event.text().upper()
        print(f"press {key}")

        if event.key() == qtbase.qt_keys.Key_F5:
            self.reload()
            self.add_log("QApp reload", color="red")
            return


        ret, data = self.arm.get_pose()
        pose: list = data['pose']
        _pose = {
            "x": pose[0],
            "y": pose[1],
            "z": pose[2],
            "R": pose[3],
            "P": pose[4],
            "Y": pose[5],
        }
        incr = {
            'x': .0,
            'y': .0,
            'z': .0,
            'R': .0,
            'P': .0,
            'Y': .0,
        }
        step = 0.01

        # xyz
        if key == "A":
            incr['x'] = step
        elif key == "D":
            incr['x'] = -step
        elif key == "W":
            incr['y'] = -step
        elif key == "S":
            incr['y'] = step
        elif key == "Q":
            incr['z'] = step
        elif key == "Z":
            incr['z'] = -step

        # RPY
        step *= 10
        if key == "U":
            incr['R'] = step
        elif key == "J":
            incr['R'] = -step
        elif key == "I":
            incr['P'] = step
        elif key == "K":
            incr['P'] = -step
        elif key == "O":
            incr['Y'] = step
        elif key == "L":
            incr['Y'] = -step

        if key == "G":
            self.arm.gozero()
            self.add_log("回到零位")
            return

        for i, k in enumerate(['x','y','z','R','P','Y']):
            _pose[k] += incr[k]

        
        ret = self.arm.robot.rm_movep_canfd(list(_pose.values()), False, 1, 60)
        print(f"ret={ret}, incr={incr}")
        print(f"ret={ret}, pose={pose}")

        if not event.isAutoRepeat():
            # self.add_key(key)

            if self.VERBOSE:
                print(f"keyPressEvent {event}")
            # incr = self.get_empty_incr()

        # return super().keyPressEvent(event)
    def keyReleaseEvent(self, event: qtbase.QKeyEvent):
        # return super().keyReleaseEvent(event)
        ...


    def close_ready(self):
        ...
    
    
def main():
    qapp = qtbase.QApplication(sys.argv)
    # 设置全局默认字体
    qapp.setFont(qtbase.QFont("微软雅黑", 11))
    mapp = MainWindow()
    mapp.show()
    sys.exit(qapp.exec())
