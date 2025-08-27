import sys
import threading
import numpy as np
from rich import print
from toolbox.core.time_op import get_time_str
from toolbox.qt import qtbase
from .ui.ui_form import Ui_DemoWindow
from . import q_appcfg, logger
from .bgtask.spacemouse import SpaceMouseListener
from .bgtask.realman_arm import RealmanArmClient


USE_SPACEMOUSE = 0
USE_KEYBOARD = 0
USE_ARM = 0


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
        self.bind_clicked(ui.btn_gripper, self.set_gripper)
        
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

        # 遥操作 -----------------------
        if USE_SPACEMOUSE:
            self.add_log("SpaceMouse 控制")
            self.spacemouse_th = SpaceMouseListener(devtype="SpaceMouse Compact")
            self.spacemouse_th.bind(on_data=self.spacemouse_cb, on_msg=self.add_log)
            self.add_th(self.TH_CTL_MODE, self.spacemouse_th, 1)
        return super().post_init()

    def __init__(self, parent = None):
        ui = self.ui = Ui_DemoWindow()
        super().__init__(ui, parent, q_appcfg)
        self.pre_init()
        self.init(ui_logger=ui.txt_log, logger=logger)
        self.post_init()
        self.load_device()


    @qtbase.Slot(dict)
    def spacemouse_cb(self, data: dict):
        if self.is_going_to_init_pos:
            print("正在返回初始位置，忽略")
            return
        
        # ZERO = 0.001
        # if data['x'] <= ZERO and data['y'] <= ZERO and data['z'] <= ZERO:
        #     return
        
        if q_appcfg.VERBOSE:
            print(f"{get_time_str(2)} data={data}")
            
        # 所有按键的数据都已同步
        incr_prev = SharedData.incr
        
        if data['gripper']:
            self.set_gripper()
            return
        
        if data['gozero']:
            # self.arm.join(1)
            # self.arm.stop()
            # self.gozero()
            print("gozero")
            return
        
        # if data['collect']:
        #     self.kb_collect()
        #     return
        
        # 将 spacemouse 数据格式转为 incr 字典
        _incr = {}
        for k in ['x','y','z']:
            _incr[k] = data[k] if data[k] else 0
        for k in ['R','P','Y']:
            _incr[k] = data[k] if data[k] else 0

        # print(f"{get_time_str(2)} {_incr}")
        # self.arm.cartesian_velocity_control(_incr)
        _p = self.arm.get_pose()
        xyzRPY = _p[1]['pose']

        pose = {
            'x': xyzRPY[0],
            'y': xyzRPY[1],
            'z': xyzRPY[2],
            'R': xyzRPY[3],
            'P': xyzRPY[4],
            'Y': xyzRPY[5],
        }

        for k in pose.keys():
            pose[k] += _incr[k]
        
        # print(pose)
        self.arm.move_p_canfd(list(pose.values()))
        SharedData.incr.update(_incr)
        

    def play(self):
        """执行任务理解逻辑"""
        self.add_log("play")


    def gozero(self):
        """回到初始位置"""
        self.add_log("G：回到初始位置")
        if not self.is_going_to_init_pos:
            self.add_log("正在回到初始位置中...")
            self.is_going_to_init_pos = 1
            # self.arm.goto_init_pos()
            self.is_going_to_init_pos = 0
            self.add_log("机械臂已归位！")

    def set_gripper(self):
        """夹爪控制"""
        self.is_gripper_open = not self.is_gripper_open
        if self.is_gripper_open:
            self.add_log("打开夹爪")
            self.arm.gripper_open()
        else:
            self.add_log("关闭夹爪")
            self.arm.gripper_close()


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
        # print(f"press {key}")

        if event.key() == qtbase.qt_keys.Key_F5:
            self.reload()
            self.add_log("QApp reload", color="red")
            return

        if USE_KEYBOARD:
            ret, data = self.arm.get_pose()
            pose: list = data['pose']
            incr = {
                'x': .0,
                'y': .0,
                'z': .0,
                'R': .0,
                'P': .0,
                'Y': .0,
            }
            step = 0.01

            if key == "A":
                incr['y'] = -step
            elif key == "D":
                incr['y'] = step
            elif key == "W":
                incr['x'] = step
            elif key == "S":
                incr['x'] = -step
            elif key == "Q":
                incr['z'] = step
            elif key == "Z":
                incr['z'] = -step

            for i, k in enumerate(['x','y','z','R','P','Y']):
                pose[i] += incr[k]

            ret = self.arm.robot.rm_movep_canfd(pose, False, 1, 60)

            print(f"ret={ret}, pose={pose}")

        if not event.isAutoRepeat():
            if self.VERBOSE:
                print(f"keyPressEvent {event}")

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
