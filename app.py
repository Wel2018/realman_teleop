import sys
import threading
import time
import numpy as np
from rich import print
from toolbox.qt import qtbase
from .ui.ui_form import Ui_DemoWindow
from . import q_appcfg, logger
from .bgtask.spacemouse import SpaceMouseListener
from .bgtask.realman_arm import RealmanArmClient


USE_SPACEMOUSE = 0
USE_ARM = 1


if q_appcfg.APPCFG_DICT['HOTRELOAD']:
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
        self.bind_clicked(ui.btn_stop, self.arm_stop)
        self.bind_clicked(ui.btn_recover, self.arm_recover)
        self.bind_clicked(ui.btn_set_collision, self.arm_set_collision)
        self.bind_clicked(ui.btn_gozero, self.gozero)
        self.bind_clicked(ui.btn_spacemouse_start, self.spacemouse_start)
        self.bind_clicked(ui.btn_spacemouse_stop, self.spacemouse_stop)
        
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
            self.spacemouse_start()

    def arm_set_collision(self):
        ret1 = self.arm.robot.rm_get_collision_stage()
        # arm.rm_get_collision_stage()
        stage = self.ui.spin_collision_state.value()
        ret = self.arm.robot.rm_set_collision_state(stage)
        ret2 = self.arm.robot.rm_get_collision_stage()
        self.add_log(f"设置碰撞等级: {stage}, ret={ret}", color='green')
        self.add_log(f"碰撞防护等级：{ret1} → {ret2}")

    def arm_recover(self):
        ret_err = self.arm.robot.rm_get_joint_err_flag()
        ret, joint_en_states = self.arm.robot.rm_get_joint_en_state()
        self.add_log(f"ret_err={ret_err}")
        self.add_log(f"joint_en_states={joint_en_states}")
        # self.arm.robot.rm_set_joint_clear_err(0)
        # self.arm.robot.rm_set_joint_en_state(0, 1)
        brake_state = ret_err['brake_state']
        for i in range(6):
            if brake_state[i]:
                self.arm.robot.rm_set_joint_clear_err(i)
                time.sleep(0.1)
                self.arm.robot.rm_set_joint_en_state(i, 1)
                time.sleep(0.1)
                self.add_log(f"机械臂第 {i} 号关节上使能", color='green')
        self.gozero()
        self.add_log("机械臂故障恢复完成", color='green')

    def arm_stop(self):
        ret = self.arm.robot.rm_set_arm_stop()
        self.add_log("机械臂急停", color="red")

    def spacemouse_start(self):
        if self.th.get(self.TH_CTL_MODE, None) is not None:
            self.add_log("SpaceMouse 服务已启动，跳过", color='yellow')
            return

        self.add_log("SpaceMouse 服务启动", color='green')
        # devtype="SpaceMouse Pro Wireless"
        devtype="SpaceMouse Compact"
        self.spacemouse_th = SpaceMouseListener(devtype=devtype)
        # self.spacemouse_th.bind(on_data=self.spacemouse_cb, on_msg=self.add_log)
        self.add_th(self.TH_CTL_MODE, self.spacemouse_th, 1)
    
        from .bgtask.realman_arm import RealmanArmTask
        self.arm_task = RealmanArmTask(self.arm, self.spacemouse_th)
        self.add_th("arm_task", self.arm_task, 1)

    def spacemouse_stop(self):
        self.add_log("SpaceMouse 服务退出", color='yellow')
        self.stop_th(self.TH_CTL_MODE)
        self.stop_th("arm_task")
        self.th.pop(self.TH_CTL_MODE, None)
        self.th.pop("arm_task", None)


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

    def gozero(self):
        """回到初始位置"""
        self.add_log("机械臂回到初始位置（快捷键：G）")
        if not self.is_going_to_init_pos:
            self.add_log("正在回到初始位置中...")
            self.is_going_to_init_pos = 1
            self.arm.gozero()
            self.is_going_to_init_pos = 0
            self.add_log("机械臂已归位！")


    def get_pose(self) -> dict:
        """获取机械臂当前位置"""
        ret, data = self.arm.get_pose()
        pose: list = data['pose']
        return {
            "x": pose[0],
            "y": pose[1],
            "z": pose[2],
            "R": pose[3],
            "P": pose[4],
            "Y": pose[5],
        }
    
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

    def get_incr(self, key: str) -> dict:
        """获取当前增量"""
        incr = self.get_empty_incr()
        step_xyz = 0.01
        step_RPY = 0.1

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

        pose = self.get_pose()
        incr = self.get_incr(key)

        if key == "G":
            self.arm.gozero()
            self.add_log("回到零位")
            return
        
        # pose_bak = pose.copy()

        # 处理 xyz 轴 --------------------------
        if self.incr_has_xyz(incr):
            pose_ = self.get_new_pose(pose, incr)
            ret = self.arm.robot.rm_movep_canfd(self.pose_to_list(pose_), False, 1, 60)

            print("xyz", "-"*50)
            print(f"ret={ret}, incr={incr}")
            print(f"new_pose={pose_}")
            return
        
        # 处理 RPY 轴 --------------------------
        if self.incr_has_RPY(incr):
            pose_ = self.get_new_pose(pose, incr)
            theta = incr['R'] if incr['R'] else incr['P'] if incr['P'] else incr['Y']

            if incr['R']:
                new_pose = self.arm.matrix_pose_rotate(self.pose_to_list(pose_), 'x', theta)
            elif incr['P']:
                new_pose = self.arm.matrix_pose_rotate(self.pose_to_list(pose_), 'y', theta)
            elif incr['Y']:
                new_pose = self.arm.matrix_pose_rotate(self.pose_to_list(pose_), 'z', theta)
            else:
                raise ValueError
            
            ret = self.arm.robot.rm_movep_canfd(new_pose, False, 0, 60)
        
            print("RPY", "-"*50)
            print(f"ret={ret}, incr={incr}")
            print(f"ret={ret}, new_pose={new_pose}")


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
