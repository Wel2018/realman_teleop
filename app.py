import os
import sys
import threading
import time
import json
import requests
from rich import print
from realman_teleop.bgtask.runner import Runner
from realman_teleop.log import init_logger, printc
from loguru import logger
from toolbox.comm.server_echo import ServerEcho
# from . import printc
from toolbox.core.file_op import open_local_file
from toolbox.qt import qtbase
from .ui.ui_form import Ui_DemoWindow
from . import q_appcfg
from . import APPCFG, IS_HAND_MODE, API_IP, API_PORT
from . import ARM_IP, API_PORT, API_PRE
from .bgtask.spacemouse import SpaceMouseListener
from .bgtask.realman_arm import RealmanArmClient
from .bgtask.realman_arm import RealmanArmTask
from .util import set_layout_visible, set_layout_enabled

echo = ServerEcho()


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

        self.set_theme("blue")

        # 绑定点击事件
        self.bind_clicked(ui.btn_clear, self.clean_log)
        self.bind_clicked(ui.btn_stop, self.arm_stop)
        self.bind_clicked(ui.btn_recover, self.arm_recover)
        self.bind_clicked(ui.btn_set_collision, self.arm_set_collision)
        self.bind_clicked(ui.btn_gozero, self.gozero)
        self.bind_clicked(ui.btn_spacemouse_start, self.spacemouse_start)
        self.bind_clicked(ui.btn_spacemouse_stop, self.spacemouse_stop)
        self.bind_clicked(ui.btn_arm_connect, self.arm_connect)
        self.bind_clicked(ui.btn_arm_disconnect, self.arm_disconnect)
        self.bind_clicked(ui.btn_setting, self.setting)

        # 灵巧手模式
        if IS_HAND_MODE:
            self.bind_clicked(ui.btn_gripper, self.set_gripper_or_hand)
            self.bind_clicked(ui.btn_2finge_pick, self._2finge_pick)
            self.bind_clicked(ui.btn_2finge_release, self._2finge_release)
            self.bind_clicked(ui.btn_4finge_pick, self._4finge_pick)
            self.bind_clicked(ui.btn_4finge_release, self._4finge_release)
            self.bind_clicked(ui.btn_rotate_thumb, self.rotate_thumb)
            ui.btn_gripper.setText("灵巧手")

        # 夹爪模式
        else:
            self.bind_clicked(ui.btn_gripper, self.set_gripper_or_hand)
            qtbase.set_enable(ui.btn_2finge_pick, 0)
            qtbase.set_enable(ui.btn_2finge_release, 0)
            qtbase.set_enable(ui.btn_4finge_pick, 0)
            qtbase.set_enable(ui.btn_4finge_release, 0)
            
            # 隐藏灵巧手相关的组件
            set_layout_enabled(ui.hand_layout, bool(0))

        # 遥操作控制步长改变
        # 默认速度
        self.ui.step_posi.setValue(APPCFG['vel_linear_keyboard'])  # 键盘速度控制，线速度
        self.ui.step_angle.setValue(APPCFG['vel_angular_keyboard'])  # 键盘速度控制，角速度
        self.ui.step_posi2.setValue(APPCFG['vel_linear_spaceouse'])  # 鼠标速度控制，线速度
        self.ui.step_angle2.setValue(APPCFG['vel_angular_spaceouse'])  # 鼠标速度控制，角速度

        # 线速度、角速度
        self.pos_vel = ui.step_posi.value()
        self.rot_vel = ui.step_angle.value()
        self.is_spacemouse_running = 0

        self.max_duration = 1000*60
        # self.keys_pressed = set()
        
        # # 在 lambda 表达式中不能使用赋值语句
        # 修改键盘控制模式下的速度
        self.bind_val_changed(
            ui.step_posi, 
            lambda val: \
                setattr(self, 'pos_vel', round(val,3))
        )
        
        self.bind_val_changed(
            ui.step_angle, 
            lambda val: \
                setattr(self, 'rot_vel', round(val,3))
        )

        # 定时器: 每秒检查一次 spacemouse_usable 的值，判断当前 3d 鼠标是否可用，可以介入机械臂的控制
        # 0->1 触发 spacemouse_start
        # 1->0 触发 spacemouse_stop
        self.ui.spacemouse_usable.setChecked(bool(0))
        self.spacemouse_usable = bool(0)
        self.spacemouse_usable_msg = ""

        self.ui.msg.setText(f"服务地址: {API_IP}:{API_PORT}")
        self.ui.arm_ip.setText(ARM_IP)

        # 机械臂实例
        self.arm = RealmanArmClient()
        self.arm.sig_connect_finished.connect(self.connect_finished)
        self.runner = Runner(ui.spacemouse_trigger_mode, ui.spacemouse_usable)
        self.runner.sig_on.connect(self.runner_on)
        self.runner.sig_off.connect(self.runner_off)
        self.runner.sig_msg.connect(self.add_log)
        self.runner.start()

        self.ui.btn_arm_connect.setEnabled(bool(1))
        self.ui.btn_arm_disconnect.setEnabled(bool(0))

        # init ok
        self.add_log("初始化完成", color="green")

    def runner_on(self):
        self.arm_connect()
        self.spacemouse_start()

    def runner_off(self):
        self.spacemouse_stop()
        self.arm_disconnect()

    def rotate_thumb(self):
        v = self.ui.rotate_thumb.value()
        self.arm.hand.rotate_thumb(v)
        self.ui.angle_f0.setValue(v)

    def _get_arm_hand_curr(self):
        if not hasattr(self.arm, "hand"):
            printc("arm 没有初始化 hand 灵巧手！")
            return
        
        self.is_t_run = 1
        while self.is_t_run:
            curr = self.arm.hand.get_current_positions()
            printc(f"curr={curr}")
            if curr is None:
                continue
            self.ui.pos_f0.setValue(curr[0])
            self.ui.pos_f1.setValue(curr[1])
            self.ui.pos_f2.setValue(curr[2])
            self.ui.pos_f3.setValue(curr[3])
            self.ui.pos_f4.setValue(curr[4])
            time.sleep(1)

    def set_gripper(self):
        self.check_arm()
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
        self.arm.hand.rotate_thumb(0)

    def _4finge_pick(self):
        self.arm.hand.close_4finger()
        self.arm.hand.rotate_thumb(100)
        self._hand_sleep()
        self.arm.hand.close_finger(0)

    def _hand_sleep(self):
        time.sleep(0.3)

    def connect_finished(self):
        """尝试连接机械臂完成，可能没有连接成功"""
        if self.arm.is_connected:
            self.add_log("机械臂连接成功", color='green')
            self.add_timer("timer_update_msg", 200, self.update_msg, 1)
            stage = self.arm_get_collision()
            self.ui.spin_collision_state.setValue(stage)
            self.add_log(f"当前碰撞等级: {stage} [范围: 1-8]", color='green')
            self.add_log(f"【遥控功能】如需使用 SpaceMouse 遥操作需手动开启服务，不再使用时请手动关闭该服务", color='#ffab70')
            self.add_log("程序初始化完成", color="green")
            self.ui.label_arm.setStyleSheet("color: green; background-color: #03db6b;")
            self.ui.btn_arm_connect.setEnabled(bool(0))
            self.ui.btn_arm_disconnect.setEnabled(bool(1))
            
            # 启动灵巧手状态实时获取线程
            if IS_HAND_MODE:
                self.t = threading.Thread(target=self._get_arm_hand_curr, daemon=True)
                self.t.start()
        else:
            self.add_log("机械臂连接失败", color='red')
            self.ui.label_arm.setStyleSheet("color: red; background-color: #f4cce4;")
            self.ui.btn_arm_connect.setEnabled(bool(1))
            self.ui.btn_arm_disconnect.setEnabled(bool(0))
        

    def arm_connect(self):
        """机械臂连接"""
        if self.arm.is_connected:
            self.add_log("机械臂已经连接")
            return
         
        self.ui.arm_ip.setText(ARM_IP)
        ip = self.ui.arm_ip.text()

        # 检查连通性
        timeout = echo.ping(ARM_IP)
        if timeout == echo.PING_TIMEOUT:
            self.add_log(f"无法 ping 通 ARM_IP={ARM_IP}，请检查配置文件！", color='red')
            return

        self.add_log(f"ping={timeout} ms, ARM_IP={ARM_IP}，正在连接...")
        self.arm.connect(ip)

        self.ui.btn_arm_connect.setEnabled(bool(0))
        self.ui.btn_arm_disconnect.setEnabled(bool(0))


    def arm_disconnect(self):
        """机械臂断开连接"""
        if not self.arm.is_connected:
            self.add_log("机械臂已经断开连接")
            return
        
        # 检查鼠标服务是否关闭
        if self.is_spacemouse_running:
             self.spacemouse_stop()
        # 断开机械臂
        self.arm.is_connected = 0
        self.remove_timer("timer_update_msg")
        self.arm.disconnect()
        self.add_log("机械臂已断开", color='green')
        self.ui.label_arm.setStyleSheet("color: red; background-color: #f4cce4;")
        if self.t.is_alive():
            #  self.t.terminate()
            self.is_t_run = 0
    
    def update_msg(self):
        """更新机械臂状态信息"""
        ret, data = self.arm.get_pose()
        j = data['joint']
        p = data['pose']
        err = data['err']
        
        p = [round(el,2) for el in p]
        j = [round(el,2) for el in j]
        # self.add_log(f"pose p={p}")
        # self.add_log(f"pose j={j}")
        self.ui.msg.setText(f"p={p}\n j={j}\n err={err}")

    def arm_set_collision(self):
        self.check_arm()
        _, stage1 = self.arm.robot.rm_get_collision_stage()
        # arm.rm_get_collision_stage()
        stage = self.ui.spin_collision_state.value()
        ret = self.arm.robot.rm_set_collision_state(stage)
        _, stage2 = self.arm.robot.rm_get_collision_stage()
        self.add_log(f"设置碰撞等级: {stage}, ret={ret}", color='green')
        self.add_log(f"碰撞防护等级状态：{stage1} → {stage2}")

    def arm_get_collision(self):
        _, stage = self.arm.robot.rm_get_collision_stage()
        return stage

    def arm_recover(self):
        self.check_arm()
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
        self.check_arm()
        ret = self.arm.robot.rm_set_arm_stop()
        self.add_log("机械臂急停", color="red")

    def spacemouse_start(self):
        self.check_arm()
        if self.task_manager.is_task_running(self.TH_CTL_MODE):
            self.add_log("SpaceMouse 服务已经启动")
            return

        self.add_log("SpaceMouse 服务启动", color='green')
        # devtype="SpaceMouse Pro Wireless"
        devtype="SpaceMouse Compact"

        # 启动 3d 鼠标监听服务
        self.spacemouse_th = SpaceMouseListener(devtype=devtype)
        self.add_th(self.TH_CTL_MODE, self.spacemouse_th, 1)
        self.ui.step_posi2.valueChanged.connect(self.spacemouse_speed_update)
        self.ui.step_angle2.valueChanged.connect(self.spacemouse_speed_update)

        # 启动机械臂循环控制服务
        self.arm_task = RealmanArmTask(self.arm, self.spacemouse_th)
        self.add_th("arm_task", self.arm_task, 1)

        # 设置状态文本
        self.ui.label_spacemouse.setStyleSheet("color: green; background-color: #03db6b;")
        self.is_spacemouse_running = 1

        # 启动时加载默认速度
        self.spacemouse_speed_update(0)

    def spacemouse_speed_update(self, value):
        if not self.is_spacemouse_running:
            return
        self.arm_task.SCALE_XYZ = self.ui.step_posi2.value()
        self.arm_task.SCALE_RPY = self.ui.step_angle2.value()

    def spacemouse_stop(self):
        self.check_arm()
        if not self.is_spacemouse_running:
            self.add_log("SpaceMouse 服务已经退出")
            return
        
        self.add_log("SpaceMouse 服务退出", color="green")
        self.stop_th(self.TH_CTL_MODE)
        self.stop_th("arm_task")
        self.ui.label_spacemouse.setStyleSheet("color: red; background-color: #f4cce4;")
        self.is_spacemouse_running = 0

    def setting(self):
         """打开配置"""
         b = os.path.dirname(__file__)
         open_local_file(os.path.join(b, "appcfg.yaml"), is_relative=1)

    def __init__(self, parent = None):
        ui = self.ui = Ui_DemoWindow()
        super().__init__(ui, parent, q_appcfg)
        self.pre_init()
        self.init(ui_logger=ui.txt_log, logger=logger)
        self.post_init()

    def check_arm(self):
        if not self.arm.is_connected:
            self.add_log("请先连接机械臂", color='red') 
        assert self.arm.is_connected == 1, "请先连接机械臂"

    def gozero(self):
        """回到初始位置"""
        self.check_arm()
        self.add_log("机械臂回到初始位置（快捷键：G）")
        if not self.is_going_to_init_pos:
            self.add_log("正在回到初始位置中...")
            self.is_going_to_init_pos = 1
            self.arm.gozero()
            self.is_going_to_init_pos = 0
            self.add_log("机械臂已归位！")


    def get_pose(self) -> dict:
        """获取机械臂当前位置"""
        self.check_arm()
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
        step_xyz = self.pos_vel
        step_RPY = self.rot_vel

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
        if not key in [
             "G", 
             "W", "A", "S", "D", 
             "Q", "Z",
             "U", "I", "O",
             "J", "K", "L",
        ]:
             return
        
        self.check_arm()
        self.ui.ctl_state.setText(f"按下 {key}")

        if event.key() == qtbase.qt_keys.Key_F5:
            self.reload()
            self.add_log("QApp reload", color="red")
            return

        pose = self.get_pose()
        incr = self.get_incr(key)
        printc(f"incr={incr}")

        if key == "G":
            self.arm.gozero()
            self.add_log("回到零位")
            return
        
        # pose_bak = pose.copy()

        # 处理 xyz 轴 --------------------------
        if self.incr_has_xyz(incr):
            pose_ = self.get_new_pose(pose, incr)
            ret = self.arm.robot.rm_movep_canfd(self.pose_to_list(pose_), False, 1, 60)
            
            if self.VERBOSE:
                printc("xyz" + "-"*50)
                printc(f"ret={ret}, incr={incr}")
                printc(f"new_pose={pose_}")
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

            if self.VERBOSE:
                printc("RPY" + "-"*50)
                printc(f"ret={ret}, incr={incr}")
                printc(f"ret={ret}, new_pose={new_pose}")


        if not event.isAutoRepeat():
            # self.add_key(key)
            if self.VERBOSE:
                printc(f"keyPressEvent {event}")
            # incr = self.get_empty_incr()

        # return super().keyPressEvent(event)
    def keyReleaseEvent(self, event: qtbase.QKeyEvent):
        # return super().keyReleaseEvent(event)
        # self.check_arm()
        self.ui.ctl_state.setText("暂无控制状态")


    def close_ready(self):
        ...

    
def main():
    init_logger()
    printc(f"q_appcfg={q_appcfg}")
    qapp = qtbase.QApplication(sys.argv)
    # 设置全局默认字体
    qapp.setFont(qtbase.QFont("微软雅黑", 11))
    mapp = MainWindow()
    mapp.show()
    sys.exit(qapp.exec())
