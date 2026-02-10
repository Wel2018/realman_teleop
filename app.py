import os
import sys
import threading
import time
import json
import requests
import functools
from toolbox.core.log import LogHelper, printc
from loguru import logger
from toolbox.comm.server_echo import ServerEcho
from toolbox.core.file_op import open_local_file
from toolbox.qt import qtbase
from .ui.ui_form import Ui_DemoWindow
from . import q_appcfg
from . import APPCFG, API_IP, API_PORT, API_PRE
from .core.shared import Shared
from .util import set_layout_visible, set_layout_enabled


# 核心组件
from .core.arm.realman import RMArm   # 机械臂
from .core.spacemouse.listener import SpaceMouseListener  # spacemouse
from .core.end_effector.gripper import Gripper   # 夹爪
from .core.end_effector.dexhand import DexterousHand   # 灵巧手
from .bgtask.listener import StateListener   # 状态监听
from .bgtask.teleop import TeleopTask        # 遥操作线程


echo = ServerEcho()


def require_arm_connected(func):
    """
    函数装饰器：要求机械臂已连接才能执行被装饰的函数
    如果 `check_arm` 校验失败，会捕获异常并打印日志，不执行原函数
    """
    @functools.wraps(func)  # 保留原函数的元信息（如函数名、文档字符串）
    def wrapper(self, *args, **kwargs):  # noqa: ANN002
        try:
            # 执行机械臂连接状态校验
            self.check_arm()
            # 校验通过，执行原函数
            return func(self, *args, **kwargs)
        except AssertionError as e:
            # 捕获校验失败的异常，打印并记录日志
            error_msg = f"执行 {func.__name__} 失败：{e!s}"
            print(error_msg)  # 控制台打印
            logger.exception(error_msg)  # 日志文件记录
        except Exception as e:
            # 捕获其他意外异常，避免程序崩溃
            error_msg = f"执行 {func.__name__} 时发生未知错误：{e!s}"
            print(error_msg)
            logger.exception(error_msg, exc_info=True)
    return wrapper


class MainWindow(qtbase.QApp):
    """应用具体实现"""
    # 定时器和线程名称
    TH_CTL_MODE = "teleop_ctl_mode"
    VERBOSE = q_appcfg.VERBOSE

    def pre_init(self):
        return super().pre_init()
    
    def post_init(self):  # noqa: PLR0915
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
        self.ui.msg.setText(f"服务地址: {API_IP}:{API_PORT}")

        #--------------------------------------------------------------------

        # 机械臂
        self.arm = RMArm()
        self.arm.sig_arm_connected.connect(self.arm_connected)
        arm_ip = self.ui.arm_ip.text()
        timeout = echo.ping(arm_ip)
        if timeout == echo.PING_TIMEOUT:
            printc(f"机械臂 {arm_ip} 响应超时！", "critical")

        # 3d 鼠标状态监听
        self.spacemouse_th = SpaceMouseListener()
        if not self.spacemouse_th.device.is_ok:
            printc(f"SpaceMouse {self.spacemouse_th.devtype} 不存在，请插入该外设！", "critical")

        # 夹爪
        self.gripper = Gripper(self.arm)

        # 灵巧手
        self.dexhand = DexterousHand(self.arm)
        self.dexhand.sig_ui_pos.connect(self.dexhand_ui_pos_update)

        # 遥操作线程
        self.teleop_th = TeleopTask(self.arm, self.spacemouse_th, self.gripper, self.dexhand)

        # 服务器接口状态监听
        self.state_listener = StateListener(self)
        self.state_listener.sig_is_01_trig_for_spacemouse_usable.connect(self.spacemouse_trig_01)
        self.state_listener.sig_is_10_trig_for_spacemouse_usable.connect(self.spacemouse_trig_10)
        self.state_listener.sig_is_trig_for_ee_type.connect(self.ee_type_trig)
        self.state_listener.initialize()
        self.state_listener.start()

        if self.state_listener.is_dev:
            self.bind_clicked(ui.btn_gripper, self.trig_ee_type)

        #--------------------------------------------------------------------

        self.ui.btn_arm_connect.setEnabled(bool(1))
        self.ui.btn_arm_disconnect.setEnabled(bool(0))
        self.ui.step_posi2.valueChanged.connect(self.spacemouse_speed_update)
        self.ui.step_angle2.valueChanged.connect(self.spacemouse_speed_update)

        # init ok
        self.add_log("初始化完成", color="green")


    def trig_ee_type(self):
        # 从夹爪切换为灵巧手
        if Shared.ee_types[0] == "gripper" and Shared.ee_types[1] == "gripper":
            Shared.ee_types[0] = "dexhand"
            Shared.ee_types[1] = "dexhand"
            self.ee_type_trig()

        # 从灵巧手切换为夹爪
        elif Shared.ee_types[0] == "dexhand" and Shared.ee_types[1] == "dexhand":
            Shared.ee_types[0] = "gripper"
            Shared.ee_types[1] = "gripper"
            self.ee_type_trig()


    def dexhand_ui_pos_update(self, pos: list):
        self.ui.pos_f0.setValue(pos[0])
        self.ui.pos_f1.setValue(pos[1])
        self.ui.pos_f2.setValue(pos[2])
        self.ui.pos_f3.setValue(pos[3])
        self.ui.pos_f4.setValue(pos[4])

    def spacemouse_trig_01(self):
        self.arm_connect()
        self.spacemouse_start()

    def spacemouse_trig_10(self):
        self.spacemouse_stop()
        self.arm_disconnect()

    def ee_type_trig(self):
        ui = self.ui
        _type = Shared.ee_types[1]
        if _type == "dexhand":
            self.ui.btn_gripper.setText("灵巧手")
            Shared.sm.set("is_dexhand", 1)
            self.dexhand.initialize()
            self.gripper.deinitialize()
        
            # 灵巧手模式
            self.bind_clicked(ui.btn_2finge_pick, self.dexhand.two_finge_pick)
            self.bind_clicked(ui.btn_2finge_release, self.dexhand.two_finge_release)
            self.bind_clicked(ui.btn_4finge_pick, self.dexhand.five_finge_pick)
            self.bind_clicked(ui.btn_4finge_release, self.dexhand.five_finge_release)
            self.bind_clicked(ui.btn_rotate_thumb, self.rotate_thumb)

        elif _type == "gripper":
            self.ui.btn_gripper.setText("夹爪")
            Shared.sm.set("is_dexhand", 0)
            self.gripper.initialize()
            self.dexhand.deinitialize()

        # 夹爪模式
        enable = _type == "dexhand"
        qtbase.set_enable(ui.btn_2finge_pick, enable)
        qtbase.set_enable(ui.btn_2finge_release, enable)
        qtbase.set_enable(ui.btn_4finge_pick, enable)
        qtbase.set_enable(ui.btn_4finge_release, enable)
        set_layout_enabled(ui.hand_layout, bool(enable))

    def rotate_thumb(self):
        v = self.ui.rotate_thumb.value()
        self.dexhand._rotate_thumb(v)  # noqa: SLF001
        self.ui.angle_f0.setValue(v)


    def arm_connected(self):
        """尝试连接机械臂完成，可能没有连接成功"""
        if self.arm.is_connected:
            self.add_log("机械臂连接成功", color='green')
            self.add_timer("timer_update_msg", 200, self.update_msg, 1)
            stage = self.arm.get_collision()
            self.ui.spin_collision_state.setValue(stage)
            self.add_log(f"碰撞等级: {stage} [范围: 1-8]", color='green')
            self.add_log(f"如需使用遥操作需手动开启服务，用完请手动关闭该服务", color='#ffab70')
            self.add_log("程序初始化完成", color="green")
            self.ui.label_arm.setStyleSheet("color: green; background-color: #03db6b;")
            self.ui.btn_arm_connect.setEnabled(bool(0))
            self.ui.btn_arm_disconnect.setEnabled(bool(1))
            printc("机械臂初始化完成", "critical")

            # 触发一次
            self.ee_type_trig()

        else:
            self.add_log("机械臂连接失败", color='red')
            self.ui.label_arm.setStyleSheet("color: red; background-color: #f4cce4;")
            self.ui.btn_arm_connect.setEnabled(bool(1))
            self.ui.btn_arm_disconnect.setEnabled(bool(0))
            if not self.arm.is_connected:
                printc("灵巧手/夹爪无法连接，因为机械臂暂时无法连接！", "critical")

    
    def arm_ip(self):
        return self.ui.arm_ip.text().strip()

    def arm_connect(self):
        """机械臂连接"""
        if self.arm.is_connected:
            self.add_log("机械臂已经连接")
            return
         
        ip = self.arm_ip()

        # 检查连通性
        timeout = echo.ping(ip)
        if timeout == echo.PING_TIMEOUT:
            self.add_log(f"无法 ping 通 ARM_IP={ip}，请检查配置文件！", color='red')
            return

        self.add_log(f"ping={timeout} ms, ARM_IP={ip}，正在连接...")
        self.arm.myconnect(ip)

        self.ui.btn_arm_connect.setEnabled(bool(0))
        self.ui.btn_arm_disconnect.setEnabled(bool(0))

    def arm_disconnect(self):
        """机械臂断开连接"""
        if not self.arm.is_connected:
            self.add_log("机械臂已经断开连接")
            return
        
        # 检查鼠标服务是否关闭
        self.spacemouse_stop()

        # 断开机械臂
        self.remove_timer("timer_update_msg")
        self.arm.mydisconnect()
        self.add_log("机械臂已断开", color='green')
        self.ui.label_arm.setStyleSheet("color: red; background-color: #f4cce4;")
    
    def update_msg(self):
        """更新机械臂状态信息"""
        ret, data = self.arm._get_pose()
        j = data['joint']
        p = data['pose']
        err = data['err']
        
        p = [round(el,2) for el in p]  # ty:ignore[no-matching-overload]
        j = [round(el,2) for el in j]  # ty:ignore[no-matching-overload]
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

    #@require_arm_connected
    def spacemouse_start(self):
        printc("启动 spacemouse 服务...")
        
        # 等待机械臂异步连接完成
        self.arm.wait_connect()
        self.check_arm()

        # 启动鼠标监听服务
        self.spacemouse_th.start()
        self.add_log("SpaceMouse 服务已启动", color='green')

        # 启动机械臂伺服
        self.teleop_th.start()
        self.add_log("机械臂伺服服务已启动", color='green')

        # 设置状态文本
        self.ui.label_spacemouse.setStyleSheet("color: green; background-color: #03db6b;")

        # 启动时加载默认速度
        self.spacemouse_speed_update(0)

    def spacemouse_speed_update(self, value):
        Shared.SCALE_XYZ = self.ui.step_posi2.value()
        Shared.SCALE_RPY = self.ui.step_angle2.value()

    def spacemouse_stop(self):
        if not self.arm.is_connected:
            self.add_log("机械臂未连接！", color="red")
            return
        self.check_arm()
        self.add_log("SpaceMouse 服务退出", color="green")
        self.spacemouse_th.stop()
        self.teleop_th.stop()
        self.ui.label_spacemouse.setStyleSheet("color: red; background-color: #f4cce4;")

    def setting(self):
         """打开配置"""
         b = os.path.dirname(__file__)  # noqa: PTH120
         open_local_file(os.path.join(b, "appcfg.yaml"), is_relative=1)  # noqa: PTH118

    def __init__(self, parent = None):
        ui = self.ui = Ui_DemoWindow()
        super().__init__(ui, parent, q_appcfg)
        self.pre_init()
        self.init(ui_logger=ui.txt_log, logger=logger)
        self.post_init()
    

    def check_arm(self):
        self.arm.check_arm()

    def gozero(self):
        """回到初始位置"""
        self.check_arm()
        self.add_log("机械臂回到初始位置（快捷键：G）")
        self.arm.gozero()


    def keyPressEvent(self, event: qtbase.QKeyEvent):
        """按下按键：键盘打开 caps lock 模式，可以实现长按模式
        （即按住 A，只会触发一次 keyPressEvent，不会连续触发，松开也是只触发一次）
        - 键盘长按会在第一次 isAutoRepeat=False, 之后是 True
        """
        key = event.text().upper()
        if key not in [
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

        self.arm.kb(key, self.pos_vel, self.rot_vel)

        if not event.isAutoRepeat():  # noqa: SIM102
            # self.add_key(key)
            if self.VERBOSE:
                printc(f"keyPressEvent {event}")
            # incr = self.get_empty_incr()

        # return super().keyPressEvent(event)
    def keyReleaseEvent(self, event: qtbase.QKeyEvent):  # noqa: ARG002
        self.ui.ctl_state.setText("暂无控制状态")


    def close_ready(self):
        self.state_listener.stop()

    
def main():
    LogHelper.init(q_appcfg.slot)
    printc(f"q_appcfg={q_appcfg}")
    qapp = qtbase.QApplication(sys.argv)
    # 设置全局默认字体
    qapp.setFont(qtbase.QFont("微软雅黑", 11))
    mapp = MainWindow()
    mapp.show()
    sys.exit(qapp.exec())
