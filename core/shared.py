"""全局共享模块"""
from .setting_manager import SettingManagerX, SettingsManager


class Shared:
    # 用于信号槽异步接受数据
    setting_manager = SettingsManager()
    sm = setting_manager
    smx = SettingManagerX(sm)

    is_ee_opened = 1   # 夹爪打开状态，默认在启动时打开夹爪/灵巧手
    is_joystick_enable = bool(1)   # 是否启用鼠标的手柄
    # is_spacemouse_usable = 0   # 3d 鼠标可用状态
    spacemouse_usables = [0,0]  # noqa: RUF012  # [prev, now]
    ee_types = []  # noqa: RUF012  # [prev, now] # gripper, hand

    # msg
    spacemouse_usable_msg = ""   # 3d 鼠标可用状态
    ee_type_msg = ""   # 3d 鼠标可用状态

    # 控制参数（可按需调整）
    SCALE_XYZ = 0.2
    SCALE_RPY = 0.3
    ZERO_THRESHOLD = 0.001  # 认为输入为零的阈值
    LOOP_SLEEP_MS = 10  # 空闲时休眠（ms）
