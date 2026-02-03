"""配置管理器"""
from toolbox.qt import qtbase_future as qtbase


def str_to_bool_map(s: str|int) -> bool:
    # 构建映射字典，包含所有支持的字符串格式
    bool_map = {
        'true': True,
        'false': False,
        'True': True,
        'False': False,
        'TRUE': True,
        'FALSE': False,
        '1': True,
        '0': False,
        1: True,
        0: False,
        'yes': True,
        'ok': True,
        'no': False,
    }
    try:
        return bool_map[s]
    except KeyError:
        raise ValueError(f"无法将字符串 '{s}' 转换为布尔值")


class SettingsManager:

    def __init__(self):
        self._s = qtbase.QtCore.QSettings("westlake", "realman_teleop")

    def get(self, k, default=None):
        assert self._s
        return self._s.value(k, default)

    def get_(self, k, type=str, default=None):
        return eval(type(self.get(k, default)))

    def set(self, k, v):
        self._s.setValue(k, v)
        self._s.sync()  # 立即写入磁盘



class SettingManagerX:
    """SettingsManager 的高级接口，内置自动转型"""
    
    def __init__(self, setting_manager: SettingsManager):
        self.sm = setting_manager # or Shared.sm

    @property
    def ee_type(self):
        """末端类型"""
        return str(self.sm.get("ee_type", default="gripper"))

    @property
    def is_dexhand(self) -> bool:
        """末端为灵巧手"""
        return str_to_bool_map(self.sm.get("is_dexhand", default=0))

    @property
    def robot_ip(self):
        """获取机器人 IP 地址"""
        return str(self.sm.get("robot_ip", default="192.168.10.18"))

    def trig_debugpy(self) -> None:
        """设置 debugpy 调试模式"""
        enabled = not self.debugpy
        self.sm.set('debug/debugpy', 1 if enabled else 0)
    
    @property
    def debugpy(self) -> bool:
        """获取 debugpy 调试模式状态"""
        return str_to_bool_map(self.sm.get('debug/debugpy', 0))
    
    def trig_verbose(self) -> None:
        """设置详细日志模式"""
        enabled = not self.verbose
        self.sm.set('debug/verbose', 1 if enabled else 0)
    
    @property
    def verbose(self) -> bool:
        """获取详细日志模式状态"""
        return str_to_bool_map(self.sm.get('debug/verbose', 0))
    
    def trig_hotreload(self) -> None:
        """设置热重载模式"""
        enabled = not self.hotreload
        self.sm.set('debug/hotreload', 1 if enabled else 0)
    
    @property
    def hotreload(self) -> bool:
        """获取热重载模式状态"""
        return str_to_bool_map(self.sm.get('debug/hotreload', 0))
    
    def trig_theme(self) -> None:
        """设置美化主题"""
        enabled = not self.is_theme
        self.sm.set('debug/theme', 1 if enabled else 0)
    
    @property
    def is_theme(self):
        """获取美化主题状态"""
        return str_to_bool_map(self.sm.get('debug/theme', 0))
    @staticmethod
    def _set_btn_checked_style(btn: qtbase.QPushButton, checked: bool):
        """设置按钮勾选样式"""
        if checked:
            btn.setStyleSheet("background-color: red; color: white;")
        else:
            btn.setStyleSheet("none;")
