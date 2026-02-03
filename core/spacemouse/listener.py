"""空间鼠标监听器"""

import time
import pyspacemouse
from toolbox.core.log import printc
from toolbox.qt import qtbase
from toolbox.qt.common.debug import enable_debugpy
# from .. import q_appcfg
from collections import deque
from .handler import SpaceMouseHandler
from .btn_detector import SpaceMouseButtonClickDetector


class SpaceMouseListener(qtbase.QAsyncTask):
    sig_data = qtbase.Signal(dict)
    
    def __init__(
        self, 
        conf: dict = {}, 
        devtype="SpaceMouse Compact",  # "SpaceMouse Pro Wireless"
        ui_label=None,
        double_interval_ms: int = 250*2
    ):
        super().__init__(conf)
        self.devtype = devtype
        self.device = SpaceMouseHandler(devtype)
        self.cur_state = {}
        self.ui_label = ui_label
        self.is_run = 0

        # 为每个可能的按键创建状态机
        # 按需只启用 SpaceMouse 实际返回的键
        self.btn_names = ["btn1", "btn2"]
        self.btn_sms: dict[str, SpaceMouseButtonClickDetector] = {}
        for name in self.btn_names:
            self.btn_sms[name] = SpaceMouseButtonClickDetector(name, double_interval_ms)

        # 控制 UI 发信号的节流：按钮事件直接发，位姿/移动事件不发（保持你原来的设计）
        # 如果想对移动事件按时间节流，也可在此添加 last_emit_time 机制

    # @enable_debugpy(1)
    def run(self):
        self.is_run = 1
        while self.is_run:
            # self.msleep(1)
            state = self.cur_state = self.device.read_state()
            # self.sig_data.emit(state)
            if self.ui_label:
                self.ui_label.setText(str(state)) # type: ignore

            # 问题定位：
            # 高频率的信号释放会导致进入缓冲队列，主线程的事件循环按照次序处理，会导致延迟问题
            # 3d 鼠标的动作到主线程存在 1-2s 的延迟，操作不跟手，因此只把侧键事件发给主线程
            # 鼠标移动事件不释放信号，而是直接在后台处理
            if self.device.devtype == "SpaceMouse Compact":
                self.process_side_btn_click(state)
            elif self.device.devtype in "SpaceMouse Pro Wireless":
                self.process_side_btn_click_for_pro(state)
            else:
                pass

            # 如果既不是移动也不是按钮变化，则继续循环（不阻塞）
            # 注意：高频循环需短暂 sleep 以让出 CPU（并避免 100% 占用）
            # 你可根据实际需要调整，这里 sleep 0.001 -> ~1000Hz
            time.sleep(0.001)


    def process_side_btn_click(self, state: dict):
        """处理普通 SpaceMouse Compact 款的侧键点击事件，设配了双击检测"""
        now = time.monotonic()
        # 处理按钮事件：只处理本次返回的按键名集合
        for btn_name, sm in self.btn_sms.items():
            if btn_name not in state:
                continue
            raw = int(state[btn_name])

            ev = sm.update(raw, now)
            if ev is not None:
                # 构造发给主线程的数据结构（你可以自定义字段）
                payload = {
                    'event': ev['type'],   # 'single' or 'double'
                    'btn': ev['btn'],      # 'btn1' / 'btn2' / ...
                    'state': state         # 当前整帧状态（含位姿）
                }
                # 立即发信号到主线程处理按钮事件（不会 flood，因为仅在状态边沿发生）
                self.sig_data.emit(payload)


    def process_side_btn_click_for_pro(self, state: dict):
        """处理 Pro 版本的侧键点击事件，只处理单击功能"""
        btn_names = ['btn1', 'btn2', 'btn3', 'btn4']
        btn_state = [state[name] for name in btn_names]

        # 仅在有按钮被按下时处理
        if any(btn_state):
            click_btn_name = ""
            for btn_name in btn_names:
                if state[btn_name]:
                    click_btn_name = btn_name
            
            # 构造发给主线程的数据结构（你可以自定义字段）
            payload = {
                'event': 'single',      # 'single' or 'double'
                'btn': click_btn_name,  # 'btn1' / 'btn2' / ...
                'state': state          # 当前整帧状态（含位姿）
            }
            # 立即发信号到主线程处理按钮事件
            self.sig_data.emit(payload)

