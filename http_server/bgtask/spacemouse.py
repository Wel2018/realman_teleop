"""空间鼠标监听器"""

import time
import pyspacemouse
from toolbox.core.log import printc
from toolbox.qt import qtbase
from toolbox.qt.common.debug import enable_debugpy
# from .. import q_appcfg

from collections import deque
import time


class ButtonClickDetector:
    """
    基于 deque 的单击/双击检测。
    每次完整的 press->release（1->0）视为一次点击。
    """
    def __init__(self, name: str, double_interval_ms: int = 250):
        self.name = name
        self.double_interval = double_interval_ms / 1000.0
        self.click_times = deque()   # 存放每次完整点击的时间戳
        self.last_raw = 0            # 上一次原始值（用于检测 1->0）
        self.pending_single = None   # 记录等待确认的单击事件时间戳

    def update(self, raw: int, now: float) -> dict | None:
        """
        raw: 当前按钮状态 (0/1)
        now: 当前时间戳（秒）
        """
        event = None

        # 检测一次完整点击：press(1) -> release(0)
        if self.last_raw == 1 and raw == 0:
            self.click_times.append(now)

        self.last_raw = raw

        # —— 无点击则不用处理 ——
        if not self.click_times:
            return None

        # —— 处理点击队列 ——
        while len(self.click_times) >= 2:
            t1 = self.click_times[0]
            t2 = self.click_times[1]

            if t2 - t1 <= self.double_interval:
                # 两次点击在间隔内 → 双击
                self.click_times.clear()
                return {'type': 'double', 'btn': self.name}
            else:
                # 间隔太长 → t1 必然是单击
                self.click_times.popleft()
                return {'type': 'single', 'btn': self.name}

        # —— 只有一次点击，需要等待查看是否会出现第二次 ——
        t1 = self.click_times[0]

        if now - t1 > self.double_interval:
            # 超时 → 单击成立
            self.click_times.clear()
            return {'type': 'single', 'btn': self.name}

        return None


class SpaceMouse:
    """SpaceMouse 鼠标状态监听器，异步反馈控制状态
    
    ### 说明
    
    > [!note]
    > 注：已适配，无需修改底层代码
    
    ```python
    # 推荐方法
    SpaceMouseListener._d['设备型号].hid_id = [Vendor ID, Product ID]
    spacemouse = SpaceMouseListener()
    ```
    
    
    - PySpaceMouse 中将设备号写死了，会导致检测不到设备
    - 空间鼠标使用 usb 接收器、蓝牙、有线三种方法连接时，Product ID 不同，需要修改
    - 修改 `PySpaceMouse/pyspacemouse/pyspacemouse.py Line 511` 的 
    `hid_id=[0x256F, 0xC632]` 为 `hid_id=[0x256F, 0xC638]` 以适配有线
    - spacemouse 的蓝牙连接需要 linux 驱动支持，在 Ubuntu 上官方已经弃更
    - usb 接收器需要近距离连接否则会丢包，使用体验不好，建议使用有线连接，稳定可靠
    
    ### next
    
    - [x] 支持侧键功能进行夹爪控制
    
    ### 环境配置
    
    - 安装（推荐）：`pip install pyspacemouse`
    - 源码安装 [PySpaceMouse](https://github.com/JakubAndrysek/PySpaceMouse) 并 `pip install -e .`
    - 使用 `lsusb` 查看设备 idVendor 和 idProduct
    
    ```
    Bus 001 Device 022: ID 256f:c638 3Dconnexion SpaceMouse Pro Wireless BT
    Bus 001 Device 011: ID 256f:c652 3Dconnexion Universal Receiver
    Bus 003 Device 005: ID 256f:c635 3Dconnexion SpaceMouse Compact
    # 256f:c652 为 Vendor ID 和 Product ID
    
    # 在 `sudo nano /etc/udev/rules.d/99-spacemouse.rules` 中添加如下配置
    SUBSYSTEM=="input", GROUP="input", MODE="0660"
    KERNEL=="hidraw*", ATTRS{idVendor}=="256f", ATTRS{idProduct}=="c652", MODE="0666"
    KERNEL=="hidraw*", ATTRS{idVendor}=="256f", ATTRS{idProduct}=="c638", MODE="0666"

    # 重新加载配置
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    sudo usermod -a -G input $USER
    ```
    
    ### 支持设备
    
    - SpaceMouse Enterprise
    - SpaceExplorer
    - SpaceNavigator
    - SpaceMouse USB
    - SpaceMouse Compact
    - SpaceMouse Pro Wireless
    - SpaceMouse Pro
    - SpaceMouse Wireless
    - SpaceMouse Wireless [NEW]
    - 3Dconnexion Universal Receiver
    - SpacePilot
    - SpacePilot Pro

    ### 测试工具

    ```
    usage: pyspacemouse [-h] [--version] [--list-spacemouse]
                        [--list-supported-devices] [--list-all-hid-devices]
                        [--test-connect]

    PySpaceMouse CLI

    options:
      -h, --help            show this help message and exit
      --version             Version of pyspacemouse
      --list-spacemouse     List connected SpaceMouse devices
      --list-supported-devices
                            List supported SpaceMouse devices
      --list-all-hid-devices
                            List all connected HID devices
      --test-connect        Test connect to the first available device

    For more information, visit https://spacemouse.kubaandrysek.cz
    ```

    ### HID

    Failed to open SpaceMouse: HID API is probably not installed. 
    Look at https://spacemouse.kubaandrysek.cz for details.

    ```
    sudo apt-get install libhidapi-dev

    sudo echo 'KERNEL=="hidraw*", SUBSYSTEM=="hidraw", MODE="0664", GROUP="plugdev"' > /etc/udev/rules.d/99-hidraw-permissions.rules
    sudo usermod -aG plugdev $USER
    newgrp plugdev
    ```

    """
    POSE_KEYS = ['x', 'y', 'z', 'R', 'P', 'Y']
    # 需要根据实际连接方式，填写对应参数（三模鼠标每种模式都对应不同的 hid）
    _d: dict[str, pyspacemouse.DeviceSpec] = pyspacemouse.device_specs
    _d['SpaceMouse Pro Wireless'].hid_id = [0x256F, 0xC638]  # 有线
    _d['SpaceMouse Compact'].hid_id = [0x256F, 0xC635]  # 有线
    # intv = 10  # ms

    def __init__(self, devtype="SpaceMouse Pro Wireless", speed_ratio=0.3) -> None:
        dev = pyspacemouse.open(
            # dof_callback=pyspacemouse.print_state,
            # button_callback=pyspacemouse.print_buttons
        )
        if dev is not None:
            self.is_ok = 1
        else:
            self.is_ok = 0
            printc(f"遥操作外设 {devtype} 未连接")
        # self.is_run = 0
        self.speed_ratio = speed_ratio
        self.btn_state = {
            "gripper": 0,
            "gozero": 0,
            "collect": 0,
        }
        # self.btn2 = 0
        self.devtype = devtype
        self.cur_state = {}

        # 最近的按钮“原始”状态缓存（用于边沿检测）
        self._last_buttons_raw: dict[str, int] = {}

    def _get_button_state(self, state: pyspacemouse.SpaceNavigator):
        """获取鼠标按钮状态"""
        if "SpaceMouse Pro" in self.devtype:
            _start_id = 5
            btn1 = state.buttons[_start_id]
            btn2 = state.buttons[_start_id+1]
            btn3 = state.buttons[_start_id+2]
            btn4 = state.buttons[_start_id+3]
            return {
                "btn1": btn1,  # [1] 用来控制夹爪开闭
                "btn2": btn2,   # [2] 用来控制回到零位
                "btn3": btn3,  # [3] 用来控制数据录制开关
                "btn4": btn4,   # [4] 用来控制自定义功能
            }
        elif self.devtype == "SpaceMouse Compact":
            btn1 = state.buttons[0]
            btn2 = state.buttons[1]
            return {
                "btn1": btn1,
                "btn2": btn2,
            }
        else:
            raise NotImplementedError
    
    def _trigger_01(self, state: pyspacemouse.SpaceNavigator):
        """记录连续两帧的状态，当出现 0-1 跳变时触发控制信号"""
        btn_state = self._get_button_state(state)
    
        def _trigger_01_impl(slot='gripper'):
            # 通过计算当前帧与上一帧的状态，判断是否触发，然后记录当前状态再返回信号状态
            if self.btn_state[slot] == 0 and btn_state[slot] == 1:
                self.btn_state[slot] = 1
                return 1
            elif self.btn_state[slot] == 1 and btn_state[slot] == 0:
                self.btn_state[slot] = 0
                return 0
            return 0
    
        ret = {}
        ret['btn1'] = _trigger_01_impl('btn1')
        ret['btn2'] = _trigger_01_impl('btn2')
        
        # pro 版本有更多功能键
        if "SpaceMouse Pro" in self.devtype:
            ret['btn3'] = _trigger_01_impl('btn3')
            ret['btn4'] = _trigger_01_impl('btn4')
        return ret
    

    def read_state(self):
        state: pyspacemouse.SpaceNavigator = pyspacemouse.read()  # type: ignore
        # R: roll 滚转，沿着 x 轴
        # P: pitch 俯仰，沿着 y 轴
        # Y: yaw 偏航，沿着 z 轴
        cur_state = {
            # "x": -state.y,  # type: ignore
            # "y": state.x,  # type: ignore
            "x": state.x,  # type: ignore
            "y": state.y,  # type: ignore
            "z": state.z,  # type: ignore
            "R": state.roll,  # type: ignore
            "P": state.pitch,  # type: ignore
            "Y": -state.yaw  # type: ignore
        }
        
        # 控制速度
        for k in self.POSE_KEYS:
            cur_state[k] *= self.speed_ratio
        

        # 把按键原始值放进去（0/1）
        try:
            btns = self._get_button_state(state)
        except NotImplementedError:
            btns = {}
        cur_state.update(btns)

        # round:3
        for k in cur_state.keys():
            cur_state[k] = round(cur_state[k], 3)
        return cur_state


class SpaceMouseListener(qtbase.QAsyncTask):
    sig_data = qtbase.Signal(dict)
    
    def __init__(
        self, 
        conf: dict = {}, 
        devtype="SpaceMouse Compact", 
        ui_label=None,
        double_interval_ms: int = 250*2
    ):
        super().__init__(conf)
        self.device = SpaceMouse(devtype)
        self.cur_state = {}
        self.ui_label = ui_label
        self.is_run = 0

        # 为每个可能的按键创建状态机
        # 按需只启用 SpaceMouse 实际返回的键
        self.btn_names = ["btn1", "btn2"]
        self.btn_sms: dict[str, ButtonClickDetector] = {}
        for name in self.btn_names:
            self.btn_sms[name] = ButtonClickDetector(name, double_interval_ms)

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


######################################################
# 测试
# python projects/realman_teleop/bgtask/spacemouse.py
######################################################


def test_spacemouse():
    spacemouse = SpaceMouse("SpaceMouse Compact")
    while 1:
        try:
            state = spacemouse.read_state()
            printc(f"state={state}")
            time.sleep(1/1000)
        except KeyboardInterrupt:
            break

