"""空间鼠标监听器"""

import time
import pyspacemouse
from toolbox.qt import qtbase
from .. import q_appcfg


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
    
    ### todo
    
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

    def __init__(self, devtype="SpaceMouse Pro Wireless") -> None:
        dev = pyspacemouse.open(
            # dof_callback=pyspacemouse.print_state,
            # button_callback=pyspacemouse.print_buttons
        )
        if dev is not None:
            self.is_ok = 1
        else:
            self.is_ok = 0
            print("空间鼠标未连接")
        # self.is_run = 0
        self.speed_ratio = q_appcfg.APPCFG_DICT['spacemouse_speed_ratio']
        self.btn_state = {
            "gripper": 0,
            "gozero": 0,
            "collect": 0,
        }
        # self.btn2 = 0
        self.devtype = devtype
        self.cur_state = {}


    def _get_button_state(self, state: pyspacemouse.SpaceNavigator):
        """获取鼠标按钮状态"""
        if "SpaceMouse Pro" in self.devtype:
            _start_id = 5
            btn1 = state.buttons[_start_id]
            btn2 = state.buttons[_start_id+1]
            btn3 = state.buttons[_start_id+2]
            btn4 = state.buttons[_start_id+3]
            return {
                "gripper": btn1,  # [1] 用来控制夹爪开闭
                "gozero": btn2,   # [2] 用来控制回到零位
                "collect": btn3,  # [3] 用来控制数据录制开关
                "custom": btn4,   # [4] 用来控制自定义功能
            }
        elif self.devtype == "SpaceMouse Compact":
            btn1 = state.buttons[0]
            btn2 = state.buttons[1]
            return {
                "gripper": btn1,
                "gozero": btn2,
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
        ret['gripper'] = _trigger_01_impl('gripper')
        ret['gozero'] = _trigger_01_impl('gozero')
        if "SpaceMouse Pro" in self.devtype:
            ret['collect'] = _trigger_01_impl('collect')
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
        
        cur_state.update(self._trigger_01(state))
        return cur_state


class SpaceMouseListener(qtbase.QAsyncTask):
    
    def __init__(self, conf: dict = {}, devtype="SpaceMouse Compact"):
        super().__init__(conf)
        self.device = SpaceMouse(devtype)
        self.cur_state = {}
    
    def run(self):
        self.is_run = 1
        while self.is_run:
            self.cur_state = self.device.read_state()
            # print(self.cur_state)
            self.msleep(10)
