from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from realman_teleop.app import MainWindow
import json
from aiohttp import ConnectionTimeoutError
import requests
from toolbox.comm.server_echo import ServerEcho
from toolbox.core.log import printc
from toolbox.qt import qtbase_future as qtbase
from .. import APPCFG, API_IP, API_PRE
from realman_teleop.core.shared import Shared


class StateListener(qtbase.QAsyncTask):
    # 触发器信号
    sig_is_01_trig_for_spacemouse_usable = qtbase.Signal()
    sig_is_10_trig_for_spacemouse_usable = qtbase.Signal()
    sig_is_trig_for_ee_type = qtbase.Signal()

    def __init__(self, win: MainWindow, conf: dict = {}) -> None:
        super().__init__(conf)
        self.win = win

    @property
    def is_dev(self):
        work_mode = APPCFG['work_mode']
        return 1 if work_mode == "dev" else 0

    def initialize(self):
        # 监听 spacemouse 可用状态
        # 开发模式：spacemouse_usable 勾选框手动触发
        # 生产模式：通过 api 共享状态触发，和网页端搭配实现    
        self.echo = ServerEcho()

        MODE_DEV_TXT = "[开发模式]"
        MODE_PROD_TXT = "[部署模式]"
        MODE_DEV_COLOR = "red"
        MODE_PROD_COLOR = "blue"
        work_mode = self.win.ui.spacemouse_trigger_mode

        # 开发模式
        if self.is_dev:
            work_mode.setText(MODE_DEV_TXT)
            work_mode.setStyleSheet(f"color: {MODE_DEV_COLOR};")
        
        # 部署模式
        else:
            self.win.ui.spacemouse_usable.setEnabled(bool(0))   # 不允许切换
            timeout = self.echo.ping(API_IP)
            if timeout < 1e3:  # noqa: PLR2004
                work_mode.setText(MODE_PROD_TXT)
                work_mode.setStyleSheet(f"color: {MODE_PROD_COLOR};")

            else:  # 超时
                msg = "无法连接服务器"
                raise Exception(msg)  # noqa: TRY002

    def _spacemouse_update_param(self, k="spacemouse_status", v={}):
        """上传参数"""
        data = requests.post(f"{API_PRE}/api/v1/hardware/robot/spacemouse/update", json={
             k: v
            #  "spacemouse": {
                #   "a": time.time(),
                #   "b": time.time(),
            #  }
        })
        res = json.loads(data.text)
        # update {"code":200,"message":"状态已更新","data":{"updated_fields":["spacemouse"]}}
        printc(f"spacemouse_update_param: {res}")

    def _spacemouse_get_shared_data(self):
        """获取和 spacemouse 有关的全局共享变量"""
        # {"code":200,"message":"success","data":{"key":"robot","value":{"space_mouse_use_robot":true,"spacemouse":{"a":1762915702.7545369,"b":1762915702.7545369}}}}
        # 获取共享变量
        data = requests.get(f"{API_PRE}/api/v1/system/shared/state?key=robot")
        res = json.loads(data.text)
        k = res['data']['key']
        v = res['data']['value']
        printc(f"state {v}")

    def _spacemouse_check_usable(self):
        """检查是否为遥操作模式"""
        data = requests.get(f"{API_PRE}/api/v1/hardware/robot/spacemouse/usable", timeout=1)
        res = json.loads(data.text)
        usable: bool = res['data']['spacemouse_usable']
        ee_type: str = res['data']['category']   # gripper, hand

        # 同步 msg
        msg1 = f"spacemouse_usable: {usable}"
        if msg1 != Shared.spacemouse_usable_msg:
            Shared.spacemouse_usable_msg = msg1
            printc(msg1)
        
        # 同步 msg
        msg2 = f"ee_type: {ee_type}"
        if msg2 != Shared.ee_type_msg:
            Shared.ee_type_msg = msg2
            printc(msg2)

        # 同步勾选状态
        self.win.ui.spacemouse_usable.setChecked(usable)
        Shared.spacemouse_usables[0] = Shared.spacemouse_usables[1]   # prev
        Shared.spacemouse_usables[1] = int(usable)   # now
        
        # 同步末端状态
        Shared.ee_types[0] = Shared.ee_types[1]   # prev
        Shared.ee_types[1] = ee_type   # now

        if self.is_trig_for_ee_type:
            self.win.ui.btn_gripper.setText(ee_type)
    
    @property
    def is_01_trig_for_spacemouse_usable(self):
        """01触发"""
        prev, now = Shared.spacemouse_usables
        printc(f"spacemouse_trigger: {prev} -> {now}")
        return not prev and now

    @property
    def is_spacemouse_running(self):
        """01触发"""
        prev, now = Shared.spacemouse_usables
        return prev and now
        
    @property
    def is_10_trig_for_spacemouse_usable(self):
        """10触发"""
        prev, now = Shared.spacemouse_usables
        printc(f"spacemouse_trigger: {prev} -> {now}")
        return prev and not now
    
    @property
    def is_trig_for_ee_type(self):
        """01触发"""
        prev, now = Shared.ee_types
        printc(f"ee_type: {prev} -> {now}")
        return prev != now

    def _prod(self):
        """定时器回调，生产模式"""
        self._spacemouse_check_usable()
        # self._spacemouse_update_param(v={})
        # self._spacemouse_get_shared_data()
        self._dev()

    def _dev(self):
        """定时器回调，开发模式"""
        if self.is_01_trig_for_spacemouse_usable:
            self.sig_is_01_trig_for_spacemouse_usable.emit()
        
        if self.is_10_trig_for_spacemouse_usable:
            self.sig_is_10_trig_for_spacemouse_usable.emit()
        
        if self.is_trig_for_ee_type:
            self.sig_is_trig_for_ee_type.emit()

    def add_log(self, msg):
        self.sig_msg.emit(msg)

    def run(self):
        self.is_run = 1
        while self.is_run:
            try:
                if self.is_dev:
                    self._dev()
                else:
                    self._prod()
            except Exception as e:
                printc(f"spacemouse_trigger err={e}", 'err')
            self.msleep(APPCFG['spacemouse_usage_intv'])
