import json
from aiohttp import ConnectionTimeoutError
import requests
from toolbox.comm.server_echo import ServerEcho
from toolbox.core.log import printc
from toolbox.qt import qtbase
from .. import APPCFG, API_IP, API_PRE


class Runner(qtbase.QAsyncTask):
    sig_on = qtbase.Signal()
    sig_off = qtbase.Signal()

    def __init__(self, ui_spacemouse_trigger_mode: qtbase.QLabel, ui_spacemouse_usable: qtbase.QCheckBox, conf: dict = {}) -> None:
        super().__init__(conf)
        self.ui_spacemouse_trigger_mode = ui_spacemouse_trigger_mode   # ui.QLabel
        self.ui_spacemouse_usable = ui_spacemouse_usable   # ui.QCheckBox
        self.spacemouse_usable = bool(0)
        self.spacemouse_usable_msg = ""

        # 监听 spacemouse 可用状态
        # 开发模式：spacemouse_usable 勾选框手动触发
        # 生产模式：通过 api 共享状态触发，和网页端搭配实现
        # mode = self.ui.spacemouse_trigger_mode        
        self.echo = ServerEcho()

        spacemouse_mode = APPCFG['spacemouse_mode']
        self.is_dev = 1 if spacemouse_mode == "dev" else 0

        MODE_DEV_TXT = "[开发模式]"
        MODE_PROD_TXT = "[生产模式]"
        MODE_DEV_COLOR = "red"
        MODE_PROD_COLOR = "blue"

        # 开发模式
        if spacemouse_mode == "dev":
            self.is_dev = 1
            self.ui_spacemouse_trigger_mode.setText(MODE_DEV_TXT)
            self.ui_spacemouse_trigger_mode.setStyleSheet(f"color: {MODE_DEV_COLOR};")
        
        # 部署模式
        else:
            timeout = self.echo.ping(API_IP)
            if timeout < 1e3:
                self.ui_spacemouse_trigger_mode.setText(MODE_PROD_TXT)
                self.ui_spacemouse_trigger_mode.setStyleSheet(f"color: {MODE_PROD_COLOR};")
                self.is_dev = 0

            else:  # 超时
                self.ui_spacemouse_trigger_mode.setText(MODE_DEV_TXT)
                self.ui_spacemouse_trigger_mode.setStyleSheet(f"color: {MODE_DEV_COLOR};")
                self.is_dev = 1

        # 在生产模式下不允许切换 ui_spacemouse_usable
        if not self.is_dev:
            self.ui_spacemouse_usable.setEnabled(False)
            printc("生产模式下不允许切换 ui_spacemouse_usable")

    def _spacemouse_update_param(self, k="spacemouse_status", v={}):
        # 上传参数
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
        spacemouse_usable = res['data']['spacemouse_usable']
        # printc(f"spacemouse_usable: {spacemouse_usable}")
        spacemouse_usable_msg = f"spacemouse_usable: {spacemouse_usable}"
        if spacemouse_usable_msg != self.spacemouse_usable_msg:
            self.spacemouse_usable_msg = spacemouse_usable_msg
            # self.add_log(spacemouse_usable_msg, color="green")
            printc(f"spacemouse_usable: {spacemouse_usable}")
        return spacemouse_usable

    def spacemouse_trigger_prod(self):
        """定时器回调，生产模式"""
        spacemouse_usable = self._spacemouse_check_usable()
        self.ui_spacemouse_usable.setChecked(spacemouse_usable)
        # self._spacemouse_update_param(v={})
        # self._spacemouse_get_shared_data()
        self.spacemouse_trigger_dev()


    def spacemouse_trigger_dev(self):
        """定时器回调，开发模式"""
        # now = self.ui.spacemouse_usable.isChecked()
        now = self.ui_spacemouse_usable.isChecked()
        prev = self.spacemouse_usable
        # 0-1
        if not prev and now:
            printc(f"spacemouse_trigger: {prev} -> {now}")
            # self.arm_connect()
            # self.spacemouse_start()
            self.sig_on.emit()
            self.spacemouse_usable = now
        elif prev and not now:
            printc(f"spacemouse_trigger: {prev} -> {now}")
            # self.spacemouse_stop()
            # self.arm_disconnect()
            self.sig_off.emit()
            self.spacemouse_usable = now
        else:
             ...

    def add_log(self, msg):
        self.sig_msg.emit(msg)


    def run(self):
        self.is_run = 1
        if self.is_dev:
            _spacemouse_trigger = self.spacemouse_trigger_dev
        else:
            _spacemouse_trigger = self.spacemouse_trigger_prod
        printc(f"spacemouse_trigger={_spacemouse_trigger}")

        while self.is_run:
            try:
                _spacemouse_trigger()
            except Exception as e:
                printc(f"spacemouse_trigger err={e}", err=1)
            self.msleep(APPCFG['spacemouse_usage_intv'])
