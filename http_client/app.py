import threading
import time
import asyncio
import json
import sys
from rich import print
from toolbox.qt import qtbase
from .bgtask.spacemouse import SpaceMouseListener
import websockets

# 针对 Windows 的异步策略优化
if sys.platform == 'win32':
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())



class TestApp(qtbase.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Realman SpaceMouse Controller")
        self.setMinimumWidth(400)

        # --- UI 界面布局 ---
        self._layout = qtbase.QVBoxLayout()
        
        # 实时数据显示 (坐标数据)
        self.label_title = qtbase.QLabel("<b>SpaceMouse Realtime XYZ:</b>")
        self.msg_xyz = qtbase.QLabel("Waiting for data...")
        self.msg_xyz.setStyleSheet("color: gray; font-family: Consolas;")
        
        # 事件数据显示 (按钮点击)
        self.label_event_title = qtbase.QLabel("<b>Last Event:</b>")
        self.msg_event = qtbase.QLabel("None")
        self.msg_event.setStyleSheet("color: #0078d7; font-weight: bold; font-size: 14px;")
        
        self._layout.addWidget(self.label_title)
        self._layout.addWidget(self.msg_xyz)
        self._layout.addSpacing(10)
        self._layout.addWidget(self.label_event_title)
        self._layout.addWidget(self.msg_event)
        self._layout.addStretch()
        self.setLayout(self._layout)

        # --- 状态与逻辑 ---
        self._is_active = True
        # self.ws_uri = "ws://127.0.0.1:8765"
        self.ws_uri = "ws://192.168.31.177:8765"
        self._ws = None
        self._ws_loop = asyncio.new_event_loop()
        self._main_task = None

        # 1. 启动硬件监听
        self.spacemouse_listener = SpaceMouseListener(
            devtype="SpaceMouse Compact", ui_label=None # 我们自己控制 UI 更新
        )
        self.spacemouse_listener.sig_data.connect(self.on_spacemouse_data)
        self.spacemouse_listener.start()

        # 2. 启动 WebSocket 线程
        self._ws_thread = threading.Thread(target=self._run_ws_loop, daemon=True)
        self._ws_thread.start()

        # 3. 启动数据发送线程 (50Hz)
        self.th_get_incr = threading.Thread(target=self._get_incr_loop, daemon=True)
        self.th_get_incr.start()

    # ---------- UI 事件处理 ----------

    def on_spacemouse_data(self, data: dict):
        """处理按钮点击等离散事件"""
        event_type = data.get("event")
        btn_name = data.get("btn")
        
        payload = {
            "event": event_type,
            "btn": btn_name,
            "timestamp": time.time()
        }
        
        # 1. 更新 UI 显示
        display_text = f"[{time.strftime('%H:%M:%S')}] {event_type}: {btn_name}"
        self.msg_event.setText(display_text)
        
        # 2. 启动 3s 定时器清空 QLabel
        # 使用 Qt 的 singleShot，它是非阻塞且线程安全的
        qtbase.QTimer.singleShot(3000, lambda: self._clear_event_label(display_text))
        
        # 3. 发送给服务端
        print(f"[Event] {payload}")
        self._ws_send_dict(payload)

    def _clear_event_label(self, original_text):
        """如果当前显示的还是刚才那条消息，则清空"""
        if self.msg_event.text() == original_text:
            self.msg_event.setText("None")

    # ---------- WebSocket 核心 (修复了报错逻辑) ----------

    def _run_ws_loop(self):
        asyncio.set_event_loop(self._ws_loop)
        self._main_task = self._ws_loop.create_task(self._ws_main())
        try:
            self._ws_loop.run_until_complete(self._main_task)
        except asyncio.CancelledError:
            pass
        finally:
            self._cleanup_loop()

    def _cleanup_loop(self):
        try:
            pending = asyncio.all_tasks(self._ws_loop)
            for task in pending:
                task.cancel()
            if pending:
                self._ws_loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            self._ws_loop.run_until_complete(self._ws_loop.shutdown_asyncgens())
        finally:
            self._ws_loop.close()
            print("[ws-client] Loop closed safely.")

    async def _ws_main(self):
        while self._is_active:
            try:
                async with websockets.connect(self.ws_uri) as ws:
                    self._ws = ws
                    print(f"[ws-client] ✅ Connected to server")
                    while self._is_active:
                        await asyncio.sleep(0.5)
            except asyncio.CancelledError:
                if self._ws:
                    await self._ws.close()
                break
            except Exception as e:
                self._ws = None
                if self._is_active:
                    await asyncio.sleep(2.0)
                else:
                    break

    def _ws_send_dict(self, data: dict):
        if not self._ws or not self._is_active:
            return
        async def _send():
            try:
                assert self._ws
                await self._ws.send(json.dumps(data, ensure_ascii=False))
            except:
                pass
        asyncio.run_coroutine_threadsafe(_send(), self._ws_loop)

    # ---------- 数据同步 ----------

    def _get_incr_loop(self):
        while self._is_active:
            time.sleep(0.02)
            state = self.spacemouse_listener.cur_state
            if state:
                xyz = {k: state.get(k, 0.0) for k in ("x", "y", "z", "R", "P", "Y")}

                # 更新 UI (在非主线程更新 UI 需要注意，Qt 推荐使用信号，这里简单演示)
                # 如果运行不稳定，请改用信号发射给 msg_xyz.setText
                self.msg_xyz.setText(f"X: {xyz['x']:.2f} Y: {xyz['y']:.2f} Z: {xyz['z']:.2f}\n"
                                     f"R: {xyz['R']:.2f} P: {xyz['P']:.2f} Y: {xyz['Y']:.2f}")
                
                # self._ws_send_dict({"xyz": xyz})
                self._ws_send_dict(xyz)

    # ---------- 退出处理 ----------

    def closeEvent(self, event: qtbase.QCloseEvent):
        print("\n[App] Closing and cleaning up...")
        self._is_active = False
        self.spacemouse_listener.stop()
        
        if self._ws_loop and self._ws_loop.is_running():
            assert self._main_task
            self._ws_loop.call_soon_threadsafe(self._main_task.cancel)
        
        self._ws_thread.join(timeout=1.0)
        event.accept()

def main():
    app = qtbase.QApplication(sys.argv)
    mapp = TestApp()
    mapp.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
