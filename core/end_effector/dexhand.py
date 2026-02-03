import threading
import time
import os
import sys

from toolbox.qt import qtbase
from toolbox.core.log import printc
from Robotic_Arm.rm_robot_interface import *  # noqa: F403
from .common.roh_registers_v1 import *  # noqa: F403
from realman_teleop.core.shared import Shared
from realman_teleop.core.arm.realman import RMArm


# ----------------------------
# Realman 灵巧手
# ----------------------------

class DexterousHand(qtbase.QObject):
    """
    灵巧手控制类：支持五指独立控制、整体开合、旋转拇指根部等操作。
    """
    sig_move_finger = qtbase.Signal(int, int) # index: int, position: int
    sig_ui_pos = qtbase.Signal(list)

    def __init__(self, arm: RMArm, com_port: int = 1, roh_addr: int = 2, delay: float = 1.0):  # noqa: F405
        super().__init__()
        self.arm = arm
        if self.arm.is_connected:
            self._arm = self.arm.robot
            self._handle = self._arm.handle
        else:
            self._arm = None
            self._handle = None
        
        self.com_port = com_port
        self.roh_addr = roh_addr
        self.delay = delay
        self.now = [0, 0, 0, 0, 0]
        self.is_initialized = 0

    def initialize(self):
        # 初始化机械臂通讯
        # self.robot = RobotArmController(arm_ip, 8080, 3)
        if not self._arm:
            return
        self._arm.rm_close_modbustcp_mode()
        self._arm.rm_set_modbus_mode(self.com_port, 115200, 1)
        self.five_finge_release()
        printc(f"✅ 机械臂灵巧手初始化完成")

        # 启动灵巧手状态实时获取线程
        self.th_get_hand_curr = threading.Thread(target=self._get_arm_hand_curr, daemon=True)
        self.th_get_hand_curr.start()
        printc(f"✅ 机械臂灵巧手状态获取线程启动完成")
        self.is_initialized = 1

    def deinitialize(self):
        # if hasattr(self, "th_get_hand_curr"):
        self.is_t_run = 0
        self.is_initialized = 0

    def _get_arm_hand_curr(self):
        if not hasattr(self.arm, "hand"):
            printc("arm 没有初始化 hand 灵巧手！")
            return
        
        self.is_t_run = 1
        while self.is_t_run:
            curr = self.get_current_positions()
            printc(f"curr={curr}")
            if curr is None:
                continue

            # self.ui.pos_f0.setValue
            self.sig_ui_pos.emit([
                curr[0],
                curr[1],
                curr[2],
                curr[3],
                curr[4],
            ])
            time.sleep(1)

    # -------------------------------------------------------------
    # 底层寄存器操作
    # -------------------------------------------------------------
    def _write_registers(self, address, values):
        if not self._arm:
            return False
        params = rm_peripheral_read_write_params_t()  # noqa: F405
        params.port = self.com_port
        params.device = self.roh_addr
        params.address = address
        params.num = len(values)

        values_bytes = []
        for v in values:
            values_bytes.extend([(v >> 8) & 0xFF, v & 0xFF])

        ret = self._arm.rm_write_registers(params, values_bytes)
        if ret != 0:
            printc(f"[ERROR] Write_Registers failed: {ret}")
            return False
        return True

    def _read_registers(self, address, num):
        params = rm_peripheral_read_write_params_t()  # noqa: F405
        params.port = self.com_port
        params.device = self.roh_addr
        params.address = address
        params.num = num

        tag, ret = self._read_registers_by_p(params)
        # tag, ret = self.robot.rm_read_modbus_tcp_input_registers(params)
        if tag != 0:
            printc(f"[ERROR] Read_Registers failed: {tag}")
            return None

        data = [(ret[i] | (ret[i + 1] << 8)) for i in range(0, num * 2, 2)]
        return data


    def _read_registers_by_p(self, read_params: rm_peripheral_read_write_params_t) -> tuple[int, list[int]]:  # noqa: F405
        """
        读多个保存寄存器

        Args:
            read_params (rm_peripheral_read_write_params_t): 多个保存寄存器读取参数结构体，要读的寄存器的数量 2 < num < 13，该指令最多一次性支持读 12 个寄存器数据， 即 24 个 byte

        Returns:
            tuple[int,list[int]]: 包含两个元素的元组
            -int 函数执行的状态码。
                - 0: 成功。
                - 1: 控制器返回false，参数错误或机械臂状态发生错误。
                - -1: 数据发送失败，通信过程中出现问题。
                - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
                - -3: 返回值解析失败，控制器返回的数据无法识别或不完整等情况。
                - -4: 四代控制器不支持该接口
            - list[int]: 返回寄存器数据列表，数据类型：int8
        """
        if read_params.num > 1:
            data_num = int(read_params.num * 2)
            data = (c_int * data_num)()
            tag = rm_read_multiple_holding_registers(
                self._handle, read_params, data)
            return tag, list(data)
        else:
            data = (c_int * 2)()
            tag = rm_read_holding_registers(self._handle, read_params, data)
            return tag, list(data)

    # -------------------------------------------------------------
    # 手指控制方法
    # -------------------------------------------------------------
    def _open_all(self):
        """打开所有手指"""
        printc("[ACTION] Open all fingers")
        # self.now = [0, 65535, 65535, 65535, 65535]
        # self._write_registers(ROH_FINGER_POS_TARGET0, self.now)  # type: ignore
        # time.sleep(3)
        self.now = [0, 0, 0, 0, 0]
        self._write_registers(ROH_FINGER_POS_TARGET0, self.now)  # type: ignore

    def _close_all(self):
        """闭合所有手指"""
        printc("[ACTION] Close all fingers")
        # self.now = [0, 65535, 65535, 65535, 65535]
        # self._write_registers(ROH_FINGER_POS_TARGET0, self.now)  # type: ignore
        # time.sleep(3)
        self._write_registers(ROH_FINGER_POS_TARGET0, [65535, 65535, 65535, 65535, 65535])

    def move_finger(self, index: int, position: int):
        """
        控制单个手指（0-4为手指，5为拇指根部）
        position: 0~65535
        """
        if index < 0 or index > 5:
            printc("[ERROR] Finger index out of range (0-5)")
            return
        printc(f"[ACTION] Move finger {index} -> {position}")
        addr = ROH_FINGER_POS_TARGET0 + index  # type: ignore
        self._write_registers(addr, [position])
        self.sig_move_finger.emit(index, position)
    
    
    def _open_4finger(self):
        """
        控制单个手指（0-4为手指，5为拇指根部）
        position: 0~65535
        """
        OPEN = 0
        self._write_registers(ROH_FINGER_POS_TARGET1, [OPEN, OPEN, OPEN, OPEN])
        # self.sig_move_finger.emit(index, position)
        # for i in range(4):
        #     self.sig_move_finger.emit(i+1, OPEN)
    
    def _open_2finger(self):
        """
        控制单个手指（0-4为手指，5为拇指根部）
        position: 0~65535
        """
        OPEN = 0
        self._write_registers(ROH_FINGER_POS_TARGET0, [OPEN, OPEN,])
        # self.sig_move_finger.emit(index, position)
        # for i in range(4):
        #     self.sig_move_finger.emit(i+1, OPEN)
        
    def _close_4finger(self):
        """
        控制单个手指（0-4为手指，5为拇指根部）
        position: 0~65535
        """
        # addr = ROH_FINGER_POS_TARGET0 + index  # type: ignore
        # self._write_registers(addr, [position])
        CLOSE = 65535
        self._write_registers(ROH_FINGER_POS_TARGET1, [CLOSE, CLOSE, CLOSE, CLOSE])
        
    def _close_2finger(self):
        """
        控制单个手指（0-4为手指，5为拇指根部）
        position: 0~65535
        """
        CLOSE = 65535
        self._write_registers(ROH_FINGER_POS_TARGET0, [CLOSE, CLOSE])
        

    def _close_finger(self, index: int, degree: float = 100):
        """
        以百分比方式闭合单个手指
        degree: 0-100，对应 0~65535
        """
        pos = int(65535 * (degree / 100.0))
        self.move_finger(index, pos)

    def _open_finger(self, index: int):
        """打开指定手指"""
        self.move_finger(index, 0)
   
    def _rotate_thumb(self, degree: float = 100):
        """旋转拇指根部"""
        pos = int(65535 * (degree / 100.0))
        self._write_registers(ROH_FINGER_POS_TARGET5, [pos])  # type: ignore
        # time.sleep(self.delay)

    # -------------------------------------------------------------
    # 状态读取
    # -------------------------------------------------------------
    def get_target_positions(self):
        """读取目标位置"""
        return self._read_registers(ROH_FINGER_POS_TARGET0, 5)  # type: ignore

    def get_current_positions(self):
        """读取当前实际位置"""
        return self._read_registers(ROH_FINGER_POS0, 5)  # type: ignore

    # -------------------------------------------------------------
    # 综合动作示例
    # -------------------------------------------------------------
    # def test_sequence(self, loops=5):
    #     for i in range(loops):
    #         printc(f"\n--- Loop {i + 1} ---")
    #         # 拇指闭合-张开
    #         self.close_finger(0)
    #         self.open_finger(0)
    #         # 拇指旋转
    #         self.rotate_thumb(100)
    #         self.rotate_thumb(0)
    #         # 其他手指闭合-张开
    #         self._write_registers(ROH_FINGER_POS_TARGET1, [65535, 65535, 65535, 65535])
    #         time.sleep(self.delay)
    #         self._write_registers(ROH_FINGER_POS_TARGET1, [0, 0, 0, 0])
    #         time.sleep(self.delay)
    #         target = self.get_target_positions()
    #         current = self.get_current_positions()
    #         printc(f"Target: {target}")
    #         printc(f"Current: {current}")
    #     printc("[INFO] Test sequence complete.")


    # -------------------------------------------------------------
    # 常用组合操作
    # -------------------------------------------------------------

    def two_finge_release(self):
        """取消两指捏住"""
        self._open_2finger()
        self._hand_sleep()
        self._rotate_thumb(0)
        Shared.is_ee_opened = 1

    def two_finge_pick(self):
        """两指捏住"""
        self._rotate_thumb(90)
        self._hand_sleep()
        self._close_2finger()
        Shared.is_ee_opened = 0

    def five_finge_release(self):
        """取消五指捏住"""
        self._open_finger(0)
        self._hand_sleep()
        self._open_4finger()
        self._rotate_thumb(100)
        Shared.is_ee_opened = 1

    def five_finge_pick(self):
        """五指捏住"""
        self._close_4finger()
        self._rotate_thumb(100)
        self._hand_sleep()
        self._close_finger(0)
        Shared.is_ee_opened = 0
    
    def pick(self):
        self.five_finge_pick()

    def release(self):
        self.five_finge_release()

    def _hand_sleep(self):
        #self.msleep(300)
        time.sleep(300/1000)
