import time
import os
import sys

import time
from Robotic_Arm.rm_robot_interface import * # type: ignore
from toolbox.core.color_print import printc
from toolbox.qt import qtbase  # type: ignore
from .common.roh_registers_v1 import *  # type: ignore


# ----------------------------
# Realman 灵巧手
# ----------------------------

class DexterousHand(qtbase.QObject):
    """
    灵巧手控制类：支持五指独立控制、整体开合、旋转拇指根部等操作。
    """
    sig_move_finger = qtbase.Signal(int, int) # index: int, position: int

    def __init__(self, robot: RoboticArm, com_port: int = 1, roh_addr: int = 2, delay: float = 1.0):
        super().__init__()
        self.robot = robot
        self.handle = self.robot.handle
        self.com_port = com_port
        self.roh_addr = roh_addr
        self.delay = delay

        # 初始化机械臂通讯
        # self.robot = RobotArmController(arm_ip, 8080, 3)
        self.robot.rm_close_modbustcp_mode()
        self.robot.rm_set_modbus_mode(com_port, 115200, 1)
        printc(f"✅ 机械臂灵巧手初始化完成")
        self.arm = self.robot
        self.now = [0, 0, 0, 0, 0]

    # -------------------------------------------------------------
    # 底层寄存器操作
    # -------------------------------------------------------------
    def _write_registers(self, address, values):
        params = rm_peripheral_read_write_params_t()  # type: ignore
        params.port = self.com_port
        params.device = self.roh_addr
        params.address = address
        params.num = len(values)

        values_bytes = []
        for v in values:
            values_bytes.extend([(v >> 8) & 0xFF, v & 0xFF])

        ret = self.robot.rm_write_registers(params, values_bytes)
        if ret != 0:
            printc(f"[ERROR] Write_Registers failed: {ret}")
            return False
        return True

    def _read_registers(self, address, num):
        params = rm_peripheral_read_write_params_t()  # type: ignore
        params.port = self.com_port
        params.device = self.roh_addr
        params.address = address
        params.num = num

        tag, ret = self.Read_Registers(params)
        # tag, ret = self.robot.rm_read_modbus_tcp_input_registers(params)
        if tag != 0:
            printc(f"[ERROR] Read_Registers failed: {tag}")
            return None

        data = [(ret[i] | (ret[i + 1] << 8)) for i in range(0, num * 2, 2)]
        return data


    def Read_Registers(self, read_params: rm_peripheral_read_write_params_t) -> tuple[int, list[int]]:
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
                self.handle, read_params, data)
            return tag, list(data)
        else:
            data = (c_int * 2)()
            tag = rm_read_holding_registers(self.handle, read_params, data)
            return tag, list(data)

    # -------------------------------------------------------------
    # 手指控制方法
    # -------------------------------------------------------------
    def open_all(self):
        """打开所有手指"""
        printc("[ACTION] Open all fingers")
        # self.now = [0, 65535, 65535, 65535, 65535]
        # self._write_registers(ROH_FINGER_POS_TARGET0, self.now)  # type: ignore
        # time.sleep(3)
        self.now = [0, 0, 0, 0, 0]
        self._write_registers(ROH_FINGER_POS_TARGET0, self.now)  # type: ignore

    def close_all(self):
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
    
    
    def open_4finger(self):
        """
        控制单个手指（0-4为手指，5为拇指根部）
        position: 0~65535
        """
        OPEN = 0
        self._write_registers(ROH_FINGER_POS_TARGET1, [OPEN, OPEN, OPEN, OPEN])
        # self.sig_move_finger.emit(index, position)
        # for i in range(4):
        #     self.sig_move_finger.emit(i+1, OPEN)
    
    def open_2finger(self):
        """
        控制单个手指（0-4为手指，5为拇指根部）
        position: 0~65535
        """
        OPEN = 0
        self._write_registers(ROH_FINGER_POS_TARGET0, [OPEN, OPEN,])
        # self.sig_move_finger.emit(index, position)
        # for i in range(4):
        #     self.sig_move_finger.emit(i+1, OPEN)
        
    def close_4finger(self):
        """
        控制单个手指（0-4为手指，5为拇指根部）
        position: 0~65535
        """
        # addr = ROH_FINGER_POS_TARGET0 + index  # type: ignore
        # self._write_registers(addr, [position])
        CLOSE = 65535
        self._write_registers(ROH_FINGER_POS_TARGET1, [CLOSE, CLOSE, CLOSE, CLOSE])
        
    def close_2finger(self):
        """
        控制单个手指（0-4为手指，5为拇指根部）
        position: 0~65535
        """
        CLOSE = 65535
        self._write_registers(ROH_FINGER_POS_TARGET0, [CLOSE, CLOSE])
        

    def close_finger(self, index: int, degree: float = 100):
        """
        以百分比方式闭合单个手指
        degree: 0-100，对应 0~65535
        """
        pos = int(65535 * (degree / 100.0))
        self.move_finger(index, pos)

    def open_finger(self, index: int):
        """打开指定手指"""
        self.move_finger(index, 0)
   
    def rotate_thumb(self, degree: float = 100):
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
    def test_sequence(self, loops=5):
        for i in range(loops):
            printc(f"\n--- Loop {i + 1} ---")

            # 拇指闭合-张开
            self.close_finger(0)
            self.open_finger(0)

            # 拇指旋转
            self.rotate_thumb(100)
            self.rotate_thumb(0)

            # 其他手指闭合-张开
            self._write_registers(ROH_FINGER_POS_TARGET1, [65535, 65535, 65535, 65535])  # type: ignore
            time.sleep(self.delay)
            self._write_registers(ROH_FINGER_POS_TARGET1, [0, 0, 0, 0])  # type: ignore
            time.sleep(self.delay)

            target = self.get_target_positions()
            current = self.get_current_positions()
            printc(f"Target: {target}")
            printc(f"Current: {current}")

        printc("[INFO] Test sequence complete.")
