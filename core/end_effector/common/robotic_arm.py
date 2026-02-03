import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))

from robotic_arm_controller.rm_robot_interface import *


class RobotArmController:
    def __init__(self, ip, port=0, level=3, mode=2):
        """Initialize and connect to the robotic arm.

        Args:
            ip (str): IP address of the robot arm.
            port (int): Port number.
            level (int, optional): Connection level. Defaults to 3.
            mode (int, optional): Thread mode (0: single, 1: dual, 2: triple). Defaults to 2.
        """
        self.thread_mode = rm_thread_mode_e(mode)
        self.robot = RoboticArm(self.thread_mode)
        self.handle = self.robot.rm_create_robot_arm(ip, port, level)

        if self.handle.id == -1:
            print("\nFailed to connect to the robot arm\n")
            exit(1)
        else:
            print(f"\nSuccessfully connected to the robot arm: {self.handle.id}\n")

    def Set_Modbus_Mode(self, port=1, baudrate=115200, timeout=1):
        """Set the Modbus RTU mode.

        Args:
            port (int): Communication port. 0 for controller RS485 port as RTU master, 1 for end interface board RS485 port as RTU master, 2 for controller RS485 port as RTU slave.
            baudrate (int): Baud rate. Supports 9600, 115200, 460800.
            timeout (int): Timeout duration in hundred milliseconds. For all read and write commands to Modbus devices, if no response data is returned within the specified timeout period, a timeout error is returned. The timeout cannot be 0; if set to 0, the robot arm will configure it as 1.

        Returns:
            None
        """
        set_result = self.robot.rm_set_modbus_mode(port, baudrate, timeout)
        if set_result == 0:
            print("\nSuccessfully set the Modbus mode\n")
        else:
            print("\nFailed to set the Modbus mode\n")

    def Close_Modbus_Mode(self, port=0):
        """Close the Modbus RTU mode.

        Args:
            port (int): Communication port. 0 for controller RS485 port, 1 for end interface board RS485 port, 3 for controller ModbusTCP device.

        Returns:
            None
        """
        close_result = self.robot.rm_close_modbus_mode(port)
        if close_result == 0:
            print("\nSuccessfully closed the Modbus mode\n")
        else:
            print("\nFailed to close the Modbus mode\n")

    def Close_Modbustcp_Mode(self) -> int:
        """
        关闭通讯端口ModbusTCP模式

        Returns:
            int: 函数执行的状态码。
            - 0: 成功。
            - 1: 读取失败，超时时间内未获取到数据。
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
            - -4: 四代控制器不支持该接口
        """
        tag = rm_close_modbustcp_mode(self.handle)
        return tag

    def Movej_Cmd(self, joint, v=20, r=0, connect=0, block=1):
        """
        Perform movej motion.

        Args:
            joint (list of float): Joint positions.
            v (float, optional): Speed of the motion. Defaults to 20.
            connect (int, optional): Trajectory connection flag. Defaults to 0.
            block (int, optional): Whether the function is blocking (1 for blocking, 0 for non-blocking). Defaults to 1.
            r (float, optional): Blending radius. Defaults to 0.

        Returns:
            None
        """
        movej_result = self.robot.rm_movej(joint, v, r, connect, block)
        if movej_result != 0:
            print("\nmovej motion fawrite_paramsiled, Error code: ", movej_result, "\n")

    def Movel_Cmd(self, pose: list[float], v: int, r: int, connect: int, block: int) -> int:
        """
        笛卡尔空间直线运动

        Args:
            pose (list[float]): 目标位姿,位置单位：米，姿态单位：弧度
            v (int): 速度百分比系数，1~100
            r (int, optional): 交融半径百分比系数，0~100。
            connect (int): 轨迹连接标志
                - 0：立即规划并执行轨迹，不与后续轨迹连接。
                - 1：将当前轨迹与下一条轨迹一起规划，但不立即执行。阻塞模式下，即使发送成功也会立即返回。
            block (int): 阻塞设置
                - 多线程模式：
                    - 0：非阻塞模式，发送指令后立即返回。
                    - 1：阻塞模式，等待机械臂到达目标位置或规划失败后才返回。
                - 单线程模式：
                    - 0：非阻塞模式。
                    - 其他值：阻塞模式并设置超时时间，单位为秒。
        """

        po1 = rm_pose_t()
        po1.position = rm_position_t(*pose[:3])
        po1.euler = rm_euler_t(*pose[3:])

        tag = rm_movel(self.handle, po1, v, r, connect, block)

        return tag
    
    def Write_Registers(self, write_params: rm_peripheral_read_write_params_t, data: list[int]) -> int:
        """
        写多个寄存器

        Args:
            write_params (rm_peripheral_read_write_params_t): 多个寄存器数据写入参数结构体。其中寄存器每次写的数量不超过10个，即该结构体成员num<=10。
            data (list[int]): 要写入寄存器的数据数组，类型：byte。

        Returns:
            int: 函数执行的状态码。
            - 0: 成功。
            - 1: 读取失败，超时时间内未获取到数据。
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
            - -4: 四代控制器不支持该接口
        """

        data_num = int(write_params.num * 2)
        datas = (c_int * data_num)(*data)
        tag = rm_write_registers(self.handle, write_params, datas)
        return tag
    
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

    def Set_Plus_Mode(self):
        return self.robot.rm_set_rm_plus_mode(115200)

    def Get_Base_info(self):
        return self.robot.rm_get_rm_plus_base_info()

    def Get_State_info(self):
        return self.robot.rm_get_rm_plus_state_info()

    def Set_Hand_Follow_pos(self, hand_pos: list[int], block:bool) -> int:
        return self.robot.rm_set_hand_follow_pos(hand_pos, block)
