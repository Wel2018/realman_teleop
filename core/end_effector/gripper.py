from toolbox.core.log import printc
from Robotic_Arm.rm_robot_interface import RoboticArm
from toolbox.qt import qtbase
from realman_teleop.core.shared import Shared
from realman_teleop import APPCFG


class Gripper(qtbase.QObject):
    """夹爪"""

    def __init__(self, robot: RoboticArm):
        self.robot = robot

    def initialize(self):
        self.release()
    def deinitialize(self):
        ...

    def pick(self):
        try:
            ret = self.robot.rm_set_gripper_pick(APPCFG['arm_gripper_speed'], APPCFG['arm_gripper_force'], True, 10)
            printc(f"gripper_close: {ret}")
            Shared.is_ee_opened = 0
            return ret
        except Exception as e:
            printc(f"gripper_close failed: {e}", 'err')

    def release(self):
        try:
            ret = self.robot.rm_set_gripper_release(APPCFG['arm_gripper_speed'], True, 10)
            printc(f"gripper_open: {ret}")
            Shared.is_ee_opened = 1
            return ret
        except Exception as e:
            printc(f"gripper_open failed: {e}", 'err')
