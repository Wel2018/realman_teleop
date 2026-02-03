from toolbox.core.log import printc
# from Robotic_Arm.rm_robot_interface import RoboticArm
from toolbox.qt import qtbase
from realman_teleop.core.shared import Shared
from realman_teleop import APPCFG
from realman_teleop.core.arm.realman import RMArm


class Gripper(qtbase.QObject):
    """夹爪"""

    def __init__(self, arm: RMArm):
        self.arm = arm
        if self.arm.is_connected:
            self._arm = self.arm.robot
        else:
            self._arm = None
        self.is_initialized = 0

    def initialize(self):
        self.release()
        self.is_initialized = 1

    def deinitialize(self):
        self.is_initialized = 0

    def pick(self):
        if not self._arm:
            return None
        try:
            ret = self._arm.rm_set_gripper_pick(APPCFG['arm_gripper_speed'], APPCFG['arm_gripper_force'], True, 10)
            printc(f"gripper_close: {ret}")
            Shared.is_ee_opened = 0
            return ret
        except Exception as e:
            printc(f"gripper_close failed: {e}", 'err')

    def release(self):
        if not self._arm:
            return None
            
        try:
            ret = self._arm.rm_set_gripper_release(APPCFG['arm_gripper_speed'], True, 10)
            printc(f"gripper_open: {ret}")
            Shared.is_ee_opened = 1
            return ret
        except Exception as e:
            printc(f"gripper_open failed: {e}", 'err')
