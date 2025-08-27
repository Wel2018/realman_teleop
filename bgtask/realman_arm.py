# 官方文档：https://develop.realman-robotics.com/robot/summarize/
# pip install Robotic_Arm
# Successfully installed Robotic_Arm-1.1.1
from Robotic_Arm.rm_robot_interface import * # type: ignore
from rich import print


"""
current c api version:  1.1.1
机械臂ID： 1

================== Arm Software Information ==================
Arm Model:  RML63III-BI
Algorithm Library Version:  1.5.5-c0d52d18
Control Layer Software Version:  V1.7.1
Dynamics Version:  2
Planning Layer Software Version:  V1.7.1
==============================================================
"""


class RealmanArmClient:

    def __init__(self) -> None:
        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.handle = self.robot.rm_create_robot_arm("192.168.10.19", 8080)
        print("机械臂ID：", self.handle.id)

        software_info = self.robot.rm_get_arm_software_info()
        if software_info[0] == 0:
            print("\n================== Arm Software Information ==================")
            print("Arm Model: ", software_info[1]['product_version'])
            print("Algorithm Library Version: ", software_info[1]['algorithm_info']['version'])
            print("Control Layer Software Version: ", software_info[1]['ctrl_info']['version'])
            print("Dynamics Version: ", software_info[1]['dynamic_info']['model_version'])
            print("Planning Layer Software Version: ", software_info[1]['plan_info']['version'])
            print("==============================================================\n")
        else:
            print("\nFailed to get arm software information, Error code: ", software_info[0], "\n")

    def move_p_canfd(self, pose: list):
        # self.robot.rm_movej_p
        ret = self.robot.rm_movep_canfd(pose, False, 1, 60)
        print(f"MOVE_P: {pose}, ret={ret}")

    def gripper_open(self):
        ret = self.robot.rm_set_gripper_release(500, True, 10)

    def gripper_close(self):
        ret = self.robot.rm_set_gripper_pick(500, 200, True, 10)

    def __del__(self):
        ret = self.robot.rm_delete_robot_arm()

    def get_pose(self):
        return self.robot.rm_get_current_arm_state()
