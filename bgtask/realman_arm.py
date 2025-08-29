# 官方文档：https://develop.realman-robotics.com/robot/apipython/classes/roboticArm/
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
        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E) # type: ignore
        self.handle = self.robot.rm_create_robot_arm("192.168.10.19", 8080)
        print("机械臂ID：", self.handle.id)
        # self.robot.rm_change_tool_frame("hhltest2")
        self.robot.rm_change_tool_frame("Arm_Tip")

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

        # pose=(0, {'joint': [90.42400360107422, 48.599998474121094, -119.23500061035156, 175.34500122070312, 17.634000778198242, -148.9080047607422], 'pose': [-0.000791, -0.275186, 0.616766, -0.766, -1.527, 2.32], 'err': {'err_len': 1, 'err': ['0']}})
        self.gozero()
    
    def gozero(self):
        ret = self.robot.rm_movej([90.42400360107422, 48.599998474121094, -119.23500061035156, 175.34500122070312, 17.634000778198242, -148.9080047607422], 20, 0, 0, 1)

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
    

from toolbox.qt import qtbase
from .spacemouse import SpaceMouseListener


class RealmanArmTask(qtbase.QAsyncTask):

    def set_gripper(self):
        """夹爪控制"""
        self.is_gripper_open = not self.is_gripper_open
        if self.is_gripper_open:
            # self.add_log("打开夹爪")
            self.arm.gripper_open()
        else:
            # self.add_log("关闭夹爪")
            self.arm.gripper_close()
    
    def __init__(self, arm: RealmanArmClient, spacemouse_th: SpaceMouseListener, conf: dict = {}):
        super().__init__(conf)
        # self.device = SpaceMouse(devtype)
        # self.cur_state = {}
        self.spacemouse_th = spacemouse_th
        self.arm = arm
        self.is_gripper_open = 1
    
    def run(self):
        self.is_run = 1
        while self.is_run:
            cur_state = self.spacemouse_th.cur_state.copy()

            if cur_state['gripper']:
                print("gripper")
                self.set_gripper()
                # return
                continue
            
            if cur_state['gozero']:
                # self.arm.join(1)
                # self.arm.stop()
                # self.gozero()
                print("gozero")
                continue
            
            # continue
            ZERO = 0.001
            is_all_zero = 1
            for k in ['x', 'y', 'z', 'R', 'P', 'Y', 'gripper', 'gozero']:
                if cur_state[k] >= ZERO:
                    is_all_zero = 0
            
            if is_all_zero:
                self.msleep(10)
                continue
            
            # 获取增量
            _incr = {
                'x': .0,
                'y': .0,
                'z': .0,
                'R': .0,
                'P': .0,
                'Y': .0,
            }
            for k in _incr.keys():
                _incr[k] = cur_state[k] * 0.1
                if _incr[k] > 0.5:
                    _incr[k] *= 0.1
                
                if k in ['x', 'y']:
                    _incr[k] = -_incr[k]

            # 获取当前机械臂状态
            _p = self.arm.get_pose()
            _xyzRPY = _p[1]['pose']
            pose = {
                'x': _xyzRPY[0],
                'y': _xyzRPY[1],
                'z': _xyzRPY[2],
                'R': _xyzRPY[3],
                'P': _xyzRPY[4],
                'Y': _xyzRPY[5],
            }

            for k in pose.keys():
                pose[k] += _incr[k]

            pose['R'], pose['P'] = -pose['P'], -pose['R']

            # pose = list(pose.values())
            ret = self.arm.robot.rm_movep_canfd(
                list(pose.values()), 
                False, 0, 60)
            # print(f"ret={ret}, pose={pose}")
            # SharedData.incr.update(_incr)

            print("-"*50)
            _incr = [round(_incr[k], 2) for k in _incr.keys()]
            pose = [round(pose[k], 2) for k in pose.keys()] # type: ignore
            print(f"incr={_incr}")
            print(f"pose={pose}")
