from __future__ import annotations
import numpy as np
from Robotic_Arm.rm_robot_interface import *  # noqa: F403


# ----------------------------
# numpy 旋转矩阵（更高效）
# ----------------------------
def rotx(theta: float) -> np.ndarray:
    """绕 X 轴旋转矩阵 3x3"""
    c = np.cos(theta); s = np.sin(theta)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]], dtype=float)

def roty(theta: float) -> np.ndarray:
    """绕 Y 轴旋转矩阵 3x3"""
    c = np.cos(theta); s = np.sin(theta)
    return np.array([[c, 0, -s],
                     [0, 1, 0],
                     [s, 0, c]], dtype=float)

def rotz(theta: float) -> np.ndarray:
    """绕 Z 轴旋转矩阵 3x3"""
    c = np.cos(theta); s = np.sin(theta)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]], dtype=float)

_ROT_MAP = {"x": rotx, "y": roty, "z": rotz}

# ----------------------------
# 将 3x3 rotation -> pose（使用 Algo 的转换函数）
# ----------------------------
def _matrix3_to_pose(algo: "Algo", R3: np.ndarray, position: list[float]) -> list[float]:  # noqa: N803
    """
    将 3x3 旋转矩阵和位置组合成 4x4 rm_matrix_t 后调用算法库转换为 pose。
    algo: 复用的 Algo 实例（必须存在）
    R3: 3x3 numpy array
    position: [x,y,z]（保留原值）
    返回 [x,y,z,roll,pitch,yaw]
    """
    # 构造 4x4 数据列表（row-major）
    data = [
        [float(R3[0, 0]), float(R3[0, 1]), float(R3[0, 2]), 0.0],
        [float(R3[1, 0]), float(R3[1, 1]), float(R3[1, 2]), 0.0],
        [float(R3[2, 0]), float(R3[2, 1]), float(R3[2, 2]), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    mat = rm_matrix_t(4, 4, data)  # noqa: F405
    pose = algo.rm_algo_matrix2pos(mat)  # [x,y,z,roll,pitch,yaw]
    # 保持原 position（x,y,z）
    pose[0:3] = [float(position[0]), float(position[1]), float(position[2])]
    return pose

def pose_by_rot_cached(
    algo: "Algo",
    pose: list[float],
    rot_axis: str = "x",
    theta: float = 0.01,
) -> list[float]:
    """
    基于传入的 Algo（避免重复构建）对 pose 做绕轴小旋转并返回新的 pose。
    pose: [x,y,z, r,p,y] 其中 r,p,y 单位与 Algo 期望一致（保持原来的行为）
    rot_axis: 'x'/'y'/'z'
    theta: 小角度（弧度）
    """
    if rot_axis not in _ROT_MAP:
        raise ValueError("rot_axis must be 'x', 'y' or 'z'")

    # 提取位置与 RPY（注意：原代码中有 prec，但为 1；如果你的 RPY 是度请在这里转换）
    pos = [float(pose[0]), float(pose[1]), float(pose[2])]
    rpy = [float(pose[3]), float(pose[4]), float(pose[5])]

    # 1) 当前 R0（4x4 matrix） -> 提取 3x3
    R0 = algo.rm_algo_euler2matrix(rpy)  # 返回 rm_matrix_t
    R_np = np.array(R0.data, dtype=float).reshape(4, 4)
    R3 = R_np[:3, :3]

    # 2) 计算 dR 并更新 R
    dR = _ROT_MAP[rot_axis](theta)
    R_new = dR @ R3  # 矩阵乘法（更快）

    # 3) 转回 pose
    return _matrix3_to_pose(algo, R_new, pos)
