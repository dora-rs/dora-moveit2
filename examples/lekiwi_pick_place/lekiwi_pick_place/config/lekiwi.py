#!/usr/bin/env python3
"""
LeKiwi Robot Configuration (SO_ARM100 6-DOF arm on omnidirectional base)
Based on SIGRobotics-UIUC/LeKiwi-sim MuJoCo model.

Arm kinematic chain (6 revolute joints):
  Joint 1 (Rotation):    Base rotation, axis -Y
  Joint 2 (Pitch):       Shoulder pitch, axis X
  Joint 3 (Elbow):       Elbow pitch, axis X
  Joint 4 (Wrist_Pitch): Wrist pitch, axis X
  Joint 5 (Wrist_Roll):  Wrist roll, axis Y
  Joint 6 (Jaw):         Gripper, axis Z

qpos layout: freejoint[0:7], wheels[7:10], arm[10:16]
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple


@dataclass
class JointConfig:
    """Single joint configuration"""
    name: str
    lower_limit: float
    upper_limit: float
    velocity_limit: float
    effort_limit: float


class LekiwiConfig:
    """LeKiwi SO_ARM100 6-DOF Arm Configuration"""

    NUM_JOINTS = 6
    NUM_ACTUATORS = 9       # 3 wheel velocity + 6 arm position actuators
    ARM_QPOS_START = 10     # arm joints start at qpos[10] (after freejoint[0:7] + 3 wheels[7:10])
    ARM_ACTUATOR_START = 3  # arm actuators start at ctrl[3] (after 3 wheel actuators)

    JOINT_CONFIGS = [
        JointConfig("Rotation", -1.92, 1.92, 5.0, 3.5),
        JointConfig("Pitch", -1.747, 1.747, 5.0, 3.5),
        JointConfig("Elbow", -1.657, 1.657, 5.0, 3.5),
        JointConfig("Wrist_Pitch", -1.66, 1.66, 5.0, 3.5),
        JointConfig("Wrist_Roll", -2.79, 2.79, 5.0, 3.5),
        JointConfig("Jaw", 0.0, 0.6, 5.0, 3.5),
    ]

    JOINT_LOWER_LIMITS = np.array([j.lower_limit for j in JOINT_CONFIGS])
    JOINT_UPPER_LIMITS = np.array([j.upper_limit for j in JOINT_CONFIGS])
    JOINT_VELOCITY_LIMITS = np.array([j.velocity_limit for j in JOINT_CONFIGS])
    JOINT_EFFORT_LIMITS = np.array([j.effort_limit for j in JOINT_CONFIGS])

    # Link transforms from so_arm100.xml kinematic chain.
    # Extracted from body positions and joint axes in the MJCF.
    LINK_TRANSFORMS = [
        # Joint 1 (Rotation): base -> rotation_pitch
        {"xyz": [0.0, -0.0452, 0.0165], "rpy": [0.7854, 0.7854, 0], "axis": [0, -1, 0]},
        # Joint 2 (Pitch): rotation_pitch -> upper_arm
        {"xyz": [0.0, 0.1025, 0.0306], "rpy": [0, 0, 0], "axis": [1, 0, 0]},
        # Joint 3 (Elbow): upper_arm -> lower_arm
        {"xyz": [0.0, 0.11257, 0.028], "rpy": [0, 0, 0], "axis": [1, 0, 0]},
        # Joint 4 (Wrist_Pitch): lower_arm -> wrist_pitch_roll
        {"xyz": [0.0, 0.0052, 0.1349], "rpy": [-1.5708, 0, 0], "axis": [1, 0, 0]},
        # Joint 5 (Wrist_Roll): wrist_pitch_roll -> fixed_jaw
        {"xyz": [0.0, -0.0601, 0.0], "rpy": [0, 1.5708, 0], "axis": [0, 1, 0]},
        # Joint 6 (Jaw): fixed_jaw -> moving_jaw
        {"xyz": [-0.0202, -0.0244, 0.0], "rpy": [0, 3.14, -0.174], "axis": [0, 0, 1]},
    ]

    # EE offset from last joint to fingertip
    EE_OFFSET = np.array([0.0, -0.05, 0.0])

    # Collision geometry (simplified spheres for each arm link)
    COLLISION_GEOMETRY = [
        ("sphere", [0.05]),   # base/rotation link
        ("sphere", [0.05]),   # shoulder link
        ("sphere", [0.04]),   # upper arm link
        ("sphere", [0.04]),   # lower arm link
        ("sphere", [0.035]),  # wrist link
        ("sphere", [0.03]),   # gripper link
    ]

    COLLISION_MARGIN = 0.015
    MAX_ACCELERATION = np.array([8.0, 8.0, 8.0, 8.0, 8.0, 8.0])

    # From so_arm100.xml keyframes
    HOME_CONFIG = np.array([0.0, 0.0, 0.0, 1.57, -1.57, 0.0])
    SAFE_CONFIG = np.array([0.0, -1.75, 1.59, 1.13, 0.0, 0.0])

    NAMED_POSES = {
        "home": HOME_CONFIG,
        "safe": SAFE_CONFIG,
        "rest": SAFE_CONFIG,
        "zero": np.zeros(6),
    }

    @staticmethod
    def get_joint_limits() -> Tuple[np.ndarray, np.ndarray]:
        return LekiwiConfig.JOINT_LOWER_LIMITS, LekiwiConfig.JOINT_UPPER_LIMITS

    @staticmethod
    def get_velocity_limits() -> np.ndarray:
        return LekiwiConfig.JOINT_VELOCITY_LIMITS

    @staticmethod
    def is_config_valid(q: np.ndarray) -> bool:
        return bool(
            np.all(q >= LekiwiConfig.JOINT_LOWER_LIMITS)
            and np.all(q <= LekiwiConfig.JOINT_UPPER_LIMITS)
        )

    @staticmethod
    def clip_to_limits(q: np.ndarray) -> np.ndarray:
        return np.clip(q, LekiwiConfig.JOINT_LOWER_LIMITS, LekiwiConfig.JOINT_UPPER_LIMITS)
