#!/usr/bin/env python3
"""
GEN72 + Robotiq 2F-85 Gripper Robot Configuration
Based on GEN72 7-DOF arm with Robotiq 2F-85 gripper attached at Link7.
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple

from .gen72 import JointConfig


class GEN72WithGripperConfig:
    """GEN72 7-DOF Robot Arm + Robotiq 2F-85 Gripper Configuration"""

    NUM_JOINTS = 7
    NUM_ACTUATORS = 8  # 7 arm + 1 gripper
    ARM_QPOS_START = 7  # cube freejoint occupies qpos[0:7]

    # Joint limits (same as GEN72)
    JOINT_CONFIGS = [
        JointConfig("joint1", -3.0014, 3.0014, 3.141, 25.0),
        JointConfig("joint2", -1.8323, 1.8323, 3.141, 25.0),
        JointConfig("joint3", -3.0014, 3.0014, 3.141, 25.0),
        JointConfig("joint4", -2.8792, 0.9597, 3.141, 25.0),
        JointConfig("joint5", -3.0014, 3.0014, 3.926, 5.0),
        JointConfig("joint6", -1.707, 1.783, 3.926, 5.0),
        JointConfig("joint7", -3.0014, 3.0014, 3.926, 5.0),
    ]

    JOINT_LOWER_LIMITS = np.array([j.lower_limit for j in JOINT_CONFIGS])
    JOINT_UPPER_LIMITS = np.array([j.upper_limit for j in JOINT_CONFIGS])
    JOINT_VELOCITY_LIMITS = np.array([j.velocity_limit for j in JOINT_CONFIGS])
    JOINT_EFFORT_LIMITS = np.array([j.effort_limit for j in JOINT_CONFIGS])

    # Link transforms from URDF (same as GEN72)
    LINK_TRANSFORMS = [
        {"xyz": [0, 0, 0.218], "rpy": [0, 0, 0]},
        {"xyz": [0, 0, 0], "rpy": [-1.5708, 0, 0]},
        {"xyz": [0, -0.28, 0], "rpy": [1.5708, 0, 0]},
        {"xyz": [0.04, 0, 0], "rpy": [1.5708, 0, 0]},
        {"xyz": [-0.019, 0.2525, 0], "rpy": [-1.5708, 0, 0]},
        {"xyz": [0, 0, 0], "rpy": [1.5708, 0, 0]},
        {"xyz": [0.0905, 0.067, 0], "rpy": [1.5708, 0, 1.5708]},
    ]

    # EE offset: gripper_mount(0.055) + base_mount(0.007) + gripper_base(0.0038) + pinch(0.145)
    EE_OFFSET = np.array([0, 0, -0.2108])

    # Collision geometry (same as GEN72)
    COLLISION_GEOMETRY = [
        ("sphere", [0.055]),
        ("sphere", [0.045]),
        ("sphere", [0.045]),
        ("sphere", [0.045]),
        ("sphere", [0.04]),
        ("sphere", [0.035]),
        ("sphere", [0.03]),
        ("sphere", [0.025]),
    ]

    COLLISION_MARGIN = 0.015
    MAX_ACCELERATION = np.array([5.0, 5.0, 5.0, 5.0, 8.0, 8.0, 8.0])

    HOME_CONFIG = np.array([0.0, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0])
    SAFE_CONFIG = np.array([0.0, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0])

    NAMED_POSES = {
        "home": HOME_CONFIG,
        "safe": SAFE_CONFIG,
        "zero": np.zeros(7),
    }

    # Gripper control range (Robotiq 2F-85: 0=open, 255=closed)
    GRIPPER_OPEN = 0.0
    GRIPPER_CLOSED = 255.0
    GRIPPER_OUTPUT_NAME = "gripper_control"
    GRIPPER_SETTLE_DURATION = 2.0
    GRIPPER_POLL_TIMEOUT = 0.05

    @staticmethod
    def get_joint_limits() -> Tuple[np.ndarray, np.ndarray]:
        return GEN72WithGripperConfig.JOINT_LOWER_LIMITS, GEN72WithGripperConfig.JOINT_UPPER_LIMITS

    @staticmethod
    def get_velocity_limits() -> np.ndarray:
        return GEN72WithGripperConfig.JOINT_VELOCITY_LIMITS

    @staticmethod
    def is_config_valid(q: np.ndarray) -> bool:
        return (np.all(q >= GEN72WithGripperConfig.JOINT_LOWER_LIMITS) and
                np.all(q <= GEN72WithGripperConfig.JOINT_UPPER_LIMITS))

    @staticmethod
    def clip_to_limits(q: np.ndarray) -> np.ndarray:
        return np.clip(q, GEN72WithGripperConfig.JOINT_LOWER_LIMITS, GEN72WithGripperConfig.JOINT_UPPER_LIMITS)
