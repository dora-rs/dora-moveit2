#!/usr/bin/env python3
"""
Dual GEN72 Robot Configuration
Two Realman GEN72 7-DOF arms mounted on a shared table.
Left arm at (-0.3, 0, 0.8), right arm at (0.3, 0, 0.8).
"""

import numpy as np
from typing import Dict, List, Tuple


class DualGEN72Config:
    """Dual GEN72 7-DOF Robot Arms Configuration"""

    # ===== Per-arm joint count =====
    NUM_JOINTS = 7

    # ===== Arm chain identifiers =====
    ARM_CHAINS = ["left_arm", "right_arm"]

    # ===== Arm base transforms (relative to world) =====
    ARM_BASE_TRANSFORMS = {
        "left_arm": {"xyz": [-0.4, 0.0, 0.8], "rpy": [0, 0, 0]},
        "right_arm": {"xyz": [0.4, 0.0, 0.8], "rpy": [0, 0, 3.14159265]},
    }

    # ===== MuJoCo qpos indices per arm =====
    # Layout: left_arm(0-6), left_gripper(7-14), right_arm(15-21), right_gripper(22-29), ball(30)
    ARM_QPOS_START_PER_CHAIN = {
        "left_arm": 0,
        "right_arm": 15,
    }

    # Default ARM_QPOS_START for backward compat (left arm)
    ARM_QPOS_START = 0

    # Total actuators in MuJoCo model
    # Layout: [left_arm(0-6), left_gripper(7), right_arm(8-14), right_gripper(15)]
    NUM_ACTUATORS = 16
    ARM_ACTUATOR_START = 0

    # Gripper actuator indices
    GRIPPER_ACTUATOR_INDEX = {
        "left_arm": 7,
        "right_arm": 15,
    }

    # Per-arm actuator start indices (arm joints only, excluding gripper)
    ARM_ACTUATOR_START_PER_CHAIN = {
        "left_arm": 0,
        "right_arm": 8,
    }

    # ===== Joint limits (same for both arms, from GEN72 URDF) =====
    JOINT_LOWER_LIMITS = np.array([-3.0014, -1.8323, -3.0014, -2.8792, -3.0014, -1.707, -3.0014])
    JOINT_UPPER_LIMITS = np.array([3.0014, 1.8323, 3.0014, 0.9597, 3.0014, 1.783, 3.0014])
    JOINT_VELOCITY_LIMITS = np.array([3.141, 3.141, 3.141, 3.141, 3.926, 3.926, 3.926])

    # ===== Link transforms from URDF (per arm, identical) =====
    _GEN72_LINK_TRANSFORMS = [
        {"xyz": [0, 0, 0.218], "rpy": [0, 0, 0]},
        {"xyz": [0, 0, 0], "rpy": [-1.5708, 0, 0]},
        {"xyz": [0, -0.28, 0], "rpy": [1.5708, 0, 0]},
        {"xyz": [0.04, 0, 0], "rpy": [1.5708, 0, 0]},
        {"xyz": [-0.019, 0.2525, 0], "rpy": [-1.5708, 0, 0]},
        {"xyz": [0, 0, 0], "rpy": [1.5708, 0, 0]},
        {"xyz": [0.0905, 0.067, 0], "rpy": [1.5708, 0, 1.5708]},
    ]

    # Single-arm default (left arm, for backward compat)
    LINK_TRANSFORMS = _GEN72_LINK_TRANSFORMS

    # Per-chain transforms
    LINK_TRANSFORMS_PER_CHAIN = {
        "left_arm": _GEN72_LINK_TRANSFORMS,
        "right_arm": _GEN72_LINK_TRANSFORMS,
    }

    # ===== Collision geometry (per arm, identical) =====
    _GEN72_COLLISION_GEOMETRY = [
        ("sphere", [0.055]),
        ("sphere", [0.045]),
        ("sphere", [0.045]),
        ("sphere", [0.045]),
        ("sphere", [0.04]),
        ("sphere", [0.035]),
        ("sphere", [0.03]),
        ("sphere", [0.025]),
    ]

    COLLISION_GEOMETRY = _GEN72_COLLISION_GEOMETRY
    COLLISION_MARGIN = 0.015

    COLLISION_GEOMETRY_PER_CHAIN = {
        "left_arm": _GEN72_COLLISION_GEOMETRY,
        "right_arm": _GEN72_COLLISION_GEOMETRY,
    }

    # ===== Home / Safe configurations =====
    _LEFT_HOME = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    _RIGHT_HOME = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    HOME_CONFIG = _LEFT_HOME
    SAFE_CONFIG = _LEFT_HOME

    HOME_CONFIG_PER_CHAIN = {
        "left_arm": _LEFT_HOME,
        "right_arm": _RIGHT_HOME,
    }

    # ===== Named poses =====
    NAMED_POSES = {
        "home": _LEFT_HOME,
        "safe": _LEFT_HOME,
        "zero": np.zeros(7),
    }

    NAMED_POSES_PER_CHAIN = {
        "left_arm": {
            "home": _LEFT_HOME,
            "safe": _LEFT_HOME,
            "zero": np.zeros(7),
        },
        "right_arm": {
            "home": _RIGHT_HOME,
            "safe": _RIGHT_HOME,
            "zero": np.zeros(7),
        },
    }

    @staticmethod
    def get_joint_limits() -> Tuple[np.ndarray, np.ndarray]:
        return DualGEN72Config.JOINT_LOWER_LIMITS, DualGEN72Config.JOINT_UPPER_LIMITS

    @staticmethod
    def is_config_valid(q: np.ndarray) -> bool:
        return (np.all(q >= DualGEN72Config.JOINT_LOWER_LIMITS)
                and np.all(q <= DualGEN72Config.JOINT_UPPER_LIMITS))

    @staticmethod
    def clip_to_limits(q: np.ndarray) -> np.ndarray:
        return np.clip(q, DualGEN72Config.JOINT_LOWER_LIMITS, DualGEN72Config.JOINT_UPPER_LIMITS)
