#!/usr/bin/env python3
"""
ADORA1 Nano Robot Configuration (SO_ARM100 6-DOF arm)
Extracted from URDF: nano/nano.urdf

Arm kinematic chain (6 revolute joints):
  Joint 1 (Revolute-45): Base rotation, axis Z
  Joint 2 (Revolute-49): Shoulder, axis X
  Joint 3 (Revolute-51): Elbow, axis X
  Joint 4 (Revolute-53): Wrist pitch, axis X
  Joint 5 (Revolute-55): Wrist roll, axis [0, 0.4226, -0.9063] (angled ~25 deg)
  Joint 6 (Revolute-57): Gripper, axis [0, -0.9063, -0.4226] (angled ~25 deg)
"""

import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass
class JointConfig:
    """Single joint configuration"""
    name: str
    lower_limit: float
    upper_limit: float
    velocity_limit: float
    effort_limit: float


class NanoConfig:
    """ADORA1 Nano SO_ARM100 6-DOF Arm Configuration"""

    NUM_JOINTS = 6
    ARM_QPOS_START = 3  # arm joints start at qpos[3] (after 3 wheel joints)

    JOINT_CONFIGS = [
        JointConfig("base_rotation", -2.618, 2.618, 5.0, 3.0),
        JointConfig("shoulder", -2.618, 2.618, 5.0, 3.0),
        JointConfig("elbow", -2.618, 2.618, 5.0, 3.0),
        JointConfig("wrist_pitch", -2.618, 2.618, 5.0, 3.0),
        JointConfig("wrist_roll", -2.618, 2.618, 5.0, 3.0),
        JointConfig("gripper", -1.57, 1.57, 5.0, 3.0),
    ]

    JOINT_LOWER_LIMITS = np.array([j.lower_limit for j in JOINT_CONFIGS])
    JOINT_UPPER_LIMITS = np.array([j.upper_limit for j in JOINT_CONFIGS])
    JOINT_VELOCITY_LIMITS = np.array([j.velocity_limit for j in JOINT_CONFIGS])
    JOINT_EFFORT_LIMITS = np.array([j.effort_limit for j in JOINT_CONFIGS])

    # Link transforms from URDF (revolute joint origins with rotation axes).
    # Each entry corresponds to one revolute joint in the arm chain.
    LINK_TRANSFORMS = [
        # Joint 1 (Revolute-45): Base rotation
        {"xyz": [-0.01025, 0.0346, -0.0328], "rpy": [1.5708, 0, 0], "axis": [0, 0, 1]},
        # Joint 2 (Revolute-49): Shoulder
        {"xyz": [0.0346, 0.01025, -0.0328], "rpy": [3.1416, 0, 0], "axis": [1, 0, 0]},
        # Joint 3 (Revolute-51): Elbow
        {"xyz": [0.0346, -0.01307, 0.03178], "rpy": [0, 0, 0], "axis": [1, 0, 0]},
        # Joint 4 (Revolute-53): Wrist pitch
        {"xyz": [0.0346, -0.03408, 0.00440], "rpy": [3.1416, 0, 0], "axis": [1, 0, 0]},
        # Joint 5 (Revolute-55): Wrist roll — angled axis (~25 deg from Y toward -Z)
        {"xyz": [-0.029, 0.00811, 0.00687], "rpy": [3.1416, 0, 0], "axis": [0, 0.4226, -0.9063]},
        # Joint 6 (Revolute-57): Gripper — angled axis (~25 deg from -Y toward -Z)
        {"xyz": [-0.0328, -0.02391, 0.02703], "rpy": [-1.5708, 0, 0], "axis": [0, -0.9063, -0.4226]},
    ]

    # EE offset from last joint (gripper jaw) to fingertip
    # Estimated from Moving_Jaw_08d geometry: ~3cm along gripper axis
    EE_OFFSET = np.array([0.0, 0.0, -0.03])

    # Collision geometry (simplified spheres for each arm link)
    COLLISION_GEOMETRY = [
        ("sphere", [0.04]),   # base rotation link
        ("sphere", [0.04]),   # shoulder link
        ("sphere", [0.035]),  # elbow link
        ("sphere", [0.035]),  # wrist pitch link
        ("sphere", [0.03]),   # wrist roll link
        ("sphere", [0.025]),  # gripper link
    ]

    COLLISION_MARGIN = 0.015
    MAX_ACCELERATION = np.array([8.0, 8.0, 8.0, 8.0, 8.0, 8.0])

    HOME_CONFIG = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    SAFE_CONFIG = np.array([0.0, -0.3, 0.3, 0.0, 0.0, 0.0])

    NAMED_POSES = {
        "home": HOME_CONFIG,
        "safe": SAFE_CONFIG,
        "zero": np.zeros(6),
    }

    @staticmethod
    def get_joint_limits() -> Tuple[np.ndarray, np.ndarray]:
        return NanoConfig.JOINT_LOWER_LIMITS, NanoConfig.JOINT_UPPER_LIMITS

    @staticmethod
    def get_velocity_limits() -> np.ndarray:
        return NanoConfig.JOINT_VELOCITY_LIMITS

    @staticmethod
    def is_config_valid(q: np.ndarray) -> bool:
        return bool(
            np.all(q >= NanoConfig.JOINT_LOWER_LIMITS)
            and np.all(q <= NanoConfig.JOINT_UPPER_LIMITS)
        )

    @staticmethod
    def clip_to_limits(q: np.ndarray) -> np.ndarray:
        return np.clip(q, NanoConfig.JOINT_LOWER_LIMITS, NanoConfig.JOINT_UPPER_LIMITS)
