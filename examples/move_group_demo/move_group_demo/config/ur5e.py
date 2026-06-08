#!/usr/bin/env python3
"""
UR5e Robot Configuration
Based on MuJoCo Menagerie official UR5e model (google-deepmind/mujoco_menagerie).
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class JointConfig:
    """Single joint configuration"""
    name: str
    lower_limit: float  # rad
    upper_limit: float  # rad
    velocity_limit: float  # rad/s
    effort_limit: float  # Nm


class UR5eConfig:
    """UR5e 6-DOF Robot Arm Configuration (Menagerie frame)"""

    # Number of joints
    NUM_JOINTS = 6

    # The MuJoCo pick-and-place scene (models/ur5e.xml) declares a free-jointed
    # `red_ball` BEFORE the arm, so the arm's 6 joints live at qpos[7:13]
    # (ball freejoint = qpos[0:7], gripper = qpos[13:21]). Without this, the
    # _extract_arm_joints fallback (len(qpos)>=20 -> qpos[13:19]) reads the
    # GRIPPER joints as the arm state, corrupting planning/execution start-state
    # so the arm never tracks commanded targets.
    ARM_QPOS_START = 7

    # Joint limits (from URDF / Menagerie)
    JOINT_CONFIGS = [
        JointConfig("shoulder_pan_joint",  -6.2832, 6.2832, 3.1416, 150.0),
        JointConfig("shoulder_lift_joint", -6.2832, 6.2832, 3.1416, 150.0),
        JointConfig("elbow_joint",         -3.1416, 3.1416, 3.1416, 150.0),
        JointConfig("wrist_1_joint",       -6.2832, 6.2832, 3.1416, 28.0),
        JointConfig("wrist_2_joint",       -6.2832, 6.2832, 3.1416, 28.0),
        JointConfig("wrist_3_joint",       -6.2832, 6.2832, 3.1416, 28.0),
    ]

    # Extract arrays for easy access
    JOINT_LOWER_LIMITS = np.array([j.lower_limit for j in JOINT_CONFIGS])
    JOINT_UPPER_LIMITS = np.array([j.upper_limit for j in JOINT_CONFIGS])
    JOINT_VELOCITY_LIMITS = np.array([j.velocity_limit for j in JOINT_CONFIGS])
    JOINT_EFFORT_LIMITS = np.array([j.effort_limit for j in JOINT_CONFIGS])

    # Link transforms matching the MuJoCo Menagerie UR5e model.
    # Each entry: xyz (translation), rpy (euler angles), axis (joint rotation axis).
    #
    # Menagerie body structure:
    #   base (quat 0 0 0 -1 = 180° about Z)
    #     shoulder_link  pos=[0, 0, 0.163]      joint axis=[0,0,1]
    #       upper_arm    pos=[0, 0.138, 0]       quat=[1,0,1,0] (90° about Y)  axis=[0,1,0]
    #         forearm    pos=[0, -0.131, 0.425]  joint axis=[0,1,0]
    #           wrist_1  pos=[0, 0, 0.392]       quat=[1,0,1,0] (90° about Y)  axis=[0,1,0]
    #             wrist_2  pos=[0, 0.127, 0]     joint axis=[0,0,1]
    #               wrist_3  pos=[0, 0, 0.1]     joint axis=[0,1,0]
    LINK_TRANSFORMS = [
        # shoulder_pan: base → shoulder  (base has 180° Z pre-rotation)
        {"xyz": [0, 0, 0.163], "rpy": [0, 0, 3.14159265], "axis": [0, 0, 1]},
        # shoulder_lift: shoulder → upper_arm (quat 1 0 1 0 = 90° about Y)
        {"xyz": [0, 0.138, 0], "rpy": [0, 1.5707963, 0], "axis": [0, 1, 0]},
        # elbow: upper_arm → forearm
        {"xyz": [0, -0.131, 0.425], "rpy": [0, 0, 0], "axis": [0, 1, 0]},
        # wrist_1: forearm → wrist_1 (quat 1 0 1 0 = 90° about Y)
        {"xyz": [0, 0, 0.392], "rpy": [0, 1.5707963, 0], "axis": [0, 1, 0]},
        # wrist_2: wrist_1 → wrist_2
        {"xyz": [0, 0.127, 0], "rpy": [0, 0, 0], "axis": [0, 0, 1]},
        # wrist_3: wrist_2 → wrist_3
        {"xyz": [0, 0, 0.1], "rpy": [0, 0, 0], "axis": [0, 1, 0]},
    ]

    # EE offset in wrist_3 frame to gripper fingertips:
    #   attachment_site [0, 0.1, 0] + gripper length rotated into wrist_3 frame [0, 0.1558, 0]
    #   gripper length = base_mount(0.007) + gripper_base(0.0038) + ee_site(0.145) = 0.1558m
    EE_OFFSET = np.array([0, 0.2558, 0])

    # Collision geometry (simplified spheres for each link)
    COLLISION_GEOMETRY = [
        ("sphere", [0.065]),  # base_link
        ("sphere", [0.06]),   # shoulder_link
        ("sphere", [0.05]),   # upper_arm_link
        ("sphere", [0.038]),  # forearm_link
        ("sphere", [0.035]),  # wrist_1_link
        ("sphere", [0.035]),  # wrist_2_link
        ("sphere", [0.03]),   # wrist_3_link
    ]

    # Safety parameters
    COLLISION_MARGIN = 0.015
    MAX_ACCELERATION = np.array([5.0, 5.0, 5.0, 8.0, 8.0, 8.0])

    # Default home configuration (matches Menagerie keyframe)
    HOME_CONFIG = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

    # Safe configuration (arm pointing straight up)
    SAFE_CONFIG = np.array([0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0])

    # Named poses for MoveGroup API
    NAMED_POSES = {
        "home": HOME_CONFIG,
        "safe": SAFE_CONFIG,
        "zero": np.zeros(6),
        "up": np.array([0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]),
    }

    @staticmethod
    def get_joint_limits() -> Tuple[np.ndarray, np.ndarray]:
        """Get joint position limits"""
        return UR5eConfig.JOINT_LOWER_LIMITS, UR5eConfig.JOINT_UPPER_LIMITS

    @staticmethod
    def get_velocity_limits() -> np.ndarray:
        """Get joint velocity limits"""
        return UR5eConfig.JOINT_VELOCITY_LIMITS

    @staticmethod
    def is_config_valid(q: np.ndarray) -> bool:
        """Check if configuration is within joint limits"""
        return (np.all(q >= UR5eConfig.JOINT_LOWER_LIMITS) and
                np.all(q <= UR5eConfig.JOINT_UPPER_LIMITS))

    @staticmethod
    def clip_to_limits(q: np.ndarray) -> np.ndarray:
        """Clip configuration to joint limits"""
        return np.clip(q, UR5eConfig.JOINT_LOWER_LIMITS, UR5eConfig.JOINT_UPPER_LIMITS)
