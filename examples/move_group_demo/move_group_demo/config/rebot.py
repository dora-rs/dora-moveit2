#!/usr/bin/env python3
"""
reBotArm B601-DM Robot Configuration

6-DOF arm (Damiao motors) + a single-actuator parallel gripper, modeled in
models/rebot_pickplace.xml (assembled from reBotArm_develop_hjx's
reBot-DevArm_gripper.xml). Mirrors the UR5eConfig interface so the same
dora-moveit2 nodes + octos skill demo run unchanged.

NOTE ON FK: the octos pick-and-place demo plans in JOINT space and solves grasp
configs with an on-demand MuJoCo IK against the `pinch` site (see
octos-dora-bridge/examples/arm_skills.py). It does NOT use the analytic FK built
from LINK_TRANSFORMS below, so those transforms are a best-effort derivation from
the MJCF body tree and are NOT validated for Cartesian move_to_pose / ik_request.
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


class RebotConfig:
    """reBotArm B601-DM 6-DOF arm configuration (MuJoCo frame)."""

    NUM_JOINTS = 6

    # The pick-and-place scene (models/rebot_pickplace.xml) declares the free
    # red_box BEFORE the arm, so the arm's 6 joints live at qpos[7:13]
    # (box freejoint = qpos[0:7], gripper fingers = qpos[13:15]). This matches
    # the UR5e demo layout so the skill/ball_state code transfers unchanged.
    ARM_QPOS_START = 7

    # Joint limits from reBot-DevArm_gripper.xml joint ranges + actuatorfrcrange.
    JOINT_CONFIGS = [
        JointConfig("joint1", -2.80, 2.80, 3.0, 27.0),
        JointConfig("joint2", -3.14, 0.00, 3.0, 27.0),
        JointConfig("joint3", -3.14, 0.00, 3.0, 27.0),
        JointConfig("joint4", -1.87, 1.57, 3.0, 7.0),
        JointConfig("joint5", -1.57, 1.57, 3.0, 7.0),
        JointConfig("joint6", -3.14, 3.14, 3.0, 7.0),
    ]

    JOINT_LOWER_LIMITS = np.array([j.lower_limit for j in JOINT_CONFIGS])
    JOINT_UPPER_LIMITS = np.array([j.upper_limit for j in JOINT_CONFIGS])
    JOINT_VELOCITY_LIMITS = np.array([j.velocity_limit for j in JOINT_CONFIGS])
    JOINT_EFFORT_LIMITS = np.array([j.effort_limit for j in JOINT_CONFIGS])

    # Link transforms derived from the MJCF body tree (child body pos + quat,
    # quat->rpy). Each entry: xyz (translation), rpy (euler), axis (joint axis).
    # Body chain (pos, quat, joint axis):
    #   base_link->link1  pos=[-8.4e-05,0,0.08465]     quat=I            axis=[0,0,1]
    #   link1->link2      pos=[0.020084,0.031625,0.05555] quat=[.707,-.707,0,0] (-90 X) axis=[0,0,-1]
    #   link2->link3      pos=[-0.264,0,0]              quat=I            axis=[0,0,1]
    #   link3->link4      pos=[0.2426,-0.054,-0.001625] quat=I            axis=[0,0,1]
    #   link4->link5      pos=[0.078308,-0.0375,-0.03]  quat=[.707,-.707,0,0] (-90 X) axis=[0,0,1]
    #   link5->link6      pos=[0.028008,0,0.04]         quat=[.707,0,.707,0] (+90 Y) axis=[0,0,1]
    LINK_TRANSFORMS = [
        {"xyz": [-8.416e-05, 0.0, 0.08465], "rpy": [0.0, 0.0, 0.0], "axis": [0, 0, 1]},
        {"xyz": [0.020084, 0.031625, 0.05555], "rpy": [-1.5707963, 0.0, 0.0], "axis": [0, 0, -1]},
        {"xyz": [-0.264, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0], "axis": [0, 0, 1]},
        {"xyz": [0.2426, -0.054, -0.001625], "rpy": [0.0, 0.0, 0.0], "axis": [0, 0, 1]},
        {"xyz": [0.078308, -0.0375, -0.03], "rpy": [-1.5707963, 0.0, 0.0], "axis": [0, 0, 1]},
        {"xyz": [0.028008, 0.0, 0.04], "rpy": [0.0, 1.5707963, 0.0], "axis": [0, 0, 1]},
    ]

    # EE offset (wrist link6 frame -> gripper grasp center). The `pinch` site sits
    # at link6 +z ~0.165 m; this mirrors that for any analytic FK consumers.
    EE_OFFSET = np.array([0.0, 0.0, 0.165])

    # Simplified collision spheres per link.
    COLLISION_GEOMETRY = [
        ("sphere", [0.06]),   # base_link
        ("sphere", [0.05]),   # link1
        ("sphere", [0.05]),   # link2
        ("sphere", [0.04]),   # link3
        ("sphere", [0.035]),  # link4
        ("sphere", [0.03]),   # link5
        ("sphere", [0.03]),   # link6
    ]

    COLLISION_MARGIN = 0.015
    MAX_ACCELERATION = np.array([5.0, 5.0, 5.0, 8.0, 8.0, 8.0])

    # Folded-ish home seed (also the model keyframe). TUNE on epyc so the IK seed
    # reaches the workspace cleanly.
    HOME_CONFIG = np.array([0.0, -1.0, -1.5, 0.0, 0.0, 0.0])

    # Safe / stowed posture.
    SAFE_CONFIG = np.array([0.0, -1.5708, -1.5708, 0.0, 0.0, 0.0])

    NAMED_POSES = {
        "home": HOME_CONFIG,
        "safe": SAFE_CONFIG,
        "zero": np.zeros(6),
        "up": SAFE_CONFIG,
    }

    @staticmethod
    def get_joint_limits() -> Tuple[np.ndarray, np.ndarray]:
        return RebotConfig.JOINT_LOWER_LIMITS, RebotConfig.JOINT_UPPER_LIMITS

    @staticmethod
    def get_velocity_limits() -> np.ndarray:
        return RebotConfig.JOINT_VELOCITY_LIMITS

    @staticmethod
    def is_config_valid(q: np.ndarray) -> bool:
        return (np.all(q >= RebotConfig.JOINT_LOWER_LIMITS) and
                np.all(q <= RebotConfig.JOINT_UPPER_LIMITS))

    @staticmethod
    def clip_to_limits(q: np.ndarray) -> np.ndarray:
        return np.clip(q, RebotConfig.JOINT_LOWER_LIMITS, RebotConfig.JOINT_UPPER_LIMITS)
