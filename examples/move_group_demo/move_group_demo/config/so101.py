#!/usr/bin/env python3
"""
SO-101 (LeRobot / TheRobotStudio SO-ARM101) Robot Configuration.

A low-cost **5-DOF arm + gripper** driven by Feetech STS3215 servos. This is the
config-tier proof that the octos/dora-moveit architecture handles a non-6-DOF arm:
NUM_JOINTS = 5. The dora-moveit MoveGroup framework is NUM_JOINTS-driven, and the
octos skill IK is now DOF-generic (a 5-DOF arm uses down-only grasp IK, since it
cannot hold a full 6-DOF orientation).

MODEL: use the official MuJoCo scene from TheRobotStudio/SO-ARM100 (Simulation/SO101,
so101_new_calib.xml), adapted object-first for the pick-and-place demo (the free
object owns qpos[0:7] so the arm sits at qpos[7:12]) — same convention as ur5e/rebot.

NOTE: joint limits + LINK_TRANSFORMS below are reasonable placeholders; sync exact
values from the SO101 MJCF before Cartesian/analytic use. The demo path (joint-space
moves + on-demand MuJoCo IK against the `pinch` site) does not depend on
LINK_TRANSFORMS.
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class JointConfig:
    name: str
    lower_limit: float
    upper_limit: float
    velocity_limit: float
    effort_limit: float


class SO101Config:
    """SO-101 5-DOF arm configuration (sim convention)."""

    NUM_JOINTS = 5

    # object freejoint = qpos[0:7], arm = qpos[7:12], gripper = qpos[12:]
    ARM_QPOS_START = 7

    # 5 arm joints (STS3215). Ranges are placeholders — sync from the SO101 MJCF.
    JOINT_CONFIGS = [
        JointConfig("shoulder_pan",  -2.0, 2.0, 3.0, 10.0),
        JointConfig("shoulder_lift", -1.75, 1.75, 3.0, 10.0),
        JointConfig("elbow_flex",    -1.69, 1.69, 3.0, 10.0),
        JointConfig("wrist_flex",    -1.66, 1.66, 3.0, 10.0),
        JointConfig("wrist_roll",    -2.79, 2.79, 3.0, 10.0),
    ]

    JOINT_LOWER_LIMITS = np.array([j.lower_limit for j in JOINT_CONFIGS])
    JOINT_UPPER_LIMITS = np.array([j.upper_limit for j in JOINT_CONFIGS])
    JOINT_VELOCITY_LIMITS = np.array([j.velocity_limit for j in JOINT_CONFIGS])
    JOINT_EFFORT_LIMITS = np.array([j.effort_limit for j in JOINT_CONFIGS])

    # Placeholder link transforms (5). Demo uses joint-space + MuJoCo IK, not this.
    LINK_TRANSFORMS = [
        {"xyz": [0.0, 0.0, 0.054], "rpy": [0.0, 0.0, 0.0], "axis": [0, 0, 1]},
        {"xyz": [0.0, 0.0, 0.028], "rpy": [0.0, 1.5708, 0.0], "axis": [0, 1, 0]},
        {"xyz": [0.0, 0.0, 0.11], "rpy": [0.0, 0.0, 0.0], "axis": [0, 1, 0]},
        {"xyz": [0.0, 0.0, 0.10], "rpy": [0.0, 0.0, 0.0], "axis": [0, 1, 0]},
        {"xyz": [0.0, 0.0, 0.07], "rpy": [0.0, 0.0, 0.0], "axis": [1, 0, 0]},
    ]

    EE_OFFSET = np.array([0.0, 0.0, 0.09])

    COLLISION_GEOMETRY = [
        ("sphere", [0.05]), ("sphere", [0.045]), ("sphere", [0.04]),
        ("sphere", [0.035]), ("sphere", [0.03]),
    ]

    COLLISION_MARGIN = 0.01
    MAX_ACCELERATION = np.array([5.0, 5.0, 5.0, 8.0, 8.0])

    HOME_CONFIG = np.array([0.0, -1.0, 1.0, 0.0, 0.0])
    SAFE_CONFIG = np.array([0.0, -1.0, 1.0, 0.0, 0.0])

    NAMED_POSES = {
        "home": HOME_CONFIG,
        "safe": SAFE_CONFIG,
        "zero": np.zeros(5),
        "up": np.array([0.0, -1.4, 1.4, 0.0, 0.0]),
    }

    @staticmethod
    def get_joint_limits() -> Tuple[np.ndarray, np.ndarray]:
        return SO101Config.JOINT_LOWER_LIMITS, SO101Config.JOINT_UPPER_LIMITS

    @staticmethod
    def get_velocity_limits() -> np.ndarray:
        return SO101Config.JOINT_VELOCITY_LIMITS

    @staticmethod
    def is_config_valid(q: np.ndarray) -> bool:
        return (np.all(q >= SO101Config.JOINT_LOWER_LIMITS) and
                np.all(q <= SO101Config.JOINT_UPPER_LIMITS))

    @staticmethod
    def clip_to_limits(q: np.ndarray) -> np.ndarray:
        return np.clip(q, SO101Config.JOINT_LOWER_LIMITS, SO101Config.JOINT_UPPER_LIMITS)
