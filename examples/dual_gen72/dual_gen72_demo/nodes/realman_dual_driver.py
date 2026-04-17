#!/usr/bin/env python3
"""
Dual-arm Realman GEN72 driver — HARDWARE TEMPLATE (not runnable in sim).

This is a stub for porting the dual-arm demo to physical hardware. The
working single-arm reference is at:
  examples/hunter_with_arm/hunter_arm_demo/robot_control/gen72_robot_node.py
which uses the Realman SDK wrapper at:
  examples/hunter_with_arm/hunter_arm_demo/robot_control/rm_robot_interface.py

To make this runnable:
  1. Set env vars GEN72_LEFT_IP and GEN72_RIGHT_IP in dual_gen72_real.yml.
  2. Fill in the TODO-marked methods below, mirroring gen72_robot_node.py
     but with one RoboticArm instance per side.
  3. The 14D joint array convention (matches the planner/executor):
       indices 0..6  → left arm (joint1..joint7)
       indices 7..13 → right arm (joint1..joint7)
"""
import os
import numpy as np
import pyarrow as pa
from dora import Node

try:
    # Reuse the single-arm Realman SDK wrapper.
    from hunter_arm_demo.robot_control.rm_robot_interface import RoboticArm
except ImportError as e:
    raise RuntimeError(
        "Realman SDK wrapper not importable. Install examples/hunter_with_arm/ "
        "or place rm_robot_interface.py on PYTHONPATH. Original error: "
        f"{e}"
    )


class DualRealmanDriver:
    def __init__(self, left_ip: str, right_ip: str):
        self.left_ip = left_ip
        self.right_ip = right_ip
        self.left_arm = None
        self.right_arm = None

    def connect(self) -> None:
        # TODO: instantiate RoboticArm(ip) for each side and verify both
        # report ready. See gen72_robot_node.py:connect() for reference.
        raise NotImplementedError("connect() — see module docstring")

    def read_joint_positions_14d(self) -> np.ndarray:
        # TODO: query both arms, concatenate into a 14-element np.float32 array
        # with left first, right second (matches planner/executor).
        raise NotImplementedError("read_joint_positions_14d() — see module docstring")

    def send_joint_positions_14d(self, joints_14d: np.ndarray) -> None:
        # TODO: split joints_14d[:7] → left, joints_14d[7:] → right, then
        # call movej (or equivalent) on each RoboticArm. See
        # gen72_robot_node.py control_input handler for reference.
        raise NotImplementedError("send_joint_positions_14d() — see module docstring")


def main():
    left_ip = os.environ.get("GEN72_LEFT_IP")
    right_ip = os.environ.get("GEN72_RIGHT_IP")
    if not (left_ip and right_ip):
        raise RuntimeError(
            "GEN72_LEFT_IP and GEN72_RIGHT_IP must both be set in the dataflow "
            "YAML env block. This driver is a TEMPLATE — it will not run "
            "without hardware and valid IPs."
        )

    driver = DualRealmanDriver(left_ip, right_ip)
    driver.connect()  # raises NotImplementedError — fill in before running

    node = Node()
    for event in node:
        if event["type"] != "INPUT":
            if event["type"] == "STOP":
                break
            continue

        if event["id"] == "tick":
            positions = driver.read_joint_positions_14d()
            node.send_output("joint_positions", pa.array(positions.astype(np.float32)))
        elif event["id"] == "control_input":
            joints_14d = event["value"].to_numpy().astype(np.float32)
            driver.send_joint_positions_14d(joints_14d)


if __name__ == "__main__":
    main()
