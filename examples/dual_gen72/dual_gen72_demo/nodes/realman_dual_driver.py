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
import time
import threading
import numpy as np
import pyarrow as pa
from dora import Node
from typing import Optional

try:
    # Reuse the single-arm Realman SDK wrapper (wildcard brings in
    # RoboticArm, rm_thread_mode_e, and other ctypes bindings).
    from hunter_arm_demo.robot_control.rm_robot_interface import *
except Exception as e:
    # Catches both ImportError (package missing) and errors raised while
    # loading the Realman ctypes wrapper (e.g. missing native .so/.dll).
    raise RuntimeError(
        "Realman SDK wrapper not importable. Install examples/hunter_with_arm/ "
        "and ensure the Realman native library is available, or place "
        f"rm_robot_interface.py on PYTHONPATH. Original error: {type(e).__name__}: {e}"
    )


class DualRealmanDriver:
    def __init__(self, left_ip: str, right_ip: str):
        self.left_ip = left_ip
        self.right_ip = right_ip
        self.left_arm: Optional[RoboticArm] = None
        self.right_arm: Optional[RoboticArm] = None
        self.left_handle = None
        self.right_handle = None

        self._traj_lock = threading.Lock()
        self._traj_thread: Optional[threading.Thread] = None
        self.is_executing = False
        self.execution_count = 0
        self.pending_trajectory: Optional[list] = None

    def connect(self) -> None:
        thread_mode = rm_thread_mode_e(2)

        self.left_arm = RoboticArm(thread_mode)
        self.left_handle = self.left_arm.rm_create_robot_arm(self.left_ip, 8080, 3)
        if self.left_handle.id == -1:
            raise RuntimeError(f"Failed to connect to left arm at {self.left_ip}")
        print(f"[DualDriver] Left arm connected: handle {self.left_handle.id}")
        self.left_arm.rm_set_arm_power(1)

        self.right_arm = RoboticArm(thread_mode)
        self.right_handle = self.right_arm.rm_create_robot_arm(self.right_ip, 8080, 3)
        if self.right_handle.id == -1:
            raise RuntimeError(f"Failed to connect to right arm at {self.right_ip}")
        print(f"[DualDriver] Right arm connected: handle {self.right_handle.id}")
        self.right_arm.rm_set_arm_power(1)

        time.sleep(0.5)

    def read_joint_positions_14d(self) -> np.ndarray:
        left_joints = self._read_arm_joints(self.left_arm, "left")
        right_joints = self._read_arm_joints(self.right_arm, "right")
        return np.concatenate([left_joints, right_joints]).astype(np.float32)

    def _read_arm_joints(self, arm: RoboticArm, side: str) -> np.ndarray:
        try:
            result = arm.rm_get_current_arm_state()
            if isinstance(result, tuple) and len(result) > 0:
                joint_data = result[1]
                if isinstance(joint_data, dict) and 'joint' in joint_data:
                    return np.deg2rad(np.array(joint_data['joint'][:7], dtype=np.float32))
                elif hasattr(joint_data, 'joint'):
                    return np.deg2rad(np.array(joint_data.joint[:7], dtype=np.float32))
        except Exception as e:
            print(f"[DualDriver] Error reading {side} arm joints: {e}")
        return np.zeros(7, dtype=np.float32)

    def send_joint_positions_14d(self, joints_14d: np.ndarray) -> None:
        with self._traj_lock:
            if self.is_executing:
                try:
                    self.left_arm.rm_set_arm_stop()
                    self.right_arm.rm_set_arm_stop()
                except Exception:
                    pass
            # Treat the single 14D waypoint as a one-waypoint trajectory
            self.pending_trajectory = [joints_14d.astype(np.float32)]

        t = threading.Thread(target=self._execute_trajectory, daemon=True)
        t.start()
        self._traj_thread = t

    def set_trajectory(self, waypoints: list) -> None:
        with self._traj_lock:
            if self.is_executing:
                try:
                    self.left_arm.rm_set_arm_stop()
                    self.right_arm.rm_set_arm_stop()
                except Exception:
                    pass
            self.pending_trajectory = waypoints

        t = threading.Thread(target=self._execute_trajectory, daemon=True)
        t.start()
        self._traj_thread = t

    def _execute_trajectory(self):
        with self._traj_lock:
            waypoints = self.pending_trajectory
            self.pending_trajectory = None
            self.is_executing = True
            self.execution_count += 1
            count = self.execution_count

        print(f"[DualDriver] Executing trajectory #{count} with {len(waypoints)} waypoints")
        try:
            for i, wp in enumerate(waypoints):
                left_deg = np.rad2deg(wp[:7]).tolist()
                right_deg = np.rad2deg(wp[7:14]).tolist()
                # Send both arms simultaneously, block on each
                self.left_arm.rm_movej(left_deg, 15, 0, 0, 1)
                self.right_arm.rm_movej(right_deg, 15, 0, 0, 1)
                print(f"[DualDriver] Waypoint {i+1}/{len(waypoints)} reached")
        except Exception as e:
            print(f"[DualDriver] Trajectory execution error: {e}")
        finally:
            with self._traj_lock:
                self.is_executing = False
            print(f"[DualDriver] Trajectory #{count} complete")

    def get_status(self) -> dict:
        return {
            "is_executing": self.is_executing,
            "execution_count": self.execution_count,
        }

    def disconnect(self):
        for arm, side in [(self.left_arm, "left"), (self.right_arm, "right")]:
            if arm is not None:
                try:
                    arm.rm_delete_robot_arm()
                    print(f"[DualDriver] {side} arm disconnected")
                except Exception:
                    pass


def main():
    import json

    left_ip = os.environ.get("GEN72_LEFT_IP")
    right_ip = os.environ.get("GEN72_RIGHT_IP")
    if not (left_ip and right_ip):
        raise RuntimeError(
            "GEN72_LEFT_IP and GEN72_RIGHT_IP must both be set in the dataflow YAML env block."
        )

    driver = DualRealmanDriver(left_ip, right_ip)
    driver.connect()

    node = Node()
    try:
        for event in node:
            if event["type"] == "STOP":
                break
            if event["type"] != "INPUT":
                continue

            if event["id"] == "tick":
                positions = driver.read_joint_positions_14d()
                node.send_output(
                    "joint_positions",
                    pa.array(positions, type=pa.float32()),
                    {"timestamp": time.time()},
                )
                status_bytes = json.dumps(driver.get_status()).encode("utf-8")
                node.send_output(
                    "execution_status",
                    pa.array(list(status_bytes), type=pa.uint8()),
                    {"timestamp": time.time()},
                )

            elif event["id"] == "control_input":
                traj_flat = event["value"].to_numpy().astype(np.float32)
                metadata = event.get("metadata", {})
                num_joints = int(metadata.get("num_joints", 14))
                num_waypoints = int(metadata.get("num_waypoints", len(traj_flat) // num_joints))
                if num_waypoints > 0 and len(traj_flat) >= num_waypoints * num_joints:
                    traj = traj_flat.reshape(num_waypoints, num_joints)
                    driver.set_trajectory([traj[i] for i in range(num_waypoints)])
    finally:
        driver.disconnect()


if __name__ == "__main__":
    main()
