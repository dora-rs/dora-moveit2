#!/usr/bin/env python3
"""
Dual GEN72 Real Robot Node
===========================
Controls two physical GEN72 arms via Realman SDK.
Receives full 14D trajectories from planner and executes them directly
(no intermediate trajectory_executor interpolation).

Based on the proven single-arm pattern in:
  examples/hunter_with_arm/hunter_arm_demo/robot_control/gen72_robot_node.py
"""

import os
import json
import threading
import numpy as np
import pyarrow as pa
from dora import Node
import time
from typing import Optional

try:
    from hunter_arm_demo.robot_control.rm_robot_interface import *
except ImportError:
    try:
        from Robotic_Arm.rm_robot_interface import *
    except ImportError:
        print("ERROR: Realman SDK not found")
        RoboticArm = None


class DualGEN72RobotNode:
    """Real dual GEN72 robot control node"""

    def __init__(self, left_ip: str, right_ip: str):
        self.left_ip = left_ip
        self.right_ip = right_ip
        self.num_joints = 7  # per arm

        self.left_arm = None
        self.right_arm = None
        self.left_handle = None
        self.right_handle = None

        self.current_joints_14d: Optional[np.ndarray] = None

        self._traj_lock = threading.Lock()
        self._traj_thread: Optional[threading.Thread] = None
        self.is_executing = False
        self.execution_count = 0

        self.connect()

    def connect(self):
        if RoboticArm is None:
            print("ERROR: Realman SDK not available")
            return False

        thread_mode = rm_thread_mode_e(2)

        try:
            self.left_arm = RoboticArm(thread_mode)
            self.left_handle = self.left_arm.rm_create_robot_arm(self.left_ip, 8080, 3)
            if self.left_handle.id == -1:
                print(f"Failed to connect to left arm at {self.left_ip}")
                return False
            print(f"[DualRobot] Left arm connected: handle {self.left_handle.id}")
            self.left_arm.rm_set_arm_power(1)
        except Exception as e:
            print(f"[DualRobot] Left arm connection error: {e}")
            return False

        try:
            self.right_arm = RoboticArm(thread_mode)
            self.right_handle = self.right_arm.rm_create_robot_arm(self.right_ip, 8080, 3)
            if self.right_handle.id == -1:
                print(f"Failed to connect to right arm at {self.right_ip}")
                return False
            print(f"[DualRobot] Right arm connected: handle {self.right_handle.id}")
            self.right_arm.rm_set_arm_power(1)
        except Exception as e:
            print(f"[DualRobot] Right arm connection error: {e}")
            return False

        time.sleep(0.5)
        self.update_joint_state()
        return True

    def update_joint_state(self):
        left_joints = self._read_arm_joints(self.left_arm, "left")
        right_joints = self._read_arm_joints(self.right_arm, "right")
        self.current_joints_14d = np.concatenate([left_joints, right_joints]).astype(np.float32)

    def _read_arm_joints(self, arm, side: str) -> np.ndarray:
        if arm is None:
            return np.zeros(self.num_joints, dtype=np.float32)
        try:
            result = arm.rm_get_current_arm_state()
            if isinstance(result, tuple) and len(result) > 0:
                joint_data = result[1]
                if isinstance(joint_data, dict) and 'joint' in joint_data:
                    return np.deg2rad(np.array(joint_data['joint'][:7], dtype=np.float32))
                elif hasattr(joint_data, 'joint'):
                    return np.deg2rad(np.array(joint_data.joint[:7], dtype=np.float32))
        except Exception as e:
            print(f"[DualRobot] Error reading {side} arm: {e}")
        return np.zeros(self.num_joints, dtype=np.float32)

    def set_trajectory(self, waypoints: list):
        """Accept a full 14D trajectory and execute it."""
        with self._traj_lock:
            if self.is_executing:
                try:
                    self.left_arm.rm_set_arm_stop()
                    self.right_arm.rm_set_arm_stop()
                except Exception:
                    pass

        t = threading.Thread(target=self._execute_trajectory, args=(waypoints,), daemon=True)
        t.start()
        self._traj_thread = t

    def _execute_trajectory(self, waypoints: list):
        with self._traj_lock:
            self.is_executing = True
            self.execution_count += 1
            count = self.execution_count

        print(f"[DualRobot] Executing trajectory #{count} with {len(waypoints)} waypoints")
        try:
            for i, wp in enumerate(waypoints):
                left_deg = np.rad2deg(wp[:7]).tolist()
                right_deg = np.rad2deg(wp[7:14]).tolist()
                # Send both arms simultaneously with block=1 (wait for completion)
                left_thread = threading.Thread(
                    target=self.left_arm.rm_movej,
                    args=(left_deg, 15, 0, 0, 1),
                    daemon=True,
                )
                right_thread = threading.Thread(
                    target=self.right_arm.rm_movej,
                    args=(right_deg, 15, 0, 0, 1),
                    daemon=True,
                )
                left_thread.start()
                right_thread.start()
                left_thread.join()
                right_thread.join()
                print(f"[DualRobot] Waypoint {i+1}/{len(waypoints)} reached")
        except Exception as e:
            print(f"[DualRobot] Trajectory execution error: {e}")
        finally:
            with self._traj_lock:
                self.is_executing = False
            print(f"[DualRobot] Trajectory #{count} complete!")

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
                    print(f"[DualRobot] {side} arm disconnected")
                except Exception:
                    pass


def main():
    print("=== Dual GEN72 Real Robot Node ===")

    left_ip = os.environ.get("GEN72_LEFT_IP")
    right_ip = os.environ.get("GEN72_RIGHT_IP")
    if not (left_ip and right_ip):
        raise RuntimeError(
            "GEN72_LEFT_IP and GEN72_RIGHT_IP must be set in the dataflow env block."
        )

    node = Node()
    robot = DualGEN72RobotNode(left_ip, right_ip)

    if robot.left_arm is None or robot.right_arm is None:
        print("Failed to initialize robot. Exiting.")
        return

    try:
        for event in node:
            if event["type"] == "INPUT":
                input_id = event["id"]

                if input_id == "control_input":
                    traj_flat = event["value"].to_numpy().astype(np.float32)
                    metadata = event.get("metadata", {})
                    num_joints = int(metadata.get("num_joints", 14))
                    num_waypoints = int(metadata.get("num_waypoints", len(traj_flat) // num_joints))
                    if num_waypoints > 0 and len(traj_flat) >= num_waypoints * num_joints:
                        traj = traj_flat.reshape(num_waypoints, num_joints)
                        waypoints = [traj[i] for i in range(num_waypoints)]
                        robot.set_trajectory(waypoints)

                elif input_id == "tick":
                    robot.update_joint_state()

                    if robot.current_joints_14d is not None:
                        node.send_output(
                            "joint_positions",
                            pa.array(robot.current_joints_14d, type=pa.float32()),
                            {"timestamp": time.time(), "encoding": "jointstate"},
                        )

                    status = robot.get_status()
                    status_bytes = json.dumps(status).encode("utf-8")
                    node.send_output(
                        "execution_status",
                        pa.array(list(status_bytes), type=pa.uint8()),
                        {"timestamp": time.time()},
                    )

            elif event["type"] == "STOP":
                print("[DualRobot] Stopping...")
                break
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
