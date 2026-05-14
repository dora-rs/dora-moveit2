#!/usr/bin/env python3
"""
GEN72 Real Robot Control Node
==============================
Controls physical GEN72 robot arm via Realman SDK.
"""

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


class GEN72RobotNode:
    """Real GEN72 robot control node"""

    def __init__(self):
        self.robot = None
        self.handle = None
        self.num_joints = 7
        self.current_joints: Optional[np.ndarray] = None

        # Trajectory execution state
        self._traj_lock = threading.Lock()
        self._traj_thread: Optional[threading.Thread] = None
        self.is_executing = False
        self.execution_count = 0
        self.pending_trajectory: Optional[list] = None  # list of np.ndarray waypoints

        self.connect()

    def connect(self):
        if RoboticArm is None:
            print("ERROR: Realman SDK not available")
            return False
        try:
            thread_mode = rm_thread_mode_e(2)
            self.robot = RoboticArm(thread_mode)
            self.handle = self.robot.rm_create_robot_arm("192.168.1.19", 8080, 3)
            if self.handle.id == -1:
                print("Failed to connect to GEN72")
                return False
            print(f"Connected to GEN72: handle {self.handle.id}")
            self.robot.rm_set_arm_power(1)
            time.sleep(0.5)
            self.update_joint_state()
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False

    def update_joint_state(self):
        if self.robot is None:
            return
        try:
            result = self.robot.rm_get_current_arm_state()
            if isinstance(result, tuple) and len(result) > 0:
                joint_data = result[1]
                if hasattr(joint_data, 'joint'):
                    self.current_joints = np.deg2rad(np.array(joint_data.joint[:7], dtype=np.float32))
        except Exception as e:
            print(f"Error reading joints: {e}")

    def set_trajectory(self, waypoints: list):
        """Accept a new trajectory (list of np.ndarray joint configs)."""
        with self._traj_lock:
            if self.is_executing:
                # Stop current motion before accepting new trajectory
                try:
                    self.robot.rm_set_arm_stop()
                except Exception:
                    pass
            self.pending_trajectory = waypoints

        # Start execution thread
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

        print(f"[Robot] Executing trajectory #{count} with {len(waypoints)} waypoints")
        try:
            for i, wp in enumerate(waypoints):
                joint_deg = np.rad2deg(wp[:self.num_joints]).tolist()
                # block=1: wait for each waypoint to complete before sending next
                self.robot.rm_movej(joint_deg, 15, 0, 0, 1)
                print(f"[Robot] Waypoint {i+1}/{len(waypoints)} reached")
        except Exception as e:
            print(f"[Robot] Trajectory execution error: {e}")
        finally:
            with self._traj_lock:
                self.is_executing = False
            print(f"[Robot] Trajectory #{count} complete!")

    def get_status(self) -> dict:
        return {
            "is_executing": self.is_executing,
            "execution_count": self.execution_count,
        }

    def disconnect(self):
        if self.robot:
            self.robot.rm_delete_robot_arm()
            print("Disconnected from GEN72")


def main():
    print("=== GEN72 Real Robot Node ===")

    node = Node()
    robot_node = GEN72RobotNode()

    if robot_node.robot is None:
        print("Failed to initialize robot. Exiting.")
        return

    try:
        for event in node:
            if event["type"] == "INPUT":
                input_id = event["id"]

                if input_id == "control_input":
                    # Receive full trajectory from trajectory_executor
                    traj_flat = event["value"].to_numpy()
                    metadata = event.get("metadata", {})
                    num_joints = metadata.get("num_joints", robot_node.num_joints)
                    num_waypoints = metadata.get("num_waypoints", len(traj_flat) // num_joints)
                    if num_waypoints > 0 and len(traj_flat) >= num_waypoints * num_joints:
                        traj = traj_flat.reshape(num_waypoints, num_joints)
                        waypoints = [traj[i] for i in range(num_waypoints)]
                        robot_node.set_trajectory(waypoints)

                elif input_id == "tick":
                    robot_node.update_joint_state()

                    if robot_node.current_joints is not None:
                        node.send_output(
                            "joint_positions",
                            pa.array(robot_node.current_joints, type=pa.float32()),
                            {"timestamp": time.time(), "encoding": "jointstate"}
                        )
                        node.send_output(
                            "joint_velocities",
                            pa.array(np.zeros(7), type=pa.float32()),
                            {"timestamp": time.time()}
                        )

                    # Publish execution status so trajectory_executor and capture stay in sync
                    status = robot_node.get_status()
                    status_bytes = json.dumps(status).encode("utf-8")
                    node.send_output(
                        "execution_status",
                        pa.array(list(status_bytes), type=pa.uint8()),
                        {"timestamp": time.time()}
                    )

            elif event["type"] == "STOP":
                print("Stopping robot node...")
                break

    finally:
        robot_node.disconnect()


if __name__ == "__main__":
    main()
