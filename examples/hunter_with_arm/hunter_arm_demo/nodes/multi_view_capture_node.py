#!/usr/bin/env python3
"""
Multi-View Capture Node for GEN72 on Hunter SE
================================================
Explicit joint-waypoint choreography:
  HOME → TURN_RIGHT → ARCH_RIGHT (capture) → TURN_RIGHT → HOME
       → TURN_LEFT  → ARCH_LEFT  (capture) → TURN_LEFT  → HOME
"""

import os
import json
import time
import numpy as np
import pyarrow as pa
from dora import Node
from hunter_arm_demo.config.gen72 import GEN72Config


class MultiViewCaptureNode:
    """Multi-view capture workflow controller using explicit joint choreography.

    Motion sequence:
      HOME → TURN (j1 rotates ±90°) → ARCH (over pipe, EE down) → capture
      → retract to TURN → return to HOME → repeat other side
    """

    # Joint configs:  [j1,    j2,   j3,    j4,    j5,  j6,   j7]
    HOME       = np.array([ 0.0,  -0.5, 0.0,  0.0,   0.0,  0.5, 0.0])
    TURN_RIGHT = np.array([-1.57, -0.5, 0.0,  0.0,   0.0,  0.5, 0.0])
    ARCH_RIGHT = np.array([-1.57,  0.26, 0.0, -1.65,  0.0,  0.68, 0.0])
    TURN_LEFT  = np.array([ 1.57, -0.5, 0.0,  0.0,   0.0,  0.5, 0.0])
    ARCH_LEFT  = np.array([ 1.57,  0.26, 0.0, -1.65,  0.0,  0.68, 0.0])

    def __init__(self):
        # Full motion plan: list of (name, goal_joints, capture_here?)
        self.steps = [
            ("turn_right",      self.TURN_RIGHT, False),
            ("arch_over_right", self.ARCH_RIGHT,  True),   # capture here
            ("retract_right",   self.TURN_RIGHT, False),
            ("home_from_right", self.HOME,       False),
        ]
        self.step_idx = 0
        self.capture_count = 0

        self.current_joints = self.HOME.copy()
        self.waiting_for_execution = False
        self.waiting_for_planning = False
        self.expected_execution_count = 0
        self.waiting_for_joint_update = False
        self.joint_update_wait_ticks = 0
        self.joint_update_max_ticks = 5
        self.vehicle_ready = False
        self.workflow_done = False

        # Camera
        self.camera_index = int(os.getenv("CAPTURE_CAMERA_INDEX", "0"))
        self.output_dir = os.getenv("CAPTURE_OUTPUT_DIR", "captures")
        os.makedirs(self.output_dir, exist_ok=True)

        self.cap = None
        try:
            import cv2 as cv
            self.cap = cv.VideoCapture(self.camera_index)
            if self.cap.isOpened():
                print(f"Camera {self.camera_index} initialized")
                for _ in range(5):
                    self.cap.read()
            else:
                print(f"Warning: Failed to open camera {self.camera_index}")
                self.cap.release()
                self.cap = None
        except ImportError:
            print("Warning: OpenCV not installed, camera capture disabled")

        print("=== Multi-View Capture Node (Choreographed) ===")
        print(f"Steps: {len(self.steps)}")

    def run(self):
        node = Node()
        self._send_robot_state(node, self.current_joints)
        time.sleep(0.5)
        print("Waiting for vehicle to complete movement...")

        for event in node:
            if event["type"] == "INPUT":
                self._handle_input(node, event)
            elif event["type"] == "STOP":
                break

        print("\nMulti-view capture workflow complete!")

    # ---------------------- Event Handling ---------------------- #

    def _handle_input(self, node: Node, event):
        event_id = event["id"]

        if event_id == "vehicle_ready":
            if not self.vehicle_ready:
                self.vehicle_ready = True
                print("Vehicle stopped. Starting arm workflow...")
                time.sleep(0.5)
                self._execute_next_step(node)

        elif event_id == "joint_positions":
            try:
                joints = event["value"].to_numpy()
                # Extract arm joints: skip freejoint(7) + steering(2) + wheels(4) = 13
                if len(joints) >= 20:
                    self.current_joints = joints[13:20].copy()
                else:
                    self.current_joints = joints[:7].copy()

                if self.waiting_for_joint_update:
                    self.joint_update_wait_ticks += 1
                    if self.joint_update_wait_ticks >= self.joint_update_max_ticks:
                        self.waiting_for_joint_update = False
                        self.joint_update_wait_ticks = 0
                        self._execute_next_step(node)
            except Exception:
                pass

        elif event_id == "execution_status":
            self._handle_execution_status(node, event["value"])

        elif event_id == "trajectory":
            self._handle_trajectory(node, event)

        elif event_id == "plan_status":
            self._handle_plan_status(node, event["value"])

    def _handle_plan_status(self, node: Node, data):
        try:
            if hasattr(data, "to_pylist"):
                status_bytes = bytes(data.to_pylist())
            else:
                status_bytes = bytes(data)
            status = json.loads(status_bytes.decode("utf-8"))
        except Exception as e:
            print(f"[Capture] Error decoding plan_status: {e}")
            return
        if not status.get("success", False):
            msg = status.get("message", status.get("error", "unknown"))
            print(f"  Planning failed: {msg}")
        else:
            print("  Planning succeeded")

    def _handle_execution_status(self, node: Node, data):
        if not self.waiting_for_execution:
            return
        try:
            status_bytes = bytes(data.to_numpy())
            status = json.loads(status_bytes.decode('utf-8'))
            exec_count = status.get("execution_count", 0)
            is_executing = status.get("is_executing", True)

            if exec_count == self.expected_execution_count and not is_executing:
                self.waiting_for_execution = False
                self._on_execution_complete(node)
        except Exception as e:
            print(f"[Capture] Error handling execution status: {e}")

    def _handle_trajectory(self, node: Node, event):
        if self.waiting_for_execution:
            return

        value = event["value"]
        metadata = event.get("metadata", {}) if isinstance(event, dict) else {}

        try:
            if hasattr(value, "to_numpy"):
                traj_flat = value.to_numpy()
            else:
                traj_flat = np.frombuffer(value, dtype=np.float32)
        except Exception as e:
            print(f"[Capture] Error decoding trajectory: {e}")
            return

        num_joints = GEN72Config.NUM_JOINTS
        num_waypoints = metadata.get("num_waypoints", len(traj_flat) // num_joints)
        if num_waypoints <= 0:
            print("[Capture] Invalid trajectory: num_waypoints <= 0")
            return

        try:
            waypoints = traj_flat.reshape(num_waypoints, num_joints)
        except Exception as e:
            print(f"[Capture] Error reshaping trajectory: {e}")
            return

        print(f"  Trajectory received: {len(waypoints)} waypoints")

        self.waiting_for_planning = False
        self.waiting_for_execution = True
        self.expected_execution_count += 1
        print(f"  Waiting for execution #{self.expected_execution_count} to complete...")

    # ---------------------- Choreography ---------------------- #

    def _execute_next_step(self, node: Node):
        """Send plan_request for the next step in the choreography."""
        if self.step_idx >= len(self.steps):
            print("\nAll steps complete! Workflow finished.")
            print("Staying idle. Press Ctrl+C to exit.")
            self.workflow_done = True
            return

        name, goal_joints, _ = self.steps[self.step_idx]
        print(f"\n[Step {self.step_idx + 1}/{len(self.steps)}] {name}")
        print(f"  From: {np.round(self.current_joints, 2)}")
        print(f"  To:   {np.round(goal_joints, 2)}")

        self.waiting_for_planning = True
        self._request_plan(node, self.current_joints, goal_joints)

    def _on_execution_complete(self, node: Node):
        """Called when trajectory execution completes for the current step."""
        name, _, do_capture = self.steps[self.step_idx]
        print(f"  Step '{name}' execution complete!")

        if do_capture:
            self._capture_image(node, name)

        # Advance to next step
        self.step_idx += 1

        # Wait for joint state to settle before planning next move
        print("  Waiting for joint state to update...")
        self.waiting_for_joint_update = True
        self.joint_update_wait_ticks = 0

    # ---------------------- Helpers ---------------------- #

    def _capture_image(self, node: Node, step_name: str):
        self.capture_count += 1
        print(f"\n  Capturing image #{self.capture_count} at step '{step_name}'")

        if self.cap is None:
            print("   Camera not available, skipping capture")
            return

        try:
            import cv2 as cv
            ok, frame = self.cap.read()
            if not ok or frame is None:
                print("   Failed to grab frame from camera")
                return

            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"{step_name}_{timestamp}.png"
            filepath = os.path.join(self.output_dir, filename)

            if cv.imwrite(filepath, frame):
                print("   Image saved:", filepath)
            else:
                print("   Failed to write image file")
        except Exception as e:
            print(f"   Capture error: {e}")

    def _request_plan(self, node: Node, start_joints: np.ndarray, goal_joints: np.ndarray):
        request = {
            "start": np.asarray(start_joints, dtype=float).tolist(),
            "goal": np.asarray(goal_joints, dtype=float).tolist(),
            "planner": "rrt_connect",
            "max_time": 5.0
        }
        request_bytes = json.dumps(request).encode("utf-8")
        node.send_output(
            "plan_request",
            pa.array(list(request_bytes), type=pa.uint8())
        )

    def _send_robot_state(self, node: Node, joints: np.ndarray):
        state = {"joints": np.asarray(joints, dtype=float).tolist()}
        state_bytes = json.dumps(state).encode("utf-8")
        node.send_output(
            "robot_state",
            pa.array(list(state_bytes), type=pa.uint8())
        )


if __name__ == "__main__":
    node = MultiViewCaptureNode()
    node.run()
