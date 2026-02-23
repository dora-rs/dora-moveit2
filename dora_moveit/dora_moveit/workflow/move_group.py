#!/usr/bin/env python3
"""
MoveGroup: High-level motion planning API for dora-moveit
==========================================================
Provides a synchronous, blocking API similar to ROS MoveIt's
MoveGroupCommander, running on top of the dora dataflow.

All dora Node operations (next/send_output) happen on a single thread
to avoid "Already borrowed" errors from dora-rs.

Usage:
    from dora_moveit.workflow.move_group import MoveGroup

    group = MoveGroup("gen72")
    group.set_named_target("home")
    group.go(wait=True)
    group.go([1.57, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0], wait=True)
    group.shutdown()
"""

import json
import time
import numpy as np
import pyarrow as pa
from dora import Node
from dora_moveit.config import load_config


# --------------- Helpers --------------- #

def _encode_json(data: dict) -> pa.Array:
    """Encode a dict as JSON bytes into a PyArrow uint8 array."""
    raw = json.dumps(data).encode("utf-8")
    return pa.array(list(raw), type=pa.uint8())


def _decode_json(value) -> dict:
    """Decode a dora PyArrow value (uint8 bytes) into a dict."""
    if hasattr(value, "to_pylist"):
        raw = bytes(value.to_pylist())
    elif hasattr(value, "to_numpy"):
        raw = bytes(value.to_numpy())
    else:
        raw = bytes(value)
    return json.loads(raw.decode("utf-8"))


def _extract_arm_joints(joints: np.ndarray, num_joints: int = 7) -> np.ndarray:
    """Extract arm joint positions from full qpos array.
    Handles hunter model (20 qpos, arm at 13:20) and standalone (7-9 qpos)."""
    if len(joints) >= 20:
        return joints[13:20].copy()
    return joints[:num_joints].copy()


# --------------- MoveGroup --------------- #

class MoveGroup:
    """High-level motion planning interface for dora-moveit.

    Wraps a dora Node and provides synchronous, blocking methods
    similar to ROS MoveIt's MoveGroupCommander.

    All dora operations run on a single thread via polling loops
    inside each blocking method.
    """

    def __init__(self, group_name: str = "gen72"):
        self._group_name = group_name
        self._config = load_config()
        self._num_joints = self._config.NUM_JOINTS

        # Planner settings
        self._planner_id = "rrt_connect"
        self._planning_time = 5.0

        # Target state
        self._target_joints = None
        self._target_pose = None

        # Custom named poses (on top of config defaults)
        self._custom_poses = {}

        # Current robot state
        self._current_joints = None

        # Execution tracking
        self._expected_exec_count = 0

        # Dora node (single-threaded access only)
        self._node = Node()
        self._stopped = False

        # Wait for first joint state
        print("[MoveGroup] Waiting for robot connection...")
        deadline = time.time() + 30.0
        while self._current_joints is None and time.time() < deadline:
            self._poll_events(timeout=0.5)

        if self._current_joints is None:
            raise RuntimeError("Timed out waiting for joint_positions from robot/sim")
        print(f"[MoveGroup] Connected. Joints: {np.round(self._current_joints, 2)}")

    # ==================== Internal event polling ==================== #

    def _poll_events(self, timeout=0.1):
        """Poll one dora event and dispatch it. Returns the event or None."""
        try:
            event = self._node.next(timeout=timeout)
        except Exception:
            return None

        if event is None:
            return None
        if event["type"] == "STOP":
            self._stopped = True
            return event
        if event["type"] != "INPUT":
            return event

        event_id = event["id"]
        try:
            if event_id == "joint_positions":
                self._handle_joint_positions(event)
            elif event_id == "plan_status":
                self._handle_plan_status(event)
            elif event_id == "trajectory":
                self._handle_trajectory(event)
            elif event_id == "execution_status":
                self._handle_execution_status(event)
            elif event_id == "ik_solution":
                self._handle_ik_solution(event)
            elif event_id == "ik_status":
                self._handle_ik_status(event)
            elif event_id == "command_result":
                self._handle_command_result(event)
        except Exception as e:
            print(f"[MoveGroup] Error handling {event_id}: {e}")

        return event

    def _poll_until(self, condition_fn, timeout, poll_interval=0.05):
        """Poll events until condition_fn() returns True or timeout expires.
        Returns True if condition was met, False on timeout."""
        deadline = time.time() + timeout
        while time.time() < deadline and not self._stopped:
            self._poll_events(timeout=min(poll_interval, deadline - time.time()))
            if condition_fn():
                return True
        return condition_fn()

    # ---- Event handlers (update internal state) ---- #

    def _handle_joint_positions(self, event):
        joints = event["value"].to_numpy()
        self._current_joints = _extract_arm_joints(joints, self._num_joints)

    # Plan/trajectory/execution state for current operation
    _plan_done = False
    _plan_success = False
    _plan_message = ""
    _plan_trajectory = None
    _exec_done = False
    _exec_success = False
    _ik_done = False
    _ik_success = False
    _ik_solution = None
    _ik_message = ""
    _scene_done = False
    _scene_success = False

    def _handle_plan_status(self, event):
        status = _decode_json(event["value"])
        self._plan_success = status.get("success", False)
        self._plan_message = status.get("message", status.get("error", ""))
        if not self._plan_success:
            self._plan_done = True

    def _handle_trajectory(self, event):
        value = event["value"]
        if hasattr(value, "to_numpy"):
            traj_flat = value.to_numpy()
        else:
            traj_flat = np.frombuffer(value, dtype=np.float32)

        n_joints = self._num_joints
        n_waypoints = len(traj_flat) // n_joints

        if n_waypoints > 0:
            try:
                waypoints = traj_flat.reshape(n_waypoints, n_joints)
                self._plan_trajectory = [waypoints[i] for i in range(n_waypoints)]
                self._plan_success = True
                self._plan_done = True
                # Note: do NOT increment _expected_exec_count here.
                # The caller (go/plan) increments it before sending plan_request.
            except Exception as e:
                print(f"[MoveGroup] Error reshaping trajectory: {e}")

    def _handle_execution_status(self, event):
        status = _decode_json(event["value"])
        exec_count = status.get("execution_count", 0)
        is_executing = status.get("is_executing", True)

        if exec_count >= self._expected_exec_count and not is_executing:
            self._exec_success = True
            self._exec_done = True

    def _handle_ik_solution(self, event):
        value = event["value"]
        if hasattr(value, "to_numpy"):
            solution = value.to_numpy()
        else:
            solution = np.frombuffer(value, dtype=np.float32)

        if len(solution) >= self._num_joints:
            self._ik_solution = solution[:self._num_joints].copy()
            self._ik_success = True
            self._ik_done = True

    def _handle_ik_status(self, event):
        status = _decode_json(event["value"])
        if not status.get("success", False):
            self._ik_success = False
            self._ik_message = status.get("message", "IK failed")
            self._ik_done = True

    def _handle_command_result(self, event):
        try:
            result = _decode_json(event["value"])
            self._scene_success = result.get("success", False)
        except Exception:
            self._scene_success = False
        self._scene_done = True

    # ==================== Joint-space motion ==================== #

    def go(self, joint_goal=None, wait=True, timeout=30.0):
        """Plan and execute motion to a joint-space goal.

        Args:
            joint_goal: 7-element list/array of target joint angles (rad).
                        If None, uses previously set target.
            wait: If True, block until execution completes.
            timeout: Max seconds to wait.

        Returns:
            True if motion completed successfully, False on failure/timeout.
        """
        if joint_goal is not None:
            self.set_joint_value_target(joint_goal)

        # If a pose target was set (not joint target), solve IK first
        if self._target_joints is None and self._target_pose is not None:
            solution = self.compute_ik(self._target_pose)
            if solution is None:
                print("[MoveGroup] IK failed for pose target")
                return False
            self._target_joints = np.array(solution)
            self._target_pose = None

        if self._target_joints is None:
            raise RuntimeError("No target set. Call set_joint_value_target() or set_named_target() first.")

        if self._current_joints is None:
            raise RuntimeError("No joint state received yet.")

        # Reset operation state
        self._plan_done = False
        self._plan_success = False
        self._plan_trajectory = None
        self._exec_done = False
        self._exec_success = False
        # Increment exec count: planner will send trajectory to executor via dataflow
        self._expected_exec_count += 1

        # Send plan request
        request = {
            "start": np.asarray(self._current_joints, dtype=float).tolist(),
            "goal": np.asarray(self._target_joints, dtype=float).tolist(),
            "planner": self._planner_id,
            "max_time": self._planning_time,
        }
        self._node.send_output("plan_request", _encode_json(request))

        if not wait:
            return True

        # Wait for planning
        deadline = time.time() + timeout
        plan_timeout = self._planning_time + 5.0
        if not self._poll_until(lambda: self._plan_done, timeout=plan_timeout):
            print("[MoveGroup] Planning timed out")
            return False

        if not self._plan_success:
            print(f"[MoveGroup] Planning failed: {self._plan_message}")
            return False

        # Wait for execution (use remaining time from overall timeout)
        exec_timeout = max(deadline - time.time(), 10.0)
        if not self._poll_until(lambda: self._exec_done, timeout=exec_timeout):
            print("[MoveGroup] Execution timed out")
            return False

        return self._exec_success

    def set_joint_value_target(self, joint_values):
        """Set joint-space target for the next go() call."""
        joint_values = np.asarray(joint_values, dtype=float)
        if len(joint_values) != self._num_joints:
            raise ValueError(f"Expected {self._num_joints} joints, got {len(joint_values)}")

        lower = self._config.JOINT_LOWER_LIMITS
        upper = self._config.JOINT_UPPER_LIMITS
        for i, (v, lo, hi) in enumerate(zip(joint_values, lower, upper)):
            if v < lo or v > hi:
                raise ValueError(f"Joint {i} value {v:.3f} outside limits [{lo:.3f}, {hi:.3f}]")

        self._target_joints = joint_values.copy()
        self._target_pose = None

    def set_named_target(self, name):
        """Set target to a named pose ('home', 'safe', 'zero', or custom)."""
        if name in self._custom_poses:
            self._target_joints = self._custom_poses[name].copy()
        elif name in self._config.NAMED_POSES:
            self._target_joints = self._config.NAMED_POSES[name].copy()
        else:
            available = list(self._config.NAMED_POSES.keys()) + list(self._custom_poses.keys())
            raise KeyError(f"Unknown pose '{name}'. Available: {available}")
        self._target_pose = None

    def remember_joint_values(self, name, values=None):
        """Store current (or given) joint values under a name."""
        if values is None:
            values = self.get_current_joint_values()
        self._custom_poses[name] = np.asarray(values, dtype=float).copy()

    def stop(self):
        """Stop current motion by holding current position."""
        current = self.get_current_joint_values()
        request = {
            "start": current,
            "goal": current,
            "planner": self._planner_id,
            "max_time": 1.0,
        }
        self._node.send_output("plan_request", _encode_json(request))

    # ==================== Pose-space motion ==================== #

    def set_pose_target(self, pose):
        """Set end-effector pose target (IK solved internally on go()).

        Args:
            pose: [x,y,z,r,p,y] (6D) or [x,y,z,qw,qx,qy,qz] (7D).
        """
        pose = np.asarray(pose, dtype=float)
        if len(pose) not in (6, 7):
            raise ValueError(f"Pose must be 6 or 7 elements, got {len(pose)}")
        self._target_pose = pose.copy()
        self._target_joints = None

    def compute_ik(self, pose, seed=None, timeout=5.0):
        """Compute inverse kinematics without planning/executing.

        Returns:
            List of 7 joint angles, or None if IK failed.
        """
        pose = np.asarray(pose, dtype=np.float32)

        if seed is not None:
            seed = np.asarray(seed, dtype=np.float32)
            if len(pose) == 6:
                data = np.concatenate([pose, seed])
            else:
                data = pose
        else:
            data = pose

        self._ik_done = False
        self._ik_success = False
        self._ik_solution = None

        self._node.send_output("ik_request", pa.array(data, type=pa.float32()))

        if not self._poll_until(lambda: self._ik_done, timeout=timeout):
            print("[MoveGroup] IK timed out")
            return None

        if self._ik_success and self._ik_solution is not None:
            return self._ik_solution.tolist()
        return None

    def compute_fk(self, joint_values=None):
        """Compute forward kinematics locally.

        Returns:
            Tuple of (position_xyz, rotation_3x3).
        """
        from dora_moveit.ik_solver.ik_op import NumericalIKSolver
        solver = NumericalIKSolver(self._num_joints)
        if joint_values is None:
            joint_values = self.get_current_joint_values()
        return solver.forward_kinematics(np.asarray(joint_values))

    # ==================== Plan / Execute separately ==================== #

    def plan(self, joint_goal=None, timeout=10.0):
        """Plan a trajectory without executing.

        Returns:
            Tuple of (success: bool, trajectory: list[list[float]]).
        """
        if joint_goal is not None:
            self.set_joint_value_target(joint_goal)

        if self._target_joints is None:
            raise RuntimeError("No target set.")

        if self._current_joints is None:
            raise RuntimeError("No joint state received.")

        self._plan_done = False
        self._plan_trajectory = None
        self._plan_success = False
        # Increment exec count: planner output auto-routes to executor via dataflow
        self._expected_exec_count += 1

        request = {
            "start": np.asarray(self._current_joints, dtype=float).tolist(),
            "goal": np.asarray(self._target_joints, dtype=float).tolist(),
            "planner": self._planner_id,
            "max_time": self._planning_time,
        }
        self._node.send_output("plan_request", _encode_json(request))

        if not self._poll_until(lambda: self._plan_done, timeout=timeout):
            return False, []

        if self._plan_success and self._plan_trajectory:
            traj = [wp.tolist() for wp in self._plan_trajectory]
            return True, traj
        return False, []

    def execute(self, trajectory, wait=True, timeout=30.0):
        """Wait for the currently executing trajectory to complete.

        In this dataflow architecture, the trajectory_executor receives
        trajectories directly from the planner, so execution starts
        automatically after plan(). This method just waits for completion.

        Args:
            trajectory: The trajectory returned by plan() (used for validation only).
            wait: If True, block until execution completes.
            timeout: Max seconds to wait.

        Returns:
            True if execution completed successfully.
        """
        if not trajectory:
            return False

        # Execution was already started by the dataflow (planner → executor).
        # exec count was already incremented by plan(). Just wait.
        self._exec_done = False
        self._exec_success = False

        if not wait:
            return True

        if not self._poll_until(lambda: self._exec_done, timeout=timeout):
            print("[MoveGroup] Execution timed out")
            return False
        return self._exec_success

    # ==================== State queries ==================== #

    def get_current_joint_values(self):
        """Get current joint positions as a list."""
        if self._current_joints is None:
            return [0.0] * self._num_joints
        return self._current_joints.tolist()

    def get_current_pose(self):
        """Get current end-effector pose via FK."""
        return self.compute_fk()

    def get_named_targets(self):
        """Return list of available named target names."""
        names = list(self._config.NAMED_POSES.keys())
        names.extend(k for k in self._custom_poses if k not in names)
        return names

    # ==================== Planner config ==================== #

    def set_planner_id(self, planner_id):
        """Set planning algorithm ('rrt_connect', 'rrt_star', 'rrt', 'prm')."""
        self._planner_id = planner_id

    def set_planning_time(self, seconds):
        """Set max planning time in seconds."""
        self._planning_time = float(seconds)

    # ==================== Scene access ==================== #

    def get_planning_scene_interface(self):
        """Get PlanningSceneInterface for adding/removing obstacles."""
        return PlanningSceneInterface(self)

    # ==================== Lifecycle ==================== #

    def shutdown(self):
        """Shut down MoveGroup."""
        self._stopped = True
        print("[MoveGroup] Shutdown complete")


# --------------- PlanningSceneInterface --------------- #

class PlanningSceneInterface:
    """Interface for modifying the planning scene (obstacles).

    Obtained via MoveGroup.get_planning_scene_interface().
    """

    def __init__(self, move_group: MoveGroup):
        self._mg = move_group

    def _send_command(self, command: dict, timeout: float = 5.0) -> bool:
        self._mg._scene_done = False
        self._mg._scene_success = False
        self._mg._node.send_output("scene_command", _encode_json(command))
        self._mg._poll_until(lambda: self._mg._scene_done, timeout=timeout)
        return self._mg._scene_success

    def add_box(self, name, position, size, color=None):
        """Add a box obstacle. position=[x,y,z], size=[sx,sy,sz]."""
        obj = {"name": name, "type": "box", "position": list(position), "dimensions": list(size)}
        if color:
            obj["color"] = list(color)
        return self._send_command({"action": "add", "object": obj})

    def add_sphere(self, name, position, radius, color=None):
        """Add a sphere obstacle."""
        obj = {"name": name, "type": "sphere", "position": list(position), "radius": float(radius)}
        if color:
            obj["color"] = list(color)
        return self._send_command({"action": "add", "object": obj})

    def add_cylinder(self, name, position, radius, height, color=None):
        """Add a cylinder obstacle."""
        obj = {"name": name, "type": "cylinder", "position": list(position),
               "radius": float(radius), "height": float(height)}
        if color:
            obj["color"] = list(color)
        return self._send_command({"action": "add", "object": obj})

    def remove_world_object(self, name):
        """Remove an obstacle by name."""
        return self._send_command({"action": "remove", "object_name": str(name)})

    def clear(self):
        """Remove all objects from the scene."""
        return self._send_command({"action": "clear"})

    def attach_object(self, name, link="link7"):
        """Attach a world object to a robot link."""
        return self._send_command({"action": "attach", "object_name": str(name), "link": str(link)})

    def detach_object(self, name):
        """Detach an object from the robot."""
        return self._send_command({"action": "detach", "object_name": str(name)})
