#!/usr/bin/env python3
"""
DualMoveGroup: High-level dual-arm motion planning API for dora-moveit
======================================================================
Provides a synchronous, blocking API for coordinating two robot arms,
building on top of the single-arm MoveGroup pattern.

Usage:
    from dora_moveit.workflow.dual_move_group import DualMoveGroup

    group = DualMoveGroup()
    group.set_named_target(left_name="home", right_name="home")
    group.go(wait=True)
    group.shutdown()
"""

import json
import time
import numpy as np
import pyarrow as pa
from dora import Node
from dora_moveit.config import load_config, is_dual_arm


def _encode_json(data: dict) -> pa.Array:
    raw = json.dumps(data).encode("utf-8")
    return pa.array(list(raw), type=pa.uint8())


def _decode_json(value) -> dict:
    if hasattr(value, "to_pylist"):
        raw = bytes(value.to_pylist())
    elif hasattr(value, "to_numpy"):
        raw = bytes(value.to_numpy())
    else:
        raw = bytes(value)
    return json.loads(raw.decode("utf-8"))


class DualMoveGroup:
    """High-level dual-arm motion planning interface.

    Manages two arm chains through a single dora Node, sending 14D
    plan requests (7D left + 7D right) to the planner.
    """

    def __init__(self, left_name: str = "left_arm", right_name: str = "right_arm", speed_scale: float = 1.0):
        self._left_name = left_name
        self._right_name = right_name
        self._speed_scale = speed_scale
        self._config = load_config()
        self._num_joints = self._config.NUM_JOINTS  # per arm
        self._total_joints = self._num_joints * 2

        if not is_dual_arm(self._config):
            raise RuntimeError(
                "DualMoveGroup requires a dual-arm config with ARM_CHAINS defined"
            )

        # Per-arm state
        self._chain_starts = getattr(self._config, "ARM_QPOS_START_PER_CHAIN", {
            left_name: 0,
            right_name: self._num_joints,
        })

        # Planner settings
        self._planner_id = "rrt_connect"
        self._planning_time = 5.0

        # Target state
        self._left_target = None
        self._right_target = None

        # Current robot state (14D concatenated)
        self._current_joints = None
        self._left_joints = None
        self._right_joints = None

        # Execution tracking
        self._expected_exec_count = 0

        # Dora node
        self._node = Node()
        self._stopped = False

        # Operation state
        self._plan_done = False
        self._plan_success = False
        self._plan_message = ""
        self._plan_trajectory = None
        self._exec_done = False
        self._exec_success = False
        self._scene_done = False
        self._scene_success = False

        # Wait for first joint state
        print("[DualMoveGroup] Waiting for robot connection...")
        deadline = time.time() + 30.0
        while self._current_joints is None and time.time() < deadline:
            self._poll_events(timeout=0.5)

        if self._current_joints is None:
            raise RuntimeError("Timed out waiting for joint_positions")
        print(f"[DualMoveGroup] Connected. Left: {np.round(self._left_joints, 2)}, "
              f"Right: {np.round(self._right_joints, 2)}")

    def _poll_events(self, timeout=0.1):
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
            elif event_id == "command_result":
                self._handle_command_result(event)
        except Exception as e:
            print(f"[DualMoveGroup] Error handling {event_id}: {e}")
        return event

    def _poll_until(self, condition_fn, timeout, poll_interval=0.05):
        deadline = time.time() + timeout
        while time.time() < deadline and not self._stopped:
            self._poll_events(timeout=min(poll_interval, deadline - time.time()))
            if condition_fn():
                return True
        return condition_fn()

    def _handle_joint_positions(self, event):
        joints = event["value"].to_numpy()
        chain_starts = self._chain_starts
        left_start = chain_starts.get(self._left_name, 0)
        right_start = chain_starts.get(self._right_name, self._num_joints)
        self._left_joints = joints[left_start:left_start + self._num_joints].copy()
        self._right_joints = joints[right_start:right_start + self._num_joints].copy()
        self._current_joints = np.concatenate([self._left_joints, self._right_joints])

    def _handle_plan_status(self, event):
        status = _decode_json(event["value"])
        self._plan_success = status.get("success", False)
        self._plan_message = status.get("message", "")
        if not self._plan_success:
            self._plan_done = True

    def _handle_trajectory(self, event):
        value = event["value"]
        traj_flat = value.to_numpy() if hasattr(value, "to_numpy") else np.frombuffer(value, dtype=np.float32)
        n_joints = self._total_joints
        n_waypoints = len(traj_flat) // n_joints
        if n_waypoints > 0:
            try:
                waypoints = traj_flat.reshape(n_waypoints, n_joints)
                self._plan_trajectory = [waypoints[i] for i in range(n_waypoints)]
                self._plan_success = True
                self._plan_done = True
            except Exception as e:
                print(f"[DualMoveGroup] Error reshaping trajectory: {e}")

    def _handle_execution_status(self, event):
        status = _decode_json(event["value"])
        exec_count = status.get("execution_count", 0)
        is_executing = status.get("is_executing", True)
        if exec_count >= self._expected_exec_count and not is_executing:
            self._exec_success = True
            self._exec_done = True

    def _handle_command_result(self, event):
        try:
            result = _decode_json(event["value"])
            self._scene_success = result.get("success", False)
        except Exception:
            self._scene_success = False
        self._scene_done = True

    # ==================== Motion commands ==================== #

    def go(self, left_joints=None, right_joints=None, wait=True, timeout=30.0):
        """Plan and execute synchronized dual-arm motion.

        Args:
            left_joints: 7D target for left arm (None = hold current)
            right_joints: 7D target for right arm (None = hold current)
            wait: Block until execution completes
            timeout: Max seconds to wait

        Returns:
            True if motion completed successfully
        """
        if left_joints is not None:
            self._left_target = np.asarray(left_joints, dtype=float)
        if right_joints is not None:
            self._right_target = np.asarray(right_joints, dtype=float)

        left_goal = self._left_target if self._left_target is not None else self._left_joints
        right_goal = self._right_target if self._right_target is not None else self._right_joints

        if left_goal is None or right_goal is None:
            raise RuntimeError("No target set and no current joint state")

        goal_14d = np.concatenate([left_goal, right_goal])
        start_14d = self._current_joints.copy()

        self._plan_done = False
        self._plan_success = False
        self._plan_trajectory = None
        self._exec_done = False
        self._exec_success = False
        self._expected_exec_count += 1

        request = {
            "start": start_14d.tolist(),
            "goal": goal_14d.tolist(),
            "planner": self._planner_id,
            "max_time": self._planning_time,
            "mode": "dual_arm",
        }
        self._node.send_output("plan_request", _encode_json(request))

        if not wait:
            return True

        plan_timeout = self._planning_time + 5.0
        if not self._poll_until(lambda: self._plan_done, timeout=plan_timeout):
            print("[DualMoveGroup] Planning timed out")
            return False
        if not self._plan_success:
            print(f"[DualMoveGroup] Planning failed: {self._plan_message}")
            return False

        exec_timeout = max(timeout - (self._planning_time + 5.0), 10.0)
        if not self._poll_until(lambda: self._exec_done, timeout=exec_timeout):
            print("[DualMoveGroup] Execution timed out")
            return False
        return self._exec_success

    def go_left(self, joints, wait=True, timeout=30.0):
        """Move only the left arm (right arm holds current position)."""
        return self.go(left_joints=joints, right_joints=self._right_joints, wait=wait, timeout=timeout)

    def go_right(self, joints, wait=True, timeout=30.0):
        """Move only the right arm (left arm holds current position)."""
        return self.go(left_joints=self._left_joints, right_joints=joints, wait=wait, timeout=timeout)

    def go_left_pose(self, pose, wait=True, timeout=30.0):
        """Move left arm end-effector to a world-frame pose.

        Args:
            pose: [x, y, z] or [x, y, z, roll, pitch, yaw] in world frame.
        """
        world_pos, rpy = self._parse_pose(pose)
        joints = self._solve_ik(world_pos, self._left_name, self._left_joints, rpy=rpy)
        if joints is None:
            return False
        return self.go_left(joints, wait=wait, timeout=timeout)

    def go_right_pose(self, pose, wait=True, timeout=30.0):
        """Move right arm end-effector to a world-frame pose.

        Args:
            pose: [x, y, z] or [x, y, z, roll, pitch, yaw] in world frame.
        """
        world_pos, rpy = self._parse_pose(pose)
        joints = self._solve_ik(world_pos, self._right_name, self._right_joints, rpy=rpy)
        if joints is None:
            return False
        return self.go_right(joints, wait=wait, timeout=timeout)

    @staticmethod
    def _parse_pose(pose):
        pose = np.asarray(pose, dtype=float)
        if len(pose) == 6:
            return pose[:3], pose[3:]
        return pose[:3], None

    @staticmethod
    def _rpy_to_rot(rpy):
        r, p, y = rpy
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        return Rz @ Ry @ Rx

    def _solve_ik(self, world_pos, arm_name, seed_joints, rpy=None):
        from dora_moveit.ik_solver.advanced_ik_solver import TracIKSolver, IKRequest
        base_tf = self._config.ARM_BASE_TRANSFORMS[arm_name]
        base_xyz = np.array(base_tf["xyz"])
        base_R = self._rpy_to_rot(base_tf["rpy"])  # world → arm-base rotation

        # Transform target position into arm-local frame
        local_pos = base_R.T @ (np.asarray(world_pos, dtype=float) - base_xyz)

        # Transform target orientation into arm-local frame
        local_rpy = None
        if rpy is not None:
            R_world = self._rpy_to_rot(np.asarray(rpy, dtype=float))
            R_local = base_R.T @ R_world
            # Convert back to RPY (ZYX)
            sy = np.sqrt(R_local[0, 0]**2 + R_local[1, 0]**2)
            if sy > 1e-6:
                local_rpy = np.array([
                    np.arctan2(R_local[2, 1], R_local[2, 2]),
                    np.arctan2(-R_local[2, 0], sy),
                    np.arctan2(R_local[1, 0], R_local[0, 0]),
                ])
            else:
                local_rpy = np.array([
                    np.arctan2(-R_local[1, 2], R_local[1, 1]),
                    np.arctan2(-R_local[2, 0], sy),
                    0.0,
                ])

        solver = TracIKSolver(self._config)
        seed = np.asarray(seed_joints, dtype=float) if seed_joints is not None else None
        result = solver.solve(IKRequest(
            target_position=local_pos,
            target_orientation=local_rpy,
            seed_joints=seed,
            orientation_type="rpy",
        ))
        if not result.success:
            print(f"[DualMoveGroup] IK failed for {arm_name} at world pos {world_pos} (err={result.error:.4f})")
            return None
        return result.joint_positions

    def set_named_target(self, left_name=None, right_name=None):
        """Set targets to named poses."""
        per_chain = getattr(self._config, "NAMED_POSES_PER_CHAIN", {})

        if left_name is not None:
            left_poses = per_chain.get(self._left_name, self._config.NAMED_POSES)
            if left_name not in left_poses:
                raise KeyError(f"Unknown left pose '{left_name}'. Available: {list(left_poses.keys())}")
            self._left_target = left_poses[left_name].copy()

        if right_name is not None:
            right_poses = per_chain.get(self._right_name, self._config.NAMED_POSES)
            if right_name not in right_poses:
                raise KeyError(f"Unknown right pose '{right_name}'. Available: {list(right_poses.keys())}")
            self._right_target = right_poses[right_name].copy()

    # ==================== Gripper control ==================== #

    def gripper_open(self, arm="both"):
        """Open gripper(s). arm: 'left', 'right', or 'both'."""
        self._send_gripper_command(0.0, arm)

    def gripper_close(self, arm="both"):
        """Close gripper(s). arm: 'left', 'right', or 'both'."""
        self._send_gripper_command(0.8, arm)

    def gripper_set(self, value, arm="both"):
        """Set gripper position (0.0=open, 0.8=closed). arm: 'left', 'right', or 'both'."""
        self._send_gripper_command(float(value), arm)

    def _send_gripper_command(self, value, arm):
        """Send gripper control command via dora output."""
        gripper_indices = getattr(self._config, "GRIPPER_ACTUATOR_INDEX", {})
        left_idx = gripper_indices.get("left_arm", 7)
        right_idx = gripper_indices.get("right_arm", 15)

        if arm == "left":
            cmd = np.array([value, 0.0], dtype=np.float32)
        elif arm == "right":
            cmd = np.array([0.0, value], dtype=np.float32)
        else:
            cmd = np.array([value, value], dtype=np.float32)

        self._node.send_output("gripper_control", pa.array(cmd, type=pa.float32()))

    def get_planning_scene_interface(self):
        """Get PlanningSceneInterface for adding/removing obstacles."""
        from dora_moveit.workflow.move_group import PlanningSceneInterface

        class _DualSceneAdapter:
            """Adapter to make DualMoveGroup look like MoveGroup for PlanningSceneInterface."""
            def __init__(self, dual_mg):
                self._node = dual_mg._node
                self._scene_done = False
                self._scene_success = False

            def _poll_until(self, fn, timeout):
                return dual_mg._poll_until(fn, timeout)

        adapter = _DualSceneAdapter(self)
        adapter._scene_done = False
        adapter._scene_success = False

        class DualPlanningSceneInterface:
            def __init__(self, dual_mg):
                self._mg = dual_mg

            def _send_command(self, command, timeout=5.0):
                self._mg._scene_done = False
                self._mg._scene_success = False
                self._mg._node.send_output("scene_command", _encode_json(command))
                self._mg._poll_until(lambda: self._mg._scene_done, timeout=timeout)
                return self._mg._scene_success

            def add_box(self, name, position, size, color=None):
                obj = {"name": name, "type": "box", "position": list(position), "dimensions": list(size)}
                if color:
                    obj["color"] = list(color)
                return self._send_command({"action": "add", "object": obj})

            def remove_world_object(self, name):
                return self._send_command({"action": "remove", "object_name": str(name)})

            def clear(self):
                return self._send_command({"action": "clear"})

        return DualPlanningSceneInterface(self)

    def shutdown(self):
        """Shut down DualMoveGroup."""
        self._stopped = True
        print("[DualMoveGroup] Shutdown complete")
