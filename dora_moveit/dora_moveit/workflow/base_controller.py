#!/usr/bin/env python3
"""
BaseController: High-level chassis velocity API for dora-moveit
================================================================
Provides a simple API to control a differential-drive mobile base
(e.g., Hunter SE) via linear/angular velocity commands.

Usage (standalone node):
    from dora_moveit.workflow.base_controller import BaseController

    base = BaseController()
    base.set_velocity(linear=0.5, angular=0.0)
    base.stop()
    base.shutdown()

Usage (shared with MoveGroup):
    from dora_moveit.workflow.move_group import MoveGroup
    from dora_moveit.workflow.base_controller import BaseController

    group = MoveGroup("gen72")
    base = BaseController(node=group._node)
    base.set_velocity(linear=0.5, angular=0.0)
"""

import numpy as np
import pyarrow as pa
from dora import Node


class BaseController:
    """Differential-drive base controller.

    Converts (linear, angular) velocity commands to left/right wheel
    speeds and sends them via the dora dataflow.

    Args:
        node: Existing dora Node to reuse. If None, creates a new one.
        wheel_separation: Track width in meters (default 0.4 for Hunter SE).
        wheel_radius: Wheel radius in meters (default 0.1).
    """

    WHEEL_SEPARATION = 0.4
    WHEEL_RADIUS = 0.1

    def __init__(self, node=None, wheel_separation: float = None, wheel_radius: float = None):
        if wheel_separation is not None:
            self.WHEEL_SEPARATION = wheel_separation
        if wheel_radius is not None:
            self.WHEEL_RADIUS = wheel_radius

        self._owns_node = node is None
        self._node = node if node is not None else Node()
        self._stopped = False
        self._current_pos = None

        if self._owns_node:
            import time
            print("[BaseController] Waiting for robot connection...")
            deadline = time.time() + 10.0
            while self._current_pos is None and time.time() < deadline:
                self._poll_events(timeout=0.5)

            if self._current_pos is not None:
                x, y, yaw = self._current_pos
                print(f"[BaseController] Connected. Position: x={x:.3f} y={y:.3f} yaw={yaw:.2f}")
            else:
                print("[BaseController] Warning: no joint_positions received, proceeding anyway")

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
        if event["type"] == "INPUT" and event["id"] == "joint_positions":
            self._update_position(event["value"].to_numpy())
        return event

    def update_from_joints(self, qpos: np.ndarray):
        """Update position from joint_positions array (call externally when sharing a Node)."""
        self._update_position(qpos)

    def _update_position(self, qpos: np.ndarray):
        if len(qpos) < 7:
            return
        x = float(qpos[0])
        y = float(qpos[1])
        qw, qx, qy, qz = qpos[3], qpos[4], qpos[5], qpos[6]
        yaw = float(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))
        self._current_pos = (x, y, yaw)

    def set_velocity(self, linear: float, angular: float):
        """Set base velocity.

        Args:
            linear: Forward speed in m/s (positive = forward).
            angular: Rotation speed in rad/s (positive = counter-clockwise).
        """
        left = (linear - angular * self.WHEEL_SEPARATION / 2.0) / self.WHEEL_RADIUS
        right = (linear + angular * self.WHEEL_SEPARATION / 2.0) / self.WHEEL_RADIUS
        wheel_cmd = np.array([left, right], dtype=np.float32)
        self._node.send_output("wheel_commands", pa.array(wheel_cmd))

    def stop(self):
        """Stop the base."""
        wheel_cmd = np.array([0.0, 0.0], dtype=np.float32)
        self._node.send_output("wheel_commands", pa.array(wheel_cmd))

    def get_position(self):
        """Get current base position (x, y, yaw).

        Returns:
            Tuple (x, y, yaw) in meters/radians, or (0, 0, 0) if unknown.
        """
        if self._owns_node:
            self._poll_events(timeout=0.05)
        if self._current_pos is not None:
            return self._current_pos
        return (0.0, 0.0, 0.0)

    def shutdown(self):
        """Stop and shut down."""
        self.stop()
        self._stopped = True
        print("[BaseController] Shutdown complete")
