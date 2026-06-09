"""Simple linear-interpolation joint-space planner — OMPL-free fallback.

Selected via ``PLANNER_TYPE=simple``. Produces a straight-line joint-space
trajectory from start to goal (no collision checking), so the dora-moveit2 stack
runs in obstacle-free simulation without the OMPL Python bindings (which
conda-forge ships C++-only). Matches the OMPL planner's dora I/O contract exactly:

  in : plan_request   (uint8 JSON: {"start":[...], "goal":[...], "max_time":...})
  out: plan_status    (uint8 JSON status dict, metadata {"success": bool})
       trajectory     (flattened float32, metadata {"num_waypoints","num_joints"})

scene_update events are accepted and ignored (no collision model in this planner).
"""
from __future__ import annotations

import json
import time

import numpy as np
import pyarrow as pa
from dora import Node

DEFAULT_WAYPOINTS = 50


def plan_linear(start, goal, num_waypoints: int = DEFAULT_WAYPOINTS) -> list[np.ndarray]:
    """Straight line from start to goal in joint space, inclusive of both ends."""
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    n = max(2, int(num_waypoints))
    return [start + (goal - start) * t for t in np.linspace(0.0, 1.0, n)]


def path_length(trajectory: list[np.ndarray]) -> float:
    return float(
        sum(
            float(np.linalg.norm(trajectory[i + 1] - trajectory[i]))
            for i in range(len(trajectory) - 1)
        )
    )


def main() -> None:
    print("=== Dora-MoveIt Simple (linear) Planner Operator ===")
    node = Node()
    plan_count = 0

    for event in node:
        if event["type"] != "INPUT":
            continue
        input_id = event["id"]

        if input_id == "plan_request":
            try:
                value = event["value"]
                request_bytes = bytes(value.to_pylist()) if hasattr(value, "to_pylist") else bytes(value)
                request_data = json.loads(request_bytes.decode("utf-8"))

                plan_count += 1
                t0 = time.time()
                trajectory = plan_linear(request_data["start"], request_data["goal"])
                status = {
                    "plan_id": plan_count,
                    "success": True,
                    "planning_time": time.time() - t0,
                    "path_length": path_length(trajectory),
                    "num_waypoints": len(trajectory),
                    "num_nodes": len(trajectory),
                    "message": "linear interpolation (simple planner)",
                }
                status_bytes = json.dumps(status).encode("utf-8")
                node.send_output(
                    "plan_status",
                    pa.array(list(status_bytes), type=pa.uint8()),
                    metadata={"success": True},
                )
                traj_flat = np.array(trajectory).flatten()
                node.send_output(
                    "trajectory",
                    pa.array(traj_flat, type=pa.float32()),
                    metadata={"num_waypoints": len(trajectory), "num_joints": len(trajectory[0])},
                )
            except Exception as e:  # noqa: BLE001
                import traceback

                traceback.print_exc()
                status = {"plan_id": plan_count, "success": False, "message": str(e)}
                node.send_output(
                    "plan_status",
                    pa.array(list(json.dumps(status).encode("utf-8")), type=pa.uint8()),
                )

        elif input_id == "scene_update":
            # No collision model in the simple planner — accept and ignore.
            pass


if __name__ == "__main__":
    main()
