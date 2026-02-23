#!/usr/bin/env python3
"""
Dora-MoveIt MoveGroup Example
===============================
Dora-rs equivalent of the standard ROS MoveIt Python example.
Demonstrates all core MoveGroup capabilities:

  1. Move to a named pose
  2. Move to a joint-space goal
  3. Move to a Cartesian pose goal (via IK)
  4. Cartesian path (straight-line in workspace)
  5. Add / avoid collision objects

Compare with ROS MoveIt:
  group = moveit_commander.MoveGroupCommander("manipulator")
  group.set_named_target("home")
  group.go(wait=True)

Usage:
  cd examples/move_group_demo
  dora up
  dora start dataflows/moveit_example_mujoco.yml
"""
import time
import numpy as np
from dora_moveit.workflow.move_group import MoveGroup


def main():
    # =========================================================
    # Initialize  (ROS equivalent: moveit_commander.roscpp_initialize)
    # =========================================================
    group = MoveGroup("gen72")
    scene = group.get_planning_scene_interface()

    print("=" * 60)
    print("  Dora-MoveIt MoveGroup Example")
    print("  (ROS MoveIt-style API on dora-rs dataflow)")
    print("=" * 60)

    # =========================================================
    # 1. Move to a named pose  (defined in robot config)
    # =========================================================
    print("\n--- 1. Named pose: 'home' ---")
    group.set_named_target("home")
    group.go(wait=True)
    group.stop()
    print("Reached home position")

    time.sleep(1)

    # =========================================================
    # 2. Move to a joint-space goal
    # =========================================================
    print("\n--- 2. Joint-space goal ---")
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 1.57    # rotate base 90 degrees
    joint_goal[1] = -0.785  # tilt shoulder
    joint_goal[2] = 0.0
    joint_goal[3] = -1.57   # bend elbow
    joint_goal[4] = 0.0
    joint_goal[5] = 0.785   # tilt wrist
    joint_goal[6] = 0.0
    group.go(joint_goal, wait=True)
    group.stop()
    print(f"Reached joint goal: {[round(j, 2) for j in joint_goal]}")

    time.sleep(1)

    # =========================================================
    # 3. Move to a Cartesian pose goal  (IK solved internally)
    # =========================================================
    print("\n--- 3. Cartesian pose goal ---")
    # [x, y, z, roll, pitch, yaw]
    pose_goal = [0.15, 0.1, 0.6, 0, 0, 0]
    group.set_pose_target(pose_goal)
    success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    if success:
        pos, rot = group.get_current_pose()
        print(f"Reached Cartesian pose: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
    else:
        print("Cartesian pose goal failed (IK or planning)")

    time.sleep(1)

    # =========================================================
    # 4. Cartesian path  (straight line in workspace)
    # =========================================================
    print("\n--- 4. Cartesian path ---")
    # Get current end-effector position via FK
    current_pos, _ = group.get_current_pose()

    waypoints = []
    # Move down 10 cm
    waypoints.append([current_pos[0], current_pos[1], current_pos[2] - 0.1, 0, 0, 0])
    # Move sideways 10 cm
    waypoints.append([current_pos[0], current_pos[1] + 0.1, current_pos[2] - 0.1, 0, 0, 0])

    (trajectory, fraction) = group.compute_cartesian_path(
        waypoints,
        eef_step=0.01,   # 1 cm resolution
    )
    print(f"Cartesian path: {len(trajectory)} waypoints, {fraction * 100:.0f}% achieved")

    if fraction > 0.5:
        group.execute(trajectory, wait=True)
        print("Cartesian path executed")
    else:
        print("Cartesian path failed — too many IK failures")

    time.sleep(1)

    # =========================================================
    # 5. Add a collision object and plan around it
    # =========================================================
    print("\n--- 5. Collision object ---")
    # Add a box obstacle in the workspace
    scene.add_box("obstacle_box", [0.0, 0.0, 0.5], [0.1, 0.1, 0.5])
    print("Added obstacle_box at [0, 0, 0.5], size [0.1, 0.1, 0.5]")
    time.sleep(1)

    # Plan to home — planner will avoid the box
    group.set_named_target("home")
    success = group.go(wait=True)
    if success:
        print("Reached home (planner avoided obstacle)")
    else:
        print("Planning failed — obstacle may be blocking")

    # Remove the obstacle
    scene.remove_world_object("obstacle_box")
    print("Removed obstacle_box")

    time.sleep(1)

    # =========================================================
    # Return to home and finish
    # =========================================================
    print("\n--- Done ---")
    group.set_named_target("home")
    group.go(wait=True)
    print("Back at home. Example complete!")

    # Keep alive so MuJoCo viewer stays open
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    group.shutdown()


if __name__ == "__main__":
    main()
