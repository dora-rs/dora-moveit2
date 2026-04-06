#!/usr/bin/env python3
"""
Dora-MoveIt MoveGroup Example — ADORA1 Nano (SO_ARM100)
=========================================================
5-step MoveGroup demo for the Nano 6-DOF arm:
  1. Named pose (home)
  2. Joint-space goal
  3. Cartesian pose goal (IK solved internally)
  4. Cartesian path (straight line in workspace)
  5. Collision object avoidance

Usage:
  cd examples/nano_pick_place
  dora up
  dora start dataflows/nano_pick_place_mujoco.yml
"""
import time
import numpy as np
from dora_moveit.workflow.move_group import MoveGroup


def main():
    group = MoveGroup("nano")
    scene = group.get_planning_scene_interface()

    print("=" * 60)
    print("  Dora-MoveIt MoveGroup Example — ADORA1 Nano")
    print("  (ROS MoveIt-style API on dora-rs dataflow)")
    print("=" * 60)

    # =========================================================
    # 1. Move to a named pose
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
    joint_goal[0] = 0.5     # rotate base
    joint_goal[1] = -0.5    # shoulder
    joint_goal[2] = 0.3     # elbow
    joint_goal[3] = 0.0     # wrist pitch
    joint_goal[4] = 0.0     # wrist roll
    joint_goal[5] = 0.0     # gripper
    group.go(joint_goal, wait=True)
    group.stop()
    print(f"Reached joint goal: {[round(j, 2) for j in joint_goal]}")

    time.sleep(1)

    # =========================================================
    # 3. Move to a Cartesian pose goal (IK solved internally)
    # =========================================================
    print("\n--- 3. Cartesian pose goal ---")
    pose_goal = [0.1, 0.05, 0.15, 0, 0, 0]
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
    # 4. Cartesian path (straight line in workspace)
    # =========================================================
    print("\n--- 4. Cartesian path ---")
    current_pos, _ = group.get_current_pose()

    waypoints = []
    waypoints.append([current_pos[0], current_pos[1], current_pos[2] - 0.03, 0, 0, 0])
    waypoints.append([current_pos[0], current_pos[1] + 0.05, current_pos[2] - 0.03, 0, 0, 0])

    (trajectory, fraction) = group.compute_cartesian_path(
        waypoints,
        eef_step=0.005,
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
    scene.add_box("obstacle_box", [0.1, 0.0, 0.1], [0.05, 0.05, 0.2])
    print("Added obstacle_box at [0.1, 0, 0.1], size [0.05, 0.05, 0.2]")
    time.sleep(1)

    group.set_named_target("home")
    success = group.go(wait=True)
    if success:
        print("Reached home (planner avoided obstacle)")
    else:
        print("Planning failed — obstacle may be blocking")

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

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    group.shutdown()


if __name__ == "__main__":
    main()
