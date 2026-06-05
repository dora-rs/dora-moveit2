#!/usr/bin/env python3
"""
Dora-MoveIt MoveGroup Example — UR5e
======================================
Same 5-step demo as the GEN72 example, but for the UR5e 6-DOF arm.

Usage:
  cd examples/move_group_demo
  dora up
  dora start dataflows/ur5e_example_mujoco.yml
"""
import time
import numpy as np
from dora_moveit.workflow.move_group import MoveGroup


def main():
    group = MoveGroup("ur5e")
    scene = group.get_planning_scene_interface()

    print("=" * 60)
    print("  Dora-MoveIt MoveGroup Example — UR5e")
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
    joint_goal[0] = 1.57     # rotate base 90 degrees
    joint_goal[1] = -1.2     # shoulder lift
    joint_goal[2] = 1.0      # elbow bend
    joint_goal[3] = -1.37    # wrist 1
    joint_goal[4] = -1.57    # wrist 2
    joint_goal[5] = 0.0      # wrist 3
    group.go(joint_goal, wait=True)
    group.stop()
    print(f"Reached joint goal: {[round(j, 2) for j in joint_goal]}")

    time.sleep(1)

    # =========================================================
    # 3. Move to a Cartesian pose goal (IK solved internally)
    # =========================================================
    print("\n--- 3. Cartesian pose goal ---")
    # [x, y, z, roll, pitch, yaw]
    pose_goal = [0.3, 0.2, 0.5, 0, 0, 0]
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
    # Move down 10 cm
    waypoints.append([current_pos[0], current_pos[1], current_pos[2] - 0.1, 0, 0, 0])
    # Move sideways 10 cm
    waypoints.append([current_pos[0], current_pos[1] + 0.1, current_pos[2] - 0.1, 0, 0, 0])

    (trajectory, fraction) = group.compute_cartesian_path(
        waypoints,
        eef_step=0.01,
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
    scene.add_box("obstacle_box", [0.3, 0.0, 0.5], [0.1, 0.1, 0.5])
    print("Added obstacle_box at [0.3, 0, 0.5], size [0.1, 0.1, 0.5]")
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
