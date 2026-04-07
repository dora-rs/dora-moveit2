#!/usr/bin/env python3
"""
Dora-MoveIt Pick-and-Place Demo — ADORA1 Nano (SO_ARM100)
===========================================================
Full pick-and-place sequence using MoveGroup API:
  1. Home position
  2. Move above pick location
  3. Lower to grasp (Cartesian path)
  4. Close gripper
  5. Lift object
  6. Move to place location
  7. Lower to place (Cartesian path)
  8. Open gripper
  9. Retreat and return home

Uses the same MoveGroup API as the UR5e demo, adapted for the
Nano arm's smaller workspace.

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
    print("  Dora-MoveIt Pick-and-Place — ADORA1 Nano")
    print("  (ROS MoveIt-style API on dora-rs dataflow)")
    print("=" * 60)

    # Workspace reference (arm-frame coordinates from FK/IK tests):
    #   Home EE:  [0.032, 0.124, 0.087]
    #   Reach:    x ~ -0.05..0.05, y ~ 0.04..0.13, z ~ -0.16..0.11
    #
    # Pick/place positions chosen within IK-reachable workspace:
    pick_above  = [0.03, 0.12, 0.06, 0, 0, 0]   # above pick point
    pick_grasp  = [0.03, 0.12, 0.04, 0, 0, 0]   # grasp height
    place_above = [0.05, 0.10, 0.06, 0, 0, 0]   # above place point
    place_lower = [0.05, 0.10, 0.04, 0, 0, 0]   # place height

    GRIPPER_CLOSED = 0.5
    GRIPPER_OPEN = 0.0

    # =========================================================
    # 1. Home position
    # =========================================================
    print("\n--- 1. Move to home ---")
    group.set_named_target("home")
    group.go(wait=True)
    group.stop()
    print("At home position")
    time.sleep(1)

    # =========================================================
    # 2. Move above pick location
    # =========================================================
    print("\n--- 2. Move above pick location ---")
    group.set_pose_target(pick_above)
    success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    if success:
        pos, _ = group.get_current_pose()
        print(f"Above pick: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
    else:
        print("Failed to reach above-pick position, trying joint goal")
        # Fallback: use a known reachable joint config
        group.go([0.0, -0.5, 0.5, 0.0, 0.0, GRIPPER_OPEN], wait=True)
        group.stop()
        print("Reached approach position via joint goal")
    time.sleep(0.5)

    # =========================================================
    # 3. Lower to grasp (Cartesian path — straight down)
    # =========================================================
    print("\n--- 3. Lower to grasp ---")
    current_pos, _ = group.get_current_pose()
    lower_target = [current_pos[0], current_pos[1], current_pos[2] - 0.02, 0, 0, 0]
    trajectory, fraction = group.compute_cartesian_path(
        [lower_target], eef_step=0.005
    )
    if fraction > 0.5:
        group.execute(trajectory, wait=True)
        print(f"Lowered to grasp height ({fraction * 100:.0f}%)")
    else:
        print("Cartesian lower failed, using joint interpolation")
    time.sleep(0.5)

    # =========================================================
    # 4. Close gripper
    # =========================================================
    print("\n--- 4. Close gripper ---")
    joints = group.get_current_joint_values()
    joints[5] = GRIPPER_CLOSED
    group.go(joints, wait=True)
    group.stop()
    print(f"Gripper closed (joint6={GRIPPER_CLOSED})")
    time.sleep(0.5)

    # =========================================================
    # 5. Lift object (Cartesian path — straight up)
    # =========================================================
    print("\n--- 5. Lift object ---")
    current_pos, _ = group.get_current_pose()
    lift_target = [current_pos[0], current_pos[1], current_pos[2] + 0.03, 0, 0, 0]
    trajectory, fraction = group.compute_cartesian_path(
        [lift_target], eef_step=0.005
    )
    if fraction > 0.5:
        group.execute(trajectory, wait=True)
        print(f"Lifted ({fraction * 100:.0f}%)")
    else:
        print("Cartesian lift failed")
    time.sleep(0.5)

    # =========================================================
    # 6. Move to place location
    # =========================================================
    print("\n--- 6. Move to place location ---")
    group.set_pose_target(place_above)
    success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    if success:
        pos, _ = group.get_current_pose()
        print(f"Above place: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
    else:
        print("Failed to reach place position, trying joint goal")
        group.go([0.8, -0.5, 0.5, 0.0, 0.0, GRIPPER_CLOSED], wait=True)
        group.stop()
    time.sleep(0.5)

    # =========================================================
    # 7. Lower to place (Cartesian path — straight down)
    # =========================================================
    print("\n--- 7. Lower to place ---")
    current_pos, _ = group.get_current_pose()
    lower_target = [current_pos[0], current_pos[1], current_pos[2] - 0.02, 0, 0, 0]
    trajectory, fraction = group.compute_cartesian_path(
        [lower_target], eef_step=0.005
    )
    if fraction > 0.5:
        group.execute(trajectory, wait=True)
        print(f"Lowered to place height ({fraction * 100:.0f}%)")
    else:
        print("Cartesian lower failed")
    time.sleep(0.5)

    # =========================================================
    # 8. Open gripper (release object)
    # =========================================================
    print("\n--- 8. Open gripper ---")
    joints = group.get_current_joint_values()
    joints[5] = GRIPPER_OPEN
    group.go(joints, wait=True)
    group.stop()
    print(f"Gripper opened (joint6={GRIPPER_OPEN})")
    time.sleep(0.5)

    # =========================================================
    # 9. Retreat and return home
    # =========================================================
    print("\n--- 9. Retreat up ---")
    current_pos, _ = group.get_current_pose()
    retreat_target = [current_pos[0], current_pos[1], current_pos[2] + 0.03, 0, 0, 0]
    trajectory, fraction = group.compute_cartesian_path(
        [retreat_target], eef_step=0.005
    )
    if fraction > 0.5:
        group.execute(trajectory, wait=True)
    time.sleep(0.5)

    print("\n--- 10. Return home ---")
    group.set_named_target("home")
    group.go(wait=True)
    group.stop()
    print("Back at home. Pick-and-place complete!")

    print("\n" + "=" * 60)
    print("  Pick-and-place sequence finished successfully")
    print("=" * 60)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    group.shutdown()


if __name__ == "__main__":
    main()
