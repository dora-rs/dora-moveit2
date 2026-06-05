#!/usr/bin/env python3
"""
Dora-MoveIt Pick-and-Place Demo — ADORA1 Nano (SO_ARM100)
===========================================================
Full pick-and-place sequence using MoveGroup API:
  1. Home position
  2. Move above pick location (joint goal)
  3. Lower to grasp (joint goal)
  4. Close gripper
  5. Lift object (joint goal)
  6. Move to place location (joint goal)
  7. Lower to place (joint goal)
  8. Open gripper
  9. Retreat up (joint goal)
  10. Return home

Joint-space goals are used because the Nano's small workspace
makes IK pose targets unreliable. Joint configs were found via
FK scan of the MuJoCo model.

Usage:
  cd examples/nano_pick_place
  dora up
  dora start dataflows/nano_pick_place_mujoco.yml
"""
import time
from dora_moveit.workflow.move_group import MoveGroup


def main():
    group = MoveGroup("nano")

    print("=" * 60)
    print("  Dora-MoveIt Pick-and-Place — ADORA1 Nano")
    print("  (ROS MoveIt-style API on dora-rs dataflow)")
    print("=" * 60)

    # Joint configs found via FK scan of the MuJoCo model:
    #   Ball at world [-0.04, 0.25, 0.06] (red sphere, far front-left)
    #   Plate at world [0.10, 0.25, 0.04] (green cylinder, far front-right)
    #   ~14cm separation between ball and plate
    #
    # Pick configs (gripper reaches ~[-0.037, 0.25, 0.063]):
    #   q = [-0.069, 1.158, -1.553, 1.5, 0, gripper]
    # Place configs (gripper reaches ~[0.101, 0.251, 0.057]):
    #   q = [0.621, 2.0, -1.868, -0.136, 0, gripper]

    GRIPPER_OPEN = 0.0
    GRIPPER_CLOSED = 0.5

    # Joint goals for pick sequence
    pick_above = [-0.069, 0.60, -0.80, 0.5, 0.0, GRIPPER_OPEN]        # above ball
    pick_grasp = [-0.069, 1.158, -1.553, 1.5, 0.0, GRIPPER_OPEN]      # at ball
    pick_lift  = [-0.069, 0.60, -0.80, 0.5, 0.0, GRIPPER_CLOSED]      # lift up

    # Joint goals for place sequence
    place_above = [0.621, 1.00, -1.00, 0.0, 0.0, GRIPPER_CLOSED]      # above plate
    place_lower = [0.621, 2.00, -1.868, -0.136, 0.0, GRIPPER_CLOSED]  # at plate
    place_retreat = [0.621, 1.00, -1.00, 0.0, 0.0, GRIPPER_OPEN]      # retreat up

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
    print("\n--- 2. Move above pick ---")
    group.go(pick_above, wait=True)
    group.stop()
    print(f"Above pick position")
    time.sleep(0.5)

    # =========================================================
    # 3. Lower to grasp
    # =========================================================
    print("\n--- 3. Lower to grasp ---")
    group.go(pick_grasp, wait=True)
    group.stop()
    print("At grasp position")
    time.sleep(0.5)

    # =========================================================
    # 4. Close gripper
    # =========================================================
    print("\n--- 4. Close gripper ---")
    joints = list(pick_grasp)
    joints[5] = GRIPPER_CLOSED
    group.go(joints, wait=True)
    group.stop()
    print(f"Gripper closed (joint6={GRIPPER_CLOSED})")
    time.sleep(0.5)

    # =========================================================
    # 5. Lift object
    # =========================================================
    print("\n--- 5. Lift object ---")
    group.go(pick_lift, wait=True)
    group.stop()
    print("Object lifted")
    time.sleep(0.5)

    # =========================================================
    # 6. Move to place location
    # =========================================================
    print("\n--- 6. Move to place location ---")
    group.go(place_above, wait=True)
    group.stop()
    print("Above place position")
    time.sleep(0.5)

    # =========================================================
    # 7. Lower to place
    # =========================================================
    print("\n--- 7. Lower to place ---")
    group.go(place_lower, wait=True)
    group.stop()
    print("At place position")
    time.sleep(0.5)

    # =========================================================
    # 8. Open gripper (release object)
    # =========================================================
    print("\n--- 8. Open gripper ---")
    joints = list(place_lower)
    joints[5] = GRIPPER_OPEN
    group.go(joints, wait=True)
    group.stop()
    print(f"Gripper opened (joint6={GRIPPER_OPEN})")
    time.sleep(0.5)

    # =========================================================
    # 9. Retreat up
    # =========================================================
    print("\n--- 9. Retreat up ---")
    group.go(place_retreat, wait=True)
    group.stop()
    print("Retreated from place position")
    time.sleep(0.5)

    # =========================================================
    # 10. Return home
    # =========================================================
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
