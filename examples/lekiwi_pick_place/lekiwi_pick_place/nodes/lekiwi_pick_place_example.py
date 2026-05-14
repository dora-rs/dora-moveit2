#!/usr/bin/env python3
"""
Dora-MoveIt Pick-and-Place Demo — LeKiwi (SO_ARM100)
=====================================================
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

Joint-space goals found via FK scan of the MuJoCo model.

Usage:
  cd examples/lekiwi_pick_place
  dora up
  dora start dataflows/lekiwi_pick_place_mujoco.yml
"""
import time
from dora_moveit.workflow.move_group import MoveGroup


def main():
    group = MoveGroup("lekiwi")

    print("=" * 60)
    print("  Dora-MoveIt Pick-and-Place — LeKiwi")
    print("  (ROS MoveIt-style API on dora-rs dataflow)")
    print("=" * 60)

    # Joint configs found via FK scan of the MuJoCo model:
    #   Ball on pedestal at [0.30, 0.0, 0.174]
    #   Plate on pedestal at [0.30, -0.18, 0.153]
    #
    # Top-down approach: gripper points downward, moves above then lowers
    #   pick_above z~0.34, pick_grasp z~0.18
    #   place_above z~0.34, place_lower z~0.18

    GRIPPER_OPEN = 0.0
    GRIPPER_CLOSED = 0.55

    # Pick: move above ball, lower down, grasp, lift
    pick_above = [0.053, 0.825, -1.1, -1.65, 1.346, GRIPPER_OPEN]      # z~0.34, above ball
    pick_grasp = [0.053, 0.417, 0.688, -1.65, 2.243, GRIPPER_OPEN]     # z~0.18, at ball
    pick_lift  = [0.053, 0.825, -1.1, -1.65, 1.346, GRIPPER_CLOSED]    # z~0.34, lift up

    # Place: move above plate, lower down, release, retreat
    place_above = [0.684, 1.058, -1.65, -0.434, 2.243, GRIPPER_CLOSED]  # z~0.34, above plate
    place_lower = [0.684, 1.0, -0.275, -1.65, 2.243, GRIPPER_CLOSED]    # z~0.18, at plate
    place_retreat = [0.684, 1.058, -1.65, -0.434, 2.243, GRIPPER_OPEN]  # z~0.34, retreat up

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
    print("Above pick position")
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
    print(f"Gripper closed (jaw={GRIPPER_CLOSED})")
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
    print(f"Gripper opened (jaw={GRIPPER_OPEN})")
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