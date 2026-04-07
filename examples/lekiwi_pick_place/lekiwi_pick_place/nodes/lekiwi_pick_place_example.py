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
    #   Ball at world [0.30, 0.0, 0.024] (far in front)
    #   Plate at world [0.30, -0.18, 0.003] (right side, next to ball)
    #
    # Pick config: approach [-0.99, 0, 0.11] — reaches straight forward
    #   with jaw opening perpendicular, ball enters between open jaws
    #   q = [0.0, 1.7, -0.282, -1.309, 0.449, open]
    #   gripper center at [0.304, 0.004, 0.002]
    #
    # Place config: q = [0.633, 1.7, -0.847, -0.436, -2.243, closed]
    #   gripper center at [0.300, -0.181, 0.001]

    GRIPPER_OPEN = 0.0
    GRIPPER_CLOSED = 0.55

    # Joint goals for pick sequence — approach from behind with open jaws
    pick_above = [0.0, 0.8, -0.3, -0.5, 0.449, GRIPPER_OPEN]           # retracted, above
    pick_grasp = [0.0, 1.7, -0.282, -1.309, 0.449, GRIPPER_OPEN]       # at ball, jaws open
    pick_lift  = [0.0, 0.8, -0.3, -0.5, 0.449, GRIPPER_CLOSED]         # lift up

    # Joint goals for place sequence
    place_above = [0.633, 0.8, -0.5, -0.3, -2.243, GRIPPER_CLOSED]     # above plate
    place_lower = [0.633, 1.7, -0.847, -0.436, -2.243, GRIPPER_CLOSED] # at plate
    place_retreat = [0.633, 0.8, -0.5, -0.3, -2.243, GRIPPER_OPEN]     # retreat up

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
