#!/usr/bin/env python3
"""
Single-arm grasping demo — course Ch6.4.

Approach → descend → grasp → lift → move → descend → release → home.
Joint targets match the course listing verbatim so students see their
course code execute.

Usage:
  cd examples/move_group_demo
  dora up
  dora start dataflows/single_arm_grasping_mujoco.yml
"""
import time
from dora_moveit.workflow.move_group import MoveGroup


def main():
    group = MoveGroup("gen72")

    print("=" * 60)
    print("  GEN72 Single-Arm Grasping Demo (Course Ch6.4)")
    print("=" * 60)

    # Home first
    group.set_named_target("home")
    group.go(wait=True)
    time.sleep(1.0)

    above_object = [0.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    near_object = [0.0, 0.5, 0.0, -1.2, 0.0, 0.5, 0.0]
    above_target = [1.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    near_target = [1.0, 0.5, 0.0, -1.2, 0.0, 0.5, 0.0]

    print("\n[1/7] Move above object")
    group.go(above_object, wait=True)
    time.sleep(0.5)

    print("[2/7] Descend to object")
    group.go(near_object, wait=True)
    time.sleep(0.5)

    print("[3/7] Close gripper (simulated)")
    time.sleep(0.5)

    print("[4/7] Lift object")
    group.go(above_object, wait=True)
    time.sleep(0.5)

    print("[5/7] Traverse to target")
    group.go(above_target, wait=True)
    time.sleep(0.5)

    print("[6/7] Descend, place, open gripper (simulated)")
    group.go(near_target, wait=True)
    time.sleep(0.5)

    print("[7/7] Return home")
    group.set_named_target("home")
    group.go(wait=True)

    print("\nGrasping sequence complete.")
    group.shutdown()


if __name__ == "__main__":
    main()
