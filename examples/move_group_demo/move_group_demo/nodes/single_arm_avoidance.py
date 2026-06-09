#!/usr/bin/env python3
"""
Single-arm collision-object avoidance demo — course Ch6.5.

Adds a box obstacle to the planning scene, plans around it, then
removes the obstacle.

Usage:
  cd examples/move_group_demo
  dora up
  dora start dataflows/single_arm_avoidance_mujoco.yml
"""
import time
from dora_moveit.workflow.move_group import MoveGroup


def main():
    group = MoveGroup("gen72")
    scene = group.get_planning_scene_interface()

    print("=" * 60)
    print("  GEN72 Single-Arm Avoidance Demo (Course Ch6.5)")
    print("=" * 60)

    # Start at home
    group.set_named_target("home")
    group.go(wait=True)
    time.sleep(1.0)

    # Drop a box in front of the arm
    print("\n[1/4] Adding obstacle_box at [0.0, 0.0, 0.5] size [0.1, 0.1, 0.5]")
    scene.add_box("obstacle_box", [0.0, 0.0, 0.5], [0.1, 0.1, 0.5])
    time.sleep(1.0)

    # Move to a joint target on the far side — planner must route around
    print("[2/4] Planning to joint target (planner avoids the box)")
    joint_goal = [1.57, -0.785, 0.0, -1.57, 0.0, 0.785, 0.0]
    success = group.go(joint_goal, wait=True)
    print(f"    result: {'reached target' if success else 'planning failed'}")
    time.sleep(1.0)

    print("[3/4] Return home (still avoiding the box)")
    group.set_named_target("home")
    group.go(wait=True)
    time.sleep(1.0)

    print("[4/4] Removing obstacle_box")
    scene.remove_world_object("obstacle_box")

    print("\nAvoidance demo complete.")
    group.shutdown()


if __name__ == "__main__":
    main()
