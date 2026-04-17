#!/usr/bin/env python3
"""
Student template — write your own MoveGroup script.

Referenced by course Ch5 and Ch6. Edit the marked section below and run:
  cd examples/move_group_demo
  dora up
  dora start dataflows/custom_demo_mujoco.yml

There is no hot reload. After every edit: Ctrl+C, `dora stop`, relaunch.
"""
import time
from dora_moveit.workflow.move_group import MoveGroup


def main():
    group = MoveGroup("gen72")
    scene = group.get_planning_scene_interface()  # noqa: F841 — available if you need it

    print("my_custom_demo connected to gen72. Edit this file to add your code.")

    # ============================================================
    # STUDENT: write your code below this line.
    #
    # Examples from course Ch5 / Ch6:
    #
    #   # Named pose
    #   group.set_named_target("home")
    #   group.go(wait=True)
    #
    #   # Joint goal
    #   group.go([1.57, -0.785, 0.0, -1.57, 0.0, 0.785, 0.0], wait=True)
    #
    #   # Cartesian pose goal (IK solved internally)
    #   group.set_pose_target([0.15, 0.1, 0.6, 0, 0, 0])
    #   group.go(wait=True)
    #
    #   # Add / remove a collision object
    #   scene.add_box("box", [0.0, 0.0, 0.5], [0.1, 0.1, 0.5])
    #   scene.remove_world_object("box")
    # ============================================================

    time.sleep(1.0)  # keeps the process alive long enough to see output
    group.shutdown()


if __name__ == "__main__":
    main()
