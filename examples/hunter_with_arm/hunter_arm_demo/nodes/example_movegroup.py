#!/usr/bin/env python3
"""
Example: Multi-view capture using MoveGroup API
=================================================
Compare this ~40-line script to the 287-line multi_view_capture_node.py.

This is the dora-moveit equivalent of the ROS MoveIt example:
  group = MoveGroupCommander("manipulator")
  group.set_named_target("home")
  group.go(wait=True)

Usage:
  dora start config/dataflow_movegroup_mujoco.yml
"""
from dora_moveit.workflow.move_group import MoveGroup


def main():
    # ---- 1. Connect to robot ----
    group = MoveGroup("gen72")
    scene = group.get_planning_scene_interface()

    # ---- 2. Go to home position ----
    group.set_named_target("home")
    group.go(wait=True)
    print("At home position")

    # ---- 3. Joint-space goal (turn right 90 degrees) ----
    TURN_RIGHT = [-1.57, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0]
    group.go(TURN_RIGHT, wait=True)
    print("Turned right")

    # ---- 4. Arch over pipe ----
    ARCH_RIGHT = [-1.57, 0.26, 0.0, -1.65, 0.0, 0.68, 0.0]
    group.go(ARCH_RIGHT, wait=True)
    print("Arched over pipe - capture image here!")

    # ---- 5. Retract and return home ----
    group.go(TURN_RIGHT, wait=True)
    group.set_named_target("home")
    group.go(wait=True)
    print("Back home")

    # ---- 6. Plan without executing ----
    success, trajectory = group.plan([-1.0, -0.3, 0.0, -0.5, 0.0, 0.5, 0.0])
    if success:
        print(f"Planned trajectory: {len(trajectory)} waypoints")
        # Execute the planned trajectory
        group.execute(trajectory, wait=True)
        print("Executed planned trajectory")

    # ---- 7. Return home and shutdown ----
    group.set_named_target("home")
    group.go(wait=True)
    print("Done! Staying idle.")

    # Keep alive (dora event loop continues in background)
    import time
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    group.shutdown()


if __name__ == "__main__":
    main()
