#!/usr/bin/env python3
"""
Dual GEN72 Pick-and-Place Demo
================================
Demonstrates dual-arm coordination:
1. Both arms go home (synchronized)
2. Left arm picks up ball (independent)
3. Right arm moves to receive position
4. Handoff: left arm places ball, right arm picks it up
5. Right arm places ball on plate
6. Both arms return home (synchronized)
"""

import time
import numpy as np
from dora_moveit.workflow.dual_move_group import DualMoveGroup
from dora_moveit.config import load_config


def main():
    print("=== Dual GEN72 Pick-and-Place Demo ===")

    config = load_config()
    group = DualMoveGroup(left_name="left_arm", right_name="right_arm")
    scene = group.get_planning_scene_interface()

    # ---- Step 1: Both arms go home ----
    print("\n[Step 1] Both arms → home")
    group.set_named_target(left_name="home", right_name="home")
    group.go(wait=True)
    time.sleep(1.0)

    # ---- Step 2: Left arm picks up ball ----
    print("\n[Step 2] Left arm → ball position")
    # Ball is at (0, 0.1, 0.83) in world frame
    # Left arm approach from above
    left_above_ball = [0.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go_left(left_above_ball, wait=True)
    time.sleep(0.5)

    # Lower to grasp
    left_grasp = [0.0, 0.5, 0.0, -1.2, 0.0, 0.5, 0.0]
    group.go_left(left_grasp, wait=True)
    print("  Grasping ball...")
    time.sleep(0.5)

    # Lift with ball
    left_lift = [0.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go_left(left_lift, wait=True)
    time.sleep(0.5)

    # ---- Step 3: Right arm moves to receive position ----
    print("\n[Step 3] Right arm → receive position")
    right_receive = [0.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go_right(right_receive, wait=True)
    time.sleep(0.5)

    # ---- Step 4: Handoff ----
    print("\n[Step 4] Dual-arm handoff")
    # Move both arms to handoff position (center, synchronized)
    left_handoff = [0.5, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    right_handoff = [-0.5, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go(left_joints=left_handoff, right_joints=right_handoff, wait=True)
    print("  Transferring ball...")
    time.sleep(1.0)

    # ---- Step 5: Right arm places ball on plate ----
    print("\n[Step 5] Right arm → plate position")
    right_above_plate = [0.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go_right(right_above_plate, wait=True)
    time.sleep(0.5)

    # Lower to place
    right_place = [0.0, 0.5, 0.0, -1.2, 0.0, 0.5, 0.0]
    group.go_right(right_place, wait=True)
    print("  Placing ball on plate...")
    time.sleep(0.5)

    # Retreat
    group.go_right(right_above_plate, wait=True)
    time.sleep(0.5)

    # ---- Step 6: Both arms return home ----
    print("\n[Step 6] Both arms → home")
    group.set_named_target(left_name="home", right_name="home")
    group.go(wait=True)

    print("\n=== Demo complete! ===")
    group.shutdown()


if __name__ == "__main__":
    main()
