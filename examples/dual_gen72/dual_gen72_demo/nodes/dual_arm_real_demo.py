#!/usr/bin/env python3
"""
Dual GEN72 Real Hardware Demo
==============================
Task sequence for physical dual-arm pick-and-place.
Mirrors dual_arm_final_demo.py but targets real hardware via the
DualMoveGroup API (same interface, different backend).

Steps:
  1. Both arms → home
  2. Left arm → ball approach (0, 0.1, 0.83)
  3. Left arm → lift (0.05, 0.22, 1.2)
  4. Right arm → receive (0.05, 0.15, 1.2)
  5. Right arm → place (0.0, -0.1, 0.83)
  6. Both arms → home
"""

import time
import numpy as np
from dora_moveit.workflow.dual_move_group import DualMoveGroup
from dora_moveit.config import load_config


# Motion speed scale for real hardware (0.0–1.0).
# Lower than sim to be safe on first run — increase once verified.
SPEED_SCALE = 0.3

# Settle time between moves (seconds) — gives the arm time to stop vibrating.
SETTLE = 0.5


def go_home(group: DualMoveGroup):
    print("[Real] Both arms → home")
    group.set_named_target(left_name="home", right_name="home")
    group.go(wait=True)
    time.sleep(SETTLE)


def main():
    print("=== Dual GEN72 Real Hardware Demo ===")
    print(f"Speed scale: {SPEED_SCALE}  Settle time: {SETTLE}s")

    config = load_config()
    group = DualMoveGroup(
        left_name="left_arm",
        right_name="right_arm",
        speed_scale=SPEED_SCALE,
    )

    # ---- Step 1: home ----
    go_home(group)

    # ---- Step 2: left arm approach ball ----
    print("[Real] Left arm → ball approach (0.0, 0.2, 1.0)")
    group.go_left_pose([0.3, 0.3, 1.0, 0.0, 0.0, 0.0], wait=True)
    time.sleep(SETTLE)

    # ---- Step 3: left arm lift ----
    print("[Real] Left arm → lift (0.05, 0.22, 1.2)")
    group.go_left_pose([0.0, 0.2, 1.0, 0.0, 0.0, 0.0], wait=True)
    time.sleep(SETTLE)

    # ---- Step 4: right arm receive ----
    print("[Real] Right arm → receive (0.05, 0.15, 1.2)")
    group.go_right_pose([0.0, 0.2, 1.0, 0.0, 0.0, 0.0], wait=True)
    time.sleep(SETTLE)

    # ---- Step 5: right arm place ----
    print("[Real] Right arm → place (0.0, -0.1, 0.83)")
    group.go_right_pose([0.1, 0.3, 1.0, 0.0, 0.0, 0.0], wait=True)
    time.sleep(SETTLE)

    # ---- Step 6: home ----
    go_home(group)

    print("=== Demo complete ===")
    group.shutdown()


if __name__ == "__main__":
    main()
