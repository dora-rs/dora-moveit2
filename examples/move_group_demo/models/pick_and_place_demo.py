#!/usr/bin/env python3
"""Pick-and-place demo: UR5e + Robotiq 2F-85 grasping a red ball.

The red ball sits on the ground. The arm approaches with gripper open,
descends to the ball, closes the gripper to grasp, lifts, moves to the
green place target, and releases.

Run from models/ directory:
    mjpython pick_and_place_demo.py
"""
import mujoco
import mujoco.viewer
import numpy as np
import time

# Pre-computed IK solutions (gripper pointing straight down, solved with MuJoCo FK)
HOME      = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
PRE_GRASP = np.array([2.2045, -1.6635, 2.1416, -2.0490, -1.5708, 0.0066])
GRASP     = np.array([2.2045, -1.4535, 2.3026, -2.4199, -1.5708, 0.4526])
LIFT      = np.array([2.2045, -1.7505, 1.9440, -1.7642, -1.5708, 0.3115])
PRE_PLACE = np.array([3.4450, -1.7505, 1.9440, -1.7642, -1.5708, 0.0378])
PLACE     = np.array([3.4450, -1.5308, 2.2623, -2.3023, -1.5708, 0.4305])

GRIPPER_OPEN  = 0
GRIPPER_CLOSE = 255


def smooth(t):
    """Smooth cubic ease in-out."""
    t = np.clip(t, 0, 1)
    return 3 * t**2 - 2 * t**3


def lerp(a, b, t):
    return a + (b - a) * np.clip(t, 0, 1)


model = mujoco.MjModel.from_xml_path("ur5e.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)

# Start: gripper open, arm at home
data.ctrl[:6] = HOME
data.ctrl[6] = GRIPPER_OPEN

# Settle physics so ball rests on ground
for _ in range(3000):
    mujoco.mj_step(model, data)

ball_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "red_ball")

print("=" * 50)
print("  Pick-and-Place Demo")
print("  UR5e + Robotiq 2F-85 + Red Ball")
print("=" * 50)
print("Close the viewer window to exit.\n")

# Timeline: (start, end, from_joints, to_joints, gripper)
timeline = [
    # Hold home, gripper open — ball visible on ground
    (0,    2,    HOME,      HOME,      GRIPPER_OPEN),
    # Move above ball (gripper open)
    (2,    5,    HOME,      PRE_GRASP, GRIPPER_OPEN),
    # Descend to ball
    (5,    7.5,  PRE_GRASP, GRASP,     GRIPPER_OPEN),
    # Pause at ball
    (7.5,  8,    GRASP,     GRASP,     GRIPPER_OPEN),
    # Close gripper on ball
    (8,    10,   GRASP,     GRASP,     "close"),
    # Hold firm grip
    (10,   11,   GRASP,     GRASP,     GRIPPER_CLOSE),
    # Lift ball
    (11,   13.5, GRASP,     LIFT,      GRIPPER_CLOSE),
    # Move to above place target
    (13.5, 17,   LIFT,      PRE_PLACE, GRIPPER_CLOSE),
    # Descend to place
    (17,   19.5, PRE_PLACE, PLACE,     GRIPPER_CLOSE),
    # Open gripper (release ball)
    (19.5, 21.5, PLACE,     PLACE,     "open"),
    # Pause after release
    (21.5, 22.5, PLACE,     PLACE,     GRIPPER_OPEN),
    # Lift away
    (22.5, 24.5, PLACE,     PRE_PLACE, GRIPPER_OPEN),
    # Return home
    (24.5, 28,   PRE_PLACE, HOME,      GRIPPER_OPEN),
    # Hold home
    (28,   35,   HOME,      HOME,      GRIPPER_OPEN),
]

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    last_print = -1
    while viewer.is_running() and time.time() - start < 35:
        t = time.time() - start

        for t0, t1, j_from, j_to, grip in timeline:
            if t0 <= t < t1:
                frac = smooth((t - t0) / (t1 - t0))
                data.ctrl[:6] = lerp(j_from, j_to, frac)

                if grip == "close":
                    data.ctrl[6] = lerp(GRIPPER_OPEN, GRIPPER_CLOSE, frac)
                elif grip == "open":
                    data.ctrl[6] = lerp(GRIPPER_CLOSE, GRIPPER_OPEN, frac)
                else:
                    data.ctrl[6] = grip
                break

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)

        # Print ball status at key moments
        sec = int(t)
        if sec != last_print and sec in (0, 8, 11, 13, 17, 21, 27):
            bpos = data.xpos[ball_id]
            labels = {0: "Start", 8: "Grasping", 11: "Lifting", 13: "Lifted",
                      17: "Moving", 21: "Releasing", 27: "Done"}
            status = "LIFTED" if bpos[2] > 0.06 else "ground"
            print(f"  t={sec:2d}s [{labels[sec]:10s}] ball=[{bpos[0]:.3f}, {bpos[1]:.3f}, {bpos[2]:.3f}] {status}")
            last_print = sec

print("\nDone.")
