#!/usr/bin/env python3
"""
Dora-MoveIt MoveGroup Example — UR5e
======================================
Same 5-step demo as the GEN72 example, but for the UR5e 6-DOF arm.

Usage:
  cd examples/move_group_demo
  dora up
  dora start dataflows/ur5e_example_mujoco.yml
"""
import time
import numpy as np
from dora_moveit.workflow.move_group import MoveGroup


def main():
    group = MoveGroup("ur5e")
    scene = group.get_planning_scene_interface()

    print("=" * 60)
    print("  Dora-MoveIt MoveGroup 演示 — UR5e")
    print("  (基于 dora-rs 数据流的 ROS MoveIt 风格 API)")
    print("=" * 60)

    # =========================================================
    # 1. Move to a named pose
    # =========================================================
    print("\n--- 1. 命名位姿: 'home' ---")
    print("[演示] 移动到预定义的命名位姿")
    print("[目标] 命名位姿: 'home'（UR5e 初始/零位姿态）")
    group.set_named_target("home")
    group.go(wait=True)
    group.stop()
    print("[完成] 已到达 home 位姿")

    time.sleep(1)

    # =========================================================
    # 2. Move to a joint-space goal
    # =========================================================
    print("\n--- 2. 关节空间目标 ---")
    print("[演示] 通过指定各关节角度值进行运动规划")
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 1.57     # rotate base 90 degrees
    joint_goal[1] = -1.57     # shoulder lift
    joint_goal[2] = 1.57      # elbow bend
    joint_goal[3] = -1.57    # wrist 1
    joint_goal[4] = -1.57    # wrist 2
    joint_goal[5] = 0.0      # wrist 3
    print("[目标] 关节角度: 基座=90°, 肩=-90°, 肘=90°, 腕1=-90°, 腕2=-90°, 腕3=0°")
    group.go(joint_goal, wait=True)
    group.stop()
    print(f"[完成] 已到达关节目标: {[round(j, 2) for j in joint_goal]}")

    time.sleep(1)

    # =========================================================
    # 3. Move to a Cartesian pose goal (IK solved internally)
    # =========================================================
    print("\n--- 3. 笛卡尔空间位姿目标 ---")
    print("[演示] 指定末端执行器的目标位姿（位置+姿态），内部通过逆运动学求解")
    # [x, y, z, roll, pitch, yaw]
    pose_goal = [0.5, 0.5, 0.6, 0, 0, 1.57]
    print(f"[目标] 位置: x=0.5m, y=0.5m, z=0.6m | 姿态: roll=0, pitch=0, yaw=π/2")
    group.set_pose_target(pose_goal)
    success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    if success:
        pos, rot = group.get_current_pose()
        print(f"[完成] 已到达笛卡尔位姿: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
    else:
        print("[失败] 笛卡尔位姿目标未达成（逆运动学或规划失败）")


    time.sleep(1)

    # =========================================================
    # 4. Cartesian path (straight line in workspace)
    # =========================================================
    print("\n--- 4. Cartesian path ---")
    current_pos, _ = group.get_current_pose()

    waypoints = []
    # Move down 10 cm
    waypoints.append([current_pos[0], current_pos[1], current_pos[2] + 0.1, 0, 0, 0])
    # Move sideways 10 cm
    waypoints.append([current_pos[0], current_pos[1] + 0.1, current_pos[2] - 0.1, 0, 0, 0])

    (trajectory, fraction) = group.compute_cartesian_path(
        waypoints,
        eef_step=0.01,
    )
    print(f"Cartesian path: {len(trajectory)} waypoints, {fraction * 100:.0f}% achieved")

    if fraction > 0.5:
        group.execute(trajectory, wait=True)
        print("Cartesian path executed")
    else:
        print("Cartesian path failed — too many IK failures")

    time.sleep(1)

    # =========================================================
    # 5. Add a collision object and plan around it
    # =========================================================
    print("\n--- 5. Collision object ---")
    scene.add_box("obstacle_box", [0.3, 0.0, 0.5], [0.1, 0.1, 0.5])
    print("Added obstacle_box at [0.3, 0, 0.5], size [0.1, 0.1, 0.5]")
    time.sleep(1)

    group.set_named_target("home")
    success = group.go(wait=True)
    if success:
        print("Reached home (planner avoided obstacle)")
    else:
        print("Planning failed — obstacle may be blocking")

    scene.remove_world_object("obstacle_box")
    print("Removed obstacle_box")

    time.sleep(1)

    # =========================================================
    # Return to home and finish
    # =========================================================
    print("\n--- Done ---")
    group.set_named_target("home")
    group.go(wait=True)
    print("Back at home. Example complete!")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    group.shutdown()


if __name__ == "__main__":
    main()
