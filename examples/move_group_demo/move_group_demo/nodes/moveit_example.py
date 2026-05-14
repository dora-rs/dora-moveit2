#!/usr/bin/env python3
"""
Dora-MoveIt MoveGroup Example
===============================
Dora-rs equivalent of the standard ROS MoveIt Python example.
Demonstrates all core MoveGroup capabilities:

  1. Move to a named pose
  2. Move to a joint-space goal
  3. Move to a Cartesian pose goal (via IK)
  4. Cartesian path (straight-line in workspace)
  5. Add / avoid collision objects

Compare with ROS MoveIt:
  group = moveit_commander.MoveGroupCommander("manipulator")
  group.set_named_target("home")
  group.go(wait=True)

Usage:
  cd examples/move_group_demo
  dora up
  dora start dataflows/moveit_example_mujoco.yml
"""
import time
import numpy as np
from dora_moveit.workflow.move_group import MoveGroup


def main():
    # =========================================================
    # Initialize  (ROS equivalent: moveit_commander.roscpp_initialize)
    # =========================================================
    group = MoveGroup("gen72")
    scene = group.get_planning_scene_interface()

    print("=" * 60)
    print("  Dora-MoveIt MoveGroup 演示")
    print("  (基于 dora-rs 数据流的 ROS MoveIt 风格 API)")
    print("=" * 60)

    # =========================================================
    # 1. Move to a named pose  (defined in robot config)
    # =========================================================
    print("\n--- 1. 命名位姿: 'home' ---")
    print("[演示] 移动到预定义的命名位姿")
    print(f"[目标] 命名位姿: 'home'（机械臂初始/零位姿态）")
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
    joint_goal[0] = 1.57    # rotate base 90 degrees
    joint_goal[1] = -0.785  # tilt shoulder
    joint_goal[2] = 0.0
    joint_goal[3] = -1.57   # bend elbow
    joint_goal[4] = 0.0
    joint_goal[5] = 0.785   # tilt wrist
    joint_goal[6] = 0.0
    print(f"[目标] 关节角度: J1=90°, J2=-45°, J3=0°, J4=-90°, J5=0°, J6=45°, J7=0°")
    group.go(joint_goal, wait=True)
    group.stop()
    print(f"[完成] 已到达关节目标: {[round(j, 2) for j in joint_goal]}")

    time.sleep(1)

    # =========================================================
    # 3. Move to a Cartesian pose goal  (IK solved internally)
    # =========================================================
    print("\n--- 3. 笛卡尔空间位姿目标 ---")
    print("[演示] 指定末端执行器的目标位姿（位置+姿态），内部通过逆运动学求解")
    # [x, y, z, roll, pitch, yaw]
    pose_goal = [0.15, 0.1, 0.6, 3.14, 0, 1.57]
    print(f"[目标] 位置: x=0.15m, y=0.1m, z=0.6m | 姿态: roll=π, pitch=0, yaw=π/2")
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
    # 4. Cartesian path  (straight line in workspace)
    # =========================================================
    print("\n--- 4. 笛卡尔路径（直线轨迹） ---")
    print("[演示] 末端执行器沿工作空间中的直线路径运动")
    # Get current end-effector position via FK
    current_pos, _ = group.get_current_pose()

    waypoints = []
    # Move down 10 cm
    waypoints.append([current_pos[0], current_pos[1], current_pos[2] - 0.1, 0, 0, 0])
    # Move sideways 10 cm
    waypoints.append([current_pos[0], current_pos[1] + 0.1, current_pos[2] - 0.1, 0, 0, 0])
    print(f"[目标] 路径点: 先下移10cm → 再侧移10cm，步长1cm")

    (trajectory, fraction) = group.compute_cartesian_path(
        waypoints,
        eef_step=0.01,   # 1 cm resolution
    )
    print(f"[规划] 笛卡尔路径: {len(trajectory)} 个路径点, 完成度 {fraction * 100:.0f}%")

    if fraction > 0.5:
        group.execute(trajectory, wait=True)
        print("[完成] 笛卡尔路径执行完毕")
    else:
        print("[失败] 笛卡尔路径规划失败 — 逆运动学求解失败点过多")

    time.sleep(1)

    # =========================================================
    # 5. Add a collision object and plan around it
    # =========================================================
    print("\n--- 5. 碰撞物体避障 ---")
    print("[演示] 在规划场景中添加障碍物，规划器自动绕障")
    # Add a box obstacle in the workspace
    scene.add_box("obstacle_box", [0.0, 0.0, 0.5], [0.1, 0.1, 0.5])
    print("[场景] 已添加障碍物 'obstacle_box': 位置=[0, 0, 0.5]m, 尺寸=[0.1, 0.1, 0.5]m")
    time.sleep(1)

    # Plan to home — planner will avoid the box
    print("[目标] 在存在障碍物的情况下规划回到 home 位姿")
    group.set_named_target("home")
    success = group.go(wait=True)
    if success:
        print("[完成] 已到达 home（规划器成功绕开障碍物）")
    else:
        print("[失败] 规划失败 — 障碍物可能阻挡了所有可行路径")

    # Remove the obstacle
    scene.remove_world_object("obstacle_box")
    print("[场景] 已移除障碍物 'obstacle_box'")

    time.sleep(1)

    # =========================================================
    # Return to home and finish
    # =========================================================
    print("\n--- 演示结束 ---")
    group.set_named_target("home")
    group.go(wait=True)
    print("[完成] 已回到 home 位姿，演示全部完成！")

    # Keep alive so MuJoCo viewer stays open
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    group.shutdown()


if __name__ == "__main__":
    main()
