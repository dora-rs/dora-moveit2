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
import matplotlib.pyplot as plt
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

    group.set_joint_value_target([1.57, 0, 0, 0, 0, 0, 0])
    group.go(wait=True)
    group.stop()

    time.sleep(1)

    # =========================================================
    # 步骤三: 验证多关节组合目标设定功能
    # =========================================================
    print("\n--- 步骤三: 多关节组合目标 ---")
    print("[演示] 设定多关节组合目标，覆盖 J1/J2/J4/J6 四个关节")

    pos_before, _ = group.get_current_pose()
    print(f"[起始] 末端位置: x={pos_before[0]:.4f}, y={pos_before[1]:.4f}, z={pos_before[2]:.4f}")

    group.set_joint_value_target([1.57, -0.785, 0, -1.57, 0, 0.785, 0])
    group.go(wait=True)
    group.stop()

    pos_after, _ = group.get_current_pose()
    print(f"[到达] 末端位置: x={pos_after[0]:.4f}, y={pos_after[1]:.4f}, z={pos_after[2]:.4f}")

    displacement = np.linalg.norm(np.array(pos_after) - np.array(pos_before))
    print(f"[结果] 末端执行器空间位置偏移量: {displacement:.4f} m")

    time.sleep(1)

    # =========================================================
    # 步骤四: 测试关节空间轨迹平滑性
    # =========================================================
    print("\n--- 步骤四: 关节轨迹平滑性验证 ---")
    print("[演示] 执行运动过程中每秒采样关节角度，验证相邻数据无跳变")

    group.set_named_target("home")
    print("[目标] 从当前多关节构型回到 home")

    samples = []
    samples.append(group.get_current_joint_values().copy())
    group.go(wait=False)

    for i in range(10):
        # 持续 poll 事件 1 秒，让关节状态得到更新
        poll_deadline = time.time() + 1.0
        while time.time() < poll_deadline:
            group._poll_events(timeout=0.05)
        current = group.get_current_joint_values().copy()
        samples.append(current)
        print(f"  采样 {i+1:2d}: {[f'{v:.3f}' for v in current]}")

    group.stop()

    print("\n[分析] 相邻采样差值检查:")
    smooth = True
    for i in range(1, len(samples)):
        diff = np.abs(np.array(samples[i]) - np.array(samples[i - 1]))
        max_diff = np.max(diff)
        if max_diff > 0.1:
            print(f"  采样 {i-1} → {i}: 最大差值 = {max_diff:.4f} rad [!跳变]")
            smooth = False
        else:
            print(f"  采样 {i-1} → {i}: 最大差值 = {max_diff:.4f} rad [平滑]")

    if smooth:
        print("[结论] 轨迹连续平滑，无跳变或突变现象")
    else:
        print("[结论] 检测到跳变，轨迹平滑性不满足要求")

    # 生成关节角度变化折线图（J1/J2/J4/J6）
    samples_arr = np.array(samples)
    timestamps = list(range(len(samples)))
    target_joints = [0, 1, 3, 5]
    joint_labels = ["J1 (base)", "J2 (shoulder)", "J4 (elbow)", "J6 (wrist)"]

    plt.figure(figsize=(10, 6))
    for idx, joint_idx in enumerate(target_joints):
        plt.plot(timestamps, samples_arr[:, joint_idx], marker='o', markersize=4,
                 label=joint_labels[idx])

    plt.xlabel("time(s)")
    plt.ylabel("joint angle(rad)")
    plt.legend(loc="best")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()

    plot_path = "joint_trajectory_smoothness.png"
    plt.savefig(plot_path, dpi=150)
    plt.close()
    print(f"[图表] 关节角度变化折线图已保存: {plot_path}")

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