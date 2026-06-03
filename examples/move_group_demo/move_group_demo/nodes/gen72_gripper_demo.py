#!/usr/bin/env python3
"""
GEN72 + Robotiq 2F-85 Gripper Demo
====================================
Demonstrates arm motion + gripper open/close using the combined model.

Sequence:
  1. Move to home pose
  2. Move to pre-grasp position
  3. Close gripper
  4. Lift (move to transport pose)
  5. Move to place position
  6. Open gripper
  7. Return home

Usage:
  cd examples/move_group_demo
  dora up
  dora start dataflows/gen72_gripper_mujoco.yml
"""
import time
import numpy as np
import pyarrow as pa
from dora_moveit.workflow.move_group import MoveGroup
from move_group_demo.config.gen72_with_gripper import GEN72WithGripperConfig


def _rotation_matrix_to_rpy(R):
    """Extract roll, pitch, yaw (XYZ extrinsic) from a 3x3 rotation matrix."""
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    if np.abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return np.array([roll, pitch, yaw])


def print_pose(group):
    pos, rot_matrix = group.compute_fk()
    rpy = _rotation_matrix_to_rpy(rot_matrix)
    print(f"  位置 [x, y, z]: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
    print(f"  姿态 [roll, pitch, yaw]: [{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}]")

def main():
    group = MoveGroup("gen72_with_gripper")

    print("=" * 60)
    print("  GEN72 + Robotiq 2F-85 夹爪演示")
    print("  (机械臂运动规划 + 夹爪开合控制)")
    print("=" * 60)

    # =========================================================
    # 1. Move to home pose
    # =========================================================
    print("\n--- 1. 移动到 home 位姿 ---")
    group.set_named_target("home")
    group.go(wait=True)
    group.stop()
    print("[完成] 已到达 home")
    time.sleep(0.05)

    # =========================================================
    # 2. Open gripper
    # =========================================================
    print("\n--- 2. 打开夹爪 ---")
    _set_gripper(group, GEN72WithGripperConfig.GRIPPER_OPEN)
    print("[完成] 夹爪已打开")
    time.sleep(0.05)

    # =========================================================
    # 3. Move to pre-grasp position (above object)
    # =========================================================
    print("\n--- 3. 移动到抓取预备位 ---")
    pre_grasp = [0.0, 0.4, 0.0, -1.6, 0.0, 0.5, 0.0]
    group.go(pre_grasp, wait=True)
    group.stop()
    print("[完成] 已到达抓取预备位")
    time.sleep(0.05)

    # =========================================================
    # 4. Descend to grasp position (Cartesian straight-line)
    # =========================================================
    print("\n--- 4. 笛卡尔路径（直线下降到抓取位） ---")
    current_pos, current_rot = group.compute_fk()
    rpy = _rotation_matrix_to_rpy(current_rot)

    waypoints = [
        [current_pos[0], current_pos[1], current_pos[2] - 0.15, rpy[0], rpy[1], rpy[2]],
    ]
    print(f"[目标] 从当前位置直线下移 10cm，步长 1cm")

    trajectory, fraction = group.compute_cartesian_path(waypoints, eef_step=0.01)
    print(f"[规划] 笛卡尔路径: {len(trajectory)} 个路径点, 完成度 {fraction * 100:.0f}%")

    if fraction > 0.5:
        group.execute(trajectory, wait=True)
        print("[完成] 笛卡尔路径执行完毕")
    else:
        print("[失败] 笛卡尔路径规划失败 — 逆运动学求解失败点过多")

    print_pose(group)
    time.sleep(0.05)

    # =========================================================
    # 5. Close gripper (grasp object)
    # =========================================================
    print("\n--- 5. 关闭夹爪（抓取） ---")
    _set_gripper(group, GEN72WithGripperConfig.GRIPPER_CLOSED)
    print("[完成] 夹爪已关闭")

    # =========================================================
    # 6. Lift object
    # =========================================================
    print("\n--- 6. 提升物体 ---")
    group.go(pre_grasp, wait=True)
    group.stop()
    print("[完成] 已提升")
    time.sleep(0.05)

    # =========================================================
    # 7. Transport to target position
    # =========================================================
    print("\n--- 7. 搬运到目标位置 ---")
    transport = [1.0, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go(transport, wait=True)
    group.stop()
    print("[完成] 已到达目标位置")
    time.sleep(0.05)

    # =========================================================
    # 8. Descend to place position
    # =========================================================
    print("\n--- 8. 下降到放置位 ---")
    place_pos = [1.0, 0.5, 0.0, -1.2, 0.0, 0.5, 0.0]
    group.go(place_pos, wait=True)
    group.stop()
    print("[完成] 已到达放置位")
    time.sleep(0.05)

    # =========================================================
    # 9. Open gripper (release object)
    # =========================================================
    print("\n--- 9. 打开夹爪（释放） ---")
    _set_gripper(group, GEN72WithGripperConfig.GRIPPER_OPEN)
    print("[完成] 夹爪已打开，物体已释放")


    # =========================================================
    # 10. Return home
    # =========================================================
    print("\n--- 10. 返回 home ---")
    group.set_named_target("home")
    group.go(wait=True)
    group.stop()
    print("[完成] 已回到 home")

    print("\n" + "=" * 60)
    print("  演示完成！")
    print("  机械臂完成了完整的 抓取→搬运→放置 流程")
    print("=" * 60)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    group.shutdown()


def _set_gripper(group: MoveGroup, value: float, duration: float = None):
    cfg = GEN72WithGripperConfig
    if duration is None:
        duration = cfg.GRIPPER_SETTLE_DURATION

    group._node.send_output(
        cfg.GRIPPER_OUTPUT_NAME,
        pa.array(np.array([value], dtype=np.float32), type=pa.float32()),
    )

    deadline = time.time() + duration
    while time.time() < deadline:
        group._poll_events(timeout=cfg.GRIPPER_POLL_TIMEOUT)


if __name__ == "__main__":
    main()
