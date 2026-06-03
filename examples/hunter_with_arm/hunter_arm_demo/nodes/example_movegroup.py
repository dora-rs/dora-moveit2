#!/usr/bin/env python3
"""
Example: MoveGroup + BaseController API demo
==============================================
Demonstrates using both the arm (MoveGroup) and chassis (BaseController)
high-level APIs together.

Usage:
  dora start dataflows/hunter_arm_mujoco.yml
"""
import time
from dora_moveit.workflow.move_group import MoveGroup
from dora_moveit.workflow.base_controller import BaseController


def main():
    # ---- 1. Connect to robot (share one dora Node) ----
    group = MoveGroup("gen72")
    base = BaseController(node=group._node)
    group._joint_callbacks.append(base.update_from_joints)

    # Arm poses
    TURN_RIGHT = [-1.57, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0]
    ARCH_RIGHT = [-1.57, 0.26, 0.0, -1.65, 0.0, 0.68, 0.0]

    # ---- 2. 初始位置 → 采集点1 ----
    print("Task: 底盘移动到采集点1")
    base.set_velocity(linear=1.5, angular=0.0)
    time.sleep(2.0) # 运动时间
    base.stop()
    x, y, yaw = base.get_position()
    print(f"Done: 到达采集点1, x={x:.3f} y={y:.3f}")

    # ---- 3. 采集点1：机械臂采集动作 ----
    print("Task: 采集点1 - 机械臂执行采集")
    group.set_named_target("home")
    group.go(wait=True)
    group.go(TURN_RIGHT, wait=True)
    group.go(ARCH_RIGHT, wait=True)
    group.go(TURN_RIGHT, wait=True)
    group.set_named_target("home")
    group.go(wait=True)
    print("Done: 采集点1 采集完成")

    # ---- 4. 采集点1 → 采集点2 ----
    print("Task: 底盘移动到采集点2")
    base.set_velocity(linear=1.5, angular=0.0)
    time.sleep(2.0)
    base.stop()
    x, y, yaw = base.get_position()
    print(f"Done: 到达采集点2, x={x:.3f} y={y:.3f}")

    # ---- 5. 采集点2：机械臂采集动作 ----
    print("Task: 采集点2 - 机械臂执行采集")
    group.set_named_target("home")
    group.go(wait=True)
    group.go(TURN_RIGHT, wait=True)
    group.go(ARCH_RIGHT, wait=True)
    group.go(TURN_RIGHT, wait=True)
    group.set_named_target("home")
    group.go(wait=True)
    print("Done: 采集点2 采集完成")

    # ---- 6. 采集点2 → 返回初始位置 ----
    print("Task: 底盘返回初始位置")
    base.set_velocity(linear=-1.5, angular=0.0)
    time.sleep(4.0)
    base.stop()
    x, y, yaw = base.get_position()
    print(f"Done: 返回初始位置, x={x:.3f} y={y:.3f}")

    # ---- 7. Shutdown ----
    print("Done: 全部任务执行完毕")
    base.shutdown()
    group.shutdown()


if __name__ == "__main__":
    main()
