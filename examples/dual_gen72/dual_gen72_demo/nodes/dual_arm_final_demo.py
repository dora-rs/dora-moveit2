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
    print("=" * 60)
    print("  双臂 GEN72 抓取-递交-放置 演示")
    print("  (双臂协调：独立运动 + 同步运动 + 物体递交)")
    print("=" * 60)

    config = load_config()
    group = DualMoveGroup(left_name="left_arm", right_name="right_arm")
    scene = group.get_planning_scene_interface()

    # ---- Step 1: Both arms go home ----
    print("\n--- 1. 双臂同步回到初始位姿 ---")
    print("[演示] 双臂同步运动：两条手臂同时规划并执行到 home 位姿")
    print("[目标] 左臂: home | 右臂: home")
    group.set_named_target(left_name="home", right_name="home")
    group.go(wait=True)
    print("[完成] 双臂已同步到达 home 位姿")
    time.sleep(1.0)

    # ---- Step 2: Left arm picks up ball ----
    print("\n--- 2. 左臂抓取球体 ---")
    print("[演示] 单臂独立运动：左臂从上方接近球体并抓取")
    print("[目标] 左臂移动到球体正上方（接近位姿）")
    # Ball is at (0, 0.1, 0.83) in world frame
    # Left arm approach from above
    left_above_ball = [0.2, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go_left(left_above_ball, wait=True)
    print("[完成] 左臂已到达球体上方")
    time.sleep(0.05)

    # Lower to grasp
    print("[目标] 左臂下降至抓取位置")
    left_grasp = [0.2, 0.5, 0.0, -1.2, 0.0, 0.5, 0.0]
    group.go_left(left_grasp, wait=True)
    print("[完成] 左臂已到达抓取位置")
    time.sleep(0.05)

    # Close left gripper to grasp ball
    print("[目标] 左臂夹爪夹取球体")
    group.gripper_close(arm="left")
    print("[完成] 左臂夹爪已夹紧")
    time.sleep(2.5)

    # Lift with ball
    print("[目标] 左臂提升，带球离开")
    left_lift = [0.2, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go_left(left_lift, wait=True)
    print("[完成] 左臂已提起球体")
    time.sleep(0.05)

    # ---- Step 3: Right arm moves to receive position ----
    print("\n--- 3. 右臂移动到接收位置 ---")
    print("[演示] 单臂独立运动：右臂移动到递交区域等待接收")
    print("[目标] 右臂到达接收位姿")
    right_receive = [0.0, 0.3, 0.0, -0.8, 0.0, 0.5, 2.0]
    group.go_right(right_receive, wait=True)
    print("[完成] 右臂已就位，准备接收")
    time.sleep(0.05)

    # ---- Step 4: Handoff ----
    print("\n--- 4. 双臂协调递交 ---")
    print("[演示] 双臂同步运动：两臂同时移动到中间递交位置，完成物体传递")
    print("[目标] 左臂递出位姿 + 右臂接收位姿（同步执行）")
    # Move both arms to handoff position (center, synchronized)
    left_handoff = [0.5, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    right_handoff = [-0.5, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go(left_joints=left_handoff, right_joints=right_handoff, wait=True)
    print("[完成] 双臂到达递交位置，球体从左臂传递到右臂")
    time.sleep(0.05)

    # ---- Step 5: Right arm places ball on plate ----
    print("\n--- 5. 右臂放置球体到托盘 ---")
    print("[演示] 单臂独立运动：右臂将球体放置到目标托盘上")
    print("[目标] 右臂移动到托盘上方")
    right_above_plate = [-0.2, 0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    group.go_right(right_above_plate, wait=True)
    print("[完成] 右臂已到达托盘上方")
    time.sleep(0.05)

    # Lower to place
    print("[目标] 右臂下降，放置球体")
    right_place = [0.2, 0.5, 0.0, -1.2, 0.0, 0.5, 0.0]
    group.go_right(right_place, wait=True)
    print("[完成] 球体已放置到托盘上")
    time.sleep(0.05)

    # Retreat
    print("[目标] 右臂抬起，撤离托盘区域")
    group.go_right(right_above_plate, wait=True)
    print("[完成] 右臂已撤离")
    time.sleep(0.05)

    # ---- Step 6: Both arms return home ----
    print("\n--- 6. 双臂同步回到初始位姿 ---")
    print("[演示] 双臂同步运动：任务完成，两臂同时回到 home")
    print("[目标] 左臂: home | 右臂: home")
    group.set_named_target(left_name="home", right_name="home")
    group.go(wait=True)
    print("[完成] 双臂已回到 home 位姿")

    print("\n" + "=" * 60)
    print("  演示全部完成！")
    print("=" * 60)
    group.shutdown()


if __name__ == "__main__":
    main()