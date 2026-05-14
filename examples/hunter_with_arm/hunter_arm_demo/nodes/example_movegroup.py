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
    print("Task: 回到初始位置")
    group.set_named_target("home")
    group.go(wait=True)
    print("Done: 回到初始位置")

    # ---- 3. Joint-space goal (turn right 90 degrees) ----
    print("Task: 底座右转 90°")
    TURN_RIGHT = [-1.57, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0]
    group.go(TURN_RIGHT, wait=True)
    print("Done: 底座右转 90°")

    # ---- 4. Arch over pipe ----
    print("Task: 拱形运动越过管道至拍摄点")
    ARCH_RIGHT = [-1.57, 0.26, 0.0, -1.65, 0.0, 0.68, 0.0]
    group.go(ARCH_RIGHT, wait=True)
    print("Done: 拱形运动越过管道至拍摄点")

    # ---- 5. Retract and return home ----
    print("Task: 缩回并回到初始位置")
    group.go(TURN_RIGHT, wait=True)
    group.set_named_target("home")
    group.go(wait=True)
    print("Done: 缩回并回到初始位置")

    # ---- 6. Plan without executing ----
    print("Task: 演示Plan without executing，规划自定义关节目标轨迹")
    success, trajectory = group.plan([-1.0, -0.3, 0.0, -0.5, 0.0, 0.5, 0.0])
    if success:
        print(f"Done: 规划完成，共 {len(trajectory)} 个路径点，开始执行")
        group.execute(trajectory, wait=True)
        print("Done: 自定义关节目标轨迹执行完毕")

    # ---- 7. Return home and shutdown ----
    print("Task: 返回初始位置并结束")
    group.set_named_target("home")
    group.go(wait=True)
    print("Done: 全部任务执行完毕，进入空闲")

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
