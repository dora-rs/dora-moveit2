#!/usr/bin/env python3
"""
Multi-View Capture Node for GEN72 on Hunter SE
================================================
Explicit joint-waypoint choreography:
  HOME → TURN_RIGHT → ARCH_RIGHT (capture) → TURN_RIGHT → HOME
       → TURN_LEFT  → ARCH_LEFT  (capture) → TURN_LEFT  → HOME
"""

import os
import json
import time
import numpy as np
import pyarrow as pa
from dora import Node
from hunter_arm_demo.config.gen72 import GEN72Config


class MultiViewCaptureNode:
    """Multi-view capture workflow controller using explicit joint choreography.

    Motion sequence:
      HOME → TURN (j1 rotates ±90°) → ARCH (over pipe, EE down) → capture
      → retract to TURN → return to HOME → repeat other side
    """

    # Joint configs:  [j1,    j2,   j3,    j4,    j5,  j6,   j7]
    HOME       = np.array([ 0.0,  -1.0, 0.0,  -2.5,   0.0,  0.5, 0.0])
    TURN_RIGHT = np.array([-1.57, -0.5, 0.0,  0.0,   0.0,  0.5, 0.0])
    ARCH_RIGHT = np.array([-1.57,  0.26, 0.0, -1.65,  0.0,  0.68, 0.0])
    TURN_LEFT  = np.array([ 1.57, -0.5, 0.0,  0.0,   0.0,  0.5, 0.0])
    ARCH_LEFT  = np.array([ 1.57,  0.26, 0.0, -1.65,  0.0,  0.68, 0.0])

    def __init__(self):
        # Full motion plan: list of (name, goal_joints, capture_here?)
        self.steps = [
            ("turn_right",      self.TURN_RIGHT, False),
            ("arch_over_right", self.ARCH_RIGHT,  True),   # capture here
            ("retract_right",   self.TURN_RIGHT, False),
            ("turn_left",   self.TURN_LEFT, False),
            ("arch_over_left",   self.ARCH_LEFT, True),
            ("retract_left",   self.TURN_LEFT, False),
            ("home_from_left", self.HOME,       False),
        ]
        self.step_idx = 0
        self.capture_count = 0

        self.current_joints = self.HOME.copy()
        self.waiting_for_execution = False
        self.waiting_for_planning = False
        self.expected_execution_count = 0
        self.vehicle_ready = False
        self.workflow_done = False

        self.output_dir = os.getenv("CAPTURE_OUTPUT_DIR", "captures")
        os.makedirs(self.output_dir, exist_ok=True)

        # Latest frame received from mujoco_sim/camera_image
        self.latest_frame: np.ndarray | None = None

        print("=" * 60)
        print("  多视角拍摄节点（编排式运动）")
        print("  Hunter SE 底盘 + GEN72 机械臂协同工作流")
        print("=" * 60)
        print(f"[初始化] 共 {len(self.steps)} 个运动步骤")
        print("[流程] HOME → 右转 → 右侧拱形拍摄 → 收回 → 左转 → 左侧拱形拍摄 → 收回 → HOME")

    def run(self):
        node = Node()
        self._send_robot_state(node, self.current_joints)
        time.sleep(0.5)

        if os.getenv("VEHICLE_MODE", "0") == "1":
            print("[等待] 底盘移动完成后开始机械臂工作流...")
        else:
            print("[模式] 无底盘模式，直接启动机械臂工作流...")
            self.vehicle_ready = True
            self._execute_next_step(node)

        for event in node:
            if event["type"] == "INPUT":
                self._handle_input(node, event)
            elif event["type"] == "STOP":
                break

        print("\n[结束] 多视角拍摄工作流全部完成！")

    # ---------------------- Event Handling ---------------------- #

    def _handle_input(self, node: Node, event):
        event_id = event["id"]

        if event_id == "vehicle_ready":
            if not self.vehicle_ready:
                self.vehicle_ready = True
                print("[事件] 底盘已停稳，启动机械臂工作流...")
                time.sleep(0.5)
                self._execute_next_step(node)

        elif event_id == "joint_positions":
            try:
                if self.waiting_for_execution:
                    return
                joints = event["value"].to_numpy()
                if len(joints) >= 20:
                    self.current_joints = joints[13:20].copy()
                else:
                    self.current_joints = joints[:7].copy()
            except Exception:
                pass

        elif event_id == "execution_status":
            self._handle_execution_status(node, event["value"])

        elif event_id == "trajectory":
            self._handle_trajectory(node, event)

        elif event_id == "plan_status":
            self._handle_plan_status(node, event["value"])

        elif event_id == "camera_image":
            try:
                meta = event.get("metadata", {}) if isinstance(event, dict) else {}
                w = int(meta.get("width", 640))
                h = int(meta.get("height", 480))
                self.latest_frame = event["value"].to_numpy().reshape(h, w, 3)
            except Exception:
                pass

    def _handle_plan_status(self, node: Node, data):
        try:
            if hasattr(data, "to_pylist"):
                status_bytes = bytes(data.to_pylist())
            else:
                status_bytes = bytes(data)
            status = json.loads(status_bytes.decode("utf-8"))
        except Exception as e:
            print(f"[Capture] Error decoding plan_status: {e}")
            return
        if not status.get("success", False):
            msg = status.get("message", status.get("error", "unknown"))
            print(f"  [规划失败] {msg}")
        else:
            print("  [规划成功] 轨迹已生成")

    def _handle_execution_status(self, node: Node, data):
        if not self.waiting_for_execution:
            return
        try:
            status_bytes = bytes(data.to_numpy())
            status = json.loads(status_bytes.decode('utf-8'))
            exec_count = status.get("execution_count", 0)
            is_executing = status.get("is_executing", True)

            if exec_count == self.expected_execution_count and not is_executing:
                self.waiting_for_execution = False
                self._on_execution_complete(node)
        except Exception as e:
            print(f"[Capture] Error handling execution status: {e}")

    def _handle_trajectory(self, node: Node, event):
        if self.waiting_for_execution:
            return

        value = event["value"]
        metadata = event.get("metadata", {}) if isinstance(event, dict) else {}

        try:
            if hasattr(value, "to_numpy"):
                traj_flat = value.to_numpy()
            else:
                traj_flat = np.frombuffer(value, dtype=np.float32)
        except Exception as e:
            print(f"[Capture] Error decoding trajectory: {e}")
            return

        num_joints = GEN72Config.NUM_JOINTS
        num_waypoints = metadata.get("num_waypoints", len(traj_flat) // num_joints)
        if num_waypoints <= 0:
            print("[Capture] Invalid trajectory: num_waypoints <= 0")
            return

        try:
            waypoints = traj_flat.reshape(num_waypoints, num_joints)
        except Exception as e:
            print(f"[Capture] Error reshaping trajectory: {e}")
            return

        print(f"  [轨迹接收] 共 {len(waypoints)} 个路径点")

        self.waiting_for_planning = False
        self.waiting_for_execution = True
        self.expected_execution_count += 1
        print(f"  [执行中] 等待第 {self.expected_execution_count} 次轨迹执行完成...")

    # ---------------------- Choreography ---------------------- #

    # Step name → Chinese description
    STEP_DESC = {
        "turn_right":      "机械臂基座右转90°，准备进入右侧拍摄区域",
        "arch_over_right": "机械臂拱形运动至右侧管道上方，末端朝下拍摄",
        "retract_right":   "从右侧拍摄位收回至右转中间位姿",
        "turn_left":       "机械臂基座左转90°，准备进入左侧拍摄区域",
        "arch_over_left":  "机械臂拱形运动至左侧管道上方，末端朝下拍摄",
        "retract_left":    "从左侧拍摄位收回至左转中间位姿",
        "home_from_left":  "从左侧收回，回到初始 HOME 位姿",
    }

    def _execute_next_step(self, node: Node):
        """Send plan_request for the next step in the choreography."""
        if self.step_idx >= len(self.steps):
            print("\n[完成] 所有步骤执行完毕！工作流结束。")
            print("[待机] 保持运行中，按 Ctrl+C 退出。")
            self.workflow_done = True
            return

        name, goal_joints, do_capture = self.steps[self.step_idx]
        desc = self.STEP_DESC.get(name, name)
        capture_tag = " [将拍摄]" if do_capture else ""
        print(f"\n{'='*50}")
        print(f"[步骤 {self.step_idx + 1}/{len(self.steps)}] {name}{capture_tag}")
        print(f"[演示] {desc}")
        print(f"[目标] 关节角度: {np.round(goal_joints, 2)}")
        print(f"[当前] 关节角度: {np.round(self.current_joints, 2)}")

        self.waiting_for_planning = True
        self._request_plan(node, self.current_joints, goal_joints)

    def _on_execution_complete(self, node: Node):
        """Called when trajectory execution completes for the current step."""
        name, goal_joints, do_capture = self.steps[self.step_idx]
        print(f"  [执行完成] 步骤 '{name}' 已到达目标位姿")

        if do_capture:
            self._capture_image(node, name)

        self.current_joints = goal_joints.copy()
        self.step_idx += 1

        time.sleep(0.3)
        self._execute_next_step(node)

    # ---------------------- Helpers ---------------------- #

    def _capture_image(self, node: Node, step_name: str):
        self.capture_count += 1
        print(f"\n  [拍摄] 第 {self.capture_count} 张图像，位于步骤 '{step_name}'")

        if self.latest_frame is None:
            print("   [跳过] 尚未收到相机帧")
            return

        try:
            import cv2 as cv
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"{step_name}_{timestamp}.png"
            filepath = os.path.join(self.output_dir, filename)
            bgr = cv.cvtColor(self.latest_frame, cv.COLOR_RGB2BGR)
            if cv.imwrite(filepath, bgr):
                print(f"   [保存成功] {filepath}")
            else:
                print("   [保存失败] 写入图像文件失败")
        except ImportError:
            print("   [跳过] OpenCV 未安装")
        except Exception as e:
            print(f"   [拍摄错误] {e}")

    def _request_plan(self, node: Node, start_joints: np.ndarray, goal_joints: np.ndarray):
        request = {
            "start": np.asarray(start_joints, dtype=float).tolist(),
            "goal": np.asarray(goal_joints, dtype=float).tolist(),
            "planner": "rrt_connect",
            "max_time": 5.0
        }
        request_bytes = json.dumps(request).encode("utf-8")
        node.send_output(
            "plan_request",
            pa.array(list(request_bytes), type=pa.uint8())
        )

    def _send_robot_state(self, node: Node, joints: np.ndarray):
        state = {"joints": np.asarray(joints, dtype=float).tolist()}
        state_bytes = json.dumps(state).encode("utf-8")
        node.send_output(
            "robot_state",
            pa.array(list(state_bytes), type=pa.uint8())
        )


if __name__ == "__main__":
    node = MultiViewCaptureNode()
    node.run()
