"""Non-blocking motion: begin_motion_async fires plan (and async IK) without
re-entering node.next(); completion is observed via the plan/exec flags."""
import numpy as np

from dora_moveit.workflow.move_group import MoveGroup


class _FakeArrow:
    def __init__(self, arr):
        self._arr = arr

    def to_numpy(self):
        return self._arr

    def to_pylist(self):
        return list(self._arr)


class RecordingNode:
    """Feeds one initial joint_positions, then records every send_output."""

    def __init__(self, num_qpos=16):
        self.sent = []  # list of (output_id, value)
        self._events = [
            {"type": "INPUT", "id": "joint_positions",
             "value": _FakeArrow(np.zeros(num_qpos, dtype=float))}
        ]

    def next(self, timeout=0.1):
        return self._events.pop(0) if self._events else None

    def send_output(self, output_id, value, metadata=None):
        self.sent.append((output_id, value))

    def sent_ids(self):
        return [oid for (oid, _v) in self.sent]


def _mg():
    return MoveGroup("ur5e", node=RecordingNode())


def test_begin_motion_async_joint_sends_plan_request_and_returns():
    mg = _mg()
    mg._node.sent.clear()  # drop the initial joint_positions handshake
    mg.begin_motion_async([0.0, -1.0, 0.0, -1.0, 0.0, 0.0])
    # Fired a plan_request, did NOT send an ik_request, and did not block.
    assert "plan_request" in mg._node.sent_ids()
    assert "ik_request" not in mg._node.sent_ids()
    assert mg._pending_pose_plan is False


def test_begin_motion_async_named_sends_plan_request():
    mg = _mg()
    mg._node.sent.clear()
    mg.set_named_target("home")
    mg.begin_motion_async()
    assert "plan_request" in mg._node.sent_ids()
    assert "ik_request" not in mg._node.sent_ids()


def test_begin_motion_async_pose_defers_ik_then_plans_on_solution():
    mg = _mg()
    mg._node.sent.clear()
    mg.set_pose_target([0.4, 0.0, 0.3, 0.0, 0.0, 0.0])
    mg.begin_motion_async()
    # Pose path: ik_request now, NO plan_request yet, IK pending.
    assert mg._node.sent_ids() == ["ik_request"]
    assert mg._pending_pose_plan is True
    assert mg._plan_done is False
    # IK solution arrives -> plan_request is fired automatically.
    n = mg._num_joints
    sol = np.array([0.1] * n, dtype=np.float32)
    mg.handle_event({"type": "INPUT", "id": "ik_solution",
                     "value": _FakeArrow(sol)})
    assert mg._pending_pose_plan is False
    assert "plan_request" in mg._node.sent_ids()
    assert mg._target_pose is None


def test_begin_motion_async_pose_ik_failure_marks_plan_failed():
    mg = _mg()
    mg._node.sent.clear()
    mg.set_pose_target([9.9, 9.9, 9.9, 0.0, 0.0, 0.0])
    mg.begin_motion_async()
    status = {"success": False, "message": "out of reach"}
    raw = list(__import__("json").dumps(status).encode("utf-8"))
    mg.handle_event({"type": "INPUT", "id": "ik_status",
                     "value": _FakeArrow(np.array(raw, dtype=np.uint8))})
    # IK failure latches a failed plan so motion_status() will report "failed".
    assert mg._pending_pose_plan is False
    assert mg._plan_done is True
    assert mg._plan_success is False
    assert "reach" in mg._plan_message
    # No plan_request was ever sent.
    assert "plan_request" not in mg._node.sent_ids()
