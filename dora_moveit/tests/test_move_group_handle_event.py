"""handle_event dispatches a single externally-pulled event to MoveGroup's handlers."""
import numpy as np

from dora_moveit.workflow.move_group import MoveGroup


class _FakeArrow:
    def __init__(self, arr):
        self._arr = arr

    def to_numpy(self):
        return self._arr


class FakeNode:
    def __init__(self, raw_joints):
        self._events = [
            {"type": "INPUT", "id": "joint_positions",
             "value": _FakeArrow(np.array(raw_joints, dtype=float))}
        ]

    def next(self, timeout=0.1):
        return self._events.pop(0) if self._events else None


def test_handle_event_updates_joint_state():
    mg = MoveGroup("ur5e", node=FakeNode([0.0] * 16))
    n = mg._num_joints
    # Feed a fresh joint update directly (as the SPEC vendor node will, from its loop).
    raw = [0.0] * 16
    for i in range(n):
        raw[mg._arm_qpos_start + i] = 0.1 * (i + 1)
    mg.handle_event({"type": "INPUT", "id": "joint_positions",
                     "value": _FakeArrow(np.array(raw, dtype=float))})
    expected = [round(0.1 * (i + 1), 3) for i in range(n)]
    assert [round(float(v), 3) for v in mg._current_joints] == expected


def test_handle_event_ignores_non_input_and_sets_stopped_on_stop():
    mg = MoveGroup("ur5e", node=FakeNode([0.0] * 16))
    mg.handle_event({"type": "STOP"})  # must not raise
    assert mg._stopped is True
