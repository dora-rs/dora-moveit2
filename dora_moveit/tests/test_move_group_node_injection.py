"""MoveGroup must accept an injected dora node (so an external owner can share it)."""
import numpy as np

from dora_moveit.workflow.move_group import MoveGroup


class _FakeArrow:
    def __init__(self, arr):
        self._arr = arr

    def to_numpy(self):
        return self._arr


class FakeNode:
    """Minimal dora-node stand-in: yields one joint_positions event then None."""

    def __init__(self, raw_joints):
        self._events = [
            {"type": "INPUT", "id": "joint_positions",
             "value": _FakeArrow(np.array(raw_joints, dtype=float))},
        ]

    def next(self, timeout=0.1):
        return self._events.pop(0) if self._events else None


def test_movegroup_accepts_injected_node_and_does_not_create_its_own():
    # Provide a generously-sized raw qpos vector so _extract_arm_joints works
    # regardless of the config's NUM_JOINTS / ARM_QPOS_START.
    fake = FakeNode([0.0] * 16)
    mg = MoveGroup("ur5e", node=fake)
    assert mg._node is fake  # used our node, not a freshly-created dora Node
    assert mg._current_joints is not None  # consumed the injected joint_positions
    assert len(mg._current_joints) == mg._num_joints
