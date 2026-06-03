"""Unit tests for the OMPL-free linear planner core (no dora/ompl needed)."""
import numpy as np

from dora_moveit.motion_planner.planner_simple_op import path_length, plan_linear


def test_endpoints_are_exact():
    start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goal = [1.0, -1.0, 0.5, -0.5, 0.2, 0.0]
    traj = plan_linear(start, goal, num_waypoints=20)
    assert len(traj) == 20
    assert np.allclose(traj[0], start)
    assert np.allclose(traj[-1], goal)


def test_waypoints_are_monotonic_linear():
    traj = plan_linear([0.0], [1.0], num_waypoints=11)
    vals = [float(w[0]) for w in traj]
    assert vals == sorted(vals)
    assert abs(vals[5] - 0.5) < 1e-9  # midpoint


def test_path_length_of_straight_line_equals_endpoint_distance():
    start = np.zeros(6)
    goal = np.array([3.0, 4.0, 0.0, 0.0, 0.0, 0.0])  # distance 5
    traj = plan_linear(start, goal, num_waypoints=50)
    assert abs(path_length(traj) - 5.0) < 1e-6


def test_minimum_two_waypoints():
    traj = plan_linear([0.0, 0.0], [1.0, 1.0], num_waypoints=1)
    assert len(traj) == 2
