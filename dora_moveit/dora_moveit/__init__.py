"""
dora-moveit: Robot motion planning library for the dora-rs dataflow framework.

Provides reusable operators for IK solving, motion planning, trajectory
execution, and collision detection. Works with any robot arm — configure
via the ROBOT_CONFIG_MODULE environment variable.

Usage:
    from dora_moveit.workflow.move_group import MoveGroup

    group = MoveGroup()
    group.set_named_target("home")
    group.go(wait=True)
"""

__version__ = "0.1.0"
