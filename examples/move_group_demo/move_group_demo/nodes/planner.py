import os

# PLANNER_TYPE=simple selects the OMPL-free linear-interpolation planner
# (obstacle-free sim, no OMPL python bindings needed). Default: OMPL.
if os.environ.get("PLANNER_TYPE", "ompl").lower() == "simple":
    from dora_moveit.motion_planner.planner_simple_op import main
else:
    from dora_moveit.motion_planner.planner_ompl_with_collision_op import main

main()
