# Chapter 8: Final Demo & Debug

## ROS2 vs Dora Comparison

| Feature | ROS2 MoveIt2 | Dora-MoveIt2 | Status |
|---------|-------------|--------------|--------|
| Demo launch | `ros2 launch` with Gazebo/RViz | `dora start` with MuJoCo | ✅ |
| Debugging | ROS2 CLI tools, RViz | Print logging, MuJoCo viewer | ⚠️ |
| TF debugging | `ros2 run tf2_tools view_frames` | N/A | ❌ |
| Topic inspection | `ros2 topic echo` | Dora logging | ⚠️ |
| Parameter tuning | Dynamic reconfigure | Restart with new env vars | ⚠️ |
| Dual-arm demo | Two MoveGroupInterface instances | DualMoveGroup API | ❌ New |

## Running the Dual-Arm Demo

```bash
pip install -e dora_moveit/
pip install -e examples/dual_gen72/
cd examples/dual_gen72
dora up
dora start dataflows/dual_gen72_mujoco.yml
```

## Debug Tips

1. **Check joint positions**: Each operator prints joint state on receipt
2. **Planning failures**: Planner prints collision details and iteration counts
3. **IK failures**: IK operator prints target position and error magnitude
4. **Execution stuck**: Check `execution_status` output for progress
5. **MuJoCo viewer**: Use built-in viewer controls to inspect robot state

## Verification Checklist

- [ ] Both arms visible in MuJoCo viewer
- [ ] Both arms move independently
- [ ] Inter-arm collision avoidance works
- [ ] Synchronized handoff demo completes
- [ ] Ball ends up on plate
