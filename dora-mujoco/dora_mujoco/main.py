"""MuJoCo simulation node for Dora with robot descriptions support.

Extended for Phase 6: LiDAR raycasting, IMU sensor, ground truth pose output.
"""

import numpy as np
import pyarrow as pa
import mujoco
import mujoco.viewer
from dora import Node
import json
import math
import os
import struct
from pathlib import Path
import time
from typing import Dict, Any, Optional
from robot_descriptions.loaders.mujoco import load_robot_description


class LidarSensor:
    """LiDAR raycasting sensor for MuJoCo — matches dora-nav C++ LidarSimulator format."""

    def __init__(self, model, data):
        self.model = model
        self.data = data

        # Configuration from environment
        self.horizontal_rays = int(os.getenv("LIDAR_RAYS", "360"))
        self.vertical_beams = int(os.getenv("LIDAR_VERTICAL_BEAMS", "1"))
        self.max_range = float(os.getenv("LIDAR_RANGE", "100.0"))
        self.min_range = float(os.getenv("LIDAR_MIN_RANGE", "0.5"))
        self.noise_stddev = 0.01
        self.add_noise = os.getenv("LIDAR_ADD_NOISE", "1") != "0"
        self.vertical_fov = 30.0  # degrees, -15 to +15
        self.seq = 0

        # Find LiDAR body
        self.body_id = -1
        lidar_name = os.getenv("LIDAR_BODY_NAME", "lidar")
        for i in range(model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name and lidar_name in name:
                self.body_id = i
                print(f"  [LiDAR] Found body '{name}' (id={i})")
                break
        if self.body_id < 0:
            print(f"  [LiDAR] WARNING: body '{lidar_name}' not found, using body 1")
            self.body_id = 1

        # Pre-compute vertical angles
        if self.vertical_beams > 1:
            half_fov = math.radians(self.vertical_fov / 2.0)
            self.v_angles = np.linspace(-half_fov, half_fov, self.vertical_beams)
        else:
            self.v_angles = np.array([0.0])

        # Pre-compute horizontal angles
        self.h_angles = np.linspace(0, 2 * math.pi, self.horizontal_rays, endpoint=False)

        print(f"  [LiDAR] {self.horizontal_rays} rays × {self.vertical_beams} beams, "
              f"range [{self.min_range}, {self.max_range}]m")

    def generate_pointcloud(self) -> bytes:
        """Generate LiDAR point cloud via MuJoCo raycasting.

        Returns binary data matching dora-nav format:
        [seq(u32)][pad(u32)][timestamp_us(u64)] + N × [x,y,z,intensity as float32]
        """
        lidar_pos = self.data.xpos[self.body_id].copy()
        lidar_mat = self.data.xmat[self.body_id].reshape(3, 3)

        geomid = np.zeros(1, dtype=np.int32)
        points = []

        for h_angle in self.h_angles:
            cos_h = math.cos(h_angle)
            sin_h = math.sin(h_angle)
            for v_angle in self.v_angles:
                cos_v = math.cos(v_angle)
                sin_v = math.sin(v_angle)

                # Ray direction in LiDAR local frame
                ray_local = np.array([cos_v * cos_h, cos_v * sin_h, sin_v])

                # Transform to world frame
                ray_world = lidar_mat @ ray_local

                # Raycast
                dist = mujoco.mj_ray(
                    self.model, self.data,
                    lidar_pos, ray_world,
                    None, 1, self.body_id, geomid
                )

                if dist > 0 and self.min_range <= dist <= self.max_range:
                    if self.add_noise:
                        dist += np.random.normal(0, self.noise_stddev)
                    # Point in LiDAR local frame
                    x = float(dist * ray_local[0])
                    y = float(dist * ray_local[1])
                    z = float(dist * ray_local[2])
                    intensity = float(100.0 / (1.0 + 0.01 * dist * dist))
                    points.append((x, y, z, intensity))

        # Serialize: 16-byte header + N × 16 bytes
        self.seq += 1
        timestamp_us = int(time.time() * 1e6)
        header = struct.pack('<IIQ', self.seq, 0, timestamp_us)
        point_data = b''.join(struct.pack('<ffff', *p) for p in points)
        return header + point_data


class IMUSensor:
    """IMU sensor — matches dora-nav C++ IMUSimulator format."""

    def __init__(self, model, data):
        self.model = model
        self.data = data

        self.accel_noise_stddev = 0.01
        self.gyro_noise_stddev = 0.001
        self.add_noise = os.getenv("IMU_ADD_NOISE", "1") != "0"
        self.gravity = 9.81

        # Find robot body for IMU
        self.body_id = -1
        robot_name = os.getenv("ROBOT_BODY_NAME", "hunter_base")
        for i in range(model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name and (robot_name in name or "chassis" in (name or "") or "base" in (name or "")):
                self.body_id = i
                print(f"  [IMU] Found body '{name}' (id={i})")
                break
        if self.body_id < 0:
            self.body_id = 1
            print(f"  [IMU] WARNING: body not found, using body 1")

        print(f"  [IMU] noise={'on' if self.add_noise else 'off'}")

    def generate_imu(self) -> bytes:
        """Generate IMU reading.

        Returns 32 bytes: [stamp(f64)] [ax,ay,az(f32)] [gx,gy,gz(f32)]
        """
        body_mat = self.data.xmat[self.body_id].reshape(3, 3)
        body_mat_T = body_mat.T  # world→body rotation

        # Linear acceleration (world frame) from cacc
        cacc = self.data.cacc[self.body_id]
        world_accel = np.array([cacc[0], cacc[1], cacc[2]])
        world_accel[2] += self.gravity  # add gravity
        body_accel = body_mat_T @ world_accel

        # Angular velocity (world frame) from cvel
        cvel = self.data.cvel[self.body_id]
        world_angvel = np.array([cvel[3], cvel[4], cvel[5]])
        body_angvel = body_mat_T @ world_angvel

        if self.add_noise:
            body_accel += np.random.normal(0, self.accel_noise_stddev, 3)
            body_angvel += np.random.normal(0, self.gyro_noise_stddev, 3)

        stamp = time.time()
        return struct.pack('<d6f',
                           stamp,
                           float(body_accel[0]), float(body_accel[1]), float(body_accel[2]),
                           float(body_angvel[0]), float(body_angvel[1]), float(body_angvel[2]))


class GraspWeld:
    """Generic, opt-in 'attach object on gripper close' for sim grasping.

    Some grippers (e.g. a small single rotating jaw) can't form a stable top-down
    rigid grasp in MuJoCo — the jaw plows the object instead of clamping it. When
    GRASP_WELD=1 and the model declares a <weld> equality (used here only to name the
    two bodies), this helper attaches the object to the gripper the moment the gripper
    actuator is commanded closed and the bodies are within reach, and releases it on
    open.

    The attach is KINEMATIC, not a constraint: while grasped, the object's free joint
    is driven directly to the gripper pose ∘ (captured relative pose) every step, and
    its velocity is zeroed. This is deliberately NOT the MuJoCo weld equality — a weld
    with a small relpose residual fights the solver during a multi-waypoint carry and
    can blow the physics up (NaN -> the sim node dies and the dataflow restart-loops).
    Kinematic attach has no solver forces, so it is rock-solid through the carry.

    All robot-specifics come from env (so the sim node stays generic):
      GRASP_WELD=1                enable
      GRASP_WELD_EQ=grasp_weld    name of the <weld> equality (names body1/body2)
      GRASP_CTRL_INDEX=5          actuator index of the gripper
      GRASP_CLOSE_CTRL=-0.12      gripper ctrl value that means "closed"
      GRASP_OPEN_CTRL=1.5         gripper ctrl value that means "open"
      GRASP_PROXIMITY=0.12        max body1<->body2 distance (m) to allow latching
    """

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.enabled = os.getenv("GRASP_WELD", "0") == "1"
        if not self.enabled:
            return
        eq_name = os.getenv("GRASP_WELD_EQ", "grasp_weld")
        self.eq_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, eq_name)
        if self.eq_id < 0:
            print(f"  [GraspWeld] WARNING: equality '{eq_name}' not in model — disabled")
            self.enabled = False
            return
        self.grip_idx = int(os.getenv("GRASP_CTRL_INDEX", str(max(0, model.nu - 1))))
        self.close_ctrl = float(os.getenv("GRASP_CLOSE_CTRL", "-0.12"))
        self.open_ctrl = float(os.getenv("GRASP_OPEN_CTRL", "1.5"))
        self.proximity = float(os.getenv("GRASP_PROXIMITY", "0.12"))
        # GRASP_SNAP_AXIAL=1: on latch, snap the object onto the gripper's approach axis
        # (zero the two lateral components of the captured relative offset, keep the axial
        # one). Use when the arm grasps slightly off-centre and that lateral error would
        # otherwise be frozen into the carry, throwing off the place (e.g. ur5e ~8 cm).
        self.snap_axial = os.getenv("GRASP_SNAP_AXIAL", "0") == "1"
        # midpoint decides closed vs open regardless of which extreme is "closed"
        self._mid = 0.5 * (self.close_ctrl + self.open_ctrl)
        self._close_is_low = self.close_ctrl < self.open_ctrl
        # weld bodies (eq_obj1/2 are body ids for a weld): b1=gripper, b2=object
        self.b1 = int(model.eq_obj1id[self.eq_id])
        self.b2 = int(model.eq_obj2id[self.eq_id])
        # object must be a free body; resolve its free-joint qpos/qvel addresses so we
        # can drive its pose directly while grasped.
        jadr = int(model.body_jntadr[self.b2])
        self.obj_qadr = int(model.jnt_qposadr[jadr]) if jadr >= 0 else -1
        self.obj_dofadr = int(model.jnt_dofadr[jadr]) if jadr >= 0 else -1
        if jadr < 0 or model.jnt_type[jadr] != mujoco.mjtJoint.mjJNT_FREE:
            print(f"  [GraspWeld] WARNING: body2 has no free joint — disabled")
            self.enabled = False
            return
        self.active = False
        self._rel_pos = None   # object origin in gripper frame, captured at latch
        self._rel_quat = None  # object orientation in gripper frame
        print(f"  [GraspWeld] enabled (kinematic): eq='{eq_name}' grip_ctrl[{self.grip_idx}] "
              f"close={self.close_ctrl} open={self.open_ctrl} prox={self.proximity}m "
              f"bodies=({self.b1},{self.b2})")

    def _commanded_closed(self) -> bool:
        c = self.data.ctrl[self.grip_idx]
        return c <= self._mid if self._close_is_low else c >= self._mid

    def update(self):
        """Called every physics step, before mj_step."""
        if not self.enabled:
            return
        closed = self._commanded_closed()
        if closed and not self.active:
            d = float(np.linalg.norm(self.data.xpos[self.b1] - self.data.xpos[self.b2]))
            if d <= self.proximity:
                # capture the object's current pose relative to the gripper frame
                p1 = self.data.xpos[self.b1]
                R1 = self.data.xmat[self.b1].reshape(3, 3)
                q1 = self.data.xquat[self.b1]
                p2 = self.data.xpos[self.b2]
                q2 = self.data.xquat[self.b2]
                self._rel_pos = R1.T @ (p2 - p1)
                if self.snap_axial:
                    # keep only the gripper's approach axis (largest |component|), zero the
                    # lateral offset so the object is held on the gripper centreline (at the
                    # TCP) instead of wherever an off-centre grasp happened to clamp it.
                    axis = int(np.argmax(np.abs(self._rel_pos)))
                    snapped = np.zeros(3)
                    snapped[axis] = self._rel_pos[axis]
                    self._rel_pos = snapped
                self._rel_quat = np.zeros(4)
                qneg = np.array([q1[0], -q1[1], -q1[2], -q1[3]])
                mujoco.mju_mulQuat(self._rel_quat, qneg, q2)
                self.active = True
                print(f"  [GraspWeld] latched (dist={d*1000:.0f}mm)")
        elif (not closed) and self.active:
            self.active = False
            print("  [GraspWeld] released")

        if self.active:
            # kinematically drive the object to gripper_pose ∘ rel_pose, zero its velocity
            p1 = self.data.xpos[self.b1]
            R1 = self.data.xmat[self.b1].reshape(3, 3)
            q1 = self.data.xquat[self.b1]
            self.data.qpos[self.obj_qadr:self.obj_qadr + 3] = p1 + R1 @ self._rel_pos
            cq = np.zeros(4)
            mujoco.mju_mulQuat(cq, q1, self._rel_quat)
            self.data.qpos[self.obj_qadr + 3:self.obj_qadr + 7] = cq
            self.data.qvel[self.obj_dofadr:self.obj_dofadr + 6] = 0.0


class MuJoCoSimulator:
    """MuJoCo simulator for Dora."""

    def __init__(self, model_path_or_name: str = None):
        """Initialize the MuJoCo simulator."""
        # Check environment variable first, then use parameter, then default
        self.model_path_or_name = (
            os.getenv("MODEL_NAME") or 
            model_path_or_name or 
            "go2_mj_description"
        )
        
        self.model = None
        self.data = None
        self.viewer = None
        self.state_data = {}

        self.last_cmd_time = time.time()
        self.cmd_timeout = 0.2
        self.target_q = None

        # NAV_MODE disables virtual springs (dora-nav handles path following)
        self.nav_mode = os.getenv("NAV_MODE", "0") == "1"

        self.load_model()
        
        print(f"MuJoCo Simulator initialized with model: {self.model_path_or_name}")

    def load_model(self) -> bool:
        """Load MuJoCo model from path or robot description name."""
        model_path = Path(self.model_path_or_name)
        if model_path.exists() and model_path.suffix in ['.xml', '.urdf']:
            print(f"Loading model from direct path: {model_path}")
            self.model = mujoco.MjModel.from_xml_path(str(model_path))
        else:
            self.model = load_robot_description(self.model_path_or_name, variant="scene")

        # Add damping to hinge joints; for freejoint add yaw damping only
        dof_idx = 0
        for j in range(self.model.njnt):
            jnt_type = self.model.jnt_type[j]
            if jnt_type == 0:  # mjJNT_FREE: 6 DOF (tx,ty,tz,rx,ry,rz)
                # Lock heading: high yaw(rz) + roll(rx) + pitch(ry) damping
                self.model.dof_damping[dof_idx + 3] = 500.0  # rx (roll)
                self.model.dof_damping[dof_idx + 4] = 500.0  # ry (pitch)
                self.model.dof_damping[dof_idx + 5] = 500.0  # rz (yaw)
                dof_idx += 6
            elif jnt_type == 1:  # mjJNT_BALL: 3 DOF
                for k in range(3):
                    self.model.dof_damping[dof_idx + k] = 10.0
                dof_idx += 3
            else:  # mjJNT_HINGE or mjJNT_SLIDE: 1 DOF
                self.model.dof_damping[dof_idx] = 10.0
                dof_idx += 1

        # Initialize simulation data
        self.data = mujoco.MjData(self.model)

        # Set num_joints from model
        self.num_joints = self.model.nu

        # Set control to neutral position
        if self.model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        else:
            mujoco.mj_resetData(self.model, self.data)

            # Set initial safe configuration for LM3 (6-DOF)
            if self.num_joints == 6:
                safe_config = np.array([0.0, -1.57, 1.57, 0.0, 0.0, 0.0])
                for i in range(self.num_joints):
                    self.data.qpos[i] = safe_config[i]
                    self.data.ctrl[i] = safe_config[i]
                print("  Initialized with LM3 SAFE_CONFIG")
            # For other robots, keep zero configuration

        # Forward kinematics to update positions
        mujoco.mj_forward(self.model, self.data)
        
        # Print model info for debugging
        print("Model loaded successfully:")
        print(f"  DOF (nq): {self.model.nq}")
        print(f"  Velocities (nv): {self.model.nv}")
        print(f"  Actuators (nu): {self.model.nu}")
        print(f"  Control inputs: {len(self.data.ctrl)}")
        
        # Print actuator info
        if self.model.nu > 0:
            print("Actuators:")
            for i in range(self.model.nu):
                actuator_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                joint_id = self.model.actuator_trnid[i, 0]  # First transmission joint
                joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) if joint_id >= 0 else "N/A"
                ctrl_range = self.model.actuator_ctrlrange[i]
                print(f"  [{i}] {actuator_name or f'actuator_{i}'} -> joint: {joint_name} | range: [{ctrl_range[0]:.2f}, {ctrl_range[1]:.2f}]")
            
        # Optional sim-grasp weld (no-op unless GRASP_WELD=1 and the model declares it)
        self.grasp_weld = GraspWeld(self.model, self.data)

        # Initialize state data
        self._update_state_data()
        return True

    def apply_control(self, control_input: np.ndarray):
        """Apply control input to the simulation.

        Args:
            control_input: Control values for actuators or joint positions

        """
        if control_input is None or len(control_input) == 0:
            return

        self.last_cmd_time = time.time()
        n_arm = min(len(control_input), self.num_joints)
        self.target_q = control_input[:n_arm].copy()

        # If no actuators, directly set joint positions (position control)
        if self.model.nu == 0:
            n_joints = min(len(control_input), self.model.nq)
            for i in range(n_joints):
                self.data.qpos[i] = control_input[i]
            return

        # Ensure we don't exceed the number of actuators
        n_controls = min(len(control_input), self.model.nu)

        # Apply control directly to actuators
        for i in range(n_controls):
            # Apply joint limits if available
            ctrl_range = self.model.actuator_ctrlrange[i]
            if ctrl_range[0] < ctrl_range[1]:  # Valid range
                control_value = np.clip(control_input[i], ctrl_range[0], ctrl_range[1])
            else:
                control_value = control_input[i]

            self.data.ctrl[i] = control_value

    def _get_available_models(self) -> dict:
        """Get available models from the mapping file."""
        config_path = Path(__file__).parent / "robot_models.json"
        with open(config_path) as f:
            return json.load(f)

    def step_simulation(self):
        """Step the simulation forward."""
        # Only update arm ctrl when actively receiving commands.
        # Position actuators naturally hold the last commanded position,
        # so no need to read qpos (which has wrong indices for freejoint models).
        if self.target_q is not None:
            now = time.time()
            if now - self.last_cmd_time <= self.cmd_timeout:
                n = min(len(self.target_q), self.model.nu)
                self.data.ctrl[:n] = self.target_q[:n]

        # Virtual spring on freejoint yaw to keep a mobile CAR BASE facing straight.
        # Disabled in NAV_MODE (dora-nav lateral controller handles path following).
        # Set DISABLE_BASE_SPRING=1 when the first freejoint is a LIGHT free object that
        # the spring would destabilize: e.g. the ur5e scene's free ball, where -2000*yaw
        # explodes its rotational DOF ("Nan/Inf/huge value in QACC at DOF 3") so MuJoCo
        # auto-resets the whole sim to qpos0 every few ms — silently pinning the arm at
        # zero config so it never reaches the object. (The so101/rebot cube/box sit at
        # y=0 with ~zero spring force, so they leave the spring on.)
        if (os.getenv("DISABLE_BASE_SPRING", "0") != "1"
                and not self.nav_mode and self.model.njnt > 0 and self.model.jnt_type[0] == 0):
            # Yaw spring: extract yaw from quaternion (qpos[3:7])
            qw, qx, qy, qz = self.data.qpos[3], self.data.qpos[4], self.data.qpos[5], self.data.qpos[6]
            yaw = 2.0 * np.arctan2(qw * qz + qx * qy, 1.0 - 2.0 * (qy * qy + qz * qz))
            self.data.qfrc_applied[5] = -2000.0 * yaw   # rz: strong spring to hold heading
            self.data.qfrc_applied[1] = -200.0 * self.data.qpos[1]  # ty: gentle spring to center Y

        # Latch/release the optional grasp weld from the commanded gripper ctrl
        if getattr(self, "grasp_weld", None) is not None:
            self.grasp_weld.update()

        mujoco.mj_step(self.model, self.data)
        self._update_state_data()
    
    def _update_state_data(self):
        """Update state data that can be sent via Dora."""
        self.state_data = {
            "time": self.data.time,                    # Current simulation time
            "qpos": self.data.qpos.copy(),            # Joint positions  
            "qvel": self.data.qvel.copy(),            # Joint velocities
            "qacc": self.data.qacc.copy(),            # Joint accelerations
            "ctrl": self.data.ctrl.copy(),            # Control inputs/actuator commands
            "qfrc_applied": self.data.qfrc_applied.copy(),  # External forces applied to joints
            "sensordata": self.data.sensordata.copy() if self.model.nsensor > 0 else np.array([])  # Sensor readings
        }
    
    def get_state(self) -> Dict[str, Any]:
        """Get current simulation state."""
        return self.state_data.copy()

    def get_ground_truth_pose(self) -> bytes:
        """Get ground truth pose as Pose2D_h binary (12 bytes: float x, y, theta_degrees).

        Matches dora-nav C++ GroundTruthPose / Pose2D_h format.
        """
        if self.model.njnt > 0 and self.model.jnt_type[0] == 0:
            # Freejoint: qpos[0:3]=xyz, qpos[3:7]=quaternion(w,x,y,z)
            x = float(self.data.qpos[0])
            y = float(self.data.qpos[1])
            qw, qx, qy, qz = self.data.qpos[3], self.data.qpos[4], self.data.qpos[5], self.data.qpos[6]
            yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            theta_deg = float(yaw_rad * 180.0 / math.pi)
        else:
            x = float(self.data.qpos[0]) if self.model.nq > 0 else 0.0
            y = float(self.data.qpos[1]) if self.model.nq > 1 else 0.0
            theta_deg = float(self.data.qpos[2] * 180.0 / math.pi) if self.model.nq > 2 else 0.0
        return struct.pack('<fff', x, y, theta_deg)


def main():
    """Run the main Dora node function."""
    node = Node()

    # Initialize simulator
    simulator = MuJoCoSimulator()

    # Load model (called again but idempotent — already called in __init__)
    if not simulator.load_model():
        print("Failed to load MuJoCo model")
        return

    print("MuJoCo simulation node started")

    # Initialize sensors (only when NAV_MODE is enabled)
    nav_mode = os.getenv("NAV_MODE", "0") == "1"
    lidar_sensor: Optional[LidarSensor] = None
    imu_sensor: Optional[IMUSensor] = None
    if nav_mode:
        print("NAV_MODE enabled — initializing LiDAR + IMU sensors")
        lidar_sensor = LidarSensor(simulator.model, simulator.data)
        imu_sensor = IMUSensor(simulator.model, simulator.data)

    # Sensor output rates
    pointcloud_rate = float(os.getenv("POINTCLOUD_RATE", "10"))
    imu_rate = float(os.getenv("IMU_RATE", "20"))
    pointcloud_period = 1.0 / pointcloud_rate if pointcloud_rate > 0 else 999
    imu_period = 1.0 / imu_rate if imu_rate > 0 else 999
    last_pointcloud_time = 0.0
    last_imu_time = 0.0

    # Physics steps per event
    steps_per_tick = int(os.getenv("PHYSICS_STEPS", "5"))

    # Launch viewer (with headless fallback for macOS)
    headless = os.getenv("MUJOCO_HEADLESS", "").lower() in ("1", "true", "yes")
    viewer = None

    if not headless:
        try:
            viewer = mujoco.viewer.launch_passive(simulator.model, simulator.data)
            print("MuJoCo viewer launched")
        except RuntimeError as e:
            if "mjpython" in str(e):
                print(f"Viewer unavailable on macOS ({e}), falling back to headless mode")
            else:
                raise

    # Main Dora event loop
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "control_input":
                control_array = event["value"].to_numpy()
                simulator.apply_control(control_array)
            elif event["id"] == "wheel_control":
                wheel_cmd = event["value"].to_numpy()
                if simulator.model.nu >= 11:
                    if len(wheel_cmd) >= 4:
                        simulator.data.ctrl[7] = wheel_cmd[0]
                        simulator.data.ctrl[8] = wheel_cmd[1]
                        simulator.data.ctrl[9] = wheel_cmd[2]
                        simulator.data.ctrl[10] = wheel_cmd[3]
                    elif len(wheel_cmd) >= 2:
                        simulator.data.ctrl[7] = 0.0
                        simulator.data.ctrl[8] = 0.0
                        simulator.data.ctrl[9] = wheel_cmd[0]
                        simulator.data.ctrl[10] = wheel_cmd[1]

        # Step simulation
        for _ in range(steps_per_tick):
            simulator.step_simulation()
        if viewer is not None:
            viewer.sync()

        # Send outputs after stepping
        if event["type"] == "INPUT":
            state = simulator.get_state()
            sim_time = state.get("time", 0.0)

            node.send_output(
                "joint_positions",
                pa.array(state["qpos"]),
                {"timestamp": sim_time, "encoding": "jointstate"}
            )

            node.send_output(
                "joint_velocities",
                pa.array(state["qvel"]),
                {"timestamp": sim_time}
            )

            node.send_output(
                "actuator_controls",
                pa.array(state["ctrl"]),
                {"timestamp": sim_time}
            )

            if len(state["sensordata"]) > 0:
                node.send_output(
                    "sensor_data",
                    pa.array(state["sensordata"]),
                    {"timestamp": sim_time}
                )

            # --- NAV_MODE sensor outputs ---
            if nav_mode:
                pose_bytes = simulator.get_ground_truth_pose()
                node.send_output(
                    "ground_truth_pose",
                    pa.array(list(pose_bytes), type=pa.uint8()),
                    {"timestamp": sim_time}
                )

                if lidar_sensor and (sim_time - last_pointcloud_time) >= pointcloud_period:
                    pc_bytes = lidar_sensor.generate_pointcloud()
                    node.send_output(
                        "pointcloud",
                        pa.array(list(pc_bytes), type=pa.uint8()),
                        {"timestamp": sim_time}
                    )
                    last_pointcloud_time = sim_time

                if imu_sensor and (sim_time - last_imu_time) >= imu_period:
                    imu_bytes = imu_sensor.generate_imu()
                    node.send_output(
                        "imu_msg",
                        pa.array(list(imu_bytes), type=pa.uint8()),
                        {"timestamp": sim_time}
                    )
                    last_imu_time = sim_time


if __name__ == "__main__":
    main()