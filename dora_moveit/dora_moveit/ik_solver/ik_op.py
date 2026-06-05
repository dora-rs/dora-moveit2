#!/usr/bin/env python3
"""
Inverse Kinematics Operator for Dora-MoveIt
============================================

Converts end-effector pose to joint positions.
This is a pluggable IK solver - can be swapped with any IK implementation:
- Numerical IK (Jacobian-based)
- Analytical IK (robot-specific)
- Machine Learning IK
- External IK service

Input: target_pose (6D: x, y, z, roll, pitch, yaw) or (7D: x, y, z, qw, qx, qy, qz)
Output: joint_positions (7D for GEN72 robot)

Dora Integration:
- Receives: ik_request (pose), joint_state (current joints for seeding)
- Sends: ik_solution (joint positions), ik_status (success/failure)
"""

import json
import os
import numpy as np
import pyarrow as pa
from dataclasses import dataclass
from typing import Optional, Tuple, List
from dora import Node
from dora_moveit.config import load_config
from dora_moveit.config import is_dual_arm, get_arm_config
from dora_moveit.ik_solver.advanced_ik_solver import TracIKSolver, DifferentialEvolutionIKSolver, IKRequest, IKResult


class NumericalIKSolver:
    """
    Numerical IK solver using Jacobian pseudo-inverse method.
    
    This is a simplified IK solver for demonstration.
    In production, use specialized libraries like:
    - PyKDL
    - ikfast
    - pytorch-kinematics
    - pinocchio
    """
    
    def __init__(self, num_joints: int = 7, config=None):
        """
        Initialize IK solver.

        Args:
            num_joints: Number of robot joints
            config: Robot config object; if None, loads via load_config()
        """
        if config is None:
            config = load_config()
        self.num_joints = num_joints
        self.max_iterations = 500
        self.position_tolerance = 0.01
        self.orientation_tolerance = 1e-1
        self.step_size = 0.5

        # Joint limits
        self.joint_limits_lower = config.JOINT_LOWER_LIMITS
        self.joint_limits_upper = config.JOINT_UPPER_LIMITS

        # Link transforms from URDF
        self.link_transforms = config.LINK_TRANSFORMS

        # Safe/home configuration for IK seeding
        self._safe_config = config.SAFE_CONFIG

        # EE offset (if defined in config)
        self._ee_offset = getattr(config, 'EE_OFFSET', None)
        
    def forward_kinematics(self, joint_positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics using link transforms from config.
        Supports per-joint rotation axes.

        Args:
            joint_positions: Joint angles

        Returns:
            Tuple of (position [3], rotation_matrix [3x3])
        """
        q = joint_positions

        def rot_z(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

        def rot_y(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

        def rot_x(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

        def rot_axis(angle, axis):
            ax = np.array(axis, dtype=float)
            norm = np.linalg.norm(ax)
            if norm < 1e-10:
                return np.eye(3)
            ax = ax / norm
            if abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6:
                return rot_z(angle * np.sign(ax[2]))
            if abs(ax[0]) < 1e-6 and abs(ax[2]) < 1e-6:
                return rot_y(angle * np.sign(ax[1]))
            if abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6:
                return rot_x(angle * np.sign(ax[0]))
            K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
            return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

        T = np.eye(4)

        for i, (joint_angle, link_tf) in enumerate(zip(q, self.link_transforms)):
            xyz = np.array(link_tf["xyz"])
            rpy = link_tf["rpy"]
            axis = link_tf.get("axis", [0, 0, 1])

            R_link = rot_z(rpy[2]) @ rot_y(rpy[1]) @ rot_x(rpy[0])
            T_link = np.eye(4)
            T_link[:3, :3] = R_link
            T_link[:3, 3] = xyz

            T = T @ T_link

            T_joint = np.eye(4)
            T_joint[:3, :3] = rot_axis(joint_angle, axis)
            T = T @ T_joint

        # Apply EE offset if defined
        ee_offset = getattr(self, '_ee_offset', None)
        if ee_offset is not None:
            T_ee = np.eye(4)
            T_ee[:3, 3] = ee_offset
            T = T @ T_ee

        position = T[:3, 3]
        rotation = T[:3, :3]

        return position, rotation
    
    def compute_jacobian(self, joint_positions: np.ndarray) -> np.ndarray:
        """
        Compute Jacobian matrix numerically.
        
        Args:
            joint_positions: Current joint positions [7]
            
        Returns:
            Jacobian matrix [6 x 7]
        """
        jacobian = np.zeros((6, self.num_joints))
        delta = 1e-6
        
        pos0, rot0 = self.forward_kinematics(joint_positions)
        
        for i in range(self.num_joints):
            q_delta = joint_positions.copy()
            q_delta[i] += delta
            
            pos1, rot1 = self.forward_kinematics(q_delta)
            
            # Position Jacobian
            jacobian[:3, i] = (pos1 - pos0) / delta
            
            # Orientation Jacobian (simplified)
            jacobian[3:, i] = 0.0  # Placeholder for angular velocity
        
        return jacobian
    
    def solve(self, request: IKRequest) -> IKResult:
        """
        Solve inverse kinematics.
        
        Args:
            request: IK request with target pose
            
        Returns:
            IKResult with solution or failure info
        """
        target_pos = request.target_position
        
        # Initialize with seed or safe config
        if request.seed_joints is not None:
            q = request.seed_joints.copy()
        else:
            q = self._safe_config.copy()
        
        for iteration in range(self.max_iterations):
            # Ensure q is 1D with correct size
            q = np.asarray(q).flatten()[:self.num_joints]

            # Forward kinematics
            current_pos, current_rot = self.forward_kinematics(q)

            # Position error
            pos_error = target_pos - current_pos
            error_norm = np.linalg.norm(pos_error)

            # Check convergence
            if error_norm < self.position_tolerance:
                return IKResult(
                    success=True,
                    joint_positions=q,
                    error=error_norm,
                    iterations=iteration + 1,
                    message="IK converged successfully"
                )

            # Compute Jacobian
            J = self.compute_jacobian(q)[:3, :]  # Position only (3x7)

            # Damped least squares (Levenberg-Marquardt)
            damping = 0.001
            JJT = J @ J.T
            J_pinv = J.T @ np.linalg.inv(JJT + damping * np.eye(3))

            # Update joints
            dq = J_pinv @ pos_error
            q = q + self.step_size * dq

            # Apply joint limits
            q = np.clip(q, self.joint_limits_lower, self.joint_limits_upper)
        
        # Failed to converge
        current_pos, _ = self.forward_kinematics(q)
        final_error = np.linalg.norm(target_pos - current_pos)
        
        return IKResult(
            success=False,
            joint_positions=q,
            error=final_error,
            iterations=self.max_iterations,
            message=f"IK failed to converge. Error: {final_error:.6f}"
        )


class IKOperator:
    """
    Dora operator for Inverse Kinematics.

    Inputs:
        - ik_request: Target pose [x, y, z, roll, pitch, yaw] or [x, y, z, qw, qx, qy, qz]
        - joint_state: Current joint positions (for seeding)

    Outputs:
        - ik_solution: Joint positions if successful
        - ik_status: JSON with success/error info
    """

    def __init__(self, num_joints: int = 7, solver_type: str = "tracik"):
        """
        Initialize IK operator.

        Args:
            num_joints: Number of joints
            solver_type: "tracik" (default), "de" (differential evolution), or "simple"
        """
        if solver_type == "tracik":
            self.solver = TracIKSolver()
            print("[IK] Using TracIK solver (advanced)")
        elif solver_type == "de":
            self.solver = DifferentialEvolutionIKSolver()
            print("[IK] Using Differential Evolution solver")
        else:
            self.solver = NumericalIKSolver(num_joints)
            print("[IK] Using simple numerical solver")

        self.solver_type = solver_type
        self.current_joints: Optional[np.ndarray] = None
        self.request_count = 0
        
    def process_joint_state(self, joint_positions: np.ndarray):
        """Update current joint state for seeding"""
        self.current_joints = joint_positions.copy()
        
    def process_ik_request(self, pose_data: np.ndarray) -> Tuple[Optional[np.ndarray], dict]:
        """
        Process an IK request.
        
        Args:
            pose_data: Target pose array (6D or 7D)
            
        Returns:
            Tuple of (joint_solution or None, status_dict)
        """
        self.request_count += 1
        
        # Parse pose (6D, 7D, or 13D with seed joints)
        seed = self.current_joints
        if len(pose_data) == 13:
            # [x, y, z, roll, pitch, yaw, seed_j1..seed_j7]
            position = pose_data[:3]
            orientation = pose_data[3:6]
            orientation_type = "rpy"
            seed = pose_data[6:13].copy()
        elif len(pose_data) == 6:
            # [x, y, z, roll, pitch, yaw]
            position = pose_data[:3]
            orientation = pose_data[3:6]
            orientation_type = "rpy"
        elif len(pose_data) == 7:
            # [x, y, z, qw, qx, qy, qz]
            position = pose_data[:3]
            orientation = pose_data[3:7]
            orientation_type = "quaternion"
        else:
            return None, {
                "success": False,
                "error": f"Invalid pose length: {len(pose_data)}. Expected 6, 7, or 13",
                "request_id": self.request_count
            }

        # Create IK request
        request = IKRequest(
            target_position=position,
            target_orientation=orientation,
            seed_joints=seed,
            orientation_type=orientation_type
        )
        
        # Solve IK
        result = self.solver.solve(request)
        
        status = {
            "success": result.success,
            "error": float(result.error),
            "iterations": result.iterations,
            "message": result.message,
            "request_id": self.request_count,
            "target_position": position.tolist()
        }
        
        if result.success:
            return result.joint_positions, status
        else:
            return None, status


def main():
    """Main entry point for Dora IK operator"""
    print("=== Dora-MoveIt IK Operator ===")

    node = Node()
    config = load_config()
    # Solver selectable via IK_SOLVER_TYPE env (default "tracik" for back-compat).
    # "de" / "simple" use the pure-numpy NumericalIKSolver — no tracikpy needed.
    solver_type = os.environ.get("IK_SOLVER_TYPE", "tracik")
    ik_op = IKOperator(num_joints=config.NUM_JOINTS, solver_type=solver_type)

    print("IK operator started, waiting for requests...")
    
    for event in node:
        event_type = event["type"]
        
        if event_type == "INPUT":
            input_id = event["id"]
            
            if input_id == "joint_state":
                # Update current joint state for seeding
                joints = event["value"].to_numpy()
                ik_op.process_joint_state(joints)
                
            elif input_id == "ik_request":
                try:
                    value = event["value"]
                    # ik_request is either a uint8 JSON payload (dual-arm request)
                    # or a float32 pose array (single-arm). Dispatch on arrow type:
                    # a float array has to_pylist() too, but bytes(floats) crashes,
                    # so the float case must be checked first and decoded via numpy.
                    if hasattr(value, 'type') and pa.types.is_floating(value.type):
                        raw = value.to_numpy()
                    elif hasattr(value, 'to_pylist'):
                        raw = bytes(value.to_pylist())
                    elif hasattr(value, 'to_numpy'):
                        raw = value.to_numpy()
                    else:
                        raw = value

                    # Try JSON parse for dual-arm request
                    dual_request = None
                    if isinstance(raw, (bytes, bytearray)):
                        try:
                            dual_request = json.loads(raw.decode('utf-8'))
                        except (json.JSONDecodeError, UnicodeDecodeError):
                            pass

                    if dual_request and ("left_pose" in dual_request or "right_pose" in dual_request):
                        # Dual IK request
                        print(f"[IK] Dual request #{ik_op.request_count + 1}")
                        result = {"success": True}
                        combined_solution = []

                        for arm_key in ("left_pose", "right_pose"):
                            if arm_key in dual_request:
                                pose = np.array(dual_request[arm_key], dtype=np.float32)
                                solution, status = ik_op.process_ik_request(pose)
                                joint_key = arm_key.replace("_pose", "_joints")
                                if solution is not None:
                                    result[joint_key] = solution.tolist() if hasattr(solution, 'tolist') else list(solution)
                                    combined_solution.extend(result[joint_key])
                                else:
                                    result["success"] = False
                                    result["error"] = status.get("message", "IK failed")
                                    break

                        result_bytes = json.dumps(result).encode('utf-8')
                        node.send_output("ik_status", pa.array(list(result_bytes), type=pa.uint8()))

                        if result["success"] and combined_solution:
                            node.send_output(
                                "ik_solution",
                                pa.array(np.array(combined_solution, dtype=np.float32), type=pa.float32()),
                                metadata={"encoding": "jointstate", "success": True, "dual": True}
                            )
                            print(f"[IK] Dual SUCCESS")
                        else:
                            print(f"[IK] Dual FAILED: {result.get('error', 'unknown')}")
                    else:
                        # Single-arm IK request (original path)
                        if isinstance(raw, np.ndarray):
                            pose = raw
                        else:
                            pose = event["value"].to_numpy()

                        print(f"[IK] Request #{ik_op.request_count + 1}: target={pose[:3]}")

                        solution, status = ik_op.process_ik_request(pose)

                        status_bytes = json.dumps(status).encode('utf-8')
                        node.send_output("ik_status", pa.array(list(status_bytes), type=pa.uint8()))

                        if solution is not None:
                            node.send_output(
                                "ik_solution",
                                pa.array(solution, type=pa.float32()),
                                metadata={"encoding": "jointstate", "success": True}
                            )
                            print(f"[IK] SUCCESS: Solution found, error={status['error']:.6f}")
                        else:
                            print(f"[IK] FAILED: {status['message']}")
                except Exception as e:
                    print(f"[IK] Error processing request: {e}")
                    import traceback
                    traceback.print_exc()
                    
        elif event_type == "STOP":
            print("IK operator stopping...")
            break


if __name__ == "__main__":
    main()

