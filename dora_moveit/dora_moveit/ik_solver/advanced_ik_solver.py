#!/usr/bin/env python3
"""
Advanced IK Solver for Dora-MoveIt
===================================

Implements multiple IK solving strategies:
1. TracIK-inspired solver (combines Jacobian and optimization)
2. BFGS optimization-based solver
3. Multi-start random sampling

Significantly better than simple Jacobian pseudo-inverse.
"""

import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass
from scipy.optimize import minimize, differential_evolution
from dora_moveit.config import load_config


@dataclass
class IKRequest:
    """IK request containing target pose and optional seed"""
    target_position: np.ndarray  # [x, y, z]
    target_orientation: Optional[np.ndarray] = None  # [qw, qx, qy, qz] or [roll, pitch, yaw]
    seed_joints: Optional[np.ndarray] = None
    orientation_type: str = "quaternion"  # "quaternion" or "rpy"


@dataclass
class IKResult:
    """Result of IK computation"""
    success: bool
    joint_positions: np.ndarray
    error: float  # Position error
    iterations: int
    message: str = ""


class ForwardKinematics:
    """
    Forward kinematics using link transforms from robot config.
    Supports per-joint rotation axes (Z, Y, or X).
    """

    def __init__(self, config=None):
        if config is None:
            config = load_config()
        self.link_transforms = config.LINK_TRANSFORMS
        self.num_joints = config.NUM_JOINTS
        self.ee_offset = getattr(config, 'EE_OFFSET', None)

    @staticmethod
    def _rot_z(angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    @staticmethod
    def _rot_y(angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    @staticmethod
    def _rot_x(angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    def _rot_axis(self, angle, axis):
        """Rotate about an arbitrary unit axis using Rodrigues' formula."""
        ax = np.array(axis, dtype=float)
        norm = np.linalg.norm(ax)
        if norm < 1e-10:
            return np.eye(3)
        ax = ax / norm
        # Fast path for pure cardinal axes
        if abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6:
            return self._rot_z(angle * np.sign(ax[2]))
        if abs(ax[0]) < 1e-6 and abs(ax[2]) < 1e-6:
            return self._rot_y(angle * np.sign(ax[1]))
        if abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6:
            return self._rot_x(angle * np.sign(ax[0]))
        # General Rodrigues' rotation formula
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

    def compute_fk(self, joint_positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics.

        Args:
            joint_positions: Joint angles

        Returns:
            Tuple of (position [3], rotation_matrix [3x3])
        """
        q = joint_positions

        T = np.eye(4)

        for i, (joint_angle, link_tf) in enumerate(zip(q, self.link_transforms)):
            xyz = np.array(link_tf["xyz"])
            rpy = link_tf["rpy"]
            axis = link_tf.get("axis", [0, 0, 1])

            # Build rotation from RPY (Z-Y-X convention)
            R_link = self._rot_z(rpy[2]) @ self._rot_y(rpy[1]) @ self._rot_x(rpy[0])
            T_link = np.eye(4)
            T_link[:3, :3] = R_link
            T_link[:3, 3] = xyz

            T = T @ T_link

            # Apply joint rotation about the specified axis
            T_joint = np.eye(4)
            T_joint[:3, :3] = self._rot_axis(joint_angle, axis)
            T = T @ T_joint

        # Apply EE offset if defined
        if self.ee_offset is not None:
            T_ee = np.eye(4)
            T_ee[:3, 3] = self.ee_offset
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
            Jacobian matrix [6 x 7] (position + orientation)
        """
        n = self.num_joints
        jacobian = np.zeros((6, n))
        delta = 1e-6

        pos0, rot0 = self.compute_fk(joint_positions)

        for i in range(n):
            q_delta = joint_positions.copy()
            q_delta[i] += delta

            pos1, rot1 = self.compute_fk(q_delta)

            # Position Jacobian
            jacobian[:3, i] = (pos1 - pos0) / delta

            # Orientation Jacobian (angular velocity)
            # Approximate using rotation matrix difference
            dR = (rot1 - rot0) / delta
            omega = np.array([
                dR[2, 1],
                dR[0, 2],
                dR[1, 0]
            ])
            jacobian[3:, i] = omega

        return jacobian


class TracIKSolver:
    """
    TracIK-inspired solver combining Jacobian and optimization.

    TracIK uses a combination of:
    1. Sequential Quadratic Programming (SQP)
    2. Jacobian-based Newton-Raphson
    3. Random restarts

    This implementation provides similar functionality.
    """

    def __init__(self, config=None):
        if config is None:
            config = load_config()
        self.fk = ForwardKinematics(config=config)
        self.max_iterations = 1000
        self.position_tolerance = 0.001  # 1mm
        self.orientation_tolerance = 0.01
        self.joint_limits_lower = config.JOINT_LOWER_LIMITS
        self.joint_limits_upper = config.JOINT_UPPER_LIMITS
        self._safe_config = config.SAFE_CONFIG

    def _position_error(self, q: np.ndarray, target_pos: np.ndarray) -> float:
        """Compute position error"""
        current_pos, _ = self.fk.compute_fk(q)
        return np.linalg.norm(target_pos - current_pos)

    @staticmethod
    def _rpy_to_rot(rpy: np.ndarray) -> np.ndarray:
        r, p, y = rpy
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        return Rz @ Ry @ Rx

    def _resolve_target_rot(self, request: IKRequest) -> Optional[np.ndarray]:
        """Convert request orientation to 3x3 rotation matrix, or None."""
        ori = request.target_orientation
        if ori is None:
            return None
        ori = np.asarray(ori, dtype=float)
        if request.orientation_type == "rpy" and ori.shape == (3,):
            return self._rpy_to_rot(ori)
        if request.orientation_type == "matrix" and ori.shape == (3, 3):
            return ori
        # quaternion [qw, qx, qy, qz]
        if ori.shape == (4,):
            qw, qx, qy, qz = ori
            return np.array([
                [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
                [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
                [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)],
            ])
        return None

    def _objective_function(
        self,
        q: np.ndarray,
        target_pos: np.ndarray,
        target_rot: Optional[np.ndarray] = None
    ) -> float:
        current_pos, current_rot = self.fk.compute_fk(q)
        pos_error = np.linalg.norm(target_pos - current_pos)
        if target_rot is not None:
            rot_error = np.linalg.norm(target_rot - current_rot, 'fro')
            return pos_error + 0.1 * rot_error
        return pos_error

    def solve_jacobian(self, request: IKRequest) -> IKResult:
        target_pos = request.target_position
        target_rot = self._resolve_target_rot(request)

        if request.seed_joints is not None:
            q = request.seed_joints.copy()
        else:
            q = self._safe_config.copy()

        step_size = 0.5
        damping = 0.01

        for iteration in range(self.max_iterations):
            q = np.asarray(q).flatten()[:len(self.joint_limits_lower)]

            current_pos, current_rot = self.fk.compute_fk(q)
            pos_error = target_pos - current_pos
            error_norm = np.linalg.norm(pos_error)

            if target_rot is not None:
                # Orientation error as axis-angle vector: skew-symmetric part of R_err = R_target @ R_cur^T
                R_err = target_rot @ current_rot.T
                ori_error = np.array([R_err[2, 1] - R_err[1, 2],
                                      R_err[0, 2] - R_err[2, 0],
                                      R_err[1, 0] - R_err[0, 1]]) * 0.5
                ori_norm = np.linalg.norm(ori_error)
            else:
                ori_error = np.zeros(3)
                ori_norm = 0.0

            converged = error_norm < self.position_tolerance and ori_norm < self.orientation_tolerance
            if converged:
                return IKResult(
                    success=True,
                    joint_positions=q,
                    error=error_norm,
                    iterations=iteration + 1,
                    message="Jacobian IK converged"
                )

            J = self.fk.compute_jacobian(q)  # 6 x n
            task_error = np.concatenate([pos_error, ori_error])  # 6D
            JJT = J @ J.T
            J_pinv = J.T @ np.linalg.inv(JJT + damping * np.eye(6))

            dq = J_pinv @ task_error
            q = q + step_size * dq
            q = np.clip(q, self.joint_limits_lower, self.joint_limits_upper)

        current_pos, _ = self.fk.compute_fk(q)
        final_error = np.linalg.norm(target_pos - current_pos)

        return IKResult(
            success=False,
            joint_positions=q,
            error=final_error,
            iterations=self.max_iterations,
            message=f"Jacobian IK failed, error={final_error:.6f}"
        )

    def solve_optimization(self, request: IKRequest) -> IKResult:
        """
        Solve IK using BFGS optimization.

        Args:
            request: IK request

        Returns:
            IK result
        """
        target_pos = request.target_position

        if request.seed_joints is not None:
            q0 = request.seed_joints.copy()
        else:
            q0 = self._safe_config.copy()

        # Bounds for optimization
        bounds = [(low, high) for low, high in zip(
            self.joint_limits_lower,
            self.joint_limits_upper
        )]

        target_rot = self._resolve_target_rot(request)

        # Optimize
        result = minimize(
            fun=lambda q: self._objective_function(q, target_pos, target_rot),
            x0=q0,
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': 500, 'ftol': 1e-6}
        )

        error = self._position_error(result.x, target_pos)
        success = error < self.position_tolerance

        return IKResult(
            success=success,
            joint_positions=result.x,
            error=error,
            iterations=result.nit,
            message="BFGS optimization completed" if success else f"BFGS failed, error={error:.6f}"
        )

    def solve_multistart(self, request: IKRequest, num_attempts: int = 5) -> IKResult:
        """
        Solve IK using multiple random starts.

        Args:
            request: IK request
            num_attempts: Number of random restarts

        Returns:
            Best IK result
        """
        best_result = None
        best_error = float('inf')

        # Try with seed first
        if request.seed_joints is not None:
            result = self.solve_jacobian(request)
            if result.success:
                return result
            best_result = result
            best_error = result.error

        # Random restarts
        for i in range(num_attempts):
            # Generate random configuration
            random_q = np.random.uniform(
                self.joint_limits_lower,
                self.joint_limits_upper
            )

            request_copy = IKRequest(
                target_position=request.target_position,
                target_orientation=request.target_orientation,
                seed_joints=random_q,
                orientation_type=request.orientation_type
            )

            result = self.solve_jacobian(request_copy)

            if result.success:
                return result

            if result.error < best_error:
                best_error = result.error
                best_result = result

        return best_result

    def solve(self, request: IKRequest) -> IKResult:
        """
        Main solve method - tries multiple strategies.

        Strategy:
        1. Try Jacobian with seed
        2. Try BFGS optimization
        3. Try multi-start random sampling

        Args:
            request: IK request

        Returns:
            Best IK result
        """
        # Strategy 1: Jacobian with seed
        result = self.solve_jacobian(request)
        if result.success:
            return result

        # Strategy 2: BFGS optimization
        result_opt = self.solve_optimization(request)
        if result_opt.success:
            return result_opt

        # Strategy 3: Multi-start
        result_multi = self.solve_multistart(request, num_attempts=3)
        if result_multi.success:
            return result_multi

        # Return best result
        results = [result, result_opt, result_multi]
        best = min(results, key=lambda r: r.error)
        return best


class DifferentialEvolutionIKSolver:
    """
    IK solver using differential evolution (global optimization).
    Slower but more robust for difficult poses.
    """

    def __init__(self, config=None):
        if config is None:
            config = load_config()
        self.fk = ForwardKinematics(config=config)
        self.position_tolerance = 0.001
        self.joint_limits_lower = config.JOINT_LOWER_LIMITS
        self.joint_limits_upper = config.JOINT_UPPER_LIMITS

    def solve(self, request: IKRequest) -> IKResult:
        """
        Solve IK using differential evolution.

        Args:
            request: IK request

        Returns:
            IK result
        """
        target_pos = request.target_position

        bounds = [(low, high) for low, high in zip(
            self.joint_limits_lower,
            self.joint_limits_upper
        )]

        def objective(q):
            current_pos, _ = self.fk.compute_fk(q)
            return np.linalg.norm(target_pos - current_pos)

        result = differential_evolution(
            func=objective,
            bounds=bounds,
            maxiter=300,
            popsize=10,
            tol=1e-6,
            atol=1e-6
        )

        error = result.fun
        success = error < self.position_tolerance

        return IKResult(
            success=success,
            joint_positions=result.x,
            error=error,
            iterations=result.nit,
            message="Differential evolution completed" if success else f"DE failed, error={error:.6f}"
        )
