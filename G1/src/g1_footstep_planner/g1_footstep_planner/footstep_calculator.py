#!/usr/bin/env python3
"""
Footstep Calculator & Impulse Generator for Unitree G1.
Accounts for SDK velocity deadband thresholds (v_min > 0.1 m/s, w_min > 0.1 rad/s).
"""

import math
from typing import List, Tuple, Dict, Any


def normalize_angle(angle: float) -> float:
    """Normalize angle to range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class Footstep:
    """Represents a discrete footstep pose."""
    def __init__(self, foot: str, x: float, y: float, yaw: float, step_index: int):
        self.foot = foot  # 'LEFT' or 'RIGHT'
        self.x = x
        self.y = y
        self.yaw = yaw
        self.step_index = step_index

    def __repr__(self):
        return f"Footstep({self.foot}, step={self.step_index}, x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.1f}°)"


class ExecutionImpulse:
    """Represents a single velocity impulse phase."""
    def __init__(self, phase_name: str, vx: float, vy: float, wz: float, duration: float):
        self.phase_name = phase_name
        self.vx = vx
        self.vy = vy
        self.wz = wz
        self.duration = duration

    def __repr__(self):
        return f"Impulse({self.phase_name}: vx={self.vx:.2f}, vy={self.vy:.2f}, wz={self.wz:.2f}, T={self.duration:.2f}s)"


class FootstepCalculator:
    """
    Calculates 3-phase execution impulses and footstep sequences for Unitree G1.
    Handles velocity deadband (v_min >= 0.10 m/s, w_min >= 0.10 rad/s).
    """

    def __init__(
        self,
        vx_exec: float = 0.12,
        vy_exec: float = 0.12,
        wz_exec: float = 0.12,
        v_min: float = 0.10,
        w_min: float = 0.10,
        max_stride_x: float = 0.10,
        max_stride_y: float = 0.05,
        max_stride_yaw: float = 0.12,
        stance_width: float = 0.20,
    ):
        self.vx_exec = abs(vx_exec)
        self.vy_exec = abs(vy_exec)
        self.wz_exec = abs(wz_exec)
        self.v_min = abs(v_min)
        self.w_min = abs(w_min)
        self.max_stride_x = max_stride_x
        self.max_stride_y = max_stride_y
        self.max_stride_yaw = max_stride_yaw
        self.stance_width = stance_width

        # Ensure execution velocities are strictly above minimum deadband
        if self.vx_exec <= self.v_min:
            self.vx_exec = self.v_min + 0.02
        if self.vy_exec <= self.v_min:
            self.vy_exec = self.v_min + 0.02
        if self.wz_exec <= self.w_min:
            self.wz_exec = self.w_min + 0.02

    def calculate_relative_pose(
        self, curr_x: float, curr_y: float, curr_yaw: float,
        goal_x: float, goal_y: float, goal_yaw: float
    ) -> Tuple[float, float, float]:
        """
        Transforms goal pose in global frame to local robot frame (dx, dy, dyaw).
        """
        dx_global = goal_x - curr_x
        dy_global = goal_y - curr_y

        cos_yaw = math.cos(curr_yaw)
        sin_yaw = math.sin(curr_yaw)

        dx_local = cos_yaw * dx_global + sin_yaw * dy_global
        dy_local = -sin_yaw * dx_global + cos_yaw * dy_global
        dyaw_local = normalize_angle(goal_yaw - curr_yaw)

        return dx_local, dy_local, dyaw_local

    def plan_impulses(self, dx: float, dy: float, dyaw: float) -> List[ExecutionImpulse]:
        """
        Generates 3-phase execution impulses (Yaw -> Strafe -> Forward/Backward).
        Guarantees all active velocities exceed SDK deadband (0.1 m/s / 0.1 rad/s).
        """
        impulses: List[ExecutionImpulse] = []

        # Phase 1: Yaw Alignment
        if abs(dyaw) > 0.01:  # > ~0.5 degrees
            wz_cmd = self.wz_exec if dyaw > 0 else -self.wz_exec
            duration_yaw = abs(dyaw) / self.wz_exec
            impulses.append(ExecutionImpulse("1_YAW", 0.0, 0.0, wz_cmd, duration_yaw))

        # Phase 2: Lateral Strafe
        if abs(dy) > 0.005:  # > 5mm
            vy_cmd = self.vy_exec if dy > 0 else -self.vy_exec
            duration_y = abs(dy) / self.vy_exec
            impulses.append(ExecutionImpulse("2_STRAFE", 0.0, vy_cmd, 0.0, duration_y))

        # Phase 3: Forward / Backward
        if abs(dx) > 0.005:  # > 5mm
            vx_cmd = self.vx_exec if dx > 0 else -self.vx_exec
            duration_x = abs(dx) / self.vx_exec
            impulses.append(ExecutionImpulse("3_FORWARD", vx_cmd, 0.0, 0.0, duration_x))

        return impulses

    def generate_discrete_footsteps(self, dx: float, dy: float, dyaw: float) -> List[Footstep]:
        """
        Calculates discrete left/right footstep poses for visualization or logging.
        """
        footsteps: List[Footstep] = []

        # Determine number of steps needed based on stride limits
        num_steps_x = int(math.ceil(abs(dx) / self.max_stride_x)) if abs(dx) > 0 else 0
        num_steps_y = int(math.ceil(abs(dy) / self.max_stride_y)) if abs(dy) > 0 else 0
        num_steps_w = int(math.ceil(abs(dyaw) / self.max_stride_yaw)) if abs(dyaw) > 0 else 0

        total_steps = max(num_steps_x, num_steps_y, num_steps_w)
        if total_steps % 2 != 0:
            total_steps += 1  # Force even number of steps to land both feet symmetrically
        total_steps = max(2, total_steps)

        curr_foot = 'LEFT'
        for i in range(1, total_steps + 1):
            alpha = i / float(total_steps)
            cx = alpha * dx
            cy = alpha * dy
            cyaw = alpha * dyaw

            # Offset feet laterally relative to center body pose
            sign = 1.0 if curr_foot == 'LEFT' else -1.0
            fx = cx - sign * (self.stance_width / 2.0) * math.sin(cyaw)
            fy = cy + sign * (self.stance_width / 2.0) * math.cos(cyaw)

            footsteps.append(Footstep(curr_foot, fx, fy, cyaw, i))
            curr_foot = 'RIGHT' if curr_foot == 'LEFT' else 'LEFT'

        return footsteps
