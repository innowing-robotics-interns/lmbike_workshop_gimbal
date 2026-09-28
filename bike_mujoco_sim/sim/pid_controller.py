"""Outer roll–steer PID producing desired steer rate (rad/s)."""

from __future__ import annotations

from .sensors import wrap_angle


class RollSteerPID:
    """Replace notebook NominalBikeController outer loop."""

    def __init__(
        self,
        *,
        kp_roll: float = 8.0,
        ki_roll: float = 1.0,
        kd_roll: float = 1.5,
        kp_steer: float = 2.0,
        max_steer_velocity: float = 10.0,
        integral_limit: float = 5.0,
    ) -> None:
        self.kp_roll = float(kp_roll)
        self.ki_roll = float(ki_roll)
        self.kd_roll = float(kd_roll)
        self.kp_steer = float(kp_steer)
        self.max_steer_velocity = float(max_steer_velocity)
        self.integral_limit = float(integral_limit)
        self.integral = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def step(
        self,
        *,
        steer: float,
        roll: float,
        roll_rate: float,
        dt: float,
        roll_ref: float = 0.0,
        steer_ref: float = 0.0,
    ) -> float:
        e_roll = float(roll_ref) - float(roll)
        e_steer = float(steer_ref) - wrap_angle(steer)
        self.integral = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral + e_roll * float(dt)),
        )
        cmd = (
            self.kp_roll * e_roll
            + self.ki_roll * self.integral
            + self.kd_roll * (-float(roll_rate))
            + self.kp_steer * e_steer
        )
        return float(max(-self.max_steer_velocity, min(self.max_steer_velocity, cmd)))


def apply_speed_refs(
    *,
    mode: str,
    time_s: float,
    speed_goal: float,
    steer_ref: float,
    cfg,
) -> tuple[float, float, float]:
    """Return (speed_ref, roll_ref, steer_ref) for the current mode."""
    if mode == "speed-schedule":
        speed_ref = min(
            time_s / max(cfg.acceleration_time, 1e-9) * cfg.target_speed,
            cfg.target_speed,
        )
        return float(speed_ref), 0.0, 0.0

    # remote
    if abs(speed_goal) < cfg.low_speed_upright:
        return 0.0, 0.0, 0.0
    steer = max(-cfg.steer_ref_limit, min(cfg.steer_ref_limit, steer_ref))
    return float(speed_goal), 0.0, float(steer)
