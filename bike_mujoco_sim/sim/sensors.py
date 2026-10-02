"""IMU → bicycle state helpers (from notebook 07)."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from .config import Config


def rotation_from_wxyz(quaternion) -> Rotation:
    q = np.asarray(quaternion, dtype=float)
    xyzw = np.r_[q[1:], q[0]]
    return Rotation.from_quat(xyzw)


def sensor_to_bike_from_config(cfg: Config) -> Rotation:
    roll, pitch, yaw = np.deg2rad(cfg.sensor_to_bike_rpy_deg)
    return Rotation.from_euler("ZYX", [yaw, pitch, roll])


def wrap_angle(angle: float) -> float:
    """Wrap radians to (-pi, pi]."""
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def bicycle_state_from_imu(
    steer: float,
    quaternion_wxyz,
    gyro_sensor,
    sensor_to_bike: Rotation,
    gyro_bias_sensor,
    *,
    wrap_steer: bool = True,
) -> tuple[np.ndarray, float, float]:
    world_from_sensor = rotation_from_wxyz(quaternion_wxyz)
    world_from_bike = world_from_sensor * sensor_to_bike.inv()
    yaw, pitch, roll = world_from_bike.as_euler("ZYX")

    p, q, r = sensor_to_bike.apply(
        np.asarray(gyro_sensor, dtype=float) - np.asarray(gyro_bias_sensor, dtype=float)
    )
    if abs(np.cos(pitch)) < 1e-3:
        raise ValueError("Pitch is too close to the Euler-angle singularity.")
    roll_rate = p + np.sin(roll) * np.tan(pitch) * q + np.cos(roll) * np.tan(pitch) * r
    steer_used = wrap_angle(steer) if wrap_steer else float(steer)
    state = np.array([[steer_used], [roll], [roll_rate]], dtype=float)
    return state, float(roll), float(roll_rate), float(yaw)
