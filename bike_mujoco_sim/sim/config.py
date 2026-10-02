"""Simulation config: dataclass + YAML load from configs/."""

from __future__ import annotations

from dataclasses import dataclass, fields, replace
from pathlib import Path
from typing import Any, Mapping

import yaml

# bike_mujoco_sim/configs/
CONFIGS_DIR = Path(__file__).resolve().parents[1] / "configs"
DEFAULT_CONFIG_PATH = CONFIGS_DIR / "default.yaml"


@dataclass
class Config:
    duration: float = 10.0
    control_dt: float = 0.02
    record_dt: float = 0.02
    wheel_radius: float = 0.07
    target_speed: float = 2.7
    acceleration_time: float = 1.5
    min_scheduling_speed: float = 1.0
    balance_start_time: float = 0.1
    max_steer_velocity: float = 10.0
    steer_kp: float = 0.01
    steer_ki: float = 0.01
    steer_integral_limit: float = 10.0
    rear_kp: float = 0.1
    rear_torque_limit: float = 2.0
    sensor_to_bike_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gyro_bias_sensor_dps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Outer roll–steer PID (replaces LQR / pole place).
    pid_kp_roll: float = 8.0
    pid_ki_roll: float = 1.0
    pid_kd_roll: float = 1.5
    pid_kp_steer: float = 2.0
    pid_integral_limit: float = 5.0
    # "cascade" = notebook rate cascade; "lean_torque" = direct lean→steer torque PD;
    # "mit" = outer PID → steer angle/rate, GIM4310 MIT inner loop (no Ki).
    balance_mode: str = "cascade"
    # Extra damping on measured steer rate (lean_torque mode).
    steer_rate_damping: float = 0.0
    # GIM4310-10 MIT position loop. kp/kd are fixed motor gains, not tuned.
    mit_kp: float = 10.0
    mit_kd: float = 0.2
    # Middle-loop gain: ω_des = mit_omega_kp * (θ_des − θ). 0 = single outer loop.
    mit_omega_kp: float = 0.0
    steer_torque_limit: float = 7.98  # GIM4310-10 GDZ34 stall, N·m
    steer_velocity_limit: float = 0.0
    # If set, override steering_joint dof damping after model load.
    steer_joint_damping: float | None = None
    # Remote mode
    speed_goal_step: float = 0.15
    speed_goal_min: float = -1.0
    speed_goal_max: float = 3.5
    steer_ref_step: float = 0.05
    steer_ref_limit: float = 0.7
    # Remote heading: left/right add this many radians.
    # heading_kp is steer-rate (rad/s) per radian of heading error.
    heading_step: float = 0.262
    heading_kp: float = 1.5
    heading_ki: float = 0.3
    # Full left/right stick deflection, as a steer-rate command (rad/s).
    heading_stick_rate: float = 1.5
    low_speed_upright: float = 0.1


_TUPLE3 = frozenset({"sensor_to_bike_rpy_deg", "gyro_bias_sensor_dps"})


def _coerce(name: str, value: Any) -> Any:
    if name in _TUPLE3:
        seq = list(value)
        if len(seq) != 3:
            raise ValueError(f"{name} must have 3 elements, got {seq!r}")
        return (float(seq[0]), float(seq[1]), float(seq[2]))
    if name == "steer_joint_damping" and value is None:
        return None
    return value


def resolve_config_path(model_spec: str, config_path: str | Path | None = None) -> Path:
    """Pick YAML: explicit --config, else new_bike.yaml for new_bike, else default."""
    if config_path is not None:
        return Path(config_path)
    spec = str(model_spec).lower()
    if "new_bike_3kg" in spec or "new-bike-3kg" in spec or spec.endswith("new_bike_3kg.xml"):
        candidate = CONFIGS_DIR / "new_bike_3kg.yaml"
        if candidate.is_file():
            return candidate
    if "new_bike" in spec or spec.endswith("new_bike.xml"):
        candidate = CONFIGS_DIR / "new_bike.yaml"
        if candidate.is_file():
            return candidate
    return DEFAULT_CONFIG_PATH


def config_from_mapping(data: Mapping[str, Any], *, base: Config | None = None) -> Config:
    """Build Config from a dict (e.g. YAML). Unknown keys raise."""
    cfg = base if base is not None else Config()
    known = {f.name for f in fields(Config)}
    unknown = set(data) - known
    if unknown:
        raise ValueError(f"Unknown config keys: {sorted(unknown)}")
    updates = {k: _coerce(k, v) for k, v in data.items()}
    return replace(cfg, **updates)


def load_config(path: str | Path | None = None) -> Config:
    """Load Config from YAML. Default: configs/default.yaml."""
    yaml_path = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    if not yaml_path.is_file():
        raise FileNotFoundError(f"Config YAML not found: {yaml_path}")
    with yaml_path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    if raw is None:
        return Config()
    if not isinstance(raw, dict):
        raise ValueError(f"Config YAML must be a mapping, got {type(raw).__name__}")
    return config_from_mapping(raw)
