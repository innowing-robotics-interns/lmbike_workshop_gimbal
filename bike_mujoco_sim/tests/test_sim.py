"""Unit tests for PID bike sim helpers."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim.config import CONFIGS_DIR, DEFAULT_CONFIG_PATH, Config, load_config, resolve_config_path
from sim.models import load_model, resolve_model_path, steering_is_unlimited
from sim.pid_controller import RollSteerPID, apply_speed_refs
from sim.sensors import wrap_angle
from sim.loop import make_sim, run_headless


def test_default_yaml_loads() -> None:
    cfg = load_config(DEFAULT_CONFIG_PATH)
    assert cfg.duration == 10.0
    assert cfg.target_speed == 2.7
    assert cfg.pid_kp_roll == 8.0
    assert cfg.balance_mode == "cascade"
    assert cfg.sensor_to_bike_rpy_deg == (0.0, 0.0, 0.0)


def test_new_bike_yaml_loads() -> None:
    path = CONFIGS_DIR / "new_bike.yaml"
    cfg = load_config(path)
    assert cfg.balance_mode == "cascade"
    assert abs(cfg.pid_kp_roll - 7.54) < 0.05
    assert cfg.steer_kp < 0.1
    assert cfg.steer_torque_limit == 7.98
    assert cfg.steer_velocity_limit == 23.9
    assert resolve_config_path("new_bike") == path


def test_new_bike_tuned_holds_upright() -> None:
    cfg = load_config(CONFIGS_DIR / "new_bike.yaml")
    sim = make_sim(model_spec="new_bike", mode="speed-schedule", cfg=cfg)
    hist = run_headless(sim, duration=cfg.duration)
    rolls = np.degrees(np.abs(np.asarray(hist["roll"], dtype=float)))
    tau = np.asarray(hist["steer_torque"], dtype=float)
    assert float(hist["time"][-1]) >= cfg.duration * 0.95
    assert float(rolls.max()) < 15.0
    assert float(np.std(tau)) < 0.05
    assert float(np.mean(np.abs(tau) > 7.2)) < 0.15


def test_resolve_model_aliases() -> None:
    orange = resolve_model_path("orange_bike")
    new = resolve_model_path("new_bike")
    assert orange.name == "orange_bike_horizontal.xml"
    assert new.name == "new_bike.xml"
    assert orange.is_file()
    assert new.is_file()


def test_wrap_angle() -> None:
    assert abs(wrap_angle(0.0)) < 1e-12
    assert abs(wrap_angle(math.pi + 0.1) + (math.pi - 0.1)) < 1e-9
    assert abs(wrap_angle(-math.pi - 0.1) - (math.pi - 0.1)) < 1e-9


def test_pid_clips_and_resets() -> None:
    pid = RollSteerPID(kp_roll=1000.0, ki_roll=0.0, kd_roll=0.0, kp_steer=0.0, max_steer_velocity=10.0)
    cmd = pid.step(steer=0.0, roll=1.0, roll_rate=0.0, dt=0.02, roll_ref=0.0)
    assert cmd == -10.0  # e_roll = -1 → large negative after clip? wait e_roll = 0-1 = -1, kp*e = -1000 → clip -10
    pid.reset()
    assert pid.integral == 0.0


def test_low_speed_upright_rule() -> None:
    cfg = Config()
    speed_ref, roll_ref, steer_ref = apply_speed_refs(
        mode="remote",
        time_s=1.0,
        speed_goal=0.05,
        steer_ref=0.3,
        cfg=cfg,
    )
    assert speed_ref == 0.0
    assert roll_ref == 0.0
    assert steer_ref == 0.0

    speed_ref, _, steer_ref = apply_speed_refs(
        mode="remote",
        time_s=1.0,
        speed_goal=1.0,
        steer_ref=0.3,
        cfg=cfg,
    )
    assert speed_ref == 1.0
    assert abs(steer_ref - 0.3) < 1e-12


def test_speed_schedule_ramp() -> None:
    cfg = Config(target_speed=2.7, acceleration_time=1.5)
    s0, _, _ = apply_speed_refs(mode="speed-schedule", time_s=0.0, speed_goal=0, steer_ref=0, cfg=cfg)
    s1, _, _ = apply_speed_refs(mode="speed-schedule", time_s=1.5, speed_goal=0, steer_ref=0, cfg=cfg)
    s2, _, _ = apply_speed_refs(mode="speed-schedule", time_s=3.0, speed_goal=0, steer_ref=0, cfg=cfg)
    assert s0 == 0.0
    assert abs(s1 - 2.7) < 1e-9
    assert abs(s2 - 2.7) < 1e-9


def test_new_bike_steer_unlimited() -> None:
    model, _, _ = load_model("new_bike")
    assert steering_is_unlimited(model)
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "steering_joint")
    assert int(model.jnt_limited[jid]) == 0


def test_cli_defaults_to_gui_unless_headless() -> None:
    from run_sim import parse_args

    old = sys.argv
    try:
        sys.argv = ["run_sim.py"]
        args = parse_args()
        assert args.headless is False
        sys.argv = ["run_sim.py", "--headless"]
        args = parse_args()
        assert args.headless is True
    finally:
        sys.argv = old


def test_orange_bike_loads() -> None:
    model, data, _ = load_model("orange_bike")
    assert model.nu == 2
    assert abs(float(model.opt.timestep) - 0.0005) < 1e-12
    mujoco.mj_step(model, data)
    assert data.time > 0
