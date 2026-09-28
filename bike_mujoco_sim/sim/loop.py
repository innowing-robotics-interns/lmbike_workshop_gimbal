"""50 Hz dual-loop control + MuJoCo stepping (notebook 07 structure, PID outer)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import mujoco
import numpy as np

from .config import Config
from .models import load_model, steering_is_unlimited
from .pid_controller import RollSteerPID, apply_speed_refs
from .sensors import bicycle_state_from_imu, sensor_to_bike_from_config, wrap_angle


@dataclass
class RideGoals:
    speed_goal: float = 0.0
    steer_ref: float = 0.0
    reset_requested: bool = False

    def stop(self) -> None:
        self.speed_goal = 0.0
        self.steer_ref = 0.0


@dataclass
class ControlState:
    steer_integral: float = 0.0
    roll_integral: float = 0.0
    next_control: float = 0.0
    next_record: float = 0.0
    pid: RollSteerPID | None = None


@dataclass
class SimHandles:
    model: mujoco.MjModel
    data: mujoco.MjData
    model_path: Any
    cfg: Config
    mode: str
    wrap_steer: bool
    goals: RideGoals = field(default_factory=RideGoals)
    ctrl: ControlState = field(default_factory=ControlState)
    history: dict[str, list] = field(default_factory=dict)
    sensor_to_bike: Any = None
    gyro_bias_sensor: np.ndarray | None = None
    debug: bool = False
    steer_dofadr: int = -1

    @property
    def steer_id(self) -> int:
        return int(self.model.actuator("cmd_steering_f").id)

    @property
    def rear_id(self) -> int:
        return int(self.model.actuator("cmd_rearwheel_f").id)


def make_sim(
    *,
    model_spec: str,
    mode: str,
    cfg: Config,
    debug: bool = False,
) -> SimHandles:
    model, data, path = load_model(model_spec)
    wrap_steer = True  # always wrap for PID centering; required for free steer
    unlimited = False
    try:
        unlimited = steering_is_unlimited(model)
    except ValueError:
        pass

    # Best-effort presets for the heavier CAD bike when still on orange defaults.
    flavor = "new_bike" if "new_bike" in path.as_posix() else "orange_bike"
    if flavor == "new_bike" and cfg.balance_mode == "cascade":
        if cfg.steer_kp == Config().steer_kp:
            cfg.steer_kp = 0.2
        if cfg.steer_ki == Config().steer_ki:
            cfg.steer_ki = 0.1
        if cfg.rear_kp == Config().rear_kp:
            cfg.rear_kp = 0.25
        if cfg.acceleration_time == Config().acceleration_time:
            cfg.acceleration_time = 2.5

    if cfg.steer_joint_damping is not None:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "steering_joint")
        if jid < 0:
            raise ValueError("steering_joint not found (needed for steer_joint_damping)")
        model.dof_damping[model.jnt_dofadr[jid]] = float(cfg.steer_joint_damping)

    if debug:
        print(
            f"model: {path}  flavor={flavor}  mode={cfg.balance_mode}  "
            f"dt={model.opt.timestep}  "
            f"steer_limited={0 if unlimited else 1}  "
            f"steer_kp={cfg.steer_kp} rear_kp={cfg.rear_kp}",
            flush=True,
        )
    pid = RollSteerPID(
        kp_roll=cfg.pid_kp_roll,
        ki_roll=cfg.pid_ki_roll,
        kd_roll=cfg.pid_kd_roll,
        kp_steer=cfg.pid_kp_steer,
        max_steer_velocity=cfg.max_steer_velocity,
        integral_limit=cfg.pid_integral_limit,
    )
    names = [
        "time",
        "steer",
        "roll",
        "roll_rate",
        "speed",
        "speed_ref",
        "steer_rate",
        "steer_rate_ref",
        "steer_torque",
        "rear_torque",
    ]
    return SimHandles(
        model=model,
        data=data,
        model_path=path,
        cfg=cfg,
        mode=mode,
        wrap_steer=wrap_steer,
        ctrl=ControlState(pid=pid),
        history={n: [] for n in names},
        sensor_to_bike=sensor_to_bike_from_config(cfg),
        gyro_bias_sensor=np.deg2rad(cfg.gyro_bias_sensor_dps),
        debug=debug,
        steer_dofadr=int(model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "steering_joint")]),
    )


def reset_sim(sim: SimHandles) -> None:
    mujoco.mj_resetData(sim.model, sim.data)
    mujoco.mj_forward(sim.model, sim.data)
    sim.ctrl.steer_integral = 0.0
    sim.ctrl.roll_integral = 0.0
    sim.ctrl.next_control = 0.0
    sim.ctrl.next_record = 0.0
    if sim.ctrl.pid is not None:
        sim.ctrl.pid.reset()
    sim.goals.reset_requested = False
    for key in sim.history:
        sim.history[key].clear()


def control_step(sim: SimHandles) -> None:
    """Run one outer+inner control update and write data.ctrl."""
    cfg = sim.cfg
    model, data = sim.model, sim.data
    assert sim.ctrl.pid is not None

    steer = float(data.sensor("steering_joint_pos_sensor").data[0])
    steer_rate = float(data.sensor("steering_joint_vel_sensor").data[0])
    rear_rate = float(data.sensor("rearwheel_joint_vel_sensor").data[0])
    _, roll, roll_rate = bicycle_state_from_imu(
        steer,
        data.sensor("ori_global").data,
        data.sensor("gyro_local").data,
        sim.sensor_to_bike,
        sim.gyro_bias_sensor,
        wrap_steer=sim.wrap_steer,
    )
    speed = cfg.wheel_radius * rear_rate
    steer_wrapped = wrap_angle(steer) if sim.wrap_steer else steer

    speed_ref, roll_ref, steer_ref = apply_speed_refs(
        mode=sim.mode,
        time_s=float(data.time),
        speed_goal=sim.goals.speed_goal,
        steer_ref=sim.goals.steer_ref,
        cfg=cfg,
    )

    if float(data.time) < cfg.balance_start_time:
        steer_rate_ref = 0.0
        steer_torque = 0.0
    elif cfg.balance_mode == "lean_torque":
        # Direct lean → steer torque (tuned for new_bike; cascade rate loop fails here).
        e_roll = float(roll_ref) - float(roll)
        e_steer = float(steer_ref) - float(steer_wrapped)
        sim.ctrl.roll_integral = float(
            np.clip(
                sim.ctrl.roll_integral + (float(roll) - float(roll_ref)) * cfg.control_dt,
                -cfg.pid_integral_limit,
                cfg.pid_integral_limit,
            )
        )
        # Steer-into-lean for this MJCF: τ ∝ +roll (i.e. opposite of cascade e_roll gain).
        steer_torque = (
            -cfg.pid_kp_roll * e_roll
            + cfg.pid_ki_roll * sim.ctrl.roll_integral
            - cfg.pid_kd_roll * float(roll_rate)
            + cfg.pid_kp_steer * e_steer
            - cfg.steer_rate_damping * float(steer_rate)
        )
        steer_rate_ref = float(steer_rate)  # logged as measured; no rate setpoint
    elif cfg.balance_mode == "mit":
        # Outer PID → θ_des, optional rate command. Motor is MIT PD (no integral).
        e_roll = float(roll) - float(roll_ref)
        sim.ctrl.roll_integral = float(
            np.clip(
                sim.ctrl.roll_integral + e_roll * cfg.control_dt,
                -cfg.pid_integral_limit,
                cfg.pid_integral_limit,
            )
        )
        theta_des = (
            cfg.pid_kp_roll * e_roll
            + cfg.pid_ki_roll * sim.ctrl.roll_integral
            - cfg.pid_kd_roll * float(roll_rate)
            + cfg.pid_kp_steer * (float(steer_ref) - float(steer_wrapped))
        )
        theta_lim = float(cfg.steer_ref_limit)
        theta_des = float(np.clip(theta_des, -theta_lim, theta_lim))
        omega_des = float(cfg.mit_omega_kp) * (theta_des - float(steer_wrapped))
        vel_lim = float(cfg.steer_velocity_limit)
        if vel_lim > 0.0:
            omega_des = float(np.clip(omega_des, -vel_lim, vel_lim))
        steer_rate_ref = omega_des
        steer_torque = cfg.mit_kp * (theta_des - float(steer_wrapped)) + cfg.mit_kd * (
            omega_des - float(steer_rate)
        )
        tlim = float(cfg.steer_torque_limit)
        steer_torque = float(np.clip(steer_torque, -tlim, tlim))
    else:
        steer_rate_ref = sim.ctrl.pid.step(
            steer=steer_wrapped,
            roll=roll,
            roll_rate=roll_rate,
            dt=cfg.control_dt,
            roll_ref=roll_ref,
            steer_ref=steer_ref,
        )
        steer_error = steer_rate_ref - steer_rate
        sim.ctrl.steer_integral = float(
            np.clip(
                sim.ctrl.steer_integral + steer_error * cfg.control_dt,
                -cfg.steer_integral_limit,
                cfg.steer_integral_limit,
            )
        )
        steer_torque = cfg.steer_kp * steer_error + cfg.steer_ki * sim.ctrl.steer_integral

    steer_range = model.actuator_ctrlrange[sim.steer_id]
    steer_torque = float(np.clip(steer_torque, steer_range[0], steer_range[1]))

    rear_rate_ref = speed_ref / cfg.wheel_radius
    rear_torque = cfg.rear_kp * (rear_rate_ref - rear_rate)
    rear_range = model.actuator_ctrlrange[sim.rear_id]
    rear_lower = max(rear_range[0], -cfg.rear_torque_limit)
    rear_upper = min(rear_range[1], cfg.rear_torque_limit)
    rear_torque = float(np.clip(rear_torque, rear_lower, rear_upper))

    data.ctrl[sim.steer_id] = steer_torque
    data.ctrl[sim.rear_id] = rear_torque

    if float(data.time) + 1e-12 >= sim.ctrl.next_record:
        sim.ctrl.next_record += cfg.record_dt
        sim.history["time"].append(float(data.time))
        sim.history["steer"].append(steer_wrapped)
        sim.history["roll"].append(roll)
        sim.history["roll_rate"].append(roll_rate)
        sim.history["speed"].append(speed)
        sim.history["speed_ref"].append(speed_ref)
        sim.history["steer_rate"].append(steer_rate)
        sim.history["steer_rate_ref"].append(steer_rate_ref)
        sim.history["steer_torque"].append(steer_torque)
        sim.history["rear_torque"].append(rear_torque)
        if sim.debug:
            print(
                f"t={data.time:5.2f} roll={np.degrees(roll):+6.2f}deg "
                f"speed={speed:5.2f}/{speed_ref:5.2f} "
                f"steer_tau={steer_torque:+.3f} rear_tau={rear_torque:+.3f}",
                flush=True,
            )


def maybe_control(sim: SimHandles) -> None:
    if float(sim.data.time) + 1e-12 >= sim.ctrl.next_control:
        sim.ctrl.next_control += sim.cfg.control_dt
        control_step(sim)


def step_physics(sim: SimHandles) -> None:
    maybe_control(sim)
    mujoco.mj_step(sim.model, sim.data)
    vel_lim = float(sim.cfg.steer_velocity_limit)
    if vel_lim > 0.0 and sim.steer_dofadr >= 0:
        adr = sim.steer_dofadr
        sim.data.qvel[adr] = float(np.clip(sim.data.qvel[adr], -vel_lim, vel_lim))


def run_headless(sim: SimHandles, duration: float | None = None) -> dict[str, list]:
    duration = float(sim.cfg.duration if duration is None else duration)
    while float(sim.data.time) < duration:
        step_physics(sim)
    return sim.history


def summarize(history: dict[str, list], *, model_path, mode: str) -> None:
    if not history["time"]:
        print("result: no samples")
        return
    rolls = np.degrees(np.asarray(history["roll"], dtype=float))
    speeds = np.asarray(history["speed"], dtype=float)
    max_abs = float(np.max(np.abs(rolls)))
    print(f"model:      {model_path}")
    print(f"mode:       {mode}")
    print(f"simulated:  {history['time'][-1]:.2f}s")
    print(f"max |roll|: {max_abs:.2f} deg")
    print(f"final speed:{float(speeds[-1]):.3f} m/s")
    print(f"mean |roll|: {float(np.mean(np.abs(rolls))):.2f} deg")
    if max_abs > 45.0:
        print(
            "WARN: large lean — likely tipped. Retune PID / inner gains "
            "(new_bike is harder than orange_bike).",
            flush=True,
        )
