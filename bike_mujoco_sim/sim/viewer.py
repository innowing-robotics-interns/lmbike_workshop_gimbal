"""GLFW remote / optional viewer with wall-clock catch-up."""

from __future__ import annotations

import math
import os
import time

import mujoco

from .config import Config
from .loop import RideGoals, SimHandles, reset_sim, step_physics

KEY_SPACE, KEY_0 = 32, 48
KEY_RIGHT, KEY_LEFT, KEY_DOWN, KEY_UP = 262, 263, 264, 265
FRAME_PERIOD = 1.0 / 60.0
MAX_SIM_PER_FRAME = 0.05


def apply_key(goals: RideGoals, key: int, cfg: Config) -> None:
    if key == KEY_UP:
        goals.speed_goal = min(cfg.speed_goal_max, goals.speed_goal + cfg.speed_goal_step)
    elif key == KEY_DOWN:
        goals.speed_goal = max(cfg.speed_goal_min, goals.speed_goal - cfg.speed_goal_step)
    elif key == KEY_RIGHT:
        goals.steer_ref = min(cfg.steer_ref_limit, goals.steer_ref + cfg.steer_ref_step)
    elif key == KEY_LEFT:
        goals.steer_ref = max(-cfg.steer_ref_limit, goals.steer_ref - cfg.steer_ref_step)
    elif key == KEY_0:
        goals.stop()
    elif key == KEY_SPACE:
        goals.reset_requested = True


def run_viewer(sim: SimHandles) -> None:
    display = os.environ.get("DISPLAY")
    if not display:
        if __import__("pathlib").Path("/tmp/.X11-unix/X1").exists():
            os.environ["DISPLAY"] = ":1"
        elif __import__("pathlib").Path("/tmp/.X11-unix/X0").exists():
            os.environ["DISPLAY"] = ":0"
        else:
            raise RuntimeError("No DISPLAY set; cannot open MuJoCo window.")

    os.environ.setdefault("MUJOCO_GL", "glfw")
    from mujoco import viewer as mj_viewer

    cfg = sim.cfg

    def key_callback(key: int) -> None:
        if sim.debug:
            print(f"debug: key={key}", flush=True)
        apply_key(sim.goals, key, cfg)

    handle = mj_viewer.launch_passive(sim.model, sim.data, key_callback=key_callback)
    print("Remote / viewer controls (arrows only; WASD left for MuJoCo UI):")
    print("  Up/Down     speed goal ±")
    print("  Left/Right  steer bias ±")
    print("  0           stop (speed=0, steer_ref=0)")
    print("  Space       reset")
    print(
        f"When |speed_goal| < {cfg.low_speed_upright} m/s, "
        "refs force upright hold (fragile at standstill).",
        flush=True,
    )

    cam_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, "chasis_camera")
    dt = float(sim.model.opt.timestep)
    max_steps = max(1, int(MAX_SIM_PER_FRAME / max(dt, 1e-6)))
    wall0 = time.perf_counter()
    sim0 = float(sim.data.time)

    try:
        while handle.is_running():
            if sim.goals.reset_requested:
                reset_sim(sim)
                wall0 = time.perf_counter()
                sim0 = float(sim.data.time)

            if sim.mode == "speed-schedule" and float(sim.data.time) >= cfg.duration:
                print(f"speed-schedule reached {cfg.duration:.1f}s — resetting", flush=True)
                reset_sim(sim)
                wall0 = time.perf_counter()
                sim0 = float(sim.data.time)

            if cam_id >= 0:
                with handle.lock():
                    handle.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                    handle.cam.fixedcamid = int(cam_id)

            now = time.perf_counter()
            target = sim0 + (now - wall0)
            frame_start = now
            steps = 0
            while float(sim.data.time) < target and steps < max_steps:
                step_physics(sim)
                steps += 1

            lean = 0.0
            try:
                from .sensors import bicycle_state_from_imu

                _, roll, _ = bicycle_state_from_imu(
                    float(sim.data.sensor("steering_joint_pos_sensor").data[0]),
                    sim.data.sensor("ori_global").data,
                    sim.data.sensor("gyro_local").data,
                    sim.sensor_to_bike,
                    sim.gyro_bias_sensor,
                )
                lean = abs(roll)
            except Exception:
                pass

            left = "Keys\nSpeed goal\nSteer ref\nLean"
            right = (
                f"↑↓ speed  ←→ steer  0 stop  space reset\n"
                f"{sim.goals.speed_goal:+.2f} m/s\n"
                f"{sim.goals.steer_ref:+.3f} rad\n"
                f"{math.degrees(lean):.1f} deg"
            )
            handle.set_texts(
                [
                    (
                        mujoco.mjtFontScale.mjFONTSCALE_200,
                        mujoco.mjtGridPos.mjGRID_TOPLEFT,
                        left,
                        right,
                    )
                ]
            )
            handle.sync()

            spent = time.perf_counter() - frame_start
            if float(sim.data.time) >= target - 1e-12 and spent < FRAME_PERIOD:
                time.sleep(FRAME_PERIOD - spent)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        try:
            handle.close()
        except Exception:
            pass
