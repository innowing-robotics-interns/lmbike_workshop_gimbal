#!/usr/bin/env python3
"""Balance the workshop bicycle in MuJoCo.

Examples:
  python run_balance.py
  python run_balance.py --model new_bike
  python run_balance.py --flat
  python run_balance.py --no-viewer --duration 10
  python run_balance.py --no-viewer --controller none --duration 4
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
from pathlib import Path

import mujoco
import numpy as np

from controller import BalanceController, gains_for_world
from environment import (
    BikeWorld,
    local_slope_deg,
    nearest_hill,
    resolve_model_path,
    terrain_zone,
)

HERE = Path(__file__).resolve().parent
FALL_PITCH_RAD = math.radians(50.0)
FALL_LEAN_RAD = math.radians(55.0)
# GLFW key codes (press-only callback from the Simulate window).
# Arrows only — WASD is reserved by MuJoCo Simulate for visibility toggles.
KEY_SPACE, KEY_0 = 32, 48
KEY_RIGHT, KEY_LEFT, KEY_DOWN, KEY_UP = 262, 263, 264, 265
CRUISE_MIN, CRUISE_MAX = -0.6, 1.8
SHOVE_N = 90.0
NEW_REAR_STEP, NEW_REAR_MAX = 0.75, 6.0
NEW_STEER_STEP, NEW_STEER_MAX = 0.5, 5.0
REALTIME_WARN = 0.7
FRAME_PERIOD = 1.0 / 60.0
MAX_SIM_PER_FRAME = 0.05  # seconds of sim catch-up per display frame


class RideState:
    """Keyboard pilot for the planar bike (cascade) or new_bike (direct torque)."""

    def __init__(
        self,
        cruise: float,
        *,
        steer_force: bool = False,
        flavor: str = "planar",
    ) -> None:
        self.cruise = float(cruise)
        self.shove = 0.0
        self.rear_cmd = 0.0
        self.steer_cmd = 0.0
        self.reset_requested = False
        self.steer_force = steer_force
        self.flavor = flavor

    def on_key(self, key: int) -> None:
        if self.flavor == "new_bike":
            self._on_key_new_bike(key)
            return
        if self.steer_force and key in (KEY_RIGHT, KEY_LEFT):
            self.shove = SHOVE_N if key == KEY_RIGHT else -SHOVE_N
            return
        if key == KEY_RIGHT:
            self.cruise = min(CRUISE_MAX, self.cruise + 0.15)
        elif key == KEY_LEFT:
            self.cruise = max(CRUISE_MIN, self.cruise - 0.15)
        elif key == KEY_UP:
            self.shove = SHOVE_N
        elif key == KEY_DOWN:
            self.shove = -SHOVE_N
        elif key == KEY_0:
            self.cruise = 0.0
            self.shove = 0.0
        elif key == KEY_SPACE:
            self.reset_requested = True

    def _on_key_new_bike(self, key: int) -> None:
        if key == KEY_UP:
            self.rear_cmd = min(NEW_REAR_MAX, self.rear_cmd + NEW_REAR_STEP)
        elif key == KEY_DOWN:
            self.rear_cmd = max(-NEW_REAR_MAX, self.rear_cmd - NEW_REAR_STEP)
        elif key == KEY_RIGHT:
            self.steer_cmd = min(NEW_STEER_MAX, self.steer_cmd + NEW_STEER_STEP)
        elif key == KEY_LEFT:
            self.steer_cmd = max(-NEW_STEER_MAX, self.steer_cmd - NEW_STEER_STEP)
        elif key == KEY_0:
            self.rear_cmd = 0.0
            self.steer_cmd = 0.0
        elif key == KEY_SPACE:
            self.reset_requested = True

    def decay_shove(self) -> None:
        self.shove *= 0.90
        if abs(self.shove) < 1.0:
            self.shove = 0.0
        # Steer slowly centers when not adjusting; rear torque stays latched.
        self.steer_cmd *= 0.985
        if abs(self.steer_cmd) < 0.05:
            self.steer_cmd = 0.0
WEB_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Bike balance</title>
  <style>
    html, body { margin: 0; height: 100%; background: #0b1118; color: #e8eef4; font-family: ui-sans-serif, system-ui, sans-serif; }
    main { min-height: 100%; display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 12px; padding: 16px; }
    img { width: min(1280px, 100%); background: #111; border-radius: 12px; }
    p { margin: 0; opacity: 0.75; }
  </style>
</head>
<body>
  <main>
    <img src="/stream" alt="MuJoCo bicycle live view"/>
    <p>Live MuJoCo stream. Close this tab or Ctrl+C in the terminal to stop.</p>
  </main>
</body>
</html>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the MuJoCo bicycle balance demo.")
    view = parser.add_mutually_exclusive_group()
    view.add_argument("--no-viewer", action="store_true", help="Headless run (no GUI window).")
    view.add_argument(
        "--web",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--host", default="127.0.0.1", help="Host for --web.")
    parser.add_argument("--port", type=int, default=43145, help="Port for --web.")
    parser.add_argument("--duration", type=float, default=10.0, help="Simulated seconds to run.")
    parser.add_argument(
        "--model",
        default="bike",
        help="Bike model: bike (default planar), new_bike, or a path to an .xml under models/.",
    )
    parser.add_argument(
        "--controller",
        choices=("pd", "none"),
        default="pd",
        help="pd: cascade balance+speed controller (planar bike). none: no cascade. new_bike is always free-drive.",
    )
    parser.add_argument("--speed", type=float, default=None, help="Cruise speed in m/s (overrides the random world cruise).")
    parser.add_argument("--lean-deg", type=float, default=None, help="Initial pitch offset in degrees (random if omitted).")
    parser.add_argument("--flat", action="store_true", help="No slope or randomization; original flat floor.")
    parser.add_argument("--seed", type=int, default=None, help="RNG seed for terrain and masses.")
    parser.add_argument(
        "--realtime",
        action="store_true",
        help="Sleep so wall-clock time tracks simulation time (viewer default).",
    )
    parser.add_argument(
        "--save-plot",
        type=Path,
        default=None,
        help="Optional path for a pitch-vs-time PNG.",
    )
    parser.add_argument(
        "--save-video",
        type=Path,
        default=None,
        help="Optional MP4 path written with the offscreen renderer.",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="After duration, reset and run again. On by default for the native window.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Verbose timing/key diagnostics and auto-WARN on slow realtime.",
    )
    return parser.parse_args()


def print_physics_banner(world: BikeWorld) -> None:
    dt = float(world.model.opt.timestep)
    override = " (overridden for interactive realtime)" if world.timestep_overridden else ""
    print(
        f"physics: flavor={world.flavor}  dt={dt}{override}  "
        f"nq={world.model.nq} nv={world.model.nv} nu={world.model.nu}",
        flush=True,
    )


def benchmark_steps(world: BikeWorld, *, n: int = 200, debug: bool = False) -> float:
    """Cold-step timing. Returns average milliseconds per mj_step. Restores state."""
    model, data = world.model, world.data
    qpos0 = data.qpos.copy()
    qvel0 = data.qvel.copy()
    ctrl0 = data.ctrl.copy()
    time0 = float(data.time)
    # Warm one step so allocation quirks don't dominate.
    mujoco.mj_step(model, data)
    t0 = time.perf_counter()
    for _ in range(n):
        mujoco.mj_step(model, data)
    elapsed = time.perf_counter() - t0
    ms = 1000.0 * elapsed / max(n, 1)
    dt = float(model.opt.timestep)
    max_rt = (dt / (elapsed / max(n, 1))) if elapsed > 0 else float("inf")
    print(f"benchmark: {ms:.3f} ms/step  max_realtime≈{max_rt:.2f}x  (n={n})", flush=True)
    if max_rt < REALTIME_WARN:
        print(
            f"WARN: physics too slow for realtime (max≈{max_rt:.2f}x < {REALTIME_WARN}). "
            "Expect lag unless the catch-up loop can keep up.",
            flush=True,
        )
    if debug:
        print(f"debug: benchmark wall={elapsed:.4f}s sim_advanced={n * dt:.4f}s", flush=True)
    data.qpos[:] = qpos0
    data.qvel[:] = qvel0
    data.ctrl[:] = ctrl0
    data.time = time0
    mujoco.mj_forward(model, data)
    return ms


def sensor(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> float:
    return float(data.sensordata[model.sensor(name).id])


def sensor_vec(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    sid = model.sensor(name).id
    adr = int(model.sensor_adr[sid])
    dim = int(model.sensor_dim[sid])
    return np.asarray(data.sensordata[adr : adr + dim], dtype=float)


def lean_rad(world: BikeWorld) -> float:
    """Largest |roll|/|pitch| for fall checks (planar pitch sensor or IMU quat)."""
    if world.flavor == "planar":
        return abs(sensor(world.model, world.data, "pitch"))
    quat = sensor_vec(world.model, world.data, "ori_global")
    if quat.shape[0] < 4:
        return 0.0
    w, x, y, z = quat
    # Roll (x) and pitch (y) from body quaternion.
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return max(abs(roll), abs(pitch))


def apply_control(model: mujoco.MjModel, data: mujoco.MjData, force: float, wind: float = 0.0) -> None:
    data.ctrl[model.actuator("drive").id] = force + wind
    wheel = max(-18.0, min(18.0, 0.12 * force))
    data.ctrl[model.actuator("rear_drive").id] = wheel
    data.ctrl[model.actuator("front_drive").id] = wheel


def apply_new_bike_control(model: mujoco.MjModel, data: mujoco.MjData, rear: float, steer: float) -> None:
    data.ctrl[model.actuator("cmd_rearwheel_f").id] = max(-NEW_REAR_MAX, min(NEW_REAR_MAX, rear))
    data.ctrl[model.actuator("cmd_steering_f").id] = max(-NEW_STEER_MAX, min(NEW_STEER_MAX, steer))


def fixed_camera_id(model: mujoco.MjModel, flavor: str) -> int | None:
    for name in (("chasis_camera", "side") if flavor == "new_bike" else ("side", "chasis_camera")):
        cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        if cid >= 0:
            return int(cid)
    return None


def reset_state(world: BikeWorld, *, reroll: bool = False, viewer=None, controller: BalanceController | None = None) -> None:
    sample = world.reset(reroll=reroll)
    print(sample.summary(), flush=True)
    _print_hill_list(sample)
    if controller is not None:
        controller.reset()
    if viewer is not None and world.flavor == "planar" and world.model.nhfield > 0:
        viewer.update_hfield(0)


def _print_hill_list(sample) -> None:
    if sample.flat or not sample.bumps:
        return
    bits = ", ".join(
        f"#{i + 1} at {b.x:.1f} m ({b.height * 100:.0f} cm)" for i, b in enumerate(sample.bumps)
    )
    print(f"hills: {bits}", flush=True)


def hud_text(
    world: BikeWorld,
    controller: BalanceController | None,
    ride: RideState | None = None,
) -> tuple[str, str]:
    sample = world.sample
    if world.flavor == "new_bike":
        left = "Keys\nModel\nLean\nDrive\nSteer"
        look = "↑/↓ rear  ←/→ steer  0 stop  space reset"
        if ride is not None:
            look = f"{look}   (rear {ride.rear_cmd:+.2f}  steer {ride.steer_cmd:+.2f})"
        lean_deg = math.degrees(lean_rad(world))
        right = f"{look}\nnew_bike (free-drive)\n{lean_deg:+.1f} deg\nlatched rear torque\nincremental steer"
        return left, right

    x = sensor(world.model, world.data, "x")
    left = "Keys\nNearest hill\nLocal slope\nController\nSpeed / force"
    look = "→ faster  ← slower  ↑ shove  space reset"
    if ride is not None:
        look = f"{look}   (cruise {ride.cruise:+.2f} m/s)"
    if sample.flat:
        nearest = "—"
        zone = "level"
        slope = 0.0
    else:
        found = nearest_hill(x, sample)
        if found is None:
            nearest = "—"
        else:
            idx, bump, dist = found
            if dist > 0.25:
                nearest = f"#{idx + 1}  {dist:.1f} m ahead, {bump.height * 100:.0f} cm"
            elif dist < -0.25:
                nearest = f"#{idx + 1}  {abs(dist):.1f} m behind, {bump.height * 100:.0f} cm"
            else:
                nearest = f"#{idx + 1}  under you, {bump.height * 100:.0f} cm"
        zone = terrain_zone(x, sample)
        slope = local_slope_deg(x, sample)
    if controller is None:
        ctrl = "off (none)"
        motion = zone
    else:
        d = controller.last
        ctrl = f"cascade  pitch {d.pitch_deg:+.1f} deg (cmd {d.pitch_ref_deg:+.1f})"
        motion = f"{d.speed:.2f} / {d.speed_ref:.2f} m/s   drive {d.force:+.0f} N"
    right = f"{look}\n{nearest}\n{slope:.1f} deg  {zone}\n{ctrl}\n{motion}"
    return left, right


def step_control(
    world: BikeWorld,
    controller: BalanceController | None,
    speed_ref: float | None = None,
    extra_force: float = 0.0,
    ride: RideState | None = None,
) -> float:
    model, data = world.model, world.data
    if world.flavor == "new_bike":
        rear = 0.0 if ride is None else ride.rear_cmd
        steer = 0.0 if ride is None else ride.steer_cmd
        apply_new_bike_control(model, data, rear, steer)
        mujoco.mj_step(model, data)
        return lean_rad(world)

    sample = world.sample
    pitch = sensor(model, data, "pitch")
    x = sensor(model, data, "x")
    if controller is not None:
        cruise = sample.cruise_vel if speed_ref is None else speed_ref
        force = controller.torque(
            pitch,
            sensor(model, data, "pitch_vel"),
            x,
            sensor(model, data, "x_vel"),
            x_vel_ref=cruise,
            slope_deg=local_slope_deg(x, sample),
            mass=sample.rider_mass + 5.6,
            dt=float(model.opt.timestep),
        )
    else:
        force = 0.0
    apply_control(model, data, force + extra_force, wind=sample.wind)
    mujoco.mj_step(model, data)
    return sensor(model, data, "pitch")


def summarize(
    args: argparse.Namespace,
    times: list[float],
    pitches: list[float],
    fallen_at: float | None,
    *,
    model_path: Path | None = None,
) -> int:
    pitch_deg = np.degrees(np.asarray(pitches, dtype=float)) if pitches else np.array([0.0])
    sim_end = times[-1] if times else 0.0
    max_abs = float(np.max(np.abs(pitch_deg)))
    mean_abs = float(np.mean(np.abs(pitch_deg)))
    ok = fallen_at is None and sim_end >= args.duration * 0.98

    print(f"model:      {model_path or resolve_model_path(args.model)}")
    print(f"controller: {args.controller}")
    print(f"simulated:  {sim_end:.2f}s / {args.duration:.2f}s")
    print(f"max |lean|: {max_abs:.2f} deg")
    print(f"mean |lean|: {mean_abs:.2f} deg")
    if fallen_at is not None:
        print(f"fell at:    {fallen_at:.2f}s")
        print("result:     FALL")
    else:
        print(f"result:     {'SUCCESS' if ok else 'STOPPED'}")

    if args.save_plot:
        _save_plot(args.save_plot, times, pitch_deg)

    if args.controller == "pd" and getattr(args, "_flavor", "planar") == "planar":
        return 0 if ok else 1
    return 0


def run(args: argparse.Namespace) -> int:
    if args.duration <= 0:
        print("duration must be positive", file=sys.stderr)
        return 2
    if args.port <= 0 or args.port > 65535:
        print("port must be in 1..65535", file=sys.stderr)
        return 2

    try:
        model_path = resolve_model_path(args.model)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    world = BikeWorld(flat=args.flat, seed=args.seed, lean_deg=args.lean_deg, model_path=model_path)
    args._flavor = world.flavor
    print(f"model: {world.model_path}  ({world.flavor})", flush=True)
    print_physics_banner(world)
    benchmark_steps(world, debug=args.debug)
    print(world.sample.summary(), flush=True)
    _print_hill_list(world.sample)

    if world.flavor == "new_bike":
        controller = None
        if args.controller == "pd":
            print("controller: off (cascade PD is planar-only; new_bike is free-drive arrows)")
        else:
            print("controller: off")
        args.controller = "none"
    elif args.controller == "none":
        controller = None
        print("controller: off")
    else:
        hold_position = args.flat and args.speed is None
        controller = BalanceController(gains_for_world(flat=hold_position))
        cruise = world.sample.cruise_vel if args.speed is None else args.speed
        print(
            f"controller: cascade  kp={controller.gains.kp:.0f} kd={controller.gains.kd:.0f} "
            f"kv={controller.gains.kv:.2f} ki={controller.gains.ki:.2f}  "
            f"cruise={cruise:.2f} m/s",
            flush=True,
        )

    if args.web:
        return run_web(args, world, controller)
    if args.no_viewer:
        return run_headless(args, world, controller)
    return run_glfw(args, world, controller)


def run_headless(
    args: argparse.Namespace,
    world: BikeWorld,
    controller: BalanceController | None,
) -> int:
    os.environ.setdefault("MUJOCO_GL", "egl")
    model, data = world.model, world.data
    times: list[float] = []
    pitches: list[float] = []
    fallen_at: float | None = None
    cam = fixed_camera_id(model, world.flavor)
    recorder = (
        VideoRecorder(model, data, args.save_video, camera=cam) if args.save_video else None
    )
    fall_lim = FALL_LEAN_RAD if world.flavor == "new_bike" else FALL_PITCH_RAD
    last_wall = time.perf_counter()
    try:
        while data.time < args.duration:
            pitch = step_control(world, controller, args.speed)
            times.append(data.time)
            pitches.append(pitch)
            if recorder is not None:
                recorder.maybe_capture(data)
            if fallen_at is None and abs(pitch) > fall_lim:
                fallen_at = data.time
                if args.controller == "pd" and world.flavor == "planar":
                    break
            if args.realtime:
                elapsed = time.perf_counter() - last_wall
                sleep_for = model.opt.timestep - elapsed
                if sleep_for > 0:
                    time.sleep(sleep_for)
                last_wall = time.perf_counter()
    finally:
        if recorder is not None:
            recorder.close()
    return summarize(args, times, pitches, fallen_at, model_path=world.model_path)


def _default_display() -> str | None:
    display = os.environ.get("DISPLAY")
    if display:
        return display
    if Path("/tmp/.X11-unix/X1").exists():
        os.environ["DISPLAY"] = ":1"
        return ":1"
    if Path("/tmp/.X11-unix/X0").exists():
        os.environ["DISPLAY"] = ":0"
        return ":0"
    return None


def _run_quiet(cmd: list[str], env: dict[str, str]) -> subprocess.CompletedProcess[str] | None:
    try:
        return subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            env=env,
            check=False,
        )
    except FileNotFoundError:
        return None
    except OSError:
        return None


def raise_mujoco_window(display: str) -> None:
    """Bring the Simulate window to the front once GLFW creates it."""
    env = os.environ.copy()
    env["DISPLAY"] = display
    xdotool = shutil.which("xdotool")
    wmctrl = shutil.which("wmctrl")
    if not xdotool and not wmctrl:
        print(
            "Note: install xdotool (or wmctrl) to auto-focus the MuJoCo window, "
            "or click it so arrow keys work.",
            flush=True,
        )
        return

    for _ in range(60):
        time.sleep(0.25)
        if xdotool:
            found = _run_quiet([xdotool, "search", "--name", "MuJoCo"], env)
            if found is None:
                xdotool = None
            else:
                ids = found.stdout.split()
                if ids:
                    wid = ids[-1]
                    for cmd in (
                        [xdotool, "windowactivate", "--sync", wid],
                        [xdotool, "windowraise", wid],
                        [xdotool, "windowmove", wid, "48", "48"],
                        [xdotool, "windowsize", wid, "1600", "1000"],
                    ):
                        _run_quiet(cmd, env)
                    print("Focused MuJoCo window (xdotool).", flush=True)
                    return
        if wmctrl:
            # -a activates a window whose title contains the string.
            result = _run_quiet([wmctrl, "-a", "MuJoCo"], env)
            if result is None:
                wmctrl = None
            elif result.returncode == 0:
                print("Focused MuJoCo window (wmctrl).", flush=True)
                return

    print("Could not auto-focus MuJoCo — click the window for keyboard control.", flush=True)


def run_glfw(
    args: argparse.Namespace,
    world: BikeWorld,
    controller: BalanceController | None,
) -> int:
    display = _default_display()
    if not display:
        print(
            "No DISPLAY is set, so the native MuJoCo window cannot open.\n"
            "Use:  python run_balance.py --no-viewer --duration 10",
            file=sys.stderr,
        )
        return 1

    os.environ.setdefault("MUJOCO_GL", "glfw")
    try:
        from mujoco import viewer as mj_viewer
    except Exception as exc:  # pragma: no cover - GUI optional
        print(f"Could not import the MuJoCo viewer ({exc}).", file=sys.stderr)
        return 1

    model, data = world.model, world.data
    cam_id = fixed_camera_id(model, world.flavor)
    cruise0 = args.speed if args.speed is not None else world.sample.cruise_vel
    ride = RideState(
        cruise0,
        steer_force=controller is None and world.flavor == "planar",
        flavor=world.flavor,
    )
    if controller is not None and world.flavor == "planar":
        # Speed-track in the window so arrow keys can change cruise.
        controller = BalanceController(gains_for_world(flat=False))

    def key_callback(key: int) -> None:
        if args.debug:
            print(f"debug: key={key}", flush=True)
        ride.on_key(key)

    viewer = mj_viewer.launch_passive(model, data, key_callback=key_callback)
    if world.flavor == "planar" and model.nhfield > 0:
        viewer.update_hfield(0)

    print("Launching the MuJoCo window. Click it, then drive with the keyboard:")
    if world.flavor == "new_bike":
        print("  Up          more rear torque")
        print("  Down        less / reverse rear torque")
        print("  Right       steer right")
        print("  Left        steer left")
        print("  0           stop")
        print("  Space       reset")
        print("No cascade balancer — free-drive the mesh bike (arrows only; WASD is for MuJoCo UI).")
    else:
        print("  Right       faster")
        print("  Left        slower (or reverse)")
        print("  Up          shove forward")
        print("  Down        shove backward")
        print("  0           stop")
        print("  Space       reset")
        print("The cascade controller keeps it upright while you set the speed.")
        print("Arrows only — WASD is reserved by MuJoCo Simulate for visibility toggles.")
    print(f"DISPLAY={display}")
    if shutil.which("xdotool") or shutil.which("wmctrl"):
        print("Trying to auto-focus the MuJoCo window…")
    else:
        print("Click the MuJoCo window for arrow keys (install xdotool to auto-focus).")
    print("Close the MuJoCo window to stop.")
    threading.Thread(target=raise_mujoco_window, args=(display,), daemon=True).start()

    dt = float(model.opt.timestep)
    max_steps = max(1, int(MAX_SIM_PER_FRAME / max(dt, 1e-6)))
    wall0 = time.perf_counter()
    sim0 = float(data.time)
    log_wall0 = wall0
    log_sim0 = sim0
    log_steps = 0
    fallen_at_wall: float | None = None
    fall_lim = FALL_LEAN_RAD if world.flavor == "new_bike" else FALL_PITCH_RAD
    try:
        while viewer.is_running():
            if cam_id is not None:
                with viewer.lock():
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                    viewer.cam.fixedcamid = cam_id

            if ride.reset_requested:
                reset_state(world, reroll=not args.flat, viewer=viewer, controller=controller)
                ride.reset_requested = False
                ride.shove = 0.0
                ride.rear_cmd = 0.0
                ride.steer_cmd = 0.0
                fallen_at_wall = None
                wall0 = time.perf_counter()
                sim0 = float(data.time)
                log_wall0 = wall0
                log_sim0 = sim0
                log_steps = 0

            lean = lean_rad(world)
            now = time.perf_counter()
            if lean > fall_lim:
                if fallen_at_wall is None:
                    fallen_at_wall = now
            else:
                fallen_at_wall = None

            should_reset = fallen_at_wall is not None and now - fallen_at_wall > 2.0
            if world.flavor == "planar":
                x = sensor(model, data, "x")
                should_reset = (
                    should_reset
                    or data.time >= max(args.duration, 12.0)
                    or abs(x) > 14.0
                )
            elif data.time >= max(args.duration, 30.0):
                should_reset = True

            if should_reset:
                reset_state(world, reroll=not args.flat, viewer=viewer, controller=controller)
                ride.rear_cmd = 0.0
                ride.steer_cmd = 0.0
                fallen_at_wall = None
                wall0 = time.perf_counter()
                sim0 = float(data.time)
                log_wall0 = wall0
                log_sim0 = sim0
                log_steps = 0
                now = wall0

            target_sim = sim0 + (now - wall0)
            frame_start = now
            steps = 0
            while data.time < target_sim and steps < max_steps:
                step_control(world, controller, ride.cruise, extra_force=ride.shove, ride=ride)
                ride.decay_shove()
                steps += 1
                log_steps += 1

            left, right = hud_text(world, controller, ride)
            viewer.set_texts(
                [
                    (
                        mujoco.mjtFontScale.mjFONTSCALE_200,
                        mujoco.mjtGridPos.mjGRID_TOPLEFT,
                        left,
                        right,
                    )
                ]
            )
            viewer.sync()

            now = time.perf_counter()
            wall_span = now - log_wall0
            if wall_span >= 2.0:
                sim_span = float(data.time) - log_sim0
                factor = sim_span / wall_span if wall_span > 0 else 0.0
                sps = log_steps / wall_span if wall_span > 0 else 0.0
                if world.flavor == "new_bike":
                    ctrl = f"rear={ride.rear_cmd:+.2f} steer={ride.steer_cmd:+.2f}"
                else:
                    ctrl = f"cruise={ride.cruise:+.2f} shove={ride.shove:+.1f}"
                line = (
                    f"timing: realtime={factor:.2f}x  steps/s={sps:.0f}  "
                    f"lean={math.degrees(lean_rad(world)):.1f}deg  {ctrl}"
                )
                if factor < REALTIME_WARN:
                    print(f"WARN: {line}", flush=True)
                elif args.debug:
                    print(f"debug: {line}", flush=True)
                else:
                    print(line, flush=True)
                log_wall0 = now
                log_sim0 = float(data.time)
                log_steps = 0

            # Pace display ~60 Hz when sim has caught wall clock.
            spent = time.perf_counter() - frame_start
            if float(data.time) >= target_sim - 1e-12 and spent < FRAME_PERIOD:
                time.sleep(FRAME_PERIOD - spent)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        try:
            viewer.close()
        except Exception:
            pass
    return 0


def run_web(
    args: argparse.Namespace,
    world: BikeWorld,
    controller: BalanceController | None,
) -> int:
    os.environ.setdefault("MUJOCO_GL", "egl")
    model, data = world.model, world.data
    renderer = mujoco.Renderer(model, height=720, width=1280)
    latest: dict[str, bytes | None] = {"jpeg": None}
    stop = threading.Event()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *log_args: object) -> None:
            return

        def do_GET(self) -> None:  # noqa: N802
            if self.path in ("/", "/index.html"):
                body = WEB_HTML.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path != "/stream":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while not stop.is_set():
                    frame = latest["jpeg"]
                    if frame is None:
                        time.sleep(0.02)
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ")
                    self.wfile.write(str(len(frame)).encode())
                    self.wfile.write(b"\r\n\r\n")
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    time.sleep(1 / 30)
            except BrokenPipeError:
                return

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f"http://{args.host}:{args.port}/"
    print(f"Live view: {url}")
    print("This is a browser stream. It will not open a native MuJoCo window.")
    print("Ctrl+C to stop.")

    times: list[float] = []
    pitches: list[float] = []
    fallen_at: float | None = None
    cam = fixed_camera_id(model, world.flavor)
    cam_name = None
    if cam is not None:
        cam_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, cam)
    recorder = (
        VideoRecorder(model, data, args.save_video, camera=cam) if args.save_video else None
    )
    fall_lim = FALL_LEAN_RAD if world.flavor == "new_bike" else FALL_PITCH_RAD
    last_wall = time.perf_counter()
    try:
        while not stop.is_set():
            if data.time >= args.duration:
                reset_state(world, reroll=not args.flat, controller=controller)
                fallen_at = None
                continue
            pitch = step_control(world, controller, args.speed)
            times.append(data.time)
            pitches.append(pitch)
            if cam_name is not None:
                renderer.update_scene(data, camera=cam_name)
            else:
                renderer.update_scene(data)
            latest["jpeg"] = _jpeg(renderer.render())
            if recorder is not None:
                recorder.maybe_capture(data)
            if fallen_at is None and abs(pitch) > fall_lim:
                fallen_at = data.time
                time.sleep(0.4)
                reset_state(world, reroll=not args.flat, controller=controller)
                fallen_at = None
                continue
            elapsed = time.perf_counter() - last_wall
            sleep_for = model.opt.timestep - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)
            last_wall = time.perf_counter()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        stop.set()
        server.shutdown()
        if recorder is not None:
            recorder.close()
    return summarize(args, times, pitches, fallen_at, model_path=world.model_path)


def _jpeg(rgb: np.ndarray) -> bytes:
    from PIL import Image

    buf = BytesIO()
    Image.fromarray(rgb).save(buf, format="JPEG", quality=80)
    return buf.getvalue()


class VideoRecorder:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        path: Path,
        camera: int | None = None,
    ) -> None:
        import subprocess

        self.path = path.expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.renderer = mujoco.Renderer(model, height=720, width=1280)
        self.camera = camera
        if camera is not None:
            self.camera_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        else:
            self.camera_name = None
        self.interval = 1.0 / 30.0
        self.next_t = 0.0
        self.proc = subprocess.Popen(
            [
                "ffmpeg",
                "-y",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                "1280x720",
                "-r",
                "30",
                "-i",
                "-",
                "-an",
                "-pix_fmt",
                "yuv420p",
                "-c:v",
                "libx264",
                "-loglevel",
                "error",
                str(self.path),
            ],
            stdin=subprocess.PIPE,
        )

    def maybe_capture(self, data: mujoco.MjData) -> None:
        if data.time + 1e-9 < self.next_t:
            return
        if self.camera_name is not None:
            self.renderer.update_scene(data, camera=self.camera_name)
        else:
            self.renderer.update_scene(data)
        frame = np.ascontiguousarray(self.renderer.render())
        assert self.proc.stdin is not None
        self.proc.stdin.write(frame.tobytes())
        self.next_t += self.interval

    def close(self) -> None:
        if self.proc.stdin is not None:
            self.proc.stdin.close()
        self.proc.wait()
        print(f"video:      {self.path}")


def _save_plot(path: Path, times: list[float], pitch_deg: np.ndarray) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path = path.expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 3.4))
    ax.plot(times, pitch_deg, color="#e06a2b", linewidth=1.6)
    ax.axhline(0.0, color="#888", linewidth=0.8)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("pitch (deg)")
    ax.set_title("Bicycle pitch while balancing")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    print(f"plot:       {path}")


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
