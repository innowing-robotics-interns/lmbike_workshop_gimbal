"""GLFW remote / optional viewer with wall-clock catch-up."""

from __future__ import annotations

import math
import os
import sys
import time

import mujoco

from .config import Config
from .joystick import HardwareJoystick, JoystickWindow, StickInput
from .loop import RideGoals, SimHandles, reset_sim, shove_bicycle_sideways, step_physics

KEY_SPACE, KEY_0 = 32, 48
KEY_RIGHT, KEY_LEFT, KEY_DOWN, KEY_UP = 262, 263, 264, 265
_ARROW_KEYS = {
    KEY_UP: "up",
    KEY_DOWN: "down",
    KEY_LEFT: "left",
    KEY_RIGHT: "right",
}
# MuJoCo only reports key-down. A key counts as released when repeats stop.
_KEY_HOLD_S = 0.18
FRAME_PERIOD = 1.0 / 60.0
MAX_SIM_PER_FRAME = 0.05


def apply_key(goals: RideGoals, key: int, cfg: Config, *, pressed: bool = True) -> None:
    """Momentary arrow keys. They spring back to center on release."""
    del cfg
    if key == KEY_UP:
        goals.stick_y = 1.0 if pressed else 0.0
    elif key == KEY_DOWN:
        goals.stick_y = -1.0 if pressed else 0.0
    elif key == KEY_LEFT:
        goals.stick_x = 1.0 if pressed else 0.0
    elif key == KEY_RIGHT:
        goals.stick_x = -1.0 if pressed else 0.0
    elif key == KEY_0 and pressed:
        goals.stop()
    elif key == KEY_SPACE and pressed:
        goals.reset_requested = True


def _linux_display() -> str | None:
    """A display GLFW can use, if this Linux machine has one."""
    if sys.platform != "linux":
        return "native"
    for name in ("DISPLAY", "WAYLAND_DISPLAY"):
        if os.environ.get(name):
            return os.environ[name]
    for candidate in (":1", ":0"):
        socket_path = os.path.join(os.sep, "tmp", ".X11-unix", "X" + candidate[1:])
        if os.path.exists(socket_path):
            os.environ["DISPLAY"] = candidate
            return candidate
    return None


def _ensure_gui_display() -> None:
    """Linux GLFW needs a display. Windows and macOS use their own window system."""
    if _linux_display() is None:
        raise RuntimeError(
            "No display found. Start this from a desktop session so the MuJoCo window can open."
        )


def _play_inline(sim: SimHandles) -> None:
    """One ride drawn in the notebook. Colab has no window and no GLFW."""
    print(
        "No desktop window here. Playing one ride in the notebook. "
        "Arrow keys and the stick window are not available.",
        flush=True,
    )
    os.environ.setdefault("MUJOCO_GL", "egl")
    duration = min(float(sim.cfg.duration), 8.0)
    height, width, fps = 240, 320, 8
    renderer = mujoco.Renderer(sim.model, height=height, width=width)
    cam_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, "chasis_camera")
    camera = "chasis_camera" if cam_id >= 0 else -1
    frame_dt = 1.0 / fps
    next_frame = 0.0
    frames: list = []
    try:
        while float(sim.data.time) < duration:
            step_physics(sim)
            if float(sim.data.time) + 1e-9 >= next_frame:
                renderer.update_scene(sim.data, camera=camera)
                frames.append(renderer.render().copy())
                next_frame += frame_dt
    finally:
        renderer.close()

    if not frames:
        print("Ride finished with no frames.")
        return

    import matplotlib.pyplot as plt
    from IPython.display import HTML, display
    from matplotlib import animation

    fig, ax = plt.subplots(figsize=(6, 4.5))
    ax.axis("off")
    image = ax.imshow(frames[0])

    def _draw(index: int):
        image.set_data(frames[index])
        return (image,)

    anim = animation.FuncAnimation(
        fig,
        _draw,
        frames=len(frames),
        interval=1000.0 / fps,
        blit=True,
    )
    html = anim.to_jshtml()
    plt.close(fig)
    display(HTML(html))
    print(f"Ride finished ({duration:.1f}s).", flush=True)


def run_viewer(sim: SimHandles) -> None:
    if "google.colab" in sys.modules:
        _play_inline(sim)
        return
    _ensure_gui_display()
    os.environ.setdefault("MUJOCO_GL", "glfw")
    from mujoco import viewer as mj_viewer

    cfg = sim.cfg
    stick = StickInput()
    hardware = HardwareJoystick()
    window: JoystickWindow | None = None
    if sim.mode == "remote":
        window = JoystickWindow(stick)
        window.start()
    key_until: dict[int, float] = {}

    def key_callback(key: int) -> None:
        if sim.debug:
            print(f"debug: key={key}", flush=True)
        if key in _ARROW_KEYS:
            key_until[key] = time.perf_counter() + _KEY_HOLD_S
            stick.set_key(_ARROW_KEYS[key], True)
            return
        apply_key(sim.goals, key, cfg)

    handle = mj_viewer.launch_passive(sim.model, sim.data, key_callback=key_callback)
    print("Remote stick (floating window, or hold arrows — they grow, then spring back):")
    print("  Up/Down     speed around cruise")
    print("  Left/Right  turn rate")
    print("  0           stop")
    print("  Space / Reset button  reset upright")
    print("  Push sideways button  a shove from the side; Strength sets how hard")
    print(
        f"Stick centered: cruise at {cfg.target_speed:.2f} m/s and hold heading.",
        flush=True,
    )

    cam_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, "chasis_camera")
    dt = float(sim.model.opt.timestep)
    max_steps = max(1, int(MAX_SIM_PER_FRAME / max(dt, 1e-6)))
    wall0 = time.perf_counter()
    sim0 = float(sim.data.time)
    last_input = time.perf_counter()

    try:
        while handle.is_running():
            now_keys = time.perf_counter()
            stick.advance_keys(now_keys - last_input)
            last_input = now_keys
            for key, name in _ARROW_KEYS.items():
                if key in key_until and key_until[key] <= now_keys:
                    stick.set_key(name, False)
                    del key_until[key]
            hx, hy = hardware.poll()
            stick.set_hardware(hx, hy)
            if window is not None:
                window.poll()
            if stick.consume_reset():
                sim.goals.reset_requested = True
            sim.goals.stick_x, sim.goals.stick_y = stick.axes()
            push_speed = stick.consume_push()
            if push_speed is not None:
                shove_bicycle_sideways(sim, push_speed)

            if sim.goals.reset_requested:
                reset_sim(sim)
                wall0 = time.perf_counter()
                sim0 = float(sim.data.time)
                if window is not None:
                    window.set_fallen(False)

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

                _, roll, _, yaw = bicycle_state_from_imu(
                    float(sim.data.sensor("steering_joint_pos_sensor").data[0]),
                    sim.data.sensor("ori_global").data,
                    sim.data.sensor("gyro_local").data,
                    sim.sensor_to_bike,
                    sim.gyro_bias_sensor,
                )
                lean = abs(roll)
                yaw_meas = float(yaw)
            except Exception:
                yaw_meas = None

            if window is not None:
                window.set_fallen(math.degrees(lean) > 45.0)

            speed_goal = cfg.target_speed if sim.goals.speed_goal is None else sim.goals.speed_goal
            if yaw_meas is None or sim.goals.captured_heading is None or abs(sim.goals.stick_x) >= 0.08:
                heading_text = f"stick {sim.goals.stick_x:+.2f}"
            else:
                heading_text = f"hold {math.degrees(sim.goals.captured_heading):+.0f} / {math.degrees(yaw_meas):+.0f}"
            left = "Stick\nSpeed\nHeading\nLean"
            right = (
                f"x {sim.goals.stick_x:+.2f}  y {sim.goals.stick_y:+.2f}\n"
                f"{speed_goal:+.2f} m/s\n"
                f"{heading_text}\n"
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
        if window is not None:
            window.close()
        hardware.close()
        try:
            handle.close()
        except Exception:
            pass
