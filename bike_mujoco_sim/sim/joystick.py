"""Spring-centered stick for remote mode, plus a floating joystick window.

The stick is not sticky: it is zero unless the mouse, an arrow key, or a
hardware axis is held. Releasing returns speed to the balance cruise and
stops the turn.
"""

from __future__ import annotations

import os
import struct
import threading

from .config import Config

STICK_DEADZONE = 0.08
_JS_EVENT = struct.Struct("IhBB")
_JS_AXIS = 0x02
_JS_INIT = 0x80


# Arrow keys grow toward full deflection while held, then drop back quickly.
_KEY_RISE_PER_S = 2.4
_KEY_FALL_PER_S = 12.0


class StickInput:
    """Combine mouse, keyboard, and an optional /dev/input/js device."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._mouse = (0.0, 0.0)
        self._mouse_down = False
        self._keys: set[str] = set()
        self._key_axes = (0.0, 0.0)
        self._hardware = (0.0, 0.0)
        self._reset_pending = False
        self._push_pending = False
        self._push_speed = 0.8

    def set_mouse(self, x: float, y: float, down: bool) -> None:
        with self._lock:
            self._mouse = (_clip(x), _clip(y))
            self._mouse_down = bool(down)

    def set_key(self, name: str, pressed: bool) -> None:
        with self._lock:
            if pressed:
                self._keys.add(name)
            else:
                self._keys.discard(name)

    def set_hardware(self, x: float, y: float) -> None:
        with self._lock:
            self._hardware = (_clip(x), _clip(y))

    def request_reset(self) -> None:
        with self._lock:
            self._reset_pending = True
            self._mouse = (0.0, 0.0)
            self._mouse_down = False
            self._keys.clear()
            self._key_axes = (0.0, 0.0)
            self._hardware = (0.0, 0.0)

    def request_push(self, speed_m_s: float = 0.8) -> None:
        with self._lock:
            self._push_pending = True
            self._push_speed = float(speed_m_s)

    def consume_push(self) -> float | None:
        with self._lock:
            if not self._push_pending:
                return None
            self._push_pending = False
            return self._push_speed

    def advance_keys(self, dt: float) -> None:
        """Grow the arrow-key stick while a key is held, and drop it after release."""
        dt = max(0.0, min(float(dt), 0.05))
        with self._lock:
            target_x = (1.0 if "left" in self._keys else 0.0) - (1.0 if "right" in self._keys else 0.0)
            target_y = (1.0 if "up" in self._keys else 0.0) - (1.0 if "down" in self._keys else 0.0)
            x, y = self._key_axes
            self._key_axes = (
                _slew_axis(x, target_x, dt),
                _slew_axis(y, target_y, dt),
            )

    def consume_reset(self) -> bool:
        with self._lock:
            pending = self._reset_pending
            self._reset_pending = False
            return pending

    def mouse_active(self) -> bool:
        with self._lock:
            return self._mouse_down

    def axes(self) -> tuple[float, float]:
        """Return (x, y) in [-1, 1]. +x is left, +y is faster."""
        with self._lock:
            if self._mouse_down:
                return self._mouse
            x, y = self._key_axes
            if abs(x) > 1e-3 or abs(y) > 1e-3:
                return (_clip(x), _clip(y))
            return self._hardware


def speed_goal_from_stick(stick_y: float, cfg: Config, *, deadzone: float = STICK_DEADZONE) -> float | None:
    """Map stick Y to a speed goal. Center returns None so the cruise ramp is used."""
    y = _clip(stick_y)
    if abs(y) < deadzone:
        return None
    if y > 0.0:
        return float(cfg.target_speed + y * (cfg.speed_goal_max - cfg.target_speed))
    return float(cfg.target_speed + y * (cfg.target_speed - cfg.speed_goal_min))


def _input_device_name(path: str) -> str:
    js = os.path.basename(path)
    name_path = os.path.join(os.path.dirname(os.path.realpath(f"/sys/class/input/{js}")), "name")
    try:
        with open(name_path, encoding="utf-8", errors="replace") as handle:
            return handle.read().strip()
    except OSError:
        return ""


def axes_from_init_snapshot(values: list[int]) -> bool:
    """True when an open() snapshot is a real stick position, not a pinned dummy.

    The kernel emits JS_EVENT_INIT for every axis. A gamepad at rest is near 0.
    Devices that only pretend to be joysticks, such as an LED controller, pin
    every axis to the same rail. Those must stay at (0, 0).
    """
    if not values:
        return True
    return not (len(set(values)) == 1 and abs(values[0]) >= 32767)


class HardwareJoystick:
    """Read the first Linux joystick. Missing or dummy devices stay at (0, 0)."""

    def __init__(self) -> None:
        self._fd: int | None = None
        self._axes = {0: 0.0, 1: 0.0}
        for path in ("/dev/input/js0", "/dev/input/js1"):
            if not os.path.exists(path):
                continue
            if "led" in _input_device_name(path).lower():
                continue
            try:
                self._fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except OSError:
                self._fd = None
                continue
            if not self._accept_initial_state():
                self.close()
                continue
            break

    def _accept_initial_state(self) -> bool:
        """Apply a real stick's initial pose. Reject a device pinned to one rail."""
        events = self._read_available()
        axis_values = [value for _t, value, etype, _n in events if etype & _JS_AXIS]
        if not axes_from_init_snapshot(axis_values):
            return False
        self._apply(events)
        return True

    def _read_available(self) -> list[tuple[int, int, int, int]]:
        fd = self._fd
        if fd is None:
            return []
        events: list[tuple[int, int, int, int]] = []
        while True:
            try:
                raw = os.read(fd, _JS_EVENT.size)
            except BlockingIOError:
                return events
            except OSError:
                self._fd = None
                return events
            if len(raw) < _JS_EVENT.size:
                return events
            events.append(_JS_EVENT.unpack(raw))

    def _apply(self, events: list[tuple[int, int, int, int]]) -> None:
        for _time_ms, value, etype, number in events:
            if etype & _JS_AXIS and number in self._axes:
                self._axes[number] = max(-1.0, min(1.0, value / 32767.0))

    def poll(self) -> tuple[float, float]:
        if self._fd is None:
            return (0.0, 0.0)
        self._apply(self._read_available())
        # Axis 0 is positive to the right, axis 1 positive downward.
        # Stick +x is a left turn and +y is faster, so both axes are flipped.
        return (-self._axes[0], -self._axes[1])

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None


class JoystickWindow:
    """Always-on-top stick. Call poll() from the simulation thread; Tk stays there."""

    def __init__(self, stick: StickInput) -> None:
        self.stick = stick
        self._root = None
        self._place = None
        self._set_fallen = None
        self._fallen = False

    def start(self) -> None:
        import tkinter as tk

        root = tk.Tk()
        self._root = root
        root.title("Bike joystick")
        root.attributes("-topmost", True)
        root.resizable(False, False)
        root.geometry("280x460+60+80")
        root.configure(bg="#1c1f24")

        radius = 92
        knob_r = 22
        cx = cy = 120
        canvas = tk.Canvas(root, width=240, height=240, bg="#1c1f24", highlightthickness=0)
        canvas.pack(pady=(12, 0))
        canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, fill="#2a3038", outline="#8aa0b8", width=2)
        knob = canvas.create_oval(cx - knob_r, cy - knob_r, cx + knob_r, cy + knob_r, fill="#e07a3d", outline="#f4d2b8", width=2)
        label = tk.Label(root, text="center: cruise, straight", fg="#d5dde6", bg="#1c1f24", font=("Sans", 11))
        label.pack(pady=8)
        status = tk.Label(root, text="", fg="#e07a3d", bg="#1c1f24", font=("Sans", 10))
        status.pack()
        tk.Label(
            root,
            text="Hold an arrow key and the stick grows.\nLet go and it returns to center quickly.",
            fg="#8aa0b8",
            bg="#1c1f24",
            font=("Sans", 9),
        ).pack()

        def place_knob(x: float, y: float) -> None:
            px = cx - x * radius
            py = cy - y * radius
            canvas.coords(knob, px - knob_r, py - knob_r, px + knob_r, py + knob_r)

        def set_from_pointer(event: tk.Event) -> None:
            dx = float(event.x - cx)
            dy = float(event.y - cy)
            scale = (dx * dx + dy * dy) ** 0.5
            if scale > radius:
                dx *= radius / scale
                dy *= radius / scale
            sx = -dx / radius
            sy = -dy / radius
            self.stick.set_mouse(sx, sy, True)
            place_knob(sx, sy)
            label.configure(text=f"turn {sx:+.2f}   speed {sy:+.2f}")

        def release(_event: tk.Event) -> None:
            self.stick.set_mouse(0.0, 0.0, False)
            place_knob(0.0, 0.0)
            label.configure(text="center: cruise, straight")

        canvas.bind("<Button-1>", set_from_pointer)
        canvas.bind("<B1-Motion>", set_from_pointer)
        canvas.bind("<ButtonRelease-1>", release)

        key_names = {"Up": "up", "Down": "down", "Left": "left", "Right": "right"}

        def on_press(event: tk.Event) -> None:
            if event.keysym in ("space", "r", "R"):
                self.stick.request_reset()
                place_knob(0.0, 0.0)
                label.configure(text="center: cruise, straight")
                return
            name = key_names.get(event.keysym)
            if name:
                self.stick.set_key(name, True)

        def on_release(event: tk.Event) -> None:
            name = key_names.get(event.keysym)
            if name:
                self.stick.set_key(name, False)

        def do_reset() -> None:
            self.stick.request_reset()
            place_knob(0.0, 0.0)
            label.configure(text="center: cruise, straight")
            status.configure(text="")
            self._fallen = False
            reset_btn.configure(bg="#3a4553", activebackground="#4a5666")

        def do_push() -> None:
            try:
                speed = float(speed_var.get())
            except (TypeError, ValueError):
                speed = 0.8
            self.stick.request_push(speed)

        push_row = tk.Frame(root, bg="#1c1f24")
        push_row.pack(pady=(8, 0))
        tk.Label(push_row, text="Strength (m/s)", fg="#d5dde6", bg="#1c1f24", font=("Sans", 10)).pack(side="left", padx=(0, 6))
        speed_var = tk.StringVar(value="0.8")
        tk.Spinbox(
            push_row,
            from_=0.1,
            to=2.0,
            increment=0.1,
            textvariable=speed_var,
            width=5,
            font=("Sans", 11),
            format="%.1f",
        ).pack(side="left")
        push_btn = tk.Button(
            root,
            text="Push sideways",
            command=do_push,
            bg="#3a4553",
            fg="#f0f4f8",
            activebackground="#4a5666",
            activeforeground="#ffffff",
            relief="flat",
            padx=18,
            pady=8,
            font=("Sans", 11, "bold"),
        )
        push_btn.pack(pady=(8, 0))

        reset_btn = tk.Button(
            root,
            text="Reset bike",
            command=do_reset,
            bg="#3a4553",
            fg="#f0f4f8",
            activebackground="#4a5666",
            activeforeground="#ffffff",
            relief="flat",
            padx=18,
            pady=8,
            font=("Sans", 11, "bold"),
        )
        reset_btn.pack(pady=(10, 12))

        root.bind("<KeyPress>", on_press)
        root.bind("<KeyRelease>", on_release)
        root.protocol("WM_DELETE_WINDOW", self.close)

        def follow() -> None:
            if self.stick.mouse_active():
                return
            x, y = self.stick.axes()
            place_knob(x, y)
            if x or y:
                label.configure(text=f"turn {x:+.2f}   speed {y:+.2f}")
            else:
                label.configure(text="center: cruise, straight")

        def set_fallen(fallen: bool) -> None:
            if fallen == self._fallen:
                return
            self._fallen = fallen
            if fallen:
                status.configure(text="Bike down — press Reset")
                reset_btn.configure(bg="#b5452a", activebackground="#d45636")
            else:
                status.configure(text="")
                reset_btn.configure(bg="#3a4553", activebackground="#4a5666")

        self._place = follow
        self._set_fallen = set_fallen
        root.update()

    def set_fallen(self, fallen: bool) -> None:
        if self._set_fallen is not None:
            self._set_fallen(fallen)

    def poll(self) -> None:
        import tkinter as tk

        root = self._root
        if root is None:
            return
        if self._place is not None:
            self._place()
        try:
            root.update()
        except tk.TclError:
            self._root = None

    def close(self) -> None:
        import tkinter as tk

        root = self._root
        self._root = None
        if root is None:
            return
        try:
            root.destroy()
        except tk.TclError:
            pass


def _clip(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


def _slew_axis(current: float, target: float, dt: float) -> float:
    rate = _KEY_FALL_PER_S if target == 0.0 else _KEY_RISE_PER_S
    step = rate * dt
    if current < target:
        return min(target, current + step)
    return max(target, current - step)
