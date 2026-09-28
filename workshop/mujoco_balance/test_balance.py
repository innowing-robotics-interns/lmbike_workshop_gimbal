"""Headless checks for the planar bicycle balancer."""

from __future__ import annotations

import math
import os
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

from controller import BalanceController, gains_for_world
from environment import (
    BikeWorld,
    ground_height,
    local_slope_deg,
    nearest_hill,
    resolve_model_path,
    slope_assist_force,
)

HERE = Path(__file__).resolve().parent
MODEL_PATH = HERE / "models" / "bike.xml"
NEW_BIKE_PATH = HERE / "models" / "new_bike.xml"
RUNNER = HERE / "run_balance.py"


def test_offscreen_frame_is_not_black() -> None:
    os.environ.setdefault("MUJOCO_GL", "egl")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=180, width=320)
    renderer.update_scene(data, camera="side")
    frame = renderer.render()
    assert frame.shape == (180, 320, 3)
    assert int(frame.max()) > 10


def test_model_compiles() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    assert model.nq == 5
    assert model.nu == 3
    assert model.geom("terrain").type == mujoco.mjtGeom.mjGEOM_HFIELD


def test_pd_holds_an_eight_degree_lean() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    ctrl = BalanceController()
    data.joint("pitch").qpos[0] = math.radians(8.0)
    mujoco.mj_forward(model, data)

    max_abs = 0.0
    while data.time < 4.0:
        pitch = float(data.sensordata[model.sensor("pitch").id])
        pitch_vel = float(data.sensordata[model.sensor("pitch_vel").id])
        x = float(data.sensordata[model.sensor("x").id])
        x_vel = float(data.sensordata[model.sensor("x_vel").id])
        torque = ctrl.torque(pitch, pitch_vel, x, x_vel)
        data.ctrl[model.actuator("drive").id] = torque
        data.ctrl[model.actuator("rear_drive").id] = 0.12 * torque
        data.ctrl[model.actuator("front_drive").id] = 0.12 * torque
        mujoco.mj_step(model, data)
        max_abs = max(max_abs, abs(pitch))

    assert data.time >= 4.0
    assert max_abs < math.radians(20.0)
    assert abs(float(data.sensordata[model.sensor("pitch").id])) < math.radians(5.0)


def test_uncontrolled_bike_falls() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    data.joint("pitch").qpos[0] = math.radians(8.0)
    mujoco.mj_forward(model, data)
    while data.time < 3.0:
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        if abs(float(data.sensordata[model.sensor("pitch").id])) > math.radians(45.0):
            break
    assert abs(float(data.sensordata[model.sensor("pitch").id])) > math.radians(45.0)


def test_cli_headless_success() -> None:
    result = subprocess.run(
        [sys.executable, str(RUNNER), "--no-viewer", "--flat", "--lean-deg", "8", "--duration", "3"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "SUCCESS" in result.stdout


def test_flat_world_keeps_bike_on_ground() -> None:
    world = BikeWorld(flat=True, seed=1, lean_deg=8.0)
    assert world.sample.flat
    assert world.model.nq == 5
    z = float(world.data.sensordata[world.model.sensor("z").id])
    assert abs(z) < 0.08


def test_random_world_has_bumps_and_runs() -> None:
    world = BikeWorld(flat=False, seed=7, lean_deg=6.0)
    assert len(world.sample.bumps) >= 3
    assert 0.06 <= world.sample.peak_height <= 0.35
    z = float(world.data.sensordata[world.model.sensor("z").id])
    assert abs(z) < 0.08
    assert world.sample.friction != 1.5 or world.sample.rider_mass != 16.0
    ctrl = BalanceController()
    max_abs = 0.0
    while world.data.time < 2.0:
        pitch = float(world.data.sensordata[world.model.sensor("pitch").id])
        x = float(world.data.sensordata[world.model.sensor("x").id])
        force = ctrl.torque(
            pitch,
            float(world.data.sensordata[world.model.sensor("pitch_vel").id]),
            x,
            float(world.data.sensordata[world.model.sensor("x_vel").id]),
            x_vel_ref=world.sample.cruise_vel,
            slope_deg=local_slope_deg(x, world.sample),
            mass=world.sample.rider_mass + 5.6,
            dt=float(world.model.opt.timestep),
        )
        world.data.ctrl[world.model.actuator("drive").id] = force + world.sample.wind
        mujoco.mj_step(world.model, world.data)
        max_abs = max(max_abs, abs(pitch))
        if abs(pitch) > math.radians(50):
            break
    assert world.data.time > 0.3
    assert max_abs < math.radians(55)


def test_local_slope_is_nonzero_on_a_bump() -> None:
    world = BikeWorld(flat=False, seed=3, lean_deg=5.0)
    bump = world.sample.bumps[0]
    climb_x = bump.x - 0.6 * bump.width
    assert ground_height(bump.x, world.sample) > 0.05
    assert local_slope_deg(climb_x, world.sample) > 2.0
    assert slope_assist_force(climb_x, world.sample) > 0.0
    flat = BikeWorld(flat=True, seed=1, lean_deg=8.0)
    assert slope_assist_force(2.0, flat.sample) == 0.0
    assert ground_height(2.0, flat.sample) == 0.0
    found = nearest_hill(0.0, world.sample)
    assert found is not None
    _, bump, dist = found
    assert dist > 1.0
    assert bump.height > 0.05


def test_bumpy_world_render_shows_bike_not_a_painted_ramp() -> None:
    os.environ.setdefault("MUJOCO_GL", "egl")
    world = BikeWorld(flat=False, seed=11, lean_deg=5.0)
    renderer = mujoco.Renderer(world.model, height=360, width=640)
    renderer.update_scene(world.data, camera="side")
    frame = renderer.render()
    assert frame.max() > 10
    orange = ((frame[:, :, 0] > 160) & (frame[:, :, 1] < 120) & (frame[:, :, 2] < 80)).sum()
    assert orange < 400, f"giant orange ramp still in view: {orange} pixels"
    brown = ((frame[:, :, 0] > 90) & (frame[:, :, 0] < 200) & (frame[:, :, 1] > 50) & (frame[:, :, 1] < 140) & (frame[:, :, 2] < 90)).sum()
    yellow = ((frame[:, :, 0] > 180) & (frame[:, :, 1] > 140) & (frame[:, :, 2] < 80)).sum()
    assert brown + yellow > 200, f"hill markers missing: brown={brown} yellow={yellow}"
    cy, cx = frame.shape[0] // 2, frame.shape[1] // 2
    crop = frame[cy - 80 : cy + 80, cx - 120 : cx + 120]
    assert crop.max() - crop.min() > 40
    # Height field actually varies; this is not a tilted floor.
    h = np.asarray(world.model.hfield_data).reshape(world.model.hfield_nrow[0], world.model.hfield_ncol[0])
    assert h.max() > 0.12
    assert (h[0] < 0.05).mean() > 0.4


def test_forward_lean_produces_forward_drive() -> None:
    ctrl = BalanceController(gains_for_world(flat=True))
    force = ctrl.torque(math.radians(6.0), 0.0, 0.0, 0.0)
    assert force > 20.0


def test_too_slow_asks_for_forward_lean() -> None:
    ctrl = BalanceController(gains_for_world(flat=False))
    ctrl.torque(0.0, 0.0, 0.0, 0.0, x_vel_ref=0.8)
    assert ctrl.last.pitch_ref_deg > 1.5


def test_controller_tracks_cruise_on_flat() -> None:
    world = BikeWorld(flat=True, seed=2, lean_deg=4.0)
    ctrl = BalanceController(gains_for_world(flat=False))
    cruise = 0.6
    while world.data.time < 5.0:
        pitch = float(world.data.sensordata[world.model.sensor("pitch").id])
        x = float(world.data.sensordata[world.model.sensor("x").id])
        x_vel = float(world.data.sensordata[world.model.sensor("x_vel").id])
        force = ctrl.torque(
            pitch,
            float(world.data.sensordata[world.model.sensor("pitch_vel").id]),
            x,
            x_vel,
            x_vel_ref=cruise,
            dt=float(world.model.opt.timestep),
        )
        world.data.ctrl[world.model.actuator("drive").id] = force
        mujoco.mj_step(world.model, world.data)
        if abs(pitch) > math.radians(40):
            break
    x_vel = float(world.data.sensordata[world.model.sensor("x_vel").id])
    pitch = float(world.data.sensordata[world.model.sensor("pitch").id])
    assert world.data.time >= 4.9
    assert abs(x_vel - cruise) < 0.28
    assert abs(pitch) < math.radians(12)


def test_controller_clears_the_first_hill() -> None:
    world = BikeWorld(flat=False, seed=11, lean_deg=5.0)
    ctrl = BalanceController(gains_for_world(flat=False))
    first = world.sample.bumps[0]
    while world.data.time < 8.0:
        pitch = float(world.data.sensordata[world.model.sensor("pitch").id])
        x = float(world.data.sensordata[world.model.sensor("x").id])
        force = ctrl.torque(
            pitch,
            float(world.data.sensordata[world.model.sensor("pitch_vel").id]),
            x,
            float(world.data.sensordata[world.model.sensor("x_vel").id]),
            x_vel_ref=max(world.sample.cruise_vel, 0.55),
            slope_deg=local_slope_deg(x, world.sample),
            mass=world.sample.rider_mass + 5.6,
            dt=float(world.model.opt.timestep),
        )
        world.data.ctrl[world.model.actuator("drive").id] = force + world.sample.wind
        mujoco.mj_step(world.model, world.data)
        if abs(pitch) > math.radians(50):
            break
    x = float(world.data.sensordata[world.model.sensor("x").id])
    pitch = float(world.data.sensordata[world.model.sensor("pitch").id])
    assert x > first.x + 0.25
    assert abs(pitch) < math.radians(45)


def test_keyboard_pilot_changes_cruise_and_reset() -> None:
    from run_balance import KEY_0, KEY_LEFT, KEY_RIGHT, KEY_SPACE, RideState

    ride = RideState(0.50)
    ride.on_key(KEY_RIGHT)
    assert abs(ride.cruise - 0.65) < 1e-9
    ride.on_key(KEY_LEFT)
    ride.on_key(KEY_LEFT)
    assert ride.cruise < 0.50
    ride.on_key(KEY_0)
    assert ride.cruise == 0.0
    ride.on_key(KEY_SPACE)
    assert ride.reset_requested


def test_resolve_model_path_aliases() -> None:
    assert resolve_model_path("bike") == MODEL_PATH
    assert resolve_model_path("new_bike") == NEW_BIKE_PATH
    assert resolve_model_path("new_bike.xml") == NEW_BIKE_PATH


def test_new_bike_model_loads() -> None:
    meshes = HERE / "models" / "step_meshes"
    if not meshes.is_dir() or not any(meshes.glob("*.stl")):
        return  # meshes optional in some checkouts
    world = BikeWorld(model_path="new_bike", seed=1)
    assert world.flavor == "new_bike"
    assert world.model.nu == 2
    assert abs(float(world.model.opt.timestep) - 0.002) < 1e-9
    assert world.timestep_overridden
    assert mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "cmd_rearwheel_f") >= 0
    from run_balance import KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_UP, RideState, step_control

    ride = RideState(0.0, flavor="new_bike")
    ride.on_key(KEY_UP)
    ride.on_key(KEY_RIGHT)
    assert ride.rear_cmd > 0
    assert ride.steer_cmd > 0
    step_control(world, None, ride=ride)
    assert world.data.time > 0
    before_rear, before_steer = ride.rear_cmd, ride.steer_cmd
    ride.on_key(KEY_DOWN)
    ride.on_key(KEY_LEFT)
    assert ride.rear_cmd < before_rear
    assert ride.steer_cmd < before_steer
    world.reset(reroll=False)
