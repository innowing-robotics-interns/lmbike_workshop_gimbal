# PID MuJoCo bike sim

Notebook-07 dual-loop balance (50 Hz outer + steer PI + rear P) with a
**roll–steer PID** outer loop instead of LQR / pole placement.

Student lessons (each file stands alone): follow [HOW_TO_RUN_NOTEBOOK.md](HOW_TO_RUN_NOTEBOOK.md), then open any of
`tutorials/01_environment_and_the_fall.ipynb`,
`tutorials/02_pid_and_the_struggle.ipynb`,
`tutorials/03_tuning_and_balance.ipynb`,
`tutorials/04_teleop_and_disturbance.ipynb`,
`tutorials/05_escaping_the_sandbox.ipynb`.

## Setup

```bash
cd bike_mujoco_sim
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
# Notebook-style speed ramp (GLFW; default 10 s then reset)
python run_sim.py --mode speed-schedule

# Same on the CAD new_bike (unlimited 360° steer)
python run_sim.py --mode speed-schedule --model new_bike

# Arrow-key remote
python run_sim.py --mode remote --model orange_bike
python run_sim.py --mode remote --model new_bike --debug

# No window (tests / batch)
python run_sim.py --headless --mode speed-schedule --duration 10

# Custom YAML (copy configs/default.yaml and edit)
python run_sim.py --config configs/default.yaml --kp-roll 10

# new_bike uses configs/new_bike.yaml automatically
python run_sim.py --mode speed-schedule --model new_bike
```

The MuJoCo window opens unless you pass `--headless`. Tunables live in **`configs/*.yaml`**. CLI flags override YAML. `--model new_bike` loads `configs/new_bike.yaml` (lean→torque PD); `orange_bike` uses `configs/default.yaml` (cascade).

### `--model`

| Alias | File |
| --- | --- |
| `orange_bike` (default) | `models/orange_bike/orange_bike_horizontal.xml` |
| `new_bike` | `models/new_bike/new_bike.xml` (vendored; steer joint **unlimited**) |

Both expose `cmd_steering_f` / `cmd_rearwheel_f` and the IMU / joint sensors used by the notebook.

`new_bike` is heavier and **raked**; vendored MJCF has **unlimited** steering.
Use `configs/new_bike.yaml` (`balance_mode: lean_torque`) — the orange cascade
rate PID does not stabilize it. Standstill upright hold (`|speed_goal| < 0.1`)
is still fragile on both models.

### `--mode`

| Mode | Behavior |
| --- | --- |
| `speed-schedule` | Ramp to `target_speed` (default 2.7 m/s) over `acceleration_time` |
| `remote` | Up/Down speed goal; Left/Right steer bias; `0` stop; Space reset |

When `|speed_goal| < 0.1` m/s in remote mode, speed/steer refs force **upright hold** (standstill balance is fragile).

Arrows only — WASD is left for MuJoCo Simulate UI toggles.

## Control chain

```text
IMU + joints → [delta, roll, roll_rate]
            → RollSteerPID → δ̇_ref
            → steer rate PI → cmd_steering_f
speed_ref   → rear P       → cmd_rearwheel_f
```

Physics timestep stays **0.0005** s (no interactive override). Control at **50 Hz**.
