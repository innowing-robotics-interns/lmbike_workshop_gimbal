#!/usr/bin/env python3
"""PID dual-loop MuJoCo bike sim (notebook 07 config, PID outer loop).

Examples:
  python run_sim.py --mode speed-schedule
  python run_sim.py --mode speed-schedule --model new_bike
  python run_sim.py --mode remote --model orange_bike
  python run_sim.py --config configs/default.yaml --kp-roll 10
  python run_sim.py --headless --mode speed-schedule --duration 10
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Allow `python run_sim.py` without installing the package.
ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.config import DEFAULT_CONFIG_PATH, Config, load_config, resolve_config_path
from sim.loop import make_sim, run_headless, summarize
from sim.viewer import run_viewer


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PID MuJoCo bike balance / remote drive.")
    p.add_argument(
        "--config",
        type=Path,
        default=None,
        help="YAML with tunable parameters. Default: configs/new_bike.yaml for --model new_bike, else configs/default.yaml.",
    )
    p.add_argument(
        "--mode",
        choices=("speed-schedule", "remote"),
        default="speed-schedule",
        help="speed-schedule: notebook ramp. remote: arrow-key speed/steer goals.",
    )
    p.add_argument(
        "--model",
        default="orange_bike",
        help="orange_bike (default), new_bike, or path to an .xml with the same actuators/sensors.",
    )
    p.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Sim seconds for speed-schedule (default from YAML). Headless exits; GUI resets and continues.",
    )
    p.add_argument("--target-speed", type=float, default=None, help="Cruise target for speed-schedule (m/s).")
    p.add_argument(
        "--headless",
        action="store_true",
        help="No GLFW window. GUI is the default for every mode.",
    )
    p.add_argument("--debug", action="store_true", help="Verbose control / key logging.")
    p.add_argument("--kp-roll", type=float, default=None)
    p.add_argument("--ki-roll", type=float, default=None)
    p.add_argument("--kd-roll", type=float, default=None)
    p.add_argument("--kp-steer", type=float, default=None)
    return p.parse_args()


def build_config(args: argparse.Namespace) -> tuple[Config, Path]:
    config_path = resolve_config_path(args.model, args.config)
    cfg = load_config(config_path)
    if args.duration is not None:
        cfg.duration = float(args.duration)
    if args.target_speed is not None:
        cfg.target_speed = float(args.target_speed)
    if args.kp_roll is not None:
        cfg.pid_kp_roll = float(args.kp_roll)
    if args.ki_roll is not None:
        cfg.pid_ki_roll = float(args.ki_roll)
    if args.kd_roll is not None:
        cfg.pid_kd_roll = float(args.kd_roll)
    if args.kp_steer is not None:
        cfg.pid_kp_steer = float(args.kp_steer)
    return cfg, config_path


def main() -> int:
    args = parse_args()
    cfg, config_path = build_config(args)
    use_viewer = not args.headless

    sim = make_sim(model_spec=args.model, mode=args.mode, cfg=cfg, debug=args.debug)
    print(
        f"config={config_path.resolve()}  balance={cfg.balance_mode}  "
        f"mode={args.mode}  model={sim.model_path}",
        flush=True,
    )
    if args.mode == "speed-schedule":
        print(f"duration: {cfg.duration:.1f}s  target_speed={cfg.target_speed:.2f} m/s", flush=True)
    print(
        f"PID: kp_r={cfg.pid_kp_roll} ki_r={cfg.pid_ki_roll} "
        f"kd_r={cfg.pid_kd_roll} kp_s={cfg.pid_kp_steer}",
        flush=True,
    )

    if use_viewer:
        if args.mode == "speed-schedule":
            # Still use schedule; goals unused.
            sim.mode = "speed-schedule"
        run_viewer(sim)
        return 0

    history = run_headless(sim, duration=cfg.duration)
    summarize(history, model_path=sim.model_path, mode=sim.mode)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
