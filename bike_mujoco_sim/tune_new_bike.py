#!/usr/bin/env python3
"""Search the new_bike lean PID for the quietest steer oscillation that still balances.

GIM4310-10 GDZ34 stall torque ±7.98 N·m, steer rate capped at 23.9 rad/s. Ki is on roll lean, not on the motor.
Among runs that stay upright, the objective is the 8–25 Hz steer-torque amplitude.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.bayes_opt import BayesOptimizer
from sim.config import CONFIGS_DIR, load_config
from sim.loop import make_sim, run_headless

# name, low, high — lean-torque PID. Motor stall ±7.98 N·m, |steer rate| ≤ 23.9 rad/s.
SPACE = (
    ("pid_kp_roll", 0.05, 2.5),
    ("pid_ki_roll", 0.0, 1.2),
    ("pid_kd_roll", 0.0, 3.0),
    ("pid_kp_steer", 0.0, 1.0),
    ("steer_rate_damping", 0.0, 0.2),
    ("steer_joint_damping", 0.005, 0.12),
)
DUR = 8.0
N_INIT = 20
N_BO = 36
N_LOCAL = 24
# Upright gate. Quieter runs that lean past this are discarded.
MAX_ROLL_DEG = 15.0
MAX_DRIFT_DEG = 4.0


def decode(unit: np.ndarray) -> dict[str, float]:
    out = {}
    for value, (name, low, high) in zip(unit, SPACE):
        out[name] = float(low + value * (high - low))
    return out


def torque_ripple(t: np.ndarray, tau: np.ndarray, mask: np.ndarray) -> float:
    """Peak-to-peak amplitude of the strongest 8–25 Hz component, in N·m."""
    y = tau[mask] - float(np.mean(tau[mask]))
    if len(y) < 16:
        return float(np.std(y) * 2.0)
    dt = float(np.median(np.diff(t)))
    if dt <= 0.0:
        return float(np.std(y) * 2.0)
    window = np.hanning(len(y))
    spec = np.abs(np.fft.rfft(y * window))
    freq = np.fft.rfftfreq(len(y), dt)
    band = (freq >= 8.0) & (freq <= 25.0)
    if not np.any(band):
        return 0.0
    k = int(np.argmax(spec[band]))
    # Hanning coherent gain is 0.5, so scale the one-sided peak back to amplitude.
    return float(2.0 * spec[band][k] / max(window.sum(), 1.0))


def evaluate(params: dict[str, float]) -> tuple[float, dict[str, float]]:
    base = load_config(CONFIGS_DIR / "new_bike.yaml")
    cfg = replace(
        base,
        duration=DUR,
        balance_mode="lean_torque",
        steer_torque_limit=7.98,
        steer_velocity_limit=23.9,
        pid_integral_limit=0.4,
        **params,
    )
    sim = make_sim(model_spec="new_bike", mode="speed-schedule", cfg=cfg)
    hist = run_headless(sim, duration=DUR)
    t = np.asarray(hist["time"], dtype=float)
    if len(t) < 4:
        return 2000.0, {"tip": 0.0, "max_roll": 180.0, "tau_std": 99.0, "sat": 1.0, "rate_std": 99.0}
    roll = np.degrees(np.asarray(hist["roll"], dtype=float))
    tau = np.asarray(hist["steer_torque"], dtype=float)
    rate = np.asarray(hist["steer_rate"], dtype=float)
    abs_roll = np.abs(roll)
    tip_i = np.where(abs_roll > 30.0)[0]
    tip = float(t[tip_i[0]]) if len(tip_i) else float(t[-1])
    mask = t >= 1.5
    if int(mask.sum()) < 6:
        mask = np.ones(len(t), dtype=bool)
    tau_std = float(np.std(tau[mask]))
    rate_std = float(np.std(rate[mask]))
    sat = float(np.mean(np.abs(tau[mask]) > 7.2))
    ripple = torque_ripple(t, tau, mask)
    mx = float(abs_roll.max())
    end = float(abs_roll[-1])
    i3 = int(np.argmin(np.abs(t - 3.0)))
    drift = float(abs_roll[-1] - abs_roll[i3]) if t[-1] > 3.0 else mx
    info = {
        "tip": tip,
        "max_roll": mx,
        "end_roll": end,
        "drift": drift,
        "tau_std": tau_std,
        "sat": sat,
        "rate_std": rate_std,
        "ripple": ripple,
    }
    upright = tip >= DUR * 0.95 and mx <= MAX_ROLL_DEG and drift <= MAX_DRIFT_DEG
    if not upright:
        # Rank falls worse than any upright chatter so the search stays in the balancing set.
        return 800.0 + (DUR - tip) * 40.0 + mx + 10.0 * max(drift, 0.0), info
    # Quietest oscillation among bikes that stay up. Roll is only a tie-break.
    cost = ripple + 0.15 * tau_std + 0.02 * mx + 0.05 * max(drift, 0.0) + 2.0 * sat
    return float(cost), info


def encode(params: dict[str, float]) -> np.ndarray:
    unit = []
    for name, low, high in SPACE:
        unit.append((params[name] - low) / (high - low))
    return np.clip(np.asarray(unit, dtype=float), 0.0, 1.0)


def local_unit(rng: np.random.Generator, center: np.ndarray, scale: float = 0.08) -> np.ndarray:
    return np.clip(center + rng.normal(0.0, scale, size=center.shape), 0.0, 1.0)


def main() -> int:
    opt = BayesOptimizer(dim=len(SPACE), seed=11)
    best_cost = 1e9
    best_params: dict[str, float] | None = None
    best_info: dict[str, float] | None = None
    current = load_config(CONFIGS_DIR / "new_bike.yaml")
    seeded = encode(
        {
            "pid_kp_roll": current.pid_kp_roll,
            "pid_ki_roll": current.pid_ki_roll,
            "pid_kd_roll": current.pid_kd_roll,
            "pid_kp_steer": current.pid_kp_steer,
            "steer_rate_damping": current.steer_rate_damping,
            "steer_joint_damping": float(current.steer_joint_damping or 0.02),
        }
    )
    total = 1 + N_INIT + N_BO + N_LOCAL
    center = seeded
    for i in range(total):
        if i == 0:
            unit = seeded
        elif i <= N_INIT:
            unit = opt.rng.random(len(SPACE))
        elif i <= N_INIT + N_BO:
            unit = opt.ask()
        else:
            unit = local_unit(opt.rng, center)
        params = decode(unit)
        cost, info = evaluate(params)
        opt.tell(unit, cost)
        mark = ""
        if cost < best_cost and cost < 800.0:
            best_cost = cost
            best_params = params
            best_info = info
            center = unit
            mark = "  *"
        print(
            f"[{i+1:02d}/{total}] cost={cost:7.2f} tip={info['tip']:.2f} "
            f"maxr={info['max_roll']:5.1f} ripple={info['ripple']:.2f} tau={info['tau_std']:.2f} "
            f"sat={info['sat']:.2f} rate={info['rate_std']:.1f}{mark}",
            flush=True,
        )
    assert best_params is not None and best_info is not None
    print("\nQUIETEST UPRIGHT", best_cost)
    print(best_params)
    print(best_info)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
