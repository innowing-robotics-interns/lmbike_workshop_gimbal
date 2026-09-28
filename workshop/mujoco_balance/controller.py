"""Controllers for the planar workshop bicycle.

The bike is an inverted pendulum on a cart. Positive pitch (right-hand about
+y) tips the rider toward +x. A positive drive force accelerates the cart
in +x, which is how the inner loop catches a forward fall.

Loops, inner to outer:
  1. Pitch PD — drive force to hold a lean command.
  2. Speed / position — that lean command, clamped.
  3. Slope feed-forward — mg sin(theta) so a bump does not stall the cart.

Edit the numbers in PDGains. --controller none disables this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PDGains:
    """Tune these. Units: pitch in rad, force in N, speeds in m/s."""

    kp: float = 620.0
    kd: float = 200.0
    kx: float = 0.04
    kv: float = 0.14
    ki: float = 0.05
    k_slope: float = 1.0
    lean_limit_deg: float = 10.0
    torque_limit: float = 250.0
    i_limit: float = 0.12


@dataclass(frozen=True)
class ControllerDebug:
    force: float
    pitch_deg: float
    pitch_ref_deg: float
    speed: float
    speed_ref: float
    slope_ff: float


def gains_for_world(*, flat: bool) -> PDGains:
    """Position-hold on a level floor; speed-track when the road has hills."""
    if flat:
        return PDGains(kx=0.045, kv=0.08, ki=0.0, k_slope=0.0, lean_limit_deg=8.0)
    return PDGains(kx=0.0, kv=0.14, ki=0.06, k_slope=1.0, lean_limit_deg=10.0)


class BalanceController:
    """Cascade balance + speed controller with optional slope compensation."""

    def __init__(self, gains: PDGains | None = None) -> None:
        self.gains = gains or PDGains()
        self._speed_i = 0.0
        self.last = ControllerDebug(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def reset(self) -> None:
        self._speed_i = 0.0
        self.last = ControllerDebug(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def torque(
        self,
        pitch: float,
        pitch_vel: float,
        x: float,
        x_vel: float,
        *,
        x_ref: float = 0.0,
        x_vel_ref: float = 0.0,
        slope_deg: float = 0.0,
        mass: float = 21.6,
        dt: float = 0.002,
    ) -> float:
        g = self.gains
        lean_limit = math.radians(g.lean_limit_deg)

        speed_err = x_vel - x_vel_ref
        self._speed_i = max(-g.i_limit, min(g.i_limit, self._speed_i + speed_err * dt))
        # Too fast → negative pitch_ref (lean back). Too slow → lean forward.
        pitch_ref = -(g.kx * (x - x_ref) + g.kv * speed_err + g.ki * self._speed_i)
        pitch_ref = max(-lean_limit, min(lean_limit, pitch_ref))

        u_balance = g.kp * (pitch - pitch_ref) + g.kd * pitch_vel
        u_slope = g.k_slope * mass * 9.81 * math.sin(math.radians(slope_deg))
        u = max(-g.torque_limit, min(g.torque_limit, u_balance + u_slope))

        self.last = ControllerDebug(
            force=u,
            pitch_deg=math.degrees(pitch),
            pitch_ref_deg=math.degrees(pitch_ref),
            speed=x_vel,
            speed_ref=x_vel_ref,
            slope_ff=u_slope,
        )
        return u
