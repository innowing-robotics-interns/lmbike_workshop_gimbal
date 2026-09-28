"""Mostly-flat ground with small hills and bumps for the planar bicycle."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
MODELS_DIR = HERE / "models"
MODEL_PATH = MODELS_DIR / "bike.xml"
NEW_BIKE_MODEL_PATH = MODELS_DIR / "new_bike.xml"
# XML ships at 0.0005 s; interactive demo needs ~realtime, so we raise this at load.
NEW_BIKE_INTERACTIVE_TIMESTEP = 0.002
MODEL_ALIASES = {
    "bike": MODEL_PATH,
    "planar": MODEL_PATH,
    "new_bike": NEW_BIKE_MODEL_PATH,
    "new-bike": NEW_BIKE_MODEL_PATH,
}
NOMINAL_FRAME_MASS = 16.0
FLAT_PAD = 1.4  # metres of level ground at spawn before the first bump
MAX_HILL_MARKERS = 6
HILL_RGBA = np.array([0.62, 0.40, 0.18, 1.0])
PIN_RGBA = np.array([1.0, 0.84, 0.12, 1.0])
HIDDEN_RGBA = np.array([0.0, 0.0, 0.0, 0.0])
IDENTITY_QUAT = np.array([1.0, 0.0, 0.0, 0.0])
MARKER_Y = 1.15  # far side of the lane so markers sit behind the bike


@dataclass(frozen=True)
class Bump:
    """One gaussian bump on an otherwise flat road, extruded across the lane."""

    x: float
    height: float
    width: float


@dataclass(frozen=True)
class WorldSample:
    """One draw of real-world-ish conditions."""

    seed: int
    bumps: tuple[Bump, ...]
    roughness: float
    friction: float
    rider_mass: float
    lean_deg: float
    cruise_vel: float
    wind: float
    flat: bool

    @property
    def peak_height(self) -> float:
        if not self.bumps:
            return 0.0
        return max(b.height for b in self.bumps)

    def summary(self) -> str:
        if self.flat:
            return f"world: flat  lean={self.lean_deg:.1f} deg"
        return (
            f"world: {len(self.bumps)} bumps  "
            f"peak={self.peak_height * 100:.0f} cm  "
            f"rough={self.roughness:.3f}  "
            f"mu={self.friction:.2f}  "
            f"rider={self.rider_mass:.1f} kg  "
            f"lean={self.lean_deg:.1f} deg  "
            f"cruise={self.cruise_vel:.2f} m/s  "
            f"wind={self.wind:.1f} N"
        )


def _gaussian(x: np.ndarray | float, bump: Bump) -> np.ndarray | float:
    return bump.height * np.exp(-0.5 * ((x - bump.x) / max(bump.width, 1e-3)) ** 2)


def ground_height(x: float, sample: WorldSample) -> float:
    if sample.flat or not sample.bumps:
        return 0.0
    return float(sum(_gaussian(x, b) for b in sample.bumps))


def local_slope_deg(x: float, sample: WorldSample) -> float:
    """Rise/run under the bike, in degrees. Positive means climbing toward +x."""
    if sample.flat or not sample.bumps:
        return 0.0
    dhdx = 0.0
    for b in sample.bumps:
        w2 = max(b.width, 1e-3) ** 2
        dhdx += float(_gaussian(x, b) * (-(x - b.x) / w2))
    return float(np.degrees(np.arctan(dhdx)))


def terrain_zone(x: float, sample: WorldSample) -> str:
    if sample.flat:
        return "flat ground"
    height = ground_height(x, sample)
    slope = local_slope_deg(x, sample)
    if height < 0.02 and abs(slope) < 2.0:
        return "flat (bump ahead or behind)"
    if slope > 2.5:
        return "climbing a bump"
    if slope < -2.5:
        return "rolling down a bump"
    return "on top of a bump"


def nearest_hill(x: float, sample: WorldSample) -> tuple[int, Bump, float] | None:
    """Closest hill to the bike. Distance is signed: positive = still ahead."""
    if sample.flat or not sample.bumps:
        return None
    idx, bump = min(enumerate(sample.bumps), key=lambda item: abs(item[1].x - x))
    return idx, bump, bump.x - x


def _set_geom(model: mujoco.MjModel, name: str, pos: np.ndarray, size: np.ndarray, rgba: np.ndarray) -> None:
    gid = model.geom(name).id
    model.geom_pos[gid] = pos
    model.geom_size[gid] = size
    model.geom_quat[gid] = IDENTITY_QUAT
    model.geom_rgba[gid] = rgba


def _hide_geom(model: mujoco.MjModel, name: str) -> None:
    _set_geom(model, name, np.array([0.0, 0.0, -3.0]), np.array([0.02, 0.02, 0.02]), HIDDEN_RGBA)


def place_hill_markers(model: mujoco.MjModel, sample: WorldSample) -> None:
    """Brown mounds + yellow pins on the far side of the lane, one per bump."""
    for i in range(MAX_HILL_MARKERS):
        if sample.flat or i >= len(sample.bumps):
            _hide_geom(model, f"hill_{i}")
            _hide_geom(model, f"pin_{i}")
            continue
        bump = sample.bumps[i]
        _set_geom(
            model,
            f"hill_{i}",
            np.array([bump.x, MARKER_Y, 0.0]),
            np.array([max(bump.width * 1.55, 0.45), 0.55, bump.height]),
            HILL_RGBA,
        )
        _set_geom(
            model,
            f"pin_{i}",
            np.array([bump.x, MARKER_Y, bump.height + 0.07]),
            np.array([0.07, 0.0, 0.0]),
            PIN_RGBA,
        )


def slope_assist_force(x: float, sample: WorldSample) -> float:
    """Small gravity compensation so the cart can crest a bump instead of stalling."""
    if sample.flat:
        return 0.0
    mass = sample.rider_mass + 5.6
    return float(mass * 9.81 * np.sin(np.deg2rad(local_slope_deg(x, sample))))


def _make_bumps(rng: np.random.Generator) -> tuple[Bump, ...]:
    """A handful of isolated hills. The road between them stays flat."""
    bumps: list[Bump] = [
        # First bump sits just ahead of spawn so you can see it from the side camera.
        Bump(
            x=float(rng.uniform(2.1, 3.3)),
            height=float(rng.uniform(0.12, 0.20)),
            width=float(rng.uniform(0.75, 1.25)),
        )
    ]
    x = bumps[0].x + bumps[0].width * 2.2 + float(rng.uniform(0.7, 1.6))
    n_extra = int(rng.integers(2, 4))
    for _ in range(n_extra):
        width = float(rng.uniform(0.55, 1.35))
        height = float(rng.uniform(0.08, 0.16))
        bumps.append(Bump(x=x, height=height, width=width))
        x += width * 2.4 + float(rng.uniform(0.8, 1.8))

    hill_x = float(rng.uniform(5.5, 9.0))
    bumps.append(
        Bump(
            x=hill_x,
            height=float(rng.uniform(0.20, 0.32)),
            width=float(rng.uniform(1.1, 1.9)),
        )
    )
    bumps.sort(key=lambda b: b.x)
    return tuple(b for b in bumps if 1.5 < b.x < 12.5)


def sample_world(
    rng: np.random.Generator,
    *,
    flat: bool = False,
    lean_deg: float | None = None,
    seed: int = 0,
) -> WorldSample:
    if flat:
        return WorldSample(
            seed=seed,
            bumps=(),
            roughness=0.0,
            friction=1.5,
            rider_mass=NOMINAL_FRAME_MASS,
            lean_deg=8.0 if lean_deg is None else lean_deg,
            cruise_vel=0.0,
            wind=0.0,
            flat=True,
        )
    return WorldSample(
        seed=seed,
        bumps=_make_bumps(rng),
        roughness=float(rng.uniform(0.002, 0.010)),
        friction=float(rng.uniform(0.85, 1.7)),
        rider_mass=float(rng.uniform(13.5, 19.5)),
        lean_deg=float(rng.uniform(3.0, 7.0) if lean_deg is None else lean_deg),
        cruise_vel=float(rng.uniform(0.45, 0.95)),
        wind=float(rng.uniform(-6.0, 6.0)),
        flat=False,
    )


def _height_profile(ncol: int, x_min: float, x_max: float, sample: WorldSample, rng: np.random.Generator) -> np.ndarray:
    x = np.linspace(x_min, x_max, ncol)
    h = np.zeros(ncol, dtype=np.float64)
    if sample.flat:
        return h

    for bump in sample.bumps:
        h += _gaussian(x, bump)

    if sample.roughness > 0:
        noise = rng.normal(0.0, sample.roughness, size=ncol)
        kernel = np.ones(9) / 9.0
        noise = np.convolve(noise, kernel, mode="same")
        # Keep the spawn pad clean so the bike starts on level ground.
        noise[x < FLAT_PAD] = 0.0
        h += noise

    return np.clip(h, 0.0, 0.44)


def resolve_model_path(spec: str | Path | None = None) -> Path:
    """Resolve ``bike`` / ``new_bike`` aliases or an XML path under models/."""
    if spec is None or spec == "":
        return MODEL_PATH
    text = str(spec).strip()
    key = text.lower().removesuffix(".xml")
    if key in MODEL_ALIASES:
        return MODEL_ALIASES[key]
    path = Path(text).expanduser()
    if not path.is_absolute():
        cand = (MODELS_DIR / path).resolve()
        if cand.is_file():
            return cand
        cand = (HERE / path).resolve()
        if cand.is_file():
            return cand
        path = path.resolve()
    if not path.is_file():
        known = ", ".join(sorted({"bike", "new_bike"}))
        raise FileNotFoundError(f"Unknown model {spec!r}. Use {known} or a path to an .xml file.")
    return path


def detect_flavor(model: mujoco.MjModel) -> str:
    """``planar`` (workshop cart-bike) or ``new_bike`` (3D freejoint mesh bike)."""
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "drive") >= 0:
        return "planar"
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "cmd_rearwheel_f") >= 0:
        return "new_bike"
    raise ValueError("Unrecognized bike model: need actuator 'drive' or 'cmd_rearwheel_f'.")


def apply_sample(model: mujoco.MjModel, data: mujoco.MjData, sample: WorldSample, rng: np.random.Generator) -> None:
    nrow = int(model.hfield_nrow[0])
    ncol = int(model.hfield_ncol[0])
    size = model.hfield_size[0]
    x_min, x_max = -float(size[0]), float(size[0])
    profile = _height_profile(ncol, x_min, x_max, sample, rng)
    max_z = max(float(size[2]), 1e-6)
    normalized = np.clip(profile / max_z, 0.0, 1.0)
    model.hfield_data[:] = np.tile(normalized, nrow)

    frame = model.body("frame").id
    model.body_mass[frame] = sample.rider_mass
    model.body_inertia[frame] = np.array([0.55, 0.95, 0.55], dtype=np.float64) * (
        sample.rider_mass / NOMINAL_FRAME_MASS
    )

    friction = np.array([sample.friction, 0.01, 0.001], dtype=np.float64)
    for name in ("terrain", "rear_tire", "front_tire"):
        model.geom_friction[model.geom(name).id] = friction

    place_hill_markers(model, sample)

    mujoco.mj_resetData(model, data)
    data.joint("pitch").qpos[0] = np.deg2rad(sample.lean_deg)
    mujoco.mj_forward(model, data)


def apply_new_bike_reset(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Spawn the freejoint mesh bike upright on the flat floor."""
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)


class BikeWorld:
    """Loaded MuJoCo model plus a fresh random (or flat) outdoor draw."""

    def __init__(
        self,
        *,
        flat: bool = False,
        seed: int | None = None,
        lean_deg: float | None = None,
        model_path: str | Path | None = None,
    ) -> None:
        self.model_path = resolve_model_path(model_path)
        self.flat = flat
        self.lean_override = lean_deg
        self.seed = int(seed if seed is not None else np.random.randint(0, 1_000_000_000))
        self.rng = np.random.default_rng(self.seed)
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.flavor = detect_flavor(self.model)
        self.timestep_overridden = False
        if self.flavor == "new_bike":
            # Mesh bike has no hfield / pitch joint; keep a trivial flat sample for HUD.
            self.flat = True
            xml_dt = float(self.model.opt.timestep)
            if xml_dt < NEW_BIKE_INTERACTIVE_TIMESTEP - 1e-12:
                self.model.opt.timestep = NEW_BIKE_INTERACTIVE_TIMESTEP
                self.timestep_overridden = True
                print(
                    f"physics: new_bike timestep {xml_dt} -> {NEW_BIKE_INTERACTIVE_TIMESTEP} "
                    f"(interactive realtime; XML left unchanged)",
                    flush=True,
                )
            self.sample = sample_world(self.rng, flat=True, lean_deg=0.0 if lean_deg is None else lean_deg, seed=self.seed)
            apply_new_bike_reset(self.model, self.data)
        else:
            self.sample = sample_world(self.rng, flat=flat, lean_deg=lean_deg, seed=self.seed)
            apply_sample(self.model, self.data, self.sample, self.rng)

    def reset(self, *, reroll: bool = True) -> WorldSample:
        if self.flavor == "new_bike":
            self.sample = sample_world(
                np.random.default_rng(self.seed),
                flat=True,
                lean_deg=self.lean_override if self.lean_override is not None else 0.0,
                seed=self.seed,
            )
            apply_new_bike_reset(self.model, self.data)
            return self.sample
        if reroll and not self.flat:
            self.seed = int(self.rng.integers(0, 1_000_000_000))
            self.rng = np.random.default_rng(self.seed)
            self.sample = sample_world(
                self.rng,
                flat=False,
                lean_deg=self.lean_override,
                seed=self.seed,
            )
        else:
            self.sample = sample_world(
                np.random.default_rng(self.seed),
                flat=self.flat,
                lean_deg=self.lean_override if self.lean_override is not None else self.sample.lean_deg,
                seed=self.seed,
            )
        apply_sample(self.model, self.data, self.sample, np.random.default_rng(self.seed + 17))
        return self.sample
