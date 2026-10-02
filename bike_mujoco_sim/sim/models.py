"""Model path resolution and MuJoCo load checks."""

from __future__ import annotations

from pathlib import Path

import mujoco

HERE = Path(__file__).resolve().parent
PACKAGE_ROOT = HERE.parent
MODELS_DIR = PACKAGE_ROOT / "models"

MODEL_ALIASES = {
    "orange_bike": MODELS_DIR / "orange_bike" / "orange_bike_horizontal.xml",
    "orange": MODELS_DIR / "orange_bike" / "orange_bike_horizontal.xml",
    "new_bike": MODELS_DIR / "new_bike" / "new_bike.xml",
    "new-bike": MODELS_DIR / "new_bike" / "new_bike.xml",
    "new_bike_3kg": MODELS_DIR / "new_bike" / "new_bike_3kg.xml",
    "new-bike-3kg": MODELS_DIR / "new_bike" / "new_bike_3kg.xml",
}

REQUIRED_ACTUATORS = ("cmd_steering_f", "cmd_rearwheel_f")
REQUIRED_SENSORS = (
    "steering_joint_pos_sensor",
    "steering_joint_vel_sensor",
    "rearwheel_joint_vel_sensor",
    "ori_global",
    "gyro_local",
    "pos_global",
)


def resolve_model_path(spec: str | Path | None = None) -> Path:
    if spec is None or str(spec).strip() == "":
        return MODEL_ALIASES["orange_bike"]
    text = str(spec).strip()
    key = text.lower().removesuffix(".xml")
    if key in MODEL_ALIASES:
        path = MODEL_ALIASES[key]
    else:
        path = Path(text).expanduser()
        if not path.is_absolute():
            for cand in (MODELS_DIR / path, PACKAGE_ROOT / path, Path(path).resolve()):
                if cand.is_file():
                    path = cand
                    break
            else:
                path = path.resolve()
    if not path.is_file():
        known = ", ".join(sorted({"orange_bike", "new_bike", "new_bike_3kg"}))
        raise FileNotFoundError(f"Unknown model {spec!r}. Use {known} or a path to an .xml file.")
    return path


def load_model(spec: str | Path | None = None) -> tuple[mujoco.MjModel, mujoco.MjData, Path]:
    path = resolve_model_path(spec)
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    _assert_interfaces(model)
    mujoco.mj_forward(model, data)
    return model, data, path


def _assert_interfaces(model: mujoco.MjModel) -> None:
    missing: list[str] = []
    for name in REQUIRED_ACTUATORS:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) < 0:
            missing.append(f"actuator:{name}")
    for name in REQUIRED_SENSORS:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name) < 0:
            missing.append(f"sensor:{name}")
    if missing:
        raise ValueError("Model missing required names: " + ", ".join(missing))


def steering_is_unlimited(model: mujoco.MjModel) -> bool:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "steering_joint")
    if jid < 0:
        raise ValueError("Model has no steering_joint")
    return int(model.jnt_limited[jid]) == 0
