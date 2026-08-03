"""Registry loading and validation shared by the launcher and GUI."""

from pathlib import Path
from typing import Any

import yaml


def load_registry(path: Path, root_key: str) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    values = document.get(root_key)
    if not isinstance(values, dict) or not values:
        raise ValueError(f"{path} must contain a non-empty '{root_key}' mapping")
    return values


def resolve_asset(package_share: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else package_share / path


def validate_registries(package_share: Path) -> tuple[dict, dict]:
    vehicles = load_registry(package_share / "config/vehicles.yaml", "vehicles")
    worlds = load_registry(package_share / "config/worlds.yaml", "worlds")
    required_vehicle = {"urdf", "nav2_config", "ekf_config", "topics"}
    for name, profile in vehicles.items():
        missing = required_vehicle - set(profile)
        if missing:
            raise ValueError(f"vehicle '{name}' is missing {sorted(missing)}")
        for key in ("urdf", "nav2_config", "ekf_config"):
            path = resolve_asset(package_share, profile[key])
            if not path.is_file():
                raise FileNotFoundError(f"vehicle '{name}' {key}: {path}")
    for name, profile in worlds.items():
        for key in ("sdf", "gazebo_name", "spawn"):
            if key not in profile:
                raise ValueError(f"world '{name}' is missing '{key}'")
        if not resolve_asset(package_share, profile["sdf"]).is_file():
            raise FileNotFoundError(f"world '{name}' SDF is missing")
    return vehicles, worlds
