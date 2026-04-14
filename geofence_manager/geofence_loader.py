#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path

from geofence_manager.common_data import GeofenceDefinition

from geofence_manager.polygon_loader import load_geofence_from_yaml
from geofence_manager.qgc_plan_loader import load_geofence_from_qgc_plan


def load_geofence(file_path: str) -> GeofenceDefinition:

    # ----------------------------------------------------------
    # Load a geofence definition from a supported file type.
    #
    # Supported formats:
    # - .yaml / .yml  -> simple YAML polygon format
    # - .plan         -> QGroundControl plan file
    #
    # Returns:
    #     GeofenceDefinition
    #
    # Raises:
    #     FileNotFoundError:
    #         If the file does not exist.
    #     ValueError:
    #         If the file extension is unsupported or the file content is invalid.
    # ----------------------------------------------------------

    path = Path(file_path)
    if not path.is_file():
        raise FileNotFoundError(f"Geofence file not found: '{file_path}'")

    suffix = path.suffix.lower()

    if suffix in (".yaml", ".yml"):
        return load_geofence_from_yaml(str(path))

    if suffix == ".plan":
        return load_geofence_from_qgc_plan(str(path))

    raise ValueError(
        f"Unsupported geofence file extension '{suffix}' for file '{file_path}'. "
        f"Supported extensions: .yaml, .yml, .plan"
    )
