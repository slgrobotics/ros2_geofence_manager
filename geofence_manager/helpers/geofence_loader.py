#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path

from geofence_manager.helpers.common_data import GeofenceDefinition

from geofence_manager.helpers.polygon_loader import load_geofence_from_yaml
from geofence_manager.helpers.qgc_plan_loader import load_geofence_from_qgc_plan
from geofence_manager.helpers.wgs84_to_local import (
    LocalFrameDefinition,
    convert_wgs84_polygon_to_local,
)


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


def load_geofence_as_local_cartesian(
    file_path: str,
    frame_id: str = "map",
) -> tuple[GeofenceDefinition, LocalFrameDefinition | None]:
    """
    Load a geofence and convert WGS84 input to a local Cartesian frame.

    Returns:
      - GeofenceDefinition
      - LocalFrameDefinition if conversion from WGS84 was performed, else None
    """
    geofence = load_geofence(file_path)

    if geofence.reference_frame.lower() == "wgs84":
        return convert_wgs84_polygon_to_local(geofence, frame_id=frame_id)

    return geofence, None