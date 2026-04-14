from dataclasses import dataclass
from typing import List, Tuple


Point2D = Tuple[float, float]  # (x, y) in a local Cartesian frame, or (lat, lon) in WGS84 depending on context.

INVALID_DISTANCE_M = -1.0
EPS = 1e-9
EARTH_RADIUS_M = 6378137.0

@dataclass
class LocalFrameDefinition:
    origin_lat_deg: float
    origin_lon_deg: float
    reference_frame: str = "local_cartesian"


@dataclass
class GeofenceDefinition:
    zone_name: str
    reference_frame: str
    points: List[Point2D]


@dataclass
class BoundaryContext:
    x: float
    y: float
    inside: bool
    distance_m: float
    closest_point: Point2D
    segment_index: int
    tangent_unit: Point2D
    inward_normal_unit: Point2D


@dataclass
class BoundaryHit:
    closest_point: Point2D
    segment_index: int
    tangent_unit: Point2D
    inward_normal_unit: Point2D
    distance_m: float


@dataclass
class BounceTargetResult:
    success: bool
    target_point: Point2D
    boundary_point: Point2D
    far_boundary_point: Point2D
    travel_direction_unit: Point2D
    segment_index: int
    used_recovery_mode: bool
    reason: str

