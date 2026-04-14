from dataclasses import dataclass
from typing import List, Tuple


Point2D = Tuple[float, float]  # (x, y) in a local Cartesian frame, or (lat, lon) in WGS84 depending on context.

INVALID_DISTANCE_M = -1.0
EPS = 1e-9

@dataclass
class GeofenceDefinition:
    name: str
    frame_id: str
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

