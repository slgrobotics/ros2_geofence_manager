#!/usr/bin/env python3

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from geofence_manager.helpers.common_data import Point2D, EPS
from geofence_manager.helpers.geometry_utils import point_in_polygon
from geofence_manager.helpers.geometry_bounce import compute_nearest_boundary_hit


@dataclass
class RandomTargetResult:
    success: bool
    target_point: Point2D
    reason: str
    attempts_used: int


def compute_random_safe_target(
    robot_xy: Point2D,
    inclusion_polygon: Sequence[Point2D],
    exclusion_polygons: Sequence[Sequence[Point2D]] | None = None,
    max_samples: int = 200,
    inclusion_boundary_margin_m: float = 0.25,
    exclusion_boundary_margin_m: float = 0.25,
) -> RandomTargetResult:
    """
    Choose a random target inside the inclusion polygon that:
    - lies inside the inclusion polygon
    - stays away from the inclusion boundary by inclusion_boundary_margin_m
    - does not lie inside any exclusion polygon
    - stays away from exclusion boundaries by exclusion_boundary_margin_m
    - has a direct robot->target segment that does not intersect any exclusion polygon

    Intended as a simple high-level target selector for wandering / patrolling.
    """
    if len(inclusion_polygon) < 3:
        return RandomTargetResult(
            success=False,
            target_point=robot_xy,
            reason="inclusion polygon must contain at least 3 points",
            attempts_used=0,
        )

    exclusion_polygons = exclusion_polygons or []

    bbox = polygon_bounding_box(inclusion_polygon)

    for attempt in range(1, max_samples + 1):
        candidate = sample_random_point_in_bbox(bbox)

        if not point_in_polygon(candidate[0], candidate[1], inclusion_polygon):
            continue

        # Keep some clearance from the outer boundary.
        try:
            outer_hit = compute_nearest_boundary_hit(candidate, inclusion_polygon)
        except ValueError:
            continue

        if outer_hit.distance_m < inclusion_boundary_margin_m:
            continue

        # Reject if the point lies inside, or too close to, any exclusion polygon.
        blocked = False
        for exclusion in exclusion_polygons:
            if len(exclusion) < 3:
                continue

            if point_in_polygon(candidate[0], candidate[1], exclusion):
                blocked = True
                break

            try:
                exclusion_hit = compute_nearest_boundary_hit(candidate, exclusion)
            except ValueError:
                continue

            if exclusion_hit.distance_m < exclusion_boundary_margin_m:
                blocked = True
                break

        if blocked:
            continue

        # Reject if direct path to target crosses any exclusion polygon.
        if path_intersects_any_polygon(robot_xy, candidate, exclusion_polygons):
            continue

        return RandomTargetResult(
            success=True,
            target_point=candidate,
            reason="ok",
            attempts_used=attempt,
        )

    return RandomTargetResult(
        success=False,
        target_point=robot_xy,
        reason="failed to find a valid random target",
        attempts_used=max_samples,
    )


def polygon_bounding_box(
    polygon: Sequence[Point2D],
) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    return (min(xs), max(xs), min(ys), max(ys))


def sample_random_point_in_bbox(
    bbox: Tuple[float, float, float, float],
) -> Point2D:
    min_x, max_x, min_y, max_y = bbox
    return (
        random.uniform(min_x, max_x),
        random.uniform(min_y, max_y),
    )


def path_intersects_any_polygon(
    a: Point2D,
    b: Point2D,
    polygons: Sequence[Sequence[Point2D]],
) -> bool:
    for polygon in polygons:
        if len(polygon) < 3:
            continue

        # If either endpoint is inside the exclusion polygon, treat as blocked.
        if point_in_polygon(a[0], a[1], polygon) or point_in_polygon(b[0], b[1], polygon):
            return True

        if segment_intersects_polygon(a, b, polygon):
            return True

    return False


def segment_intersects_polygon(
    a: Point2D,
    b: Point2D,
    polygon: Sequence[Point2D],
) -> bool:
    n = len(polygon)
    for i in range(n):
        c = polygon[i]
        d = polygon[(i + 1) % n]

        if segments_intersect(a, b, c, d):
            return True

    return False


def segments_intersect(
    a: Point2D,
    b: Point2D,
    c: Point2D,
    d: Point2D,
) -> bool:
    o1 = orientation(a, b, c)
    o2 = orientation(a, b, d)
    o3 = orientation(c, d, a)
    o4 = orientation(c, d, b)

    # General case
    if o1 * o2 < 0.0 and o3 * o4 < 0.0:
        return True

    # Collinear / touching cases
    if abs(o1) < EPS and on_segment(a, c, b):
        return True
    if abs(o2) < EPS and on_segment(a, d, b):
        return True
    if abs(o3) < EPS and on_segment(c, a, d):
        return True
    if abs(o4) < EPS and on_segment(c, b, d):
        return True

    return False


def orientation(
    a: Point2D,
    b: Point2D,
    c: Point2D,
) -> float:
    return cross2d((b[0] - a[0], b[1] - a[1]), (c[0] - a[0], c[1] - a[1]))


def on_segment(
    a: Point2D,
    p: Point2D,
    b: Point2D,
) -> bool:
    return (
        min(a[0], b[0]) - EPS <= p[0] <= max(a[0], b[0]) + EPS
        and min(a[1], b[1]) - EPS <= p[1] <= max(a[1], b[1]) + EPS
    )


def cross2d(
    u: Point2D,
    v: Point2D,
) -> float:
    return u[0] * v[1] - u[1] * v[0]
