#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import signal
import sys
from pathlib import Path
from typing import List, Sequence, Tuple

import cv2
import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[1]))

from geofence_manager.polygon_loader import load_geofence_from_yaml
from geofence_manager.geometry_bounce import (
    compute_bounce_target,
    compute_nearest_boundary_hit,
)

#
# cd ~/robot_ws/src/ros2_geofence_manager/test
# ./test_bounce.py --yaml ../config/geofence_polygon.yaml --x 1.2 --y 3.2 --angle-deg 30
#

XY = Tuple[float, float]

_RUNNING = True


def _handle_sigint(signum, frame):
    global _RUNNING
    _RUNNING = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone geofence bounce visualizer."
    )
    parser.add_argument(
        "--yaml",
        required=True,
        help="Path to geofence polygon YAML file.",
    )
    parser.add_argument(
        "--x",
        type=float,
        required=True,
        help="Initial robot x in polygon frame.",
    )
    parser.add_argument(
        "--y",
        type=float,
        required=True,
        help="Initial robot y in polygon frame.",
    )
    parser.add_argument(
        "--angle-deg",
        type=float,
        default=0.0,
        help="Bounce angle in degrees. 0 = orthogonal inward.",
    )
    parser.add_argument(
        "--start-inset",
        type=float,
        default=0.25,
        help="Inset from nearest boundary before ray cast.",
    )
    parser.add_argument(
        "--goal-inset",
        type=float,
        default=0.50,
        help="Inset from far boundary for final target.",
    )
    parser.add_argument(
        "--step-size",
        type=float,
        default=0.08,
        help="Motion step size per frame in map units.",
    )
    parser.add_argument(
        "--arrival-tol",
        type=float,
        default=0.12,
        help="Distance threshold for considering target reached.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Visualization rate.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1000,
        help="Window width in pixels.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=1000,
        help="Window height in pixels.",
    )
    return parser.parse_args()


def compute_bounds(points: Sequence[XY], margin_ratio: float = 0.1) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)

    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)

    margin_x = span_x * margin_ratio
    margin_y = span_y * margin_ratio

    return (
        min_x - margin_x,
        max_x + margin_x,
        min_y - margin_y,
        max_y + margin_y,
    )


def world_to_image(
    p: XY,
    bounds: Tuple[float, float, float, float],
    width: int,
    height: int,
) -> Tuple[int, int]:
    min_x, max_x, min_y, max_y = bounds

    sx = (p[0] - min_x) / (max_x - min_x)
    sy = (p[1] - min_y) / (max_y - min_y)

    px = int(round(sx * (width - 1)))
    py = int(round((1.0 - sy) * (height - 1)))
    return px, py


def draw_polygon(
    image: np.ndarray,
    polygon: Sequence[XY],
    bounds: Tuple[float, float, float, float],
) -> None:
    pts = [world_to_image(p, bounds, image.shape[1], image.shape[0]) for p in polygon]
    pts_np = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(image, [pts_np], isClosed=True, color=(200, 200, 200), thickness=2)


def draw_point(
    image: np.ndarray,
    p: XY,
    bounds: Tuple[float, float, float, float],
    color: Tuple[int, int, int],
    radius: int = 5,
    filled: bool = True,
) -> None:
    center = world_to_image(p, bounds, image.shape[1], image.shape[0])
    thickness = -1 if filled else 1
    cv2.circle(image, center, radius, color, thickness)


def draw_line(
    image: np.ndarray,
    a: XY,
    b: XY,
    bounds: Tuple[float, float, float, float],
    color: Tuple[int, int, int],
    thickness: int = 1,
) -> None:
    pa = world_to_image(a, bounds, image.shape[1], image.shape[0])
    pb = world_to_image(b, bounds, image.shape[1], image.shape[0])
    cv2.line(image, pa, pb, color, thickness)


def draw_text(image: np.ndarray, text: str, x: int, y: int) -> None:
    cv2.putText(
        image,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (220, 220, 220),
        1,
        cv2.LINE_AA,
    )


def normalize(v: XY) -> XY:
    n = math.hypot(v[0], v[1])
    if n <= 1e-12:
        return (0.0, 0.0)
    return (v[0] / n, v[1] / n)


def dist(a: XY, b: XY) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def main() -> int:
    global _RUNNING

    args = parse_args()
    signal.signal(signal.SIGINT, _handle_sigint)

    yaml_path = Path(args.yaml)
    geofence = load_geofence_from_yaml(str(yaml_path))
    polygon: List[XY] = list(geofence.points)

    if len(polygon) < 3:
        print("Polygon must contain at least 3 points.", file=sys.stderr)
        return 1

    bounds = compute_bounds(polygon)

    robot_xy: XY = (args.x, args.y)
    trail: List[XY] = [robot_xy]

    bounce_angle_deg = args.angle_deg

    bounce = compute_bounce_target(
        robot_xy=robot_xy,
        polygon=polygon,
        bounce_angle_deg=bounce_angle_deg,
        start_inset_m=args.start_inset,
        goal_inset_m=args.goal_inset,
    )
    if not bounce.success:
        print(f"Initial bounce target failed: {bounce.reason}", file=sys.stderr)
        return 1

    target_xy: XY = bounce.target_point

    cv2.namedWindow("Geofence Bounce Test", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Geofence Bounce Test", args.width, args.height)

    wait_ms = max(1, int(round(1000.0 / args.fps)))

    while _RUNNING:
        d = dist(robot_xy, target_xy)
        if d <= args.arrival_tol:
            bounce = compute_bounce_target(
                robot_xy=robot_xy,
                polygon=polygon,
                bounce_angle_deg=bounce_angle_deg,
                start_inset_m=args.start_inset,
                goal_inset_m=args.goal_inset,
            )
            if not bounce.success:
                print(f"Bounce target failed: {bounce.reason}", file=sys.stderr)
                break
            target_xy = bounce.target_point
            # Invert bounce angle for next run to demonstrate both sides of the bounce logic.
            bounce_angle_deg = -bounce_angle_deg

        direction = normalize((target_xy[0] - robot_xy[0], target_xy[1] - robot_xy[1]))
        robot_xy = (
            robot_xy[0] + direction[0] * args.step_size,
            robot_xy[1] + direction[1] * args.step_size,
        )
        trail.append(robot_xy)
        if len(trail) > 3000:
            trail = trail[-3000:]

        hit = compute_nearest_boundary_hit(robot_xy, polygon)
        bounce_dbg = compute_bounce_target(
            robot_xy=robot_xy,
            polygon=polygon,
            bounce_angle_deg=bounce_angle_deg,
            start_inset_m=args.start_inset,
            goal_inset_m=args.goal_inset,
        )

        frame = np.zeros((args.height, args.width, 3), dtype=np.uint8)

        draw_polygon(frame, polygon, bounds)

        if len(trail) >= 2:
            for i in range(1, len(trail)):
                draw_line(frame, trail[i - 1], trail[i], bounds, (90, 90, 255), 1)

        draw_point(frame, robot_xy, bounds, (0, 255, 0), radius=6)
        draw_point(frame, target_xy, bounds, (0, 180, 255), radius=6)

        draw_point(frame, hit.closest_point, bounds, (255, 255, 0), radius=5)
        draw_line(frame, robot_xy, hit.closest_point, bounds, (255, 255, 0), 1)

        if bounce_dbg.success:
            draw_point(frame, bounce_dbg.far_boundary_point, bounds, (255, 0, 255), radius=5)
            draw_line(frame, bounce_dbg.boundary_point, bounce_dbg.far_boundary_point, bounds, (120, 120, 120), 1)
            draw_line(frame, robot_xy, bounce_dbg.target_point, bounds, (0, 180, 255), 1)

        draw_text(frame, f"zone: {geofence.name}", 15, 25)
        draw_text(frame, f"frame: {geofence.frame_id}", 15, 50)
        draw_text(frame, f"robot: ({robot_xy[0]:.2f}, {robot_xy[1]:.2f})", 15, 75)
        draw_text(frame, f"target: ({target_xy[0]:.2f}, {target_xy[1]:.2f})", 15, 100)
        draw_text(frame, f"nearest boundary dist: {hit.distance_m:.2f}", 15, 125)
        draw_text(frame, f"bounce angle: {bounce_angle_deg:.1f} deg", 15, 150)
        draw_text(frame, "q: quit", 15, 175)

        cv2.imshow("Geofence Bounce Test", frame)
        key = cv2.waitKey(wait_ms) & 0xFF
        if key == ord("q"):
            break

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
