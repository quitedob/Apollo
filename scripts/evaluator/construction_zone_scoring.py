#!/usr/bin/env python3
"""Construction-zone evaluator for competition runs.

The evaluator supports two zone modes:
1) forbidden: any entry yields score=0 immediately.
2) slowdown: passing is allowed, but overspeed incurs per-frame penalties.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from typing import Any, Dict, List, Optional, Tuple


Frame = Dict[str, Any]
Zone = Dict[str, Any]


def _point_in_box(x: float, y: float, zone: Zone) -> bool:
    xmin = zone.get("xmin", zone.get("x_min", float("-inf")))
    xmax = zone.get("xmax", zone.get("x_max", float("inf")))
    ymin = zone.get("ymin", zone.get("y_min", float("-inf")))
    ymax = zone.get("ymax", zone.get("y_max", float("inf")))
    return xmin <= x <= xmax and ymin <= y <= ymax


def _point_in_circle(x: float, y: float, zone: Zone) -> bool:
    center_x = zone.get("center_x", 0.0)
    center_y = zone.get("center_y", 0.0)
    radius = zone.get("radius", 0.0)
    return (x - center_x) ** 2 + (y - center_y) ** 2 <= radius * radius


def _point_in_polygon(x: float, y: float, zone: Zone) -> bool:
    points = zone.get("points", [])
    if len(points) < 3:
        return False

    inside = False
    p1x, p1y = points[0]
    for i in range(1, len(points) + 1):
        p2x, p2y = points[i % len(points)]
        if y > min(p1y, p2y) and y <= max(p1y, p2y) and x <= max(p1x, p2x):
            if p1y != p2y:
                xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
            else:
                xinters = p1x
            if p1x == p2x or x <= xinters:
                inside = not inside
        p1x, p1y = p2x, p2y
    return inside


def point_in_zone(x: float, y: float, zone: Zone) -> bool:
    zone_type = zone.get("type", "box")
    if zone_type == "box":
        return _point_in_box(x, y, zone)
    if zone_type == "circle":
        return _point_in_circle(x, y, zone)
    if zone_type == "polygon":
        return _point_in_polygon(x, y, zone)
    return _point_in_box(x, y, zone)


def load_json(path: str) -> Any:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def load_frames(path: str) -> List[Frame]:
    data = load_json(path)
    if isinstance(data, list):
        return data
    if isinstance(data, dict) and "trajectory" in data and isinstance(data["trajectory"], list):
        return data["trajectory"]
    raise ValueError(f"Invalid trajectory json format: {path}")


def load_zones(path: str) -> List[Zone]:
    data = load_json(path)
    if isinstance(data, list):
        return data
    if isinstance(data, dict) and "construction_zones" in data and isinstance(
        data["construction_zones"], list
    ):
        return data["construction_zones"]
    raise ValueError(f"Invalid construction-zones json format: {path}")


def _zone_mode(zone: Zone, default_mode: str) -> str:
    mode = str(zone.get("mode", default_mode)).lower()
    if mode not in {"forbidden", "slowdown"}:
        return default_mode
    return mode


def evaluate_construction_zone(
    frames: List[Frame],
    zones: List[Zone],
    default_speed_limit: float,
    penalty_per_frame_per_mps: float,
    default_mode: str,
) -> Dict[str, Any]:
    violations: List[Dict[str, Any]] = []
    total_penalty = 0.0
    entered_forbidden = False

    for frame_idx, frame in enumerate(frames):
        x = float(frame.get("x", 0.0))
        y = float(frame.get("y", 0.0))
        v = float(frame.get("v", 0.0))
        timestamp = float(frame.get("timestamp", frame_idx * 0.1))

        current_slowdown_limits: List[float] = []
        for zone in zones:
            if not point_in_zone(x, y, zone):
                continue

            mode = _zone_mode(zone, default_mode)
            zone_name = zone.get("name", zone.get("id", "unnamed_zone"))
            if mode == "forbidden":
                entered_forbidden = True
                violations.append(
                    {
                        "type": "CONSTRUCTION_ZONE_ENTRY",
                        "frame": frame_idx,
                        "timestamp": timestamp,
                        "zone": zone_name,
                        "x": x,
                        "y": y,
                        "message": "Vehicle entered forbidden construction zone",
                    }
                )
                return {
                    "scenario": "construction_zone",
                    "total_frames": len(frames),
                    "entered_forbidden_zone": True,
                    "total_penalty": 100.0,
                    "final_score": 0.0,
                    "violations": violations,
                    "violation_count": len(violations),
                }

            zone_limit = float(zone.get("speed_limit", default_speed_limit))
            current_slowdown_limits.append(zone_limit)

        if current_slowdown_limits:
            effective_limit = min(current_slowdown_limits)
            excess = max(0.0, v - effective_limit)
            if excess > 0.0:
                frame_penalty = math.floor(excess) * penalty_per_frame_per_mps
                total_penalty += frame_penalty
                violations.append(
                    {
                        "type": "CONSTRUCTION_SPEED_VIOLATION",
                        "frame": frame_idx,
                        "timestamp": timestamp,
                        "speed": v,
                        "speed_limit": effective_limit,
                        "excess_speed": excess,
                        "frame_penalty": frame_penalty,
                        "message": (
                            f"Speed {v:.3f} m/s exceeds limit {effective_limit:.3f} m/s "
                            f"by {excess:.3f} m/s"
                        ),
                    }
                )

    final_score = max(0.0, 100.0 - total_penalty)
    return {
        "scenario": "construction_zone",
        "total_frames": len(frames),
        "entered_forbidden_zone": entered_forbidden,
        "total_penalty": total_penalty,
        "final_score": final_score,
        "violations": violations,
        "violation_count": len(violations),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Construction-zone scoring evaluator")
    parser.add_argument("--trajectory", required=True, help="Trajectory json path")
    parser.add_argument(
        "--construction-zones", required=True, help="Construction-zone json path"
    )
    parser.add_argument(
        "--speed-limit",
        type=float,
        default=8.333333,
        help="Default construction-zone speed limit in m/s",
    )
    parser.add_argument(
        "--penalty-rate",
        type=float,
        default=2.0,
        help="Penalty points per frame per overspeed 1 m/s",
    )
    parser.add_argument(
        "--default-zone-mode",
        choices=["forbidden", "slowdown"],
        default="forbidden",
        help="Default zone mode when zone.mode is omitted",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="",
        help="Optional output json file path",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    frames = load_frames(args.trajectory)
    zones = load_zones(args.construction_zones)
    result = evaluate_construction_zone(
        frames=frames,
        zones=zones,
        default_speed_limit=args.speed_limit,
        penalty_per_frame_per_mps=args.penalty_rate,
        default_mode=args.default_zone_mode,
    )

    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)

    return 0 if result["final_score"] > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
