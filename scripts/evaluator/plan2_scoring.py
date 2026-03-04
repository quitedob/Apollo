#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plan 2 scoring checks (offline evaluator).

Rules implemented:
1) Longitudinal acceleration <= 3.0 m/s^2
2) Longitudinal deceleration >= -6.0 m/s^2
3) Lateral acceleration absolute <= 2.0 m/s^2
4) Speed <= 16.67 m/s
5) Red-light stop distance in [1.5, 2.0] m:
   - outside interval: -20 points
   - crossed stop line / no stop: fail (0 points)
6) Side-pass constraints:
   - min lateral distance >= 1.0 m
   - max side-pass speed <= 5.0 m/s
7) Optional right-turn-on-red check:
   - if right_turn_required=true but did_right_turn_on_red=false -> fail
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from typing import Any, Dict, List, Optional


@dataclass
class Plan2Thresholds:
    max_accel: float = 3.0
    min_accel: float = -6.0
    max_lateral_accel_abs: float = 2.0
    max_speed: float = 16.67
    min_red_stop_distance: float = 1.5
    max_red_stop_distance: float = 2.0
    red_stop_distance_penalty: int = 20
    min_side_pass_lateral_distance: float = 1.0
    max_side_pass_speed: float = 5.0


class Plan2Scorer:
    def __init__(self, thresholds: Optional[Plan2Thresholds] = None) -> None:
        self.th = thresholds or Plan2Thresholds()

    def score(self, data: Dict[str, Any]) -> Dict[str, Any]:
        trajectory = data.get("trajectory", [])
        if not trajectory:
            return self._fail("MISSING_TRAJECTORY", "trajectory is empty")

        max_ax = max(float(frame.get("ax", 0.0)) for frame in trajectory)
        min_ax = min(float(frame.get("ax", 0.0)) for frame in trajectory)
        max_abs_ay = max(abs(float(frame.get("ay", 0.0))) for frame in trajectory)
        max_speed = max(float(frame.get("v", 0.0)) for frame in trajectory)

        if max_ax > self.th.max_accel:
            return self._fail("ACCELERATION_VIOLATION", f"max_ax={max_ax:.3f} > {self.th.max_accel:.3f}")
        if min_ax < self.th.min_accel:
            return self._fail("DECELERATION_VIOLATION", f"min_ax={min_ax:.3f} < {self.th.min_accel:.3f}")
        if max_abs_ay > self.th.max_lateral_accel_abs:
            return self._fail(
                "LATERAL_ACCELERATION_VIOLATION",
                f"max_abs_ay={max_abs_ay:.3f} > {self.th.max_lateral_accel_abs:.3f}",
            )
        if max_speed > self.th.max_speed:
            return self._fail("SPEED_VIOLATION", f"max_speed={max_speed:.3f} > {self.th.max_speed:.3f}")

        score = 100
        violations: List[Dict[str, Any]] = []

        red_light = data.get("red_light", {})
        if red_light.get("required", True):
            stopped = bool(red_light.get("stopped", False))
            right_turn_required = bool(red_light.get("right_turn_required", False))
            did_right_turn = bool(red_light.get("did_right_turn_on_red", False))

            if right_turn_required and not did_right_turn:
                return self._fail(
                    "RIGHT_TURN_ON_RED_MISSED",
                    "right_turn_required=true but did_right_turn_on_red=false",
                )

            if not right_turn_required:
                if not stopped:
                    return self._fail("RED_LIGHT_NOT_STOPPED", "vehicle did not stop for red light")
                stop_line_s = red_light.get("stop_line_s")
                stop_pose_s = red_light.get("stop_pose_s")
                if stop_line_s is None or stop_pose_s is None:
                    violations.append(
                        {
                            "type": "RED_STOP_DISTANCE_UNKNOWN",
                            "message": "stop_line_s or stop_pose_s missing; red stop distance not checked",
                            "penalty": 0,
                        }
                    )
                else:
                    stop_distance = float(stop_line_s) - float(stop_pose_s)
                    if stop_distance < 0.0:
                        return self._fail(
                            "RED_LIGHT_CROSSED_STOP_LINE",
                            f"stop_distance={stop_distance:.3f} < 0.0",
                        )
                    if (
                        stop_distance < self.th.min_red_stop_distance
                        or stop_distance > self.th.max_red_stop_distance
                    ):
                        score -= self.th.red_stop_distance_penalty
                        violations.append(
                            {
                                "type": "RED_STOP_DISTANCE_PENALTY",
                                "message": (
                                    f"stop_distance={stop_distance:.3f} outside "
                                    f"[{self.th.min_red_stop_distance:.3f}, {self.th.max_red_stop_distance:.3f}]"
                                ),
                                "penalty": self.th.red_stop_distance_penalty,
                            }
                        )

        side_pass_segments = data.get("side_pass_segments", [])
        for idx, seg in enumerate(side_pass_segments):
            min_lat = float(seg.get("min_lateral_distance", 0.0))
            seg_max_speed = float(seg.get("max_speed", 0.0))
            if min_lat < self.th.min_side_pass_lateral_distance:
                return self._fail(
                    "SIDE_PASS_LATERAL_VIOLATION",
                    (
                        f"segment[{idx}] min_lateral_distance={min_lat:.3f} < "
                        f"{self.th.min_side_pass_lateral_distance:.3f}"
                    ),
                )
            if seg_max_speed > self.th.max_side_pass_speed:
                return self._fail(
                    "SIDE_PASS_SPEED_VIOLATION",
                    f"segment[{idx}] max_speed={seg_max_speed:.3f} > {self.th.max_side_pass_speed:.3f}",
                )

        status = "PASS" if score == 100 else "PENALTY"
        return {
            "scenario": "plan2_traffic_light_side_pass",
            "status": status,
            "pass": score > 0,
            "final_score": score,
            "penalty_total": 100 - score,
            "violations": violations,
            "violation_count": len(violations),
            "metrics": {
                "max_ax": max_ax,
                "min_ax": min_ax,
                "max_abs_ay": max_abs_ay,
                "max_speed": max_speed,
                "side_pass_segments_checked": len(side_pass_segments),
            },
        }

    def _fail(self, violation_type: str, message: str) -> Dict[str, Any]:
        return {
            "scenario": "plan2_traffic_light_side_pass",
            "status": "FAIL",
            "pass": False,
            "final_score": 0,
            "penalty_total": 100,
            "violations": [{"type": violation_type, "message": message, "penalty": 100}],
            "violation_count": 1,
            "metrics": {},
        }


def load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plan 2 scoring evaluator")
    parser.add_argument("--input", required=True, help="input JSON file")
    parser.add_argument("--output", default="", help="output JSON file")
    args = parser.parse_args()

    scorer = Plan2Scorer()
    result = scorer.score(load_json(args.input))

    print(f"scenario={result['scenario']} status={result['status']} score={result['final_score']}")
    if result["violations"]:
        for idx, v in enumerate(result["violations"], 1):
            print(f"{idx}. {v['type']}: {v['message']}")

    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()

