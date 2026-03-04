#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plan 3 scoring checks (offline evaluator).

Rules implemented:
1) Scenario timeout > 90s -> fail (0 points)
2) Collision -> fail (0 points)
3) Yield failure / no-avoidance -> fail (0 points)
4) Optional dynamic bounds:
   - speed <= 16.67 m/s
   - longitudinal acceleration <= 3.0 m/s^2
   - longitudinal deceleration >= -6.0 m/s^2
   - lateral acceleration abs <= 2.0 m/s^2
5) Stop distance scoring:
   - in [2.0, 2.5] m -> no penalty
   - outside interval but still before stop line -> -20
   - crossed stop line / not stopped -> fail (0 points)
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from typing import Any, Dict, List, Optional


@dataclass
class Plan3Thresholds:
    timeout_sec: float = 90.0
    max_accel: float = 3.0
    min_accel: float = -6.0
    max_lateral_accel_abs: float = 2.0
    max_speed: float = 16.67
    min_stop_distance: float = 2.0
    max_stop_distance: float = 2.5
    stop_distance_penalty: int = 20


class Plan3Scorer:
    def __init__(self, thresholds: Optional[Plan3Thresholds] = None) -> None:
        self.th = thresholds or Plan3Thresholds()

    def score(self, data: Dict[str, Any]) -> Dict[str, Any]:
        scenario_duration = float(data.get("scenario_duration", 0.0))
        if scenario_duration > self.th.timeout_sec:
            return self._fail(
                "TIMEOUT",
                f"scenario_duration={scenario_duration:.3f} > {self.th.timeout_sec:.3f}",
            )

        if bool(data.get("collision", False)):
            return self._fail("COLLISION", "collision=true")
        if bool(data.get("yield_failure", False)):
            return self._fail("YIELD_FAILURE", "yield_failure=true")

        trajectory = data.get("trajectory", [])
        if trajectory:
            max_ax = max(float(frame.get("ax", 0.0)) for frame in trajectory)
            min_ax = min(float(frame.get("ax", 0.0)) for frame in trajectory)
            max_abs_ay = max(abs(float(frame.get("ay", 0.0))) for frame in trajectory)
            max_speed = max(float(frame.get("v", 0.0)) for frame in trajectory)
            if max_ax > self.th.max_accel:
                return self._fail(
                    "ACCELERATION_VIOLATION",
                    f"max_ax={max_ax:.3f} > {self.th.max_accel:.3f}",
                )
            if min_ax < self.th.min_accel:
                return self._fail(
                    "DECELERATION_VIOLATION",
                    f"min_ax={min_ax:.3f} < {self.th.min_accel:.3f}",
                )
            if max_abs_ay > self.th.max_lateral_accel_abs:
                return self._fail(
                    "LATERAL_ACCELERATION_VIOLATION",
                    f"max_abs_ay={max_abs_ay:.3f} > {self.th.max_lateral_accel_abs:.3f}",
                )
            if max_speed > self.th.max_speed:
                return self._fail("SPEED_VIOLATION", f"max_speed={max_speed:.3f} > {self.th.max_speed:.3f}")
        else:
            max_ax = min_ax = max_abs_ay = max_speed = 0.0

        score = 100
        violations: List[Dict[str, Any]] = []

        stop_rule = data.get("stop_rule", {})
        if bool(stop_rule.get("required", True)):
            stopped = bool(stop_rule.get("stopped", False))
            if not stopped:
                return self._fail("NOT_STOPPED", "stop_rule.required=true but stopped=false")

            stop_line_s = stop_rule.get("stop_line_s")
            stop_pose_s = stop_rule.get("stop_pose_s")
            if stop_line_s is None or stop_pose_s is None:
                violations.append(
                    {
                        "type": "STOP_DISTANCE_UNKNOWN",
                        "message": "stop_line_s or stop_pose_s missing; stop distance not checked",
                        "penalty": 0,
                    }
                )
                stop_distance = None
            else:
                stop_distance = float(stop_line_s) - float(stop_pose_s)
                if stop_distance < 0.0:
                    return self._fail(
                        "CROSSED_STOP_LINE",
                        f"stop_distance={stop_distance:.3f} < 0.0",
                    )
                if (
                    stop_distance < self.th.min_stop_distance
                    or stop_distance > self.th.max_stop_distance
                ):
                    score -= self.th.stop_distance_penalty
                    violations.append(
                        {
                            "type": "STOP_DISTANCE_PENALTY",
                            "message": (
                                f"stop_distance={stop_distance:.3f} outside "
                                f"[{self.th.min_stop_distance:.3f}, {self.th.max_stop_distance:.3f}]"
                            ),
                            "penalty": self.th.stop_distance_penalty,
                        }
                    )
        else:
            stop_distance = None

        status = "PASS" if score == 100 else "PENALTY"
        metrics: Dict[str, Any] = {
            "scenario_duration": scenario_duration,
            "max_ax": max_ax,
            "min_ax": min_ax,
            "max_abs_ay": max_abs_ay,
            "max_speed": max_speed,
        }
        if stop_distance is not None:
            metrics["stop_distance"] = stop_distance

        return {
            "scenario": "plan3_timeout_stopline",
            "status": status,
            "pass": score > 0,
            "final_score": score,
            "penalty_total": 100 - score,
            "violations": violations,
            "violation_count": len(violations),
            "metrics": metrics,
        }

    def _fail(self, violation_type: str, message: str) -> Dict[str, Any]:
        return {
            "scenario": "plan3_timeout_stopline",
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
    parser = argparse.ArgumentParser(description="Plan 3 scoring evaluator")
    parser.add_argument("--input", required=True, help="input JSON file")
    parser.add_argument("--output", default="", help="output JSON file")
    args = parser.parse_args()

    scorer = Plan3Scorer()
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

