#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plan 1 scoring checks (offline evaluator).

Rules implemented:
1) Longitudinal acceleration upper bound <= 3.0 m/s^2
2) Longitudinal deceleration lower bound >= -6.0 m/s^2
3) Lateral acceleration absolute bound <= 2.0 m/s^2
4) Speed upper bound <= 16.67 m/s
5) Stop distance in [2.0, 2.5] meters:
   - outside interval: -20 points
   - crossed stop line or did not stop: fail (0 points)
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from typing import Any, Dict, List, Optional


@dataclass
class Plan1Thresholds:
    max_accel: float = 3.0
    min_accel: float = -6.0
    max_lateral_accel_abs: float = 2.0
    max_speed: float = 16.67
    min_stop_distance: float = 2.0
    max_stop_distance: float = 2.5
    stop_distance_penalty: int = 20


class Plan1Scorer:
    def __init__(self, thresholds: Optional[Plan1Thresholds] = None) -> None:
        self.th = thresholds or Plan1Thresholds()

    def score(self, data: Dict[str, Any]) -> Dict[str, Any]:
        trajectory = data.get("trajectory", [])
        if not trajectory:
            return self._fail("MISSING_TRAJECTORY", "trajectory is empty")

        max_ax = max(float(frame.get("ax", 0.0)) for frame in trajectory)
        min_ax = min(float(frame.get("ax", 0.0)) for frame in trajectory)
        max_abs_ay = max(abs(float(frame.get("ay", 0.0))) for frame in trajectory)
        max_speed = max(float(frame.get("v", 0.0)) for frame in trajectory)

        if max_ax > self.th.max_accel:
            return self._fail(
                "ACCELERATION_VIOLATION",
                f"max_ax={max_ax:.3f} > {self.th.max_accel:.3f}",
                metrics=self._metrics(max_ax, min_ax, max_abs_ay, max_speed),
            )
        if min_ax < self.th.min_accel:
            return self._fail(
                "DECELERATION_VIOLATION",
                f"min_ax={min_ax:.3f} < {self.th.min_accel:.3f}",
                metrics=self._metrics(max_ax, min_ax, max_abs_ay, max_speed),
            )
        if max_abs_ay > self.th.max_lateral_accel_abs:
            return self._fail(
                "LATERAL_ACCELERATION_VIOLATION",
                f"max_abs_ay={max_abs_ay:.3f} > {self.th.max_lateral_accel_abs:.3f}",
                metrics=self._metrics(max_ax, min_ax, max_abs_ay, max_speed),
            )
        if max_speed > self.th.max_speed:
            return self._fail(
                "SPEED_VIOLATION",
                f"max_speed={max_speed:.3f} > {self.th.max_speed:.3f}",
                metrics=self._metrics(max_ax, min_ax, max_abs_ay, max_speed),
            )

        violations: List[Dict[str, Any]] = []
        score = 100

        stopped = bool(data.get("stopped", trajectory[-1].get("stopped", False)))
        if not stopped:
            return self._fail(
                "NOT_STOPPED",
                "vehicle did not stop",
                metrics=self._metrics(max_ax, min_ax, max_abs_ay, max_speed),
            )

        stop_line_s = data.get("stop_line_s")
        stop_pose_s = data.get("stop_pose_s", trajectory[-1].get("s"))
        if stop_line_s is not None and stop_pose_s is not None:
            stop_distance = float(stop_line_s) - float(stop_pose_s)
            if stop_distance < 0.0:
                return self._fail(
                    "CROSSED_STOP_LINE",
                    f"stop_distance={stop_distance:.3f} < 0.0",
                    metrics=self._metrics(max_ax, min_ax, max_abs_ay, max_speed, stop_distance),
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
            metrics = self._metrics(max_ax, min_ax, max_abs_ay, max_speed, stop_distance)
        else:
            metrics = self._metrics(max_ax, min_ax, max_abs_ay, max_speed)
            violations.append(
                {
                    "type": "STOP_DISTANCE_UNKNOWN",
                    "message": "stop_line_s or stop_pose_s missing; stop distance not checked",
                    "penalty": 0,
                }
            )

        status = "PASS" if score == 100 else "PENALTY"
        return {
            "scenario": "plan1_stop_sign",
            "status": status,
            "pass": score > 0,
            "final_score": score,
            "penalty_total": 100 - score,
            "violations": violations,
            "violation_count": len(violations),
            "metrics": metrics,
        }

    def _metrics(
        self,
        max_ax: float,
        min_ax: float,
        max_abs_ay: float,
        max_speed: float,
        stop_distance: Optional[float] = None,
    ) -> Dict[str, Any]:
        metrics: Dict[str, Any] = {
            "max_ax": max_ax,
            "min_ax": min_ax,
            "max_abs_ay": max_abs_ay,
            "max_speed": max_speed,
        }
        if stop_distance is not None:
            metrics["stop_distance"] = stop_distance
        return metrics

    def _fail(
        self,
        violation_type: str,
        message: str,
        metrics: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        return {
            "scenario": "plan1_stop_sign",
            "status": "FAIL",
            "pass": False,
            "final_score": 0,
            "penalty_total": 100,
            "violations": [{"type": violation_type, "message": message, "penalty": 100}],
            "violation_count": 1,
            "metrics": metrics or {},
        }


def load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plan 1 scoring evaluator")
    parser.add_argument("--input", required=True, help="input JSON file")
    parser.add_argument("--output", default="", help="output JSON file")
    args = parser.parse_args()

    scorer = Plan1Scorer()
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

