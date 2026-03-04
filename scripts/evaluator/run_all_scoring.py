#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unified scoring runner for plan1..plan5.

Usage examples:
1) Single plan:
   python scripts/evaluator/run_all_scoring.py --plan 1 --input scripts/evaluator/examples/plan1.json

2) All plans from a folder containing plan1.json ... plan5.json:
   python scripts/evaluator/run_all_scoring.py --plan all --input-dir scripts/evaluator/examples
"""

from __future__ import annotations

import argparse
import json
import os
from typing import Any, Callable, Dict, List

from construction_zone_scoring import ConstructionZoneScorer
from parking_evaluation import ParkingScorer
from plan1_scoring import Plan1Scorer
from plan2_scoring import Plan2Scorer
from plan3_scoring import Plan3Scorer


def load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path: str, payload: Dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)


def normalize_result(plan: str, raw: Dict[str, Any]) -> Dict[str, Any]:
    score = int(raw.get("final_score", 0))
    status = raw.get("status")
    if not status:
        if score <= 0:
            status = "FAIL"
        elif score == 100:
            status = "PASS"
        else:
            status = "PENALTY"
    return {
        "plan": plan,
        "scenario": raw.get("scenario", f"plan{plan}"),
        "status": status,
        "pass": bool(raw.get("pass", score > 0)),
        "final_score": score,
        "penalty_total": int(raw.get("penalty_total", max(0, 100 - score))),
        "violations": raw.get("violations", []),
        "violation_count": int(raw.get("violation_count", len(raw.get("violations", [])))),
        "metrics": raw.get("metrics", {}),
        "raw": raw,
    }


def run_plan1(payload: Dict[str, Any]) -> Dict[str, Any]:
    return normalize_result("1", Plan1Scorer().score(payload))


def run_plan2(payload: Dict[str, Any]) -> Dict[str, Any]:
    return normalize_result("2", Plan2Scorer().score(payload))


def run_plan3(payload: Dict[str, Any]) -> Dict[str, Any]:
    return normalize_result("3", Plan3Scorer().score(payload))


def run_plan4(payload: Dict[str, Any]) -> Dict[str, Any]:
    params = payload.get("params", {})
    scorer = ConstructionZoneScorer(
        speed_limit=float(params.get("speed_limit", 8.333)),
        penalty_per_frame_per_mps=float(params.get("penalty_per_frame_per_mps", 2.0)),
    )
    raw = scorer.score_trajectory(payload.get("trajectory", []), payload.get("construction_zones", []))
    return normalize_result("4", raw)


def run_plan5(payload: Dict[str, Any]) -> Dict[str, Any]:
    params = payload.get("params", {})
    scorer = ParkingScorer(
        time_limit_sec=float(params.get("time_limit_sec", 90.0)),
        line_violation_penalty=int(params.get("line_violation_penalty", 20)),
    )
    log_data = payload.get("log_data", payload)
    raw = scorer.score_parking_scenario(log_data)
    return normalize_result("5", raw)


PLAN_RUNNERS: Dict[str, Callable[[Dict[str, Any]], Dict[str, Any]]] = {
    "1": run_plan1,
    "2": run_plan2,
    "3": run_plan3,
    "4": run_plan4,
    "5": run_plan5,
}


def run_single(plan: str, input_path: str) -> Dict[str, Any]:
    payload = load_json(input_path)
    return PLAN_RUNNERS[plan](payload)


def run_all(input_dir: str) -> Dict[str, Any]:
    results: List[Dict[str, Any]] = []
    for plan in ["1", "2", "3", "4", "5"]:
        plan_file = os.path.join(input_dir, f"plan{plan}.json")
        if not os.path.exists(plan_file):
            results.append(
                {
                    "plan": plan,
                    "status": "ERROR",
                    "pass": False,
                    "final_score": 0,
                    "violation_count": 1,
                    "violations": [
                        {
                            "type": "MISSING_INPUT_FILE",
                            "message": f"missing {plan_file}",
                            "penalty": 100,
                        }
                    ],
                    "metrics": {},
                    "raw": {},
                }
            )
            continue
        results.append(run_single(plan, plan_file))

    scored = [r for r in results if r.get("status") != "ERROR"]
    avg_score = sum(r["final_score"] for r in scored) / len(scored) if scored else 0.0
    passed = sum(1 for r in scored if r["status"] in ("PASS", "PENALTY"))
    failed = sum(1 for r in scored if r["status"] == "FAIL")
    errored = sum(1 for r in results if r["status"] == "ERROR")

    return {
        "mode": "all",
        "results": results,
        "summary": {
            "plans_total": 5,
            "plans_scored": len(scored),
            "plans_pass_or_penalty": passed,
            "plans_fail": failed,
            "plans_error": errored,
            "avg_score": round(avg_score, 2),
            "total_score": int(sum(r["final_score"] for r in scored)),
        },
    }


def print_result(result: Dict[str, Any]) -> None:
    if result.get("mode") == "all":
        print("Unified scoring summary (plan1..plan5)")
        for item in result["results"]:
            print(
                f"plan{item['plan']}: status={item['status']} score={item['final_score']} "
                f"violations={item['violation_count']}"
            )
        summary = result["summary"]
        print(
            "summary: "
            f"scored={summary['plans_scored']}/5 "
            f"pass_or_penalty={summary['plans_pass_or_penalty']} "
            f"fail={summary['plans_fail']} error={summary['plans_error']} "
            f"avg_score={summary['avg_score']}"
        )
        return

    print(
        f"plan{result['plan']}: status={result['status']} score={result['final_score']} "
        f"violations={result['violation_count']}"
    )
    for idx, violation in enumerate(result["violations"], 1):
        print(f"{idx}. {violation.get('type', 'UNKNOWN')}: {violation.get('message', '')}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run scoring checks for plan1..plan5")
    parser.add_argument("--plan", required=True, choices=["1", "2", "3", "4", "5", "all"], help="plan id")
    parser.add_argument("--input", default="", help="single input json when --plan != all")
    parser.add_argument("--input-dir", default="", help="folder containing plan1.json..plan5.json when --plan=all")
    parser.add_argument("--output", default="", help="output json path")
    args = parser.parse_args()

    if args.plan == "all":
        input_dir = args.input_dir or "scripts/evaluator/examples"
        result = run_all(input_dir)
    else:
        if not args.input:
            raise SystemExit("--input is required when --plan is not all")
        result = run_single(args.plan, args.input)

    print_result(result)
    if args.output:
        save_json(args.output, result)


if __name__ == "__main__":
    main()

