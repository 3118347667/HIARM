#!/usr/bin/env python3

import argparse
import csv
import re
from pathlib import Path


PROFILE_PATTERN = re.compile(
    r"\[PROFILE\] module=(?P<module>\S+) samples=(?P<samples>\d+) "
    r"mean_ms=(?P<mean_ms>[0-9.eE+-]+) p95_ms=(?P<p95_ms>[0-9.eE+-]+) "
    r"max_ms=(?P<max_ms>[0-9.eE+-]+) rate_hz=(?P<rate_hz>[0-9.eE+-]+)"
)

PERIOD_MS = {
    "vins_state_estimation": 1000.0 / 30.0,
    "local_mapping": 1000.0 / 30.0,
    "target_detection": 1000.0 / 30.0,
    "target_pose_update": 1000.0 / 30.0,
    "target_filter_prediction": 1000.0 / 10.0,
    "target_prediction": 1000.0 / 10.0,
    "grasp_path_search": 1000.0 / 20.0,
    "visibility_region_construction": 1000.0 / 20.0,
    "corridor_generation": 1000.0 / 20.0,
    "trajectory_optimization": 1000.0 / 20.0,
    "planning_total": 1000.0 / 20.0,
    "trajectory_tracking_control": 1000.0 / 333.0,
}


def collect_latest(log_paths):
    latest = {}
    for log_path in log_paths:
        for line in log_path.read_text(errors="replace").splitlines():
            match = PROFILE_PATTERN.search(line)
            if match:
                values = match.groupdict()
                latest[values["module"]] = values
    return latest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_root", nargs="?", default=str(Path.home() / ".ros/log/latest"))
    parser.add_argument("--output", default="bags/runtime_timing.csv")
    args = parser.parse_args()

    log_root = Path(args.log_root)
    log_paths = [log_root] if log_root.is_file() else list(log_root.rglob("*.log"))
    latest = collect_latest(log_paths)
    if not latest:
        raise RuntimeError(f"No [PROFILE] records found under {log_root}")

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as output_file:
        fieldnames = [
            "module",
            "samples",
            "rate_hz",
            "period_ms",
            "mean_ms",
            "p95_ms",
            "max_ms",
            "margin_percent",
        ]
        writer = csv.DictWriter(output_file, fieldnames=fieldnames)
        writer.writeheader()
        for module, values in sorted(latest.items()):
            period_ms = PERIOD_MS.get(module)
            max_ms = float(values["max_ms"])
            margin_percent = (
                (1.0 - max_ms / period_ms) * 100.0 if period_ms is not None else ""
            )
            writer.writerow(
                {
                    "module": module,
                    "samples": values["samples"],
                    "rate_hz": values["rate_hz"],
                    "period_ms": f"{period_ms:.6f}" if period_ms is not None else "",
                    "mean_ms": values["mean_ms"],
                    "p95_ms": values["p95_ms"],
                    "max_ms": values["max_ms"],
                    "margin_percent": (
                        f"{margin_percent:.6f}" if margin_percent != "" else ""
                    ),
                }
            )
    print(f"Wrote timing summary to {output_path}")


if __name__ == "__main__":
    main()
