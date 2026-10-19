#!/usr/bin/env python3

import argparse
import csv
import json
import os
import re
import statistics
import time
from pathlib import Path
from xmlrpc.client import ServerProxy

import rosgraph
import rosnode


DEFAULT_NODES = [
    "/far_vins_node",
    "/yolo_detector",
    "/target_ekf_node",
    "/drone0/manager",
    "/drone0/traj_server",
    "/flygripper/flygripper_flight_node",
]


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, int(len(ordered) * fraction + 0.999999) - 1)
    return ordered[index]


def resolve_node_pids(node_names):
    master = rosgraph.Master("/dynamic_grasp_resource_profiler")
    resolved = {}
    for node_name in node_names:
        try:
            api_uri = rosnode.get_api_uri(master, node_name)
            if not api_uri:
                continue
            code, _, pid = ServerProxy(api_uri).getPid(
                "/dynamic_grasp_resource_profiler"
            )
            if code == 1:
                resolved.setdefault(int(pid), []).append(node_name)
        except Exception as error:
            print(f"Unable to resolve {node_name}: {error}")
    return resolved


def read_process(pid):
    stat_fields = Path(f"/proc/{pid}/stat").read_text().split()
    cpu_ticks = int(stat_fields[13]) + int(stat_fields[14])
    rss_kb = 0
    for line in Path(f"/proc/{pid}/status").read_text().splitlines():
        if line.startswith("VmRSS:"):
            rss_kb = int(line.split()[1])
            break
    return cpu_ticks, rss_kb / 1024.0


def read_accelerator_percent(path):
    values = [float(value) for value in re.findall(r"([0-9.]+)\s*%", path.read_text())]
    return statistics.fmean(values) if values else None


def summarize(samples):
    cpu_core = [sample["cpu_core_percent"] for sample in samples]
    cpu_total = [sample["cpu_total_percent"] for sample in samples]
    rss_mb = [sample["rss_mb"] for sample in samples]
    return {
        "samples": len(samples),
        "cpu_core_percent_mean": statistics.fmean(cpu_core),
        "cpu_core_percent_p95": percentile(cpu_core, 0.95),
        "cpu_core_percent_max": max(cpu_core),
        "cpu_total_percent_mean": statistics.fmean(cpu_total),
        "cpu_total_percent_p95": percentile(cpu_total, 0.95),
        "cpu_total_percent_max": max(cpu_total),
        "rss_mb_mean": statistics.fmean(rss_mb),
        "rss_mb_peak": max(rss_mb),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--output", default="bags/runtime_resources.csv")
    parser.add_argument("--nodes", nargs="+", default=DEFAULT_NODES)
    parser.add_argument("--accelerator-path")
    args = parser.parse_args()

    pid_nodes = resolve_node_pids(args.nodes)
    if not pid_nodes:
        raise RuntimeError("No requested ROS nodes are currently running")

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    clock_ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    cpu_count = os.cpu_count() or 1
    previous = {}
    samples_by_pid = {pid: [] for pid in pid_nodes}
    aggregate_samples = []
    accelerator_samples = []
    accelerator_path = (
        Path(args.accelerator_path) if args.accelerator_path is not None else None
    )
    start_time = time.monotonic()

    with output_path.open("w", newline="") as output_file:
        writer = csv.DictWriter(
            output_file,
            fieldnames=[
                "elapsed_s",
                "pid",
                "nodes",
                "cpu_core_percent",
                "cpu_total_percent",
                "rss_mb",
            ],
        )
        writer.writeheader()

        while time.monotonic() - start_time < args.duration:
            sample_time = time.monotonic()
            current_rows = []
            for pid, node_names in list(pid_nodes.items()):
                try:
                    cpu_ticks, rss_mb = read_process(pid)
                except FileNotFoundError:
                    continue
                previous_sample = previous.get(pid)
                previous[pid] = (sample_time, cpu_ticks)
                if previous_sample is None:
                    continue
                wall_delta = sample_time - previous_sample[0]
                tick_delta = cpu_ticks - previous_sample[1]
                cpu_core_percent = tick_delta / clock_ticks / wall_delta * 100.0
                row = {
                    "elapsed_s": sample_time - start_time,
                    "pid": pid,
                    "nodes": ";".join(node_names),
                    "cpu_core_percent": cpu_core_percent,
                    "cpu_total_percent": cpu_core_percent / cpu_count,
                    "rss_mb": rss_mb,
                }
                samples_by_pid[pid].append(row)
                current_rows.append(row)
                writer.writerow(row)
            if current_rows:
                aggregate_samples.append(
                    {
                        "cpu_core_percent": sum(
                            row["cpu_core_percent"] for row in current_rows
                        ),
                        "cpu_total_percent": sum(
                            row["cpu_total_percent"] for row in current_rows
                        ),
                        "rss_mb": sum(row["rss_mb"] for row in current_rows),
                    }
                )
            if accelerator_path is not None:
                accelerator_percent = read_accelerator_percent(accelerator_path)
                if accelerator_percent is not None:
                    accelerator_samples.append(accelerator_percent)
            output_file.flush()
            time.sleep(max(0.0, args.interval - (time.monotonic() - sample_time)))

    summary = {
        ";".join(pid_nodes[pid]): summarize(samples)
        for pid, samples in samples_by_pid.items()
        if samples
    }
    if aggregate_samples:
        summary["complete_onboard_pipeline"] = summarize(aggregate_samples)
    if accelerator_samples:
        summary["accelerator_percent"] = {
            "samples": len(accelerator_samples),
            "mean": statistics.fmean(accelerator_samples),
            "p95": percentile(accelerator_samples, 0.95),
            "max": max(accelerator_samples),
        }
    summary_path = output_path.with_suffix(".summary.json")
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Wrote samples to {output_path}")
    print(f"Wrote summary to {summary_path}")


if __name__ == "__main__":
    main()
