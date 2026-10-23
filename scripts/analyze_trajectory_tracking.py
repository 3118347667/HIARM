#!/usr/bin/env python3
"""Compute per-trajectory position RMSE and plot controller tracking from a ROS 1 bag.

Run without ROS installed:
  uv run --with rosbags --with numpy --with matplotlib \
    python scripts/analyze_trajectory_tracking.py /path/to/recording.bag

Tracking windows come from /rosout's "Ref: ... => TRAJ" and
"Ref: TRAJ => ..." transitions. Each sample pairs reference/current position
and velocity from the same /flygripper/geometric_controller/data message.
Peak speed is the largest sampled XYZ velocity norm in that tracking window.
The controller data message has no header, so plots use rosbag record time.
"""

import argparse
import csv
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader


DEFAULT_BAG = Path("/Users/lzf/Documents/wfzf/8/2026-10-23-11-18-10.bag")
ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")
REF_TRANSITION = re.compile(r"\bRef:\s*(\w+)\s*=>\s*(\w+)")


@dataclass
class Event:
    time: float
    source: str
    target: str


@dataclass
class Trajectory:
    record_time: float
    start_time: float
    duration: float
    identifier: int


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def read_bag(path, namespace):
    controller_topic = f"/{namespace}/geometric_controller/data"
    trajectory_topic = f"/{namespace}/trajectory"
    goals_topic = "/move_base_simple/goal"
    requested = {controller_topic, trajectory_topic, goals_topic, "/rosout"}
    events, trajectories, goals, samples = [], [], [], []

    with AnyReader([path]) as reader:
        present = {connection.topic for connection in reader.connections}
        missing = {controller_topic, trajectory_topic, "/rosout"} - present
        if missing:
            raise ValueError(f"Required bag topics missing: {', '.join(sorted(missing))}")
        bag_start = reader.start_time * 1e-9
        connections = [c for c in reader.connections if c.topic in requested]
        for connection, timestamp, raw in reader.messages(connections=connections):
            message = reader.deserialize(raw, connection.msgtype)
            record_time = timestamp * 1e-9
            if connection.topic == "/rosout":
                match = REF_TRANSITION.search(ANSI_ESCAPE.sub("", message.msg))
                if match:
                    events.append(Event(record_time, *match.groups()))
            elif connection.topic == trajectory_topic:
                durations = [piece.duration for piece in message.xyz_pieces]
                if not durations or any(not math.isfinite(x) or x <= 0 for x in durations):
                    raise ValueError(f"Invalid XYZ pieces in trajectory {message.id}")
                trajectories.append(Trajectory(record_time, stamp_seconds(message.header.stamp),
                                               sum(durations), message.id))
            elif connection.topic == goals_topic:
                goals.append(record_time)
            elif connection.topic == controller_topic:
                ref = message.position_reference
                actual = message.position_current
                ref_vel = message.velocity_reference
                actual_vel = message.velocity_current
                row = (record_time, ref.x, ref.y, ref.z,
                       actual.x, actual.y, actual.z,
                       ref_vel.x, ref_vel.y, ref_vel.z,
                       actual_vel.x, actual_vel.y, actual_vel.z)
                if all(math.isfinite(x) for x in row):
                    samples.append(row)

    if not samples:
        raise ValueError(f"No finite controller position samples in {controller_topic}")
    return bag_start, sorted(events, key=lambda x: x.time), sorted(
        trajectories, key=lambda x: x.record_time), sorted(goals), np.asarray(samples)


def tracking_windows(events, trajectories, goals, samples):
    windows = []
    active_start = None
    for event in events:
        if event.target == "TRAJ" and event.source != "TRAJ":
            if active_start is not None:
                raise ValueError("A TRAJ entry has no preceding TRAJ exit")
            active_start = event.time
        elif event.source == "TRAJ" and event.target != "TRAJ":
            if active_start is None:
                raise ValueError("A TRAJ exit has no preceding TRAJ entry")
            windows.append((active_start, event.time))
            active_start = None
    if active_start is not None:
        # A recording stopped during tracking: use its final controller sample.
        windows.append((active_start, float(samples[-1, 0])))
    if not windows:
        raise ValueError("No reference-type TRAJ intervals found in /rosout")

    result = []
    for index, (start, end) in enumerate(windows):
        following_start = windows[index + 1][0] if index + 1 < len(windows) else math.inf
        candidates = [t for t in trajectories
                      if start - 0.1 <= t.record_time < min(end, following_start)
                      and abs(t.start_time - start) < 5.0]
        if len(candidates) != 1:
            raise ValueError(f"TRAJ interval {index + 1} has {len(candidates)} matching "
                             "trajectory messages; cannot assign an ID unambiguously")
        trajectory = candidates[0]
        selected = samples[(samples[:, 0] >= start) & (samples[:, 0] < end)]
        if len(selected) < 2:
            raise ValueError(f"TRAJ interval {index + 1} has fewer than two controller samples")
        preceding_goals = [g for g in goals if start - 5.0 <= g <= trajectory.record_time]
        goal = preceding_goals[-1] if preceding_goals else None
        result.append((start, end, trajectory, goal, selected))
    return result


def set_plot_style():
    plt.rcParams.update({
        "figure.dpi": 120,
        "savefig.dpi": 180,
        "font.size": 10,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "grid.alpha": 0.25,
    })


def plot_tracking_panel(number, trajectory_id, values, axis_rmse, rmse_xy, output):
    elapsed = values[:, 0] - values[0, 0]
    figure = plt.figure(figsize=(14, 13), layout="constrained")
    grid = figure.add_gridspec(4, 2, height_ratios=(1.1, 1.1, 0.8, 0.8))
    xy_axis = figure.add_subplot(grid[0, 0])
    x_axis = figure.add_subplot(grid[0, 1])
    y_axis = figure.add_subplot(grid[1, 0])
    z_axis = figure.add_subplot(grid[1, 1])
    speed_axis = figure.add_subplot(grid[2, :])
    error_axis = figure.add_subplot(grid[3, :], sharex=speed_axis)
    xy_axis.plot(values[:, 1], values[:, 2], color="#2367a8", lw=1.8,
                 label="Reference")
    xy_axis.plot(values[:, 4], values[:, 5], color="#dc6b26", lw=1.3,
                 label="Actual")
    xy_axis.plot(values[0, 1], values[0, 2], "o", color="#2367a8", ms=5)
    xy_axis.plot(values[0, 4], values[0, 5], "o", color="#dc6b26", ms=5)
    xy_axis.set(xlabel="X (m)", ylabel="Y (m)", title=f"XY plane · RMSE {rmse_xy:.4f} m")
    xy_axis.set_aspect("equal", adjustable="datalim")
    xy_axis.legend(loc="best")

    for axis, label, plot_axis in zip(range(3), "XYZ", (x_axis, y_axis, z_axis)):
        plot_axis.plot(elapsed, values[:, 1 + axis], color="#2367a8", lw=1.8)
        plot_axis.plot(elapsed, values[:, 4 + axis], color="#dc6b26", lw=1.0, alpha=0.9)
        plot_axis.set(xlabel="Time since tracking start (s)",
                      ylabel=f"{label} position (m)",
                      title=f"{label} tracking · RMSE {axis_rmse[axis]:.4f} m",
                      xlim=(0, elapsed[-1]))

    reference_speed = np.linalg.norm(values[:, 7:10], axis=1)
    actual_speed = np.linalg.norm(values[:, 10:13], axis=1)
    speed_axis.plot(elapsed, reference_speed, color="#2367a8", lw=1.7,
                    label="Reference speed")
    speed_axis.plot(elapsed, actual_speed, color="#dc6b26", lw=1.1,
                    label="Actual speed")
    speed_axis.set(ylabel="Speed (m/s)", title="3D speed tracking", xlim=(0, elapsed[-1]))
    speed_axis.legend(loc="best", ncol=2)

    position_error_3d = np.linalg.norm(values[:, 1:4] - values[:, 4:7], axis=1)
    rmse_xyz = float(np.sqrt(np.mean(np.square(position_error_3d))))
    error_axis.plot(elapsed, position_error_3d, color="#7c4da5", lw=1.3,
                    label="3D position error")
    error_axis.axhline(rmse_xyz, color="#5c6370", ls="--", lw=1.1,
                       label=f"3D RMSE {rmse_xyz:.4f} m")
    error_axis.set(xlabel="Time since tracking start (s)",
                   ylabel="Position error (m)", title="3D position error over time",
                   xlim=(0, elapsed[-1]), ylim=(0, None))
    error_axis.legend(loc="best", ncol=2)
    figure.suptitle(f"Trajectory tracking {number}  |  trajectory ID {trajectory_id}", fontsize=15)
    figure.savefig(output)
    plt.close(figure)


def write_csv(path, rows, fields):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def analyze(bag, output, namespace, on_plot=None):
    bag_start, events, trajectories, goals, samples = read_bag(bag, namespace)
    windows = tracking_windows(events, trajectories, goals, samples)
    output.mkdir(parents=True, exist_ok=True)
    set_plot_style()
    summaries = []
    sample_rows = []
    for number, (start, end, trajectory, goal, values) in enumerate(windows, 1):
        error = values[:, 1:4] - values[:, 4:7]
        mean_squared = np.mean(np.square(error), axis=0)
        axis_rmse = np.sqrt(mean_squared)
        rmse_xy = float(np.sqrt(mean_squared[:2].sum()))
        rmse_xyz = float(np.sqrt(mean_squared.sum()))
        reference_speed = np.linalg.norm(values[:, 7:10], axis=1)
        actual_speed = np.linalg.norm(values[:, 10:13], axis=1)
        reference_peak = int(np.argmax(reference_speed))
        actual_peak = int(np.argmax(actual_speed))
        planned_end = trajectory.start_time + trajectory.duration
        summary = {
            "run": number,
            "trajectory_id": trajectory.identifier,
            "goal_bag_offset_s": None if goal is None else round(goal - bag_start, 6),
            "trajectory_message_bag_offset_s": round(trajectory.record_time - bag_start, 6),
            "tracking_start_bag_offset_s": round(start - bag_start, 6),
            "tracking_end_bag_offset_s": round(end - bag_start, 6),
            "tracking_duration_s": round(end - start, 6),
            "planned_duration_s": round(trajectory.duration, 6),
            "planned_end_minus_tracking_end_s": round(planned_end - end, 6),
            "samples": len(values),
            "rmse_x_m": float(axis_rmse[0]),
            "rmse_y_m": float(axis_rmse[1]),
            "rmse_z_m": float(axis_rmse[2]),
            "rmse_xy_m": rmse_xy,
            "rmse_xyz_m": rmse_xyz,
            "max_reference_speed_m_s": float(reference_speed[reference_peak]),
            "max_actual_speed_m_s": float(actual_speed[actual_peak]),
            "reference_peak_bag_offset_s": round(float(values[reference_peak, 0] - bag_start), 6),
            "actual_peak_bag_offset_s": round(float(values[actual_peak, 0] - bag_start), 6),
        }
        summaries.append(summary)
        for row, delta, ref_speed, cur_speed in zip(values, error, reference_speed, actual_speed):
            sample_rows.append({
                "run": number, "trajectory_id": trajectory.identifier,
                "bag_offset_s": round(float(row[0] - bag_start), 6),
                "tracking_elapsed_s": round(float(row[0] - values[0, 0]), 6),
                "reference_x_m": row[1], "reference_y_m": row[2], "reference_z_m": row[3],
                "actual_x_m": row[4], "actual_y_m": row[5], "actual_z_m": row[6],
                "error_x_m": delta[0], "error_y_m": delta[1], "error_z_m": delta[2],
                "reference_vx_m_s": row[7], "reference_vy_m_s": row[8],
                "reference_vz_m_s": row[9], "actual_vx_m_s": row[10],
                "actual_vy_m_s": row[11], "actual_vz_m_s": row[12],
                "reference_speed_m_s": ref_speed, "actual_speed_m_s": cur_speed,
            })
        prefix = f"tracking_{number:02d}_id_{trajectory.identifier}"
        panel_path = output / f"{prefix}_overview.png"
        plot_tracking_panel(number, trajectory.identifier, values, axis_rmse, rmse_xy, panel_path)
        if on_plot is not None:
            on_plot(panel_path)
    write_csv(output / "rmse_summary.csv", summaries, list(summaries[0]))
    write_csv(output / "tracking_samples.csv", sample_rows, list(sample_rows[0]))
    (output / "rmse_summary.json").write_text(
        json.dumps({"bag": str(bag), "namespace": namespace,
                    "window_source": "/rosout Ref transitions",
                    "position_source": f"/{namespace}/geometric_controller/data",
                    "rmse_definition": "sqrt(mean((reference - actual)^2)) per controller sample; XY/XYZ sum axis MSE before square root",
                    "peak_speed_definition": "max(sqrt(vx^2 + vy^2 + vz^2)) over controller samples in each actual TRAJ interval",
                    "runs": summaries}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", nargs="?", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--output", type=Path, help="Output directory; default is next to the bag")
    parser.add_argument("--namespace", default="flygripper", help="Vehicle ROS namespace")
    args = parser.parse_args()
    bag = args.bag.expanduser().resolve()
    if not bag.is_file():
        parser.error(f"Bag does not exist: {bag}")
    output = args.output.expanduser().resolve() if args.output else bag.with_suffix("").with_name(
        bag.stem + "_tracking")
    try:
        summaries = analyze(bag, output, args.namespace.strip("/"))
    except (OSError, ValueError) as exc:
        parser.exit(1, f"Analysis failed: {exc}\n")
    print(f"Analyzed {len(summaries)} tracking runs from {bag}")
    for row in summaries:
        print(f"  Run {row['run']} (ID {row['trajectory_id']}): "
              f"X={row['rmse_x_m']:.4f}, Y={row['rmse_y_m']:.4f}, Z={row['rmse_z_m']:.4f}, "
              f"XY={row['rmse_xy_m']:.4f}, XYZ={row['rmse_xyz_m']:.4f} m | "
              f"max ref/actual speed={row['max_reference_speed_m_s']:.3f}/"
              f"{row['max_actual_speed_m_s']:.3f} m/s "
              f"({row['samples']} samples, {row['tracking_duration_s']:.2f} s)")
    print(f"Saved CSV, JSON and plots to {output}")


if __name__ == "__main__":
    main()
