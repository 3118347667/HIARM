#!/usr/bin/env python3
"""Save the actual /flygripper/trajectory as a dependency-free SVG plot."""

import argparse
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import rospy
from flygripper_msgs.msg import PolynomialTrajectory


def sample_trajectory(message):
    if not message.xyz_pieces:
        raise ValueError("trajectory contains no XYZ polynomial pieces")

    samples = []
    elapsed = 0.0
    for piece in message.xyz_pieces:
        dim, order = piece.dim, piece.order
        if dim != 3 or order > 32 or len(piece.coeff) != dim * (order + 1):
            raise ValueError("unsupported or incomplete XYZ polynomial piece")
        if not math.isfinite(piece.duration) or piece.duration <= 0:
            raise ValueError("trajectory piece has an invalid duration")
        if not all(math.isfinite(value) for value in piece.coeff):
            raise ValueError("trajectory piece has a non-finite coefficient")

        intervals = max(20, min(1000, math.ceil(piece.duration / 0.05)))
        for index in range(intervals + 1):
            if samples and index == 0:
                continue
            local_time = piece.duration * index / intervals
            position = []
            for axis in range(3):
                value = 0.0
                # Eigen's column-major matrix stores highest power first.
                for column in range(order + 1):
                    value = value * local_time + piece.coeff[dim * column + axis]
                position.append(value)
            if not all(math.isfinite(value) for value in position):
                raise ValueError("trajectory evaluates to a non-finite position")
            samples.append((elapsed + local_time, *position))
        elapsed += piece.duration

    return samples


def add(parent, tag, **attributes):
    return ET.SubElement(parent, tag, {key.replace("_", "-"): str(value)
                                       for key, value in attributes.items()})


def label(parent, x, y, content, **attributes):
    element = add(parent, "text", x=x, y=y, **attributes)
    element.text = content
    return element


def read_box():
    base = "/flygripper/poly_planner/"
    lower = rospy.get_param(base + "BoxMin", None)
    upper = rospy.get_param(base + "BoxMax", None)
    if (not isinstance(lower, list) or not isinstance(upper, list) or
            len(lower) != 3 or len(upper) != 3 or
            not all(isinstance(value, (int, float)) and math.isfinite(value)
                    for value in lower + upper) or
            any(low >= high for low, high in zip(lower, upper))):
        return None
    return lower, upper


def save_svg(message, samples, output, box):
    width, height = 1000, 900
    root = ET.Element("svg", xmlns="http://www.w3.org/2000/svg",
                      width=str(width), height=str(height),
                      viewBox=f"0 0 {width} {height}")
    add(root, "rect", x=0, y=0, width=width, height=height, fill="#fff")
    label(root, 100, 45, "Eight trajectory · planned reference", fill="#172336",
          font_size=25, font_family="sans-serif", font_weight="bold")

    xs = [sample[1] for sample in samples]
    ys = [sample[2] for sample in samples]
    zs = [sample[3] for sample in samples]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    if box:
        lower, upper = box
        xmin, xmax = min(xmin, lower[0]), max(xmax, upper[0])
        ymin, ymax = min(ymin, lower[1]), max(ymax, upper[1])
    margin = 0.4
    xmin, xmax = xmin - margin, xmax + margin
    ymin, ymax = ymin - margin, ymax + margin
    scale = min(800 / max(xmax - xmin, 0.1), 560 / max(ymax - ymin, 0.1))
    xmid, ymid = (xmin + xmax) / 2, (ymin + ymax) / 2

    def project(x, y):
        return 500 + (x - xmid) * scale, 370 - (y - ymid) * scale

    add(root, "rect", x=85, y=75, width=830, height=590,
        fill="#fbfcfe", stroke="#d7dfeb")
    for tick in range(math.ceil(xmin), math.floor(xmax) + 1):
        px, _ = project(tick, ymid)
        add(root, "line", x1=f"{px:.1f}", y1=75, x2=f"{px:.1f}", y2=665,
            stroke="#e5eaf1")
        label(root, f"{px:.1f}", 683, str(tick), text_anchor="middle",
              fill="#627083", font_size=13, font_family="sans-serif")
    for tick in range(math.ceil(ymin), math.floor(ymax) + 1):
        _, py = project(xmid, tick)
        add(root, "line", x1=85, y1=f"{py:.1f}", x2=915, y2=f"{py:.1f}",
            stroke="#e5eaf1")
        label(root, 75, f"{py + 4:.1f}", str(tick), text_anchor="end",
              fill="#627083", font_size=13, font_family="sans-serif")
    if box:
        left, top = project(lower[0], upper[1])
        right, bottom = project(upper[0], lower[1])
        add(root, "rect", x=f"{left:.1f}", y=f"{top:.1f}",
            width=f"{right-left:.1f}", height=f"{bottom-top:.1f}",
            fill="none", stroke="#8e9aaf", stroke_width=2,
            stroke_dasharray="7 5")

    points = [project(x, y) for _, x, y, _ in samples]
    add(root, "polyline", points=" ".join(f"{x:.2f},{y:.2f}" for x, y in points),
        fill="none", stroke="#1269c7", stroke_width=3.5,
        stroke_linecap="round", stroke_linejoin="round")
    for fraction in (0.25, 0.7):
        index = min(len(points) - 2, int((len(points) - 1) * fraction))
        x0, y0 = points[index]
        x1, y1 = points[index + 1]
        angle = math.atan2(y1 - y0, x1 - x0)
        if math.hypot(x1 - x0, y1 - y0) > 0.001:
            arrow = [(x0 + 11 * math.cos(angle), y0 + 11 * math.sin(angle)),
                     (x0 - 7 * math.cos(angle - 0.55), y0 - 7 * math.sin(angle - 0.55)),
                     (x0 - 7 * math.cos(angle + 0.55), y0 - 7 * math.sin(angle + 0.55))]
            add(root, "polygon", points=" ".join(f"{x:.1f},{y:.1f}" for x, y in arrow),
                fill="#1269c7")
    add(root, "circle", cx=f"{points[0][0]:.2f}", cy=f"{points[0][1]:.2f}",
        r=7, fill="#18945d", stroke="#fff", stroke_width=2)
    add(root, "circle", cx=f"{points[-1][0]:.2f}", cy=f"{points[-1][1]:.2f}",
        r=7, fill="#ce493e", stroke="#fff", stroke_width=2)
    label(root, 500, 710, "X (m)", text_anchor="middle", fill="#344054",
          font_size=16, font_family="sans-serif")
    label(root, 42, 370, "Y (m)", transform="rotate(-90 42 370)",
          text_anchor="middle", fill="#344054", font_size=16,
          font_family="sans-serif")

    zmin, zmax = min(zs), max(zs)
    zpad = max(0.03, (zmax - zmin) * 0.15)
    zlow, zhigh = zmin - zpad, zmax + zpad
    add(root, "rect", x=100, y=750, width=800, height=95,
        fill="#fbfcfe", stroke="#d7dfeb")
    z_points = []
    for stamp, _, _, z in samples:
        x = 100 + 800 * stamp / samples[-1][0]
        y = 845 - 95 * (z - zlow) / (zhigh - zlow)
        z_points.append(f"{x:.2f},{y:.2f}")
    add(root, "polyline", points=" ".join(z_points), fill="none",
        stroke="#e29329", stroke_width=2.5)
    label(root, 100, 740, f"Z (m) · {zmin:.2f}–{zmax:.2f}",
          fill="#344054", font_size=14, font_family="sans-serif")
    label(root, 100, 867, "0 s", fill="#627083", font_size=12,
          font_family="sans-serif")
    label(root, 900, 867, f"{samples[-1][0]:.2f} s", text_anchor="end",
          fill="#627083", font_size=12, font_family="sans-serif")
    label(root, 100, 891,
          f"Trajectory ID {message.id} · green=start · red=end · dashed=planner box",
          fill="#627083", font_size=12, font_family="sans-serif")

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description="Save one real planned eight trajectory as SVG")
    parser.add_argument("--output", type=Path, default=Path("~/eight_trajectory.svg"),
                        help="SVG output path (default: ~/eight_trajectory.svg)")
    parser.add_argument("--topic", default="/flygripper/trajectory")
    parser.add_argument("--timeout", type=float, default=None,
                        help="optional wait limit in seconds; default waits until a trajectory arrives")
    args = parser.parse_args(rospy.myargv()[1:])
    if args.output.suffix.lower() != ".svg":
        parser.error("--output must end in .svg")
    if args.timeout is not None and (not math.isfinite(args.timeout) or args.timeout <= 0):
        parser.error("--timeout must be a positive finite number")

    rospy.init_node("save_eight_trajectory", anonymous=True)
    print(f"Waiting for the next planned trajectory on {args.topic}...", flush=True)
    try:
        message = rospy.wait_for_message(args.topic, PolynomialTrajectory,
                                         timeout=args.timeout)
        samples = sample_trajectory(message)
        output = args.output.expanduser().resolve()
        save_svg(message, samples, output, read_box())
    except (rospy.ROSException, ValueError, OSError) as exc:
        print(f"Could not save trajectory: {exc}", file=sys.stderr)
        return 1
    print(f"Saved trajectory ID {message.id} ({samples[-1][0]:.2f} s) to {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
