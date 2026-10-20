#!/usr/bin/env python3
"""Export every ArduPilot parameter to a complete five-column .params file."""

import argparse
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import uuid


DEFAULT_OUTPUT = Path(__file__).resolve().parent.parent / "params" / "hiarm.params"
INTEGER_TYPES = {2, 4, 6}
SUPPORTED_TYPES = INTEGER_TYPES | {9}


def ensure_serial_free(device):
    fuser = shutil.which("fuser")
    if fuser is None:
        raise RuntimeError("未找到 fuser，无法检查串口占用；请先安装 psmisc")
    result = subprocess.run(
        [fuser, "-s", device], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        check=False,
    )
    if result.returncode == 0:
        raise RuntimeError(f"{device} 正被其他进程占用；请先停止 MAVROS")


def wait_autopilot(connection, mavlink, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        heartbeat = connection.recv_match(
            type="HEARTBEAT", blocking=True,
            timeout=max(0.0, min(1.0, deadline - time.monotonic())),
        )
        if heartbeat is None:
            continue
        if (heartbeat.get_srcComponent() == mavlink.MAV_COMP_ID_AUTOPILOT1
                and heartbeat.autopilot == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA):
            return heartbeat.get_srcSystem(), heartbeat.get_srcComponent()
    raise RuntimeError(f"{timeout:g} 秒内未收到 ArduPilot 飞控心跳")


def parameter_name(message):
    name = message.param_id
    if isinstance(name, bytes):
        name = name.decode("ascii", errors="strict")
    return name.rstrip("\0")


def collect_parameter(message, system, component, expected, by_index, by_name):
    if (message is None or message.get_type() != "PARAM_VALUE"
            or message.get_srcSystem() != system
            or message.get_srcComponent() != component):
        return expected, False
    count = message.param_count
    index = message.param_index
    if not 1 <= count <= 10000:
        raise RuntimeError(f"飞控返回异常参数总数 {count}")
    if expected is not None and count != expected:
        raise RuntimeError(f"参数总数从 {expected} 变为 {count}；拒绝保存不一致的快照")
    if not 0 <= index < count:
        return expected, False  # Unsolicited PARAM_VALUE has no usable list index.
    name = parameter_name(message)
    if (not 1 <= len(name) <= 16 or not name.isascii()
            or not name.isupper() or any(char.isspace() for char in name)):
        raise RuntimeError(f"参数序号 {index} 返回无效名称 {name!r}")
    if message.param_type not in SUPPORTED_TYPES:
        raise RuntimeError(f"{name} 的 MAVLink 类型 {message.param_type} 不受支持")
    value = message.param_value
    if not math.isfinite(value):
        raise RuntimeError(f"{name} 的参数值不是有限数")
    if message.param_type in INTEGER_TYPES and (
        not value.is_integer() or abs(value) > 2**24
    ):
        raise RuntimeError(f"{name} 的整数值无法从 MAVLink float 无损保存")
    previous_name = by_index.get(index)
    if previous_name is not None and (
        parameter_name(previous_name) != name
        or previous_name.param_type != message.param_type
        or previous_name.param_value != value
    ):
        raise RuntimeError(f"参数序号 {index} 的名称、类型或值发生变化")
    previous_index = by_name.get(name)
    if previous_index is not None and previous_index != index:
        raise RuntimeError(f"参数名 {name} 出现在序号 {previous_index} 和 {index}")
    by_index[index] = message
    by_name[name] = index
    return count, previous_name is None


def download_parameters(connection, system, component, total_timeout, read_timeout, retries):
    by_index = {}
    by_name = {}
    expected = None
    deadline = time.monotonic() + total_timeout
    last_progress = time.monotonic()
    list_attempts = 1
    connection.mav.param_request_list_send(system, component)
    print("已请求飞控参数列表...")

    while time.monotonic() < deadline:
        if expected is not None and len(by_index) == expected:
            return by_index
        remaining = deadline - time.monotonic()
        message = connection.recv_match(
            blocking=True, timeout=max(0.0, min(1.0, remaining)),
        )
        expected, added = collect_parameter(
            message, system, component, expected, by_index, by_name,
        )
        if added:
            last_progress = time.monotonic()
            if len(by_index) % 100 == 0:
                print(f"已接收 {len(by_index)}/{expected}")
        idle = time.monotonic() - last_progress
        if expected is None and idle >= 5:
            if list_attempts >= retries:
                raise RuntimeError("飞控没有返回参数列表")
            connection.mav.param_request_list_send(system, component)
            list_attempts += 1
            last_progress = time.monotonic()
            continue
        if expected is not None and (
            (expected - 1 in by_index and idle >= 1) or idle >= 5
        ):
            break

    if expected is None:
        raise RuntimeError("超时：未收到任何飞控参数")
    missing = [index for index in range(expected) if index not in by_index]
    if missing:
        print(f"参数列表缺少 {len(missing)} 项，正在按序号补读...")
    for index in missing:
        if index in by_index:
            continue
        for _ in range(retries):
            if time.monotonic() >= deadline:
                break
            connection.mav.param_request_read_send(system, component, b"", index)
            request_deadline = min(deadline, time.monotonic() + read_timeout)
            while time.monotonic() < request_deadline and index not in by_index:
                message = connection.recv_match(
                    blocking=True,
                    timeout=max(0.0, min(1.0, request_deadline - time.monotonic())),
                )
                expected, _ = collect_parameter(
                    message, system, component, expected, by_index, by_name,
                )
            if index in by_index:
                break
        if index not in by_index:
            raise RuntimeError(
                f"参数序号 {index} 补读失败；已收到 {len(by_index)}/{expected} 项，"
                "原参数文件未修改"
            )
        if len(by_index) % 100 == 0:
            print(f"已接收 {len(by_index)}/{expected}")
    if len(by_index) != expected:
        raise RuntimeError(f"参数快照不完整：{len(by_index)}/{expected}；原参数文件未修改")
    return by_index


def format_parameter(message, system, component):
    name = parameter_name(message)
    if message.param_type in INTEGER_TYPES:
        value = str(int(message.param_value))
    else:
        value = format(message.param_value, ".17g")
    return f"{system}\t{component}\t{name}\t{value}\t{message.param_type}\n"


def save_complete_snapshot(output, parameters, system, component):
    output.parent.mkdir(parents=True, exist_ok=True)
    backup = None
    if output.exists():
        backup_dir = output.parent / "backups"
        backup_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        backup = backup_dir / f"{output.stem}-before-export-{stamp}-{uuid.uuid4().hex[:8]}{output.suffix}"
        with output.open("rb") as original, backup.open("xb") as copy:
            shutil.copyfileobj(original, copy)
            copy.flush()
            os.fsync(copy.fileno())

    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="ascii", dir=output.parent,
            prefix=f".{output.name}.", suffix=".tmp", delete=False,
        ) as handle:
            temporary = Path(handle.name)
            handle.write(f"# Onboard parameters for Vehicle {system}\n")
            handle.write("# Stack: ArduPilot\n")
            handle.write(f"# Exported UTC: {datetime.now(timezone.utc).isoformat()}\n")
            handle.write("# Vehicle-Id Component-Id Name Value Type\n")
            for message in sorted(parameters.values(), key=parameter_name):
                handle.write(format_parameter(message, system, component))
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, output)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return backup


def main():
    parser = argparse.ArgumentParser(description="从 ArduPilot 读取完整参数并保存为 params/hiarm.params")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="输出 .params 路径")
    parser.add_argument("--device", default="/dev/ttyACM0", help="飞控串口，默认 /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=57600, help="串口波特率，默认 57600")
    parser.add_argument("--timeout", type=float, default=180.0, help="完整下载最长秒数，默认 180")
    parser.add_argument("--read-timeout", type=float, default=2.0, help="补读单项等待秒数，默认 2")
    parser.add_argument("--retries", type=int, default=3, help="请求重试次数，默认 3")
    args = parser.parse_args()
    if min(args.baud, args.timeout, args.read_timeout, args.retries) <= 0:
        parser.error("波特率、超时和重试次数必须大于 0")

    try:
        from pymavlink import mavutil
    except ImportError as exc:
        raise RuntimeError("未安装 pymavlink；请先在开发板的 Python 3 环境安装") from exc
    ensure_serial_free(args.device)
    try:
        connection = mavutil.mavlink_connection(args.device, baud=args.baud, autoreconnect=False)
    except (OSError, ValueError) as exc:
        raise RuntimeError(f"无法打开飞控串口 {args.device}: {exc}") from exc

    try:
        system, component = wait_autopilot(connection, mavutil.mavlink, min(args.timeout, 10.0))
        print(f"已连接 ArduPilot 飞控 {system}/{component}")
        parameters = download_parameters(
            connection, system, component, args.timeout, args.read_timeout, args.retries,
        )
    finally:
        connection.close()

    backup = save_complete_snapshot(args.output, parameters, system, component)
    print(f"已保存 {len(parameters)} 项完整参数到 {args.output}")
    if backup is not None:
        print(f"原文件备份：{backup}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, OSError, ValueError, KeyboardInterrupt) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        sys.exit(1)
