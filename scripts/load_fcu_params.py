#!/usr/bin/env python3
"""Apply a QGroundControl five-column ArduPilot parameter export over MAVLink."""

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
from decimal import Decimal, InvalidOperation
import math
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import time


DEFAULT_FILE = Path(__file__).resolve().parent.parent / "params" / "hiarm.params"
SKIP_NAMES = {
    "ARSPD_OFFSET", "CMD_INDEX", "CMD_TOTAL", "FENCE_TOTAL",
    "FORMAT_VERSION", "GND_ABS_PRESS", "GND_TEMP", "LOG_LASTFILE",
    "MIS_TOTAL", "SYSID_SW_MREV", "SYS_NUM_RESETS",
}
INT_BOUNDS = {2: (-128, 127), 4: (-32768, 32767), 6: (-2147483648, 2147483647)}
LINK_NAMES = {"MAV_SYSID", "MAV_GCS_SYSID", "MAV_GCS_SYSID_HI"}


@dataclass(frozen=True)
class Parameter:
    system: int
    component: int
    name: str
    value: float
    param_type: int


def float32(value):
    return struct.unpack("<f", struct.pack("<f", value))[0]


def parse_file(path):
    parameters = []
    seen = set()
    try:
        lines = path.read_text(encoding="utf-8-sig").splitlines()
    except OSError as exc:
        raise RuntimeError(f"无法读取参数文件 {path}: {exc}") from exc
    for line_number, raw in enumerate(lines, 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 5:
            raise RuntimeError(f"{path}:{line_number}: 需要 5 列：系统 ID、组件 ID、名称、值、类型")
        system_text, component_text, name, value_text, type_text = fields
        try:
            system = int(system_text)
            component = int(component_text)
            param_type = int(type_text)
            value = Decimal(value_text)
        except (ValueError, InvalidOperation) as exc:
            raise RuntimeError(f"{path}:{line_number}: ID、参数值或类型无效") from exc
        if not (1 <= system <= 255 and 1 <= component <= 255):
            raise RuntimeError(f"{path}:{line_number}: 系统或组件 ID 超出范围")
        if not name.isascii() or not name.isupper() or not 1 <= len(name) <= 16:
            raise RuntimeError(f"{path}:{line_number}: 参数名必须为 1–16 个 ASCII 大写字符")
        if name in seen:
            raise RuntimeError(f"{path}:{line_number}: 重复参数 {name}")
        if param_type not in (*INT_BOUNDS, 9):
            raise RuntimeError(f"{path}:{line_number}: 不支持 {name} 的类型 {param_type}")
        if not value.is_finite():
            raise RuntimeError(f"{path}:{line_number}: {name} 的参数值不是有限数")
        if param_type in INT_BOUNDS:
            if value != value.to_integral_value():
                raise RuntimeError(f"{path}:{line_number}: {name} 的整数值含小数")
            integer = int(value)
            lower, upper = INT_BOUNDS[param_type]
            if not lower <= integer <= upper or abs(integer) > 2**24:
                raise RuntimeError(f"{path}:{line_number}: {name} 超出类型范围或无法用 MAVLink float 精确传输")
            numeric = float(integer)
        else:
            try:
                numeric = float32(float(value))
            except (OverflowError, struct.error) as exc:
                raise RuntimeError(f"{path}:{line_number}: {name} 的浮点值超出范围") from exc
            if not math.isfinite(numeric):
                raise RuntimeError(f"{path}:{line_number}: {name} 的浮点值超出范围")
        parameters.append(Parameter(system, component, name, numeric, param_type))
        seen.add(name)
    if not parameters:
        raise RuntimeError(f"{path}: 没有可读取的参数")
    identities = {(param.system, param.component) for param in parameters}
    if len(identities) != 1:
        raise RuntimeError(f"{path}: 包含多个飞控系统或组件 ID")
    return parameters


def ensure_serial_free(device):
    fuser = shutil.which("fuser")
    if fuser is None:
        raise RuntimeError("未找到 fuser，无法检查串口占用；请先安装 psmisc")
    used = subprocess.run(
        [fuser, "-s", device], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        check=False,
    )
    if used.returncode == 0:
        raise RuntimeError(f"{device} 正被其他进程占用；请先停止 MAVROS")


def param_name(message):
    name = message.param_id
    if isinstance(name, bytes):
        name = name.decode("ascii", errors="replace")
    return name.rstrip("\0")


def receive_parameter(connection, mavlink, parameter, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = connection.recv_match(
            blocking=True, timeout=max(0.0, min(1.0, deadline - time.monotonic())),
        )
        if message is None:
            continue
        if (message.get_type() == "HEARTBEAT"
                and message.get_srcSystem() == parameter.system
                and message.get_srcComponent() == parameter.component
                and message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            raise RuntimeError("飞控变为解锁状态，立即停止写入")
        if (message.get_type() == "PARAM_VALUE"
                and message.get_srcSystem() == parameter.system
                and message.get_srcComponent() == parameter.component
                and param_name(message) == parameter.name):
            return message
    return None


def read_parameter(connection, mavlink, parameter, timeout, retries):
    for _ in range(retries):
        connection.mav.param_request_read_send(
            parameter.system, parameter.component, parameter.name.encode("ascii"), -1,
        )
        value = receive_parameter(connection, mavlink, parameter, timeout)
        if value is not None:
            return value
    raise RuntimeError(f"读取 {parameter.name} 超时；未找到参数或 MAVLink 链路不稳定")


def check_heartbeat(connection, mavlink, system, component, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        heartbeat = connection.recv_match(
            type="HEARTBEAT", blocking=True,
            timeout=max(0.0, min(1.0, deadline - time.monotonic())),
        )
        if heartbeat is None:
            continue
        if (heartbeat.get_srcSystem() != system
                or heartbeat.get_srcComponent() != component
                or heartbeat.autopilot != mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA):
            continue
        if heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise RuntimeError("飞控处于解锁状态，拒绝写入参数")
        return
    raise RuntimeError(f"未收到 ArduPilot 飞控 {system}/{component} 的心跳")


def write_backup(path, changed, current):
    backup_dir = path.parent / "backups"
    backup_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup = backup_dir / f"before-hiarm-{stamp}-{time.time_ns() % 1000000:06d}.params"
    with backup.open("x", encoding="ascii") as output:
        output.write("# Parameters before applying hiarm.params; UTC backup\n")
        output.write("# Vehicle-Id Component-Id Name Value Type\n")
        for item in changed:
            previous = current[item.name]
            output.write(
                f"{item.system}\t{item.component}\t{item.name}\t"
                f"{previous.param_value:.17g}\t{previous.param_type}\n"
            )
    return backup


def main():
    parser = argparse.ArgumentParser(description="把 params/hiarm.params 的差异写入已上锁的 ArduPilot")
    parser.add_argument("--file", type=Path, default=DEFAULT_FILE, help="五列 .params 文件")
    parser.add_argument("--device", default="/dev/ttyACM0", help="飞控串口，默认 /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=57600, help="串口波特率，默认 57600")
    parser.add_argument("--timeout", type=float, default=2.0, help="单次读取/写入等待秒数，默认 2")
    parser.add_argument("--retries", type=int, default=3, help="单参数读取重试次数，默认 3")
    parser.add_argument("--dry-run", action="store_true", help="只比较参数，不写入飞控")
    parser.add_argument("--allow-link-changes", action="store_true",
                        help="允许修改可能中断当前连接的串口或系统 ID 参数")
    args = parser.parse_args()
    if args.baud <= 0 or args.timeout <= 0 or args.retries <= 0:
        parser.error("--baud、--timeout 和 --retries 必须大于 0")

    parameters = parse_file(args.file)
    system, component = parameters[0].system, parameters[0].component
    selected = [param for param in parameters if param.name not in SKIP_NAMES]
    skipped = len(parameters) - len(selected)
    print(f"文件包含 {len(parameters)} 项；跳过 {skipped} 项只读/系统状态参数。")

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
        check_heartbeat(connection, mavutil.mavlink, system, component, args.timeout * args.retries)
        current = {}
        print(f"已连接飞控 {system}/{component}，正在读取 {len(selected)} 项当前参数...")
        for number, item in enumerate(selected, 1):
            response = read_parameter(connection, mavutil.mavlink, item, args.timeout, args.retries)
            if response.param_type != item.param_type:
                raise RuntimeError(
                    f"{item.name} 类型不一致：文件={item.param_type}，飞控={response.param_type}；"
                    "未开始写入"
                )
            if item.param_type in INT_BOUNDS and abs(response.param_value) > 2**24:
                raise RuntimeError(
                    f"{item.name} 的飞控整数值无法用 MAVLink float 精确备份；未开始写入"
                )
            current[item.name] = response
            if number % 100 == 0 or number == len(selected):
                print(f"已读取 {number}/{len(selected)}")

        changed = [item for item in selected if current[item.name].param_value != item.value]
        print(f"检查完成：{len(changed)} 项需要写入，{len(selected) - len(changed)} 项已一致。")
        link_changes = [
            item.name for item in changed
            if item.name in LINK_NAMES or (
                item.name.startswith("SERIAL") and item.name.endswith(
                    ("_PROTOCOL", "_BAUD", "_OPTIONS")
                )
            )
        ]
        if args.dry_run:
            for item in changed[:20]:
                print(f"  {item.name}: {current[item.name].param_value:g} -> {item.value:g}")
            if len(changed) > 20:
                print(f"  ...另有 {len(changed) - 20} 项差异")
            if link_changes:
                print("连接相关参数存在差异：" + ", ".join(link_changes))
        if not changed or args.dry_run:
            return 0

        if link_changes and not args.allow_link_changes:
            raise RuntimeError(
                "连接相关参数存在差异，未开始写入：" + ", ".join(link_changes)
                + "；确认连接方式后可传 --allow-link-changes"
            )

        backup = write_backup(args.file, changed, current)
        print(f"已保存将被修改参数的原值：{backup}")
        # Connection settings go last, so other parameters can be verified first.
        changed.sort(
            key=lambda item: 3 if item.name == "MAV_SYSID"
            else 2 if item.name.startswith("SERIAL0_")
            else 1 if item.name in link_changes else 0
        )
        confirmed = 0
        try:
            for item in changed:
                if confirmed % 20 == 0:
                    check_heartbeat(
                        connection, mavutil.mavlink, system, component,
                        args.timeout * args.retries,
                    )
                connection.mav.param_set_send(
                    system, component, item.name.encode("ascii"), item.value, item.param_type,
                )
                echoed = receive_parameter(connection, mavutil.mavlink, item, args.timeout)
                if echoed is None or echoed.param_value != item.value:
                    raise RuntimeError(f"{item.name} 的写入回执缺失或值不一致")
                # ArduPilot may echo a proposed value before reverting a read-only parameter.
                time.sleep(0.3)
                actual = read_parameter(
                    connection, mavutil.mavlink, item, args.timeout, args.retries,
                )
                if actual.param_value != item.value:
                    raise RuntimeError(
                        f"{item.name} 回读值 {actual.param_value} 与文件值 {item.value} 不一致"
                    )
                confirmed += 1
                if confirmed % 20 == 0 or confirmed == len(changed):
                    print(f"已写入并回读确认 {confirmed}/{len(changed)}")
        except (RuntimeError, OSError) as exc:
            raise RuntimeError(
                f"写入中断：已确认 {confirmed}/{len(changed)} 项；"
                f"部分参数可能已改变，原值备份在 {backup}。原因：{exc}"
            ) from exc
        print(f"完成：{confirmed} 项参数已写入并回读确认。")
        return 0
    finally:
        connection.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, OSError, KeyboardInterrupt) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        sys.exit(1)
