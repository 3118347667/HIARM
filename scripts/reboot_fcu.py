#!/usr/bin/env python3
"""Reboot a disarmed flight controller over its direct MAVLink serial port."""

import argparse
import shutil
import subprocess
import sys
import time


def check_serial_port(device):
    """Avoid opening the same serial port while MAVROS already owns it."""
    fuser = shutil.which("fuser")
    if fuser is None:
        raise RuntimeError("未找到 fuser，无法检查串口占用；请先安装 psmisc")
    result = subprocess.run(
        [fuser, "-s", device], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        check=False,
    )
    if result.returncode == 0:
        raise RuntimeError(f"{device} 正被其他进程占用；请先停止 MAVROS，再重试")


def wait_autopilot_heartbeat(connection, mavlink, timeout, system=None, component=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        heartbeat = connection.recv_match(
            type="HEARTBEAT", blocking=True,
            timeout=max(0.0, min(1.0, deadline - time.monotonic())),
        )
        if heartbeat is None:
            continue
        if heartbeat.get_srcComponent() != mavlink.MAV_COMP_ID_AUTOPILOT1:
            continue
        if heartbeat.autopilot == mavlink.MAV_AUTOPILOT_INVALID:
            continue
        if system is not None and heartbeat.get_srcSystem() != system:
            continue
        if component is not None and heartbeat.get_srcComponent() != component:
            continue
        if heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise RuntimeError("飞控仍处于解锁状态，拒绝重启；请先安全降落并上锁")
        return heartbeat
    raise RuntimeError(f"{timeout:g} 秒内未收到飞控心跳，未发送重启命令")


def wait_reboot_ack(connection, mavlink, system, component, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ack = connection.recv_match(
            type="COMMAND_ACK", blocking=True,
            timeout=max(0.0, min(1.0, deadline - time.monotonic())),
        )
        if (ack is not None
                and ack.command == mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
                and ack.get_srcSystem() == system
                and ack.get_srcComponent() == component):
            return ack
    return None


def main():
    parser = argparse.ArgumentParser(description="通过 pymavlink 串口连接重启已上锁的飞控")
    parser.add_argument("--device", default="/dev/ttyACM0", help="飞控串口，默认 /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=57600, help="串口波特率，默认 57600")
    parser.add_argument("--timeout", type=float, default=5.0, help="等待心跳和回执的秒数，默认 5")
    args = parser.parse_args()
    if args.baud <= 0 or args.timeout <= 0:
        parser.error("--baud 和 --timeout 必须大于 0")

    try:
        from pymavlink import mavutil
    except ImportError as exc:
        raise RuntimeError("未安装 pymavlink；请先在开发板的 Python 3 环境安装") from exc

    check_serial_port(args.device)
    try:
        connection = mavutil.mavlink_connection(args.device, baud=args.baud, autoreconnect=False)
    except (OSError, ValueError) as exc:
        raise RuntimeError(f"无法打开飞控串口 {args.device}: {exc}") from exc

    try:
        heartbeat = wait_autopilot_heartbeat(connection, mavutil.mavlink, args.timeout)
        system = heartbeat.get_srcSystem()
        component = heartbeat.get_srcComponent()
        print(f"已连接飞控 system={system}, component={component}；当前已上锁")
        print("发送飞控重启命令...")
        connection.mav.command_long_send(
            system, component, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        ack = wait_reboot_ack(connection, mavutil.mavlink, system, component, args.timeout)
        if ack is None:
            raise RuntimeError("未收到重启回执；飞控可能已开始重启，请检查心跳，勿立即重复发送")
        if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"飞控未确认接受重启命令 (MAV_RESULT={ack.result})；请先检查飞控状态")
        print("飞控接受了重启命令；请等待心跳断开并重新出现，以确认重启完成。")
        return 0
    finally:
        connection.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, OSError, KeyboardInterrupt) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        sys.exit(1)
