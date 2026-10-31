#!/usr/bin/env python3
"""Download the latest ArduPilot DataFlash log on the onboard computer."""
import argparse
from datetime import datetime
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import uuid


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def worker(device, baud, output):
    from pymavlink import mavutil

    if not shutil.which('fuser'):
        raise RuntimeError('板端缺少 fuser，无法检查串口占用')
    if not os.path.exists(device):
        raise RuntimeError(f'串口不存在：{device}')
    if subprocess.run(['fuser', '-s', device], check=False).returncode == 0:
        raise RuntimeError(f'{device} 正被占用，请先停止 MAVROS 或其他串口程序')
    conn = mavutil.mavlink_connection(device, baud=baud, source_system=253)
    system = component = None

    def receive(kind, timeout):
        nonlocal system, component
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = conn.recv_match(blocking=True, timeout=min(1, max(0, deadline-time.monotonic())))
            if msg is None:
                continue
            if msg.get_type() == 'HEARTBEAT' and msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1 and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                if system is None:
                    system, component = msg.get_srcSystem(), msg.get_srcComponent()
                if msg.get_srcSystem() == system and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    raise RuntimeError('飞控已解锁，停止下载')
                if kind == 'HEARTBEAT' and msg.get_srcSystem() == system:
                    return msg
            if system is not None and msg.get_srcSystem() == system and msg.get_srcComponent() == component and msg.get_type() == kind:
                return msg
        return None

    try:
        if receive('HEARTBEAT', 10) is None:
            raise RuntimeError('未收到 ArduPilot 飞控心跳')
        print('飞控心跳正常，正在读取日志目录…', flush=True)
        latest = None
        for _ in range(4):
            conn.mav.log_request_list_send(system, component, 0, 65535)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                entry = receive('LOG_ENTRY', min(1, deadline-time.monotonic()))
                if entry is None:
                    continue
                if entry.num_logs == 0:
                    raise RuntimeError('飞控没有日志')
                if entry.id == entry.last_log_num:
                    latest = entry
                    break
            if latest is not None:
                break
        if latest is None:
            raise RuntimeError('未收到最新日志的目录项')
        if latest.size <= 0:
            raise RuntimeError(f'最新日志 ID {latest.id} 为空')
        print(f'最新日志 ID {latest.id}，大小 {latest.size} 字节，开始下载…', flush=True)
        with open(output, 'wb') as f:
            offset = 0
            last_progress = -1
            while offset < latest.size:
                end = min(offset + 4096, latest.size)
                pending = {}
                retries = 0
                while offset < end:
                    conn.mav.log_request_data_send(system, component, latest.id, offset, end-offset)
                    deadline = time.monotonic() + 2
                    start = offset
                    while offset < end and time.monotonic() < deadline:
                        msg = receive('LOG_DATA', max(0, deadline-time.monotonic()))
                        if msg is None:
                            break
                        if msg.id != latest.id or msg.count <= 0 or msg.ofs < offset or msg.ofs >= end:
                            continue
                        data = bytes(msg.data[:msg.count])[:end-msg.ofs]
                        pending[msg.ofs] = data
                        while offset in pending:
                            data = pending.pop(offset)
                            f.write(data)
                            offset += len(data)
                    retries = retries + 1 if offset == start else 0
                    if retries >= 5:
                        raise RuntimeError(f'日志下载超时，停在 {offset}/{latest.size} 字节')
                progress = offset * 100 // latest.size
                if progress // 10 > last_progress // 10 or offset == latest.size:
                    print(f'下载进度 {progress}%', flush=True)
                    last_progress = progress
        conn.mav.log_request_end_send(system, component)
        if os.path.getsize(output) != latest.size:
            raise RuntimeError('下载文件大小与飞控目录不符')
        return latest.id, latest.size
    finally:
        conn.close()


def main():
    p = argparse.ArgumentParser(description='在机载开发板下载飞控最新 DataFlash 日志到项目 bin 目录。')
    p.add_argument('--device', default='/dev/ttyACM0', help='飞控串口（默认 /dev/ttyACM0）')
    p.add_argument('--baud', type=int, default=57600, help='飞控串口波特率（默认 57600）')
    p.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parent.parent / 'bin', help='保存目录（默认工作区 bin）')
    args = p.parse_args()
    if args.baud <= 0:
        raise RuntimeError('波特率必须大于零')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    token = uuid.uuid4().hex
    tmp = args.output_dir / f'.download_{token}.tmp'
    try:
        log_id, size = worker(args.device, args.baud, tmp)
        digest = sha256_file(tmp)
        destination = args.output_dir / f"fcu_log_{log_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{token[:8]}.BIN"
        tmp.rename(destination)
        print(f'已保存：{destination.resolve()}（{size} 字节）')
        print(f'SHA-256：{digest}')
    finally:
        tmp.unlink(missing_ok=True)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, subprocess.CalledProcessError, ImportError) as exc:
        print(f'错误：{exc}', file=sys.stderr)
        sys.exit(1)
