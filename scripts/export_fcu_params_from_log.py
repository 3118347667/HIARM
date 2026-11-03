#!/usr/bin/env python3
"""Export the latest DataFlash PARM values as an ArduPilot .param file."""

import argparse
import math
from pathlib import Path
import sys


DEFAULT_DIR = Path('/Users/lzf/Documents/wfzf/chat')


def read_parameters(path):
    try:
        from pymavlink import mavutil
    except ImportError as exc:
        raise RuntimeError('未安装 pymavlink；请运行 python3 -m pip install pymavlink') from exc

    parameters = {}
    records = 0
    updates = 0
    bad_records = 0
    connection = mavutil.mavlink_connection(str(path))
    try:
        while True:
            message = connection.recv_match()
            if message is None:
                break
            if message.get_type() == 'BAD_DATA':
                bad_records += 1
                continue
            if message.get_type() != 'PARM':
                continue
            name = message.Name
            if isinstance(name, bytes):
                name = name.decode('ascii', errors='strict')
            name = name.rstrip('\0')
            value = message.Value
            if (not name.isascii() or not name.isupper() or not 1 <= len(name) <= 16
                    or any(char.isspace() or char == ',' for char in name)):
                raise RuntimeError(f'日志中有无效参数名称：{name!r}')
            if not math.isfinite(value):
                raise RuntimeError(f'日志中参数 {name} 的值不是有限数')
            records += 1
            if name in parameters and parameters[name] != value:
                updates += 1
            parameters[name] = value
    finally:
        connection.close()
    if not parameters:
        raise RuntimeError('日志中没有找到 PARM 参数记录，未生成文件')
    return parameters, records, updates, bad_records


def main():
    parser = argparse.ArgumentParser(
        description='从 ArduPilot DataFlash BIN 日志提取参数，生成地面站可导入的两列 .param 文件。'
    )
    parser.add_argument('log', type=Path, help='DataFlash .BIN 日志路径')
    parser.add_argument('-o', '--output', type=Path, help='输出路径；默认保存到工作区 chat 目录')
    args = parser.parse_args()
    if not args.log.is_file():
        raise RuntimeError(f'日志文件不存在或不是普通文件：{args.log}')
    output = args.output or DEFAULT_DIR / f'{args.log.stem}.param'
    if output.resolve() == args.log.resolve():
        raise RuntimeError('输出路径不能与日志路径相同')
    parameters, records, updates, bad_records = read_parameters(args.log)
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        with output.open('x', encoding='ascii', newline='\n') as handle:
            for name, value in sorted(parameters.items()):
                handle.write(f'{name},{value:.17g}\n')
    except FileExistsError as exc:
        raise RuntimeError(f'输出文件已存在，请用 --output 指定新路径：{output}') from exc
    print(f'已导出 {len(parameters)} 个参数到 {output}（{records} 条记录，{updates} 次值变化）。')
    if bad_records:
        print(f'警告：解析时遇到 {bad_records} 条损坏数据，日志可能不完整。', file=sys.stderr)
    print('注意：文件仅包含日志中记录的参数，不能保证完整；重复参数使用最后一次记录。')
    print('导入其他飞机前请核对固件、硬件及各机独有的校准和配置参数。')


if __name__ == '__main__':
    try:
        main()
    except (OSError, UnicodeError, ValueError, RuntimeError) as exc:
        print(f'错误：{exc}', file=sys.stderr)
        sys.exit(1)
