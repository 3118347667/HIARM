#!/usr/bin/env python3
"""Audit the entire can closure and the first six seconds of extraction."""
# 对易拉罐收拢夹爪到开始取出后的六秒做仿真回归检查，不代表真实硬件安全界限。
import json
import math
import sys
from pathlib import Path


def evaluate(report, rows):
    # 从规划器日志定位易拉罐收拢开始和 task_2 搬运开始的时刻。
    start = next((e['sim_time'] for e in report['events']
                  if 'stage 1 with morphing SHRINKING' in e['message']), None)
    extraction = next((e['sim_time'] for e in report['events']
                       if 'stage 2 with velocity' in e['message']), None)
    if start is None or not rows:
        return dict(complete=False, stable=False, reason='Missing can closing motion')
    end = extraction + 6 if extraction is not None else rows[-1]['t']
    complete = extraction is not None and rows[-1]['t'] >= end
    selected = [r for r in rows if start <= r['t'] <= end]
    if not selected:
        return dict(complete=False, stable=False, reason='Missing samples in interval')
    # 先检查采样数值有效，再由姿态四元数和角速度计算最大倾角/转速。
    finite = all(math.isfinite(v) for r in selected for k in ('p', 'q', 'v', 'omega') for v in r[k])
    def tilt(q):
        return math.degrees(math.acos(max(-1, min(1, 1-2*(q[0]**2+q[1]**2)))))
    peak_tilt = max(tilt(r['q']) for r in selected) if finite else None
    peak_rate = max(math.hypot(*r['omega']) for r in selected) if finite else None
    contacts = [e for e in report.get('support_contacts', []) if start <= e['sim_time'] <= end]
    rotor_contacts = [e for e in report.get('can_contacts', []) if '_rotor_link' in e['message'] and start <= e['sim_time'] <= end]
    # stable 还要求采样覆盖完整区间且没有支撑台或旋翼接触；阈值仅用于仿真回归。
    # Local simulation regression limits; these are not hardware safety limits.
    return dict(interval=[start, end], complete=complete, samples=len(selected), finite=finite,
                peak_tilt_deg=peak_tilt, peak_angular_speed_rad_s=peak_rate,
                peak_speed_m_s=max(math.hypot(*r['v']) for r in selected) if finite else None,
                min_height_m=min(r['p'][2] for r in selected) if finite else None,
                support_contacts=contacts, rotor_contacts=rotor_contacts,
                limits=dict(peak_tilt_deg=10, peak_angular_speed_rad_s=1),
                stable=bool(complete and finite and peak_tilt <= 10 and peak_rate <= 1 and not contacts and not rotor_contacts))


if __name__ == '__main__':
    # 命令行模式读取 mission 报告和运动 JSONL，打印易拉罐稳定性检查结果。
    report = json.loads(Path(sys.argv[1]).read_text())
    rows = [json.loads(line) for line in Path(sys.argv[2]).read_text().splitlines()]
    result = evaluate(report, rows)
    result.update(grasp_events=report['grasp_events'],
                  all_transfers_verified=report.get('all_transfers_verified', False))
    print(json.dumps(result, indent=2))
