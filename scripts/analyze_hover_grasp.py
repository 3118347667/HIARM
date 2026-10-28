#!/usr/bin/env python3
"""Offline EKF position tracking evaluation; see analyze_hover_grasp.ipynb.

Only reads bags: does not connect to ROS, arm, or command a vehicle.
"""
from dataclasses import asdict, dataclass
from pathlib import Path
import csv
import hashlib
import json
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader

ODOM = '/ekf/ekf_odom'
CONTROL = '/flygripper/geometric_controller/data'
RC = '/mavros/rc/in'
STATE = '/mavros/state'
LAND = '/mavros/extended_state'
SNAPSHOT = '/flygripper/rosparam_snapshot'
CHAT_OUTPUT_ROOT = Path(__file__).resolve().parents[2] / 'chat'
PARAMETER_TOPICS = {
    '/flygripper/flygripper_flight_node/parameter_updates',
    '/flygripper/geometric_controller/parameter_updates',
    '/flygripper/robot_parameter/parameter_updates',
    '/flygripper/servo_node/parameter_updates',
}
PARAMETER_NAMESPACES = ('flygripper_flight_node', 'geometric_controller',
                        'robot_parameter', 'servo_node')


@dataclass
class Settings:
    threshold_m: float = 0.02
    window_s: float = 10.0
    max_gap_s: float = 0.10    # Analysis quality guard, NOT an experimental tolerance.
    max_header_age_s: float = 0.10
    min_grasp_z_m: float = 0.5  # EKF world-frame Z, not height above the takeoff point.
    rc_max_gap_s: float = 1.0  # Matches servo_control.hpp RC freshness timeout.
    max_reference_motion_m: float = 0.001


def xyz(v):
    return [v.x, v.y, v.z]


def stamp_ns(header):
    return header.stamp.sec * 10**9 + header.stamp.nanosec


def deduplicate_odom(odom):
    """Drop consecutive identical timestamp/position samples in acquisition order."""
    if len(odom) < 2:
        return odom
    duplicate = (odom[1:, 1] == odom[:-1, 1]) & np.all(odom[1:, 2:5] == odom[:-1, 2:5], axis=1)
    return odom[np.r_[True, ~duplicate]]


def read_bag(path):
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    arrays = {k: [] for k in ('odom', 'control', 'rc', 'land')}
    states, transitions, frames, logs = [], [], set(), []
    snapshots, updates, parameter_warnings = [], [], []
    requested = {ODOM, CONTROL, RC, STATE, LAND, '/rosout', SNAPSHOT} | PARAMETER_TOPICS
    with AnyReader([path]) as reader:
        inventory = [{'topic': c.topic, 'type': c.msgtype, 'count': c.msgcount}
                     for c in reader.connections]
        present = {c.topic for c in reader.connections}
        if not {ODOM, CONTROL} <= present:
            raise ValueError(f'Bag must contain {ODOM} and {CONTROL}')
        start_ns, end_ns = reader.start_time, reader.end_time
        # An empty rosbags connection list means ALL messages; never use it for a missing topic.
        connections = [c for c in reader.connections if c.topic in requested]
        for c, t, raw in reader.messages(connections=connections):
            m = reader.deserialize(raw, c.msgtype)
            sec = (t - start_ns) * 1e-9
            if c.topic == ODOM:
                arrays['odom'].append([sec, (stamp_ns(m.header)-start_ns)*1e-9,
                                       *xyz(m.pose.pose.position)])
                frames.add(m.header.frame_id)
            elif c.topic == CONTROL:
                arrays['control'].append([sec, *xyz(m.position_reference)])
            elif c.topic == RC and len(m.channels) >= 10:
                arrays['rc'].append([sec, int(m.channels[8]), int(m.channels[9])])
            elif c.topic == STATE:
                states.append((sec, bool(m.armed), m.mode))
            elif c.topic == LAND:
                arrays['land'].append([sec, m.landed_state])
            elif c.topic == SNAPSHOT:
                try:
                    snapshot = json.loads(m.data)
                    params = snapshot['parameters']
                    if not isinstance(params, dict):
                        raise ValueError('parameters is not an object')
                    snapshots.append({'offset_s': sec, 'captured_at_utc': snapshot.get('captured_at_utc'),
                                      'parameters': {k: params[k] for k in PARAMETER_NAMESPACES if k in params}})
                except (ValueError, KeyError, TypeError) as exc:
                    parameter_warnings.append(f'参数快照解析失败 at {sec:.3f}s: {exc}')
            elif c.topic in PARAMETER_TOPICS:
                values = {p.name: p.value for group in (m.bools, m.ints, m.doubles, m.strs) for p in group}
                updates.append({'offset_s': sec, 'topic': c.topic, 'parameters': values})
            elif c.topic == '/rosout':
                msg = re.sub(r'\x1b\[[0-9;]*m', '', m.msg)
                match = re.search(r'\bRef:\s*(\w+)\s*=>\s*(\w+)', msg)
                if match and m.name.endswith('/flygripper_flight_node'):
                    transitions.append((sec, *match.groups()))
                if match or any(word in msg.lower() for word in ('handover', 'takeoff', 'stale', 'teleop mode')):
                    logs.append({'offset_s': sec, 'node': m.name, 'message': msg})
    widths = dict(odom=5, control=4, rc=3, land=2)
    data = {k: np.asarray(v, dtype=float).reshape(-1, widths[k]) for k, v in arrays.items()}
    # Identical adjacent EKF messages provide no new sample; keep conflicting
    # duplicates and backward timestamps so the coverage audit rejects them.
    data['odom'] = deduplicate_odom(data['odom'])
    data.update(path=path, duration_s=(end_ns-start_ns)*1e-9, start_ns=start_ns,
                inventory=inventory, states=states, transitions=transitions,
                frames=sorted(frames), logs=logs,
                parameter_snapshots=snapshots, parameter_updates=updates,
                parameter_warnings=parameter_warnings)
    return data


def detect_candidates(data, cfg=None):
    """CH9 leaving open (>=1900 us), remaining <1900 for >=0.10 s.

    Uses first departure, including the 1500 us transition, not full-close time.
    CH10 override is checked separately.
    """
    cfg = cfg or Settings()
    rc, odom = data['rc'], data['odom']
    result = []
    for i in range(1, len(rc)):
        if rc[i-1, 1] < 1900 or rc[i, 1] >= 1900:
            continue
        t = float(rc[i, 0])
        j = int(np.searchsorted(rc[:, 0], t+0.1))
        if j >= len(rc) or np.any(rc[i:j+1, 1] >= 1900):
            continue
        # Both topics use bag record time to identify the latest observation
        # available when the CH9 signal arrives; reject stale EKF headers too.
        k = int(np.searchsorted(odom[:, 0], t, side='right')) - 1
        if k < 0:
            continue
        record_t, header_t, z = odom[k, 0], odom[k, 1], odom[k, 4]
        if (not np.isfinite([record_t, header_t, z]).all()
                or t - record_t > cfg.max_gap_s
                or abs(record_t - header_t) > cfg.max_header_age_s
                or z <= cfg.min_grasp_z_m):
            continue
        result.append({'start_s': t, 'source': 'CH9 open-to-close departure (bag record time)',
                       'previous_rc_s': float(rc[i-1, 0]), 'ekf_z_m': float(z),
                       'ekf_record_s': float(record_t), 'ekf_header_s': float(header_t),
                       'ch10_override': bool(np.any(rc[i:j+1, 2] > 1625))})
    return result


def coverage(times, start, end, max_gap):
    times = np.asarray(times)
    if not len(times) or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        return False, None
    left = np.searchsorted(times, start, side='right')-1
    right = np.searchsorted(times, end, side='left')
    if left < 0 or right >= len(times):
        return False, None
    gap = float(np.max(np.diff(times[left:right+1])))
    return gap <= max_gap, gap


def held_indices(times, query):
    # ZOH, never np.interp across reference jumps; caller checks coverage first.
    return np.searchsorted(times, query, side='right')-1


def evaluate_trial(data, event, cfg):
    start, end = event['start_s'], event['start_s'] + cfg.window_s
    issues = []
    if not np.isfinite(start) or start < 0 or end > data['duration_s']:
        issues.append('抓取后完整 10 秒未被 bag 覆盖')
    odom, c = data['odom'], data['control']
    # Limit monotonicity audit to this window plus boundary support, preserving acquisition order.
    near = odom[(odom[:, 0] >= start-1) & (odom[:, 0] <= end+1)]
    ot = near[:, 1]  # EKF header time; controller references use bag record time.
    ok, odom_gap = coverage(ot, start, end, cfg.max_gap_s)
    if not ok:
        issues.append('EKF 时间覆盖/时间单调性/采样间隙不满足要求')
    cnear = c[(c[:, 0] >= start-1) & (c[:, 0] <= end+1)]
    cok, control_gap = coverage(cnear[:, 0], start, end, cfg.max_gap_s)
    if not cok:
        issues.append('控制参考时间覆盖/采样间隙不满足要求')
    selected = near[(ot >= start) & (ot < end)]
    ages = selected[:, 0]-selected[:, 1]
    if len(ages) and np.max(np.abs(ages)) > cfg.max_header_age_s:
        issues.append('EKF header 与 record 时间差超出配置上限')
    if len(selected) < 2 or not ok or not cok or not np.isfinite(selected).all() or not np.isfinite(cnear).all():
        return {'start_s': start, 'end_s': end, 'issues': issues or ['非有限值或样本不足'],
                'data_valid': False, 'numeric_pass': None, 'event': event}, None
    t = selected[:, 1]
    ref0 = cnear[held_indices(cnear[:, 0], [start])[0], 1:4]
    reference = cnear[held_indices(cnear[:, 0], t), 1:4]
    window_ref = cnear[(cnear[:, 0] >= start) & (cnear[:, 0] < end), 1:4]
    ref_motion = float(np.max(np.linalg.norm(window_ref-ref0, axis=1)))
    if ref_motion > cfg.max_reference_motion_m:
        issues.append('10 秒内目标位置改变，不是固定目标悬停')
    position = selected[:, 2:5]
    error = position - reference
    e3 = np.linalg.norm(error, axis=1)
    # Exact 10 s support for supplemental time average (linear error interpolation at boundaries).
    tt = np.r_[start, t, end]
    pp = np.column_stack([np.interp(tt, ot, near[:, 2+j]) for j in range(3)])
    rr = cnear[held_indices(cnear[:, 0], tt), 1:4]
    time_mean = float(np.trapezoid(np.linalg.norm(pp-rr, axis=1), tt) / cfg.window_s)
    # Flight state: require state messages covering the interval with bounded staleness.
    st = data['states']
    if not st:
        issues.append('缺少飞行状态')
    else:
        stt = np.array([v[0] for v in st])
        sok, _ = coverage(stt, start, end, 2.0)
        indices = held_indices(stt, np.r_[start, stt[(stt > start) & (stt < end)]])
        if not sok or np.any(indices < 0) or any(not st[i][1] or st[i][2] != 'GUIDED_NOGPS' for i in indices):
            issues.append('窗口内未持续解锁并保持 GUIDED_NOGPS 或状态覆盖不足')
    land = data['land']
    # Landed-state messages often arrive at 1 Hz with scheduling jitter.
    # Require boundary support and ordered records, but no maximum interval.
    lok, _ = coverage(land[:, 0], start, end, float('inf'))
    if not lok:
        issues.append('空中状态记录覆盖不足')
    else:
        idx = held_indices(land[:, 0], np.r_[start, land[(land[:, 0] > start) & (land[:, 0] < end), 0]])
        if np.any(land[idx, 1] != 2):
            issues.append('窗口内 landed_state 未持续为 IN_AIR')
    transitions = data['transitions']
    previous = [v for v in transitions if v[0] <= start]
    if not previous or previous[-1][2] != 'HOVER' or any(start < v[0] < end and v[2] != 'HOVER' for v in transitions):
        issues.append('未证实完整窗口持续为 HOVER')
    rc = data['rc']
    rok, rc_gap = coverage(rc[:, 0], start, end, cfg.rc_max_gap_s)
    rw = rc[(rc[:, 0] >= start) & (rc[:, 0] < end)]
    if event['source'].startswith('CH9') and (not rok or np.any(rw[:, 1] >= 1900) or np.any(rw[:, 2] > 1625)):
        issues.append('RC 覆盖不足、窗口内重新张开或 CH10 覆盖')
    mean = float(e3.mean())
    result = dict(start_s=start, end_s=end, samples=len(t), event=event,
                  data_valid=not issues, issues=issues,
                  mean_3d_m=mean, time_mean_3d_m=time_mean,
                  rmse_3d_m=float(np.sqrt(np.mean(e3**2))), max_3d_m=float(e3.max()),
                  mean_xy_m=float(np.linalg.norm(error[:, :2], axis=1).mean()),
                  mae_xyz_m=np.abs(error).mean(axis=0).tolist(),
                  rmse_xyz_m=np.sqrt(np.mean(error**2, axis=0)).tolist(),
                  reference_start_m=ref0.tolist(), reference_motion_m=ref_motion,
                  odom_max_gap_s=odom_gap, control_max_gap_s=control_gap,
                  rc_max_gap_s=rc_gap,
                  header_age_range_s=[float(ages.min()), float(ages.max())],
                  threshold_m=cfg.threshold_m, numeric_pass=bool(mean <= cfg.threshold_m))
    values = np.column_stack([t, t-start, position, reference, error, e3])
    return result, values


def write_csv(path, rows, fields):
    with Path(path).open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def plot_overview(data, events, cfg, output):
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True, layout='constrained')
    for key, axis, cols, labels in [('rc', axes[0], [1, 2], ['CH9', 'CH10']),
                                  ('odom', axes[1], [4], ['EKF Z'])]:
        a = data[key]
        for col, label in zip(cols, labels):
            axis.plot(a[:, 0], a[:, col], label=label, lw=1)
        axis.grid(alpha=.25)
        axis.legend(loc='upper left')
        for i, event in enumerate(events, 1):
            t = event['start_s']
            axis.axvspan(t, t+cfg.window_s, color='tab:orange', alpha=.17)
            axis.axvline(t, color='tab:red', ls='--', lw=.8)
            if axis is axes[0]:
                axis.text(t, 1700, f' #{i} {t:.3f}s')
    axes[0].set_ylabel('PWM (us)')
    axes[1].set(ylabel='Height (m)', xlabel='Seconds since bag start (record time)')
    fig.suptitle(data['path'].name + ' | candidate grasp windows')
    fig.savefig(output, dpi=150)
    plt.close(fig)


def plot_trial(result, values, cfg, output):
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, layout='constrained')
    t = values[:, 1]
    for j, label in enumerate('XYZ'):
        axes[j].plot(t, values[:, 2+j], label='EKF position', lw=1.1)
        axes[j].plot(t, values[:, 5+j], '--', label='Controller reference', lw=1.1)
        axes[j].set_ylabel(label+' (m)')
        axes[j].legend(loc='best')
    axes[3].plot(t, values[:, -1]*100, color='tab:purple', label='3D error')
    axes[3].axhline(cfg.threshold_m*100, color='tab:red', ls='--', label='Mean-error criterion (not peak limit)')
    axes[3].axhline(result['mean_3d_m']*100, color='tab:green', ls=':', label=f"Mean {result['mean_3d_m']*100:.3f} cm")
    axes[3].set(xlabel='Seconds since grasp candidate', ylabel='Error (cm)', xlim=(0, cfg.window_s), ylim=(0, None))
    axes[3].legend(loc='best')
    for ax in axes:
        ax.grid(alpha=.25)
    fig.suptitle(f"Grasp +10 s | start {result['start_s']:.3f} s | EKF tracking error")
    fig.savefig(output, dpi=150)
    plt.close(fig)


def default_output_path(bag):
    """Return the shared chat output directory for a bag's hover-grasp report."""
    return CHAT_OUTPUT_ROOT / f'{Path(bag).expanduser().stem}_hover_grasp'


def analyze(bags, output=None, cfg=None, manual_starts=None, on_plot=None):
    """manual_starts maps absolute bag path to [offset_s, ...]; [] excludes that bag's events."""
    cfg = cfg or Settings()
    if (cfg.threshold_m <= 0 or not np.isfinite(cfg.threshold_m)
            or cfg.window_s != 10.0):
        raise ValueError('Use a positive threshold and a 10 s window')
    paths = [Path(p).expanduser().resolve() for p in bags]
    if not paths or len(set(paths)) != len(paths):
        raise ValueError('Supply distinct bag paths; duplicate files are not repeats')
    if output is None:
        output = default_output_path(paths[0])
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    # Only retire files belonging to this report's bag prefixes. Keep unrelated
    # files in a shared output directory, and defer removal until analysis succeeds.
    prefixes = {f'{i:02d}_{path.stem}' for i, path in enumerate(paths, 1)}
    previous_report = output / 'report.json'
    if previous_report.is_file():
        try:
            previous = json.loads(previous_report.read_text())
            previous_prefixes = {
                f"{i:02d}_{Path(bag['bag']).stem}"
                for i, bag in enumerate(previous['bags'], 1)
            }
        except (ValueError, KeyError, TypeError):
            previous_prefixes = set()
        prefixes.update(previous_prefixes)
    old_artifacts = set()
    for path in output.iterdir():
        match = re.fullmatch(r'(.+)_(?:overview\.png|trial_[1-9]\d*(?:\.png|_samples\.csv))', path.name)
        if path.is_file() and match and match[1] in prefixes:
            old_artifacts.add(path)
    generated_artifacts = set()
    results, manifests, hashes = [], [], set()
    for bag_index, path in enumerate(paths, 1):
        with path.open('rb') as f:
            digest = hashlib.file_digest(f, 'sha256').hexdigest()
        if digest in hashes:
            raise ValueError('Identical bag content cannot count as independent repeats')
        hashes.add(digest)
        data = read_bag(path)
        events = detect_candidates(data, cfg)
        auto_events = list(events)
        if manual_starts and str(path) in manual_starts:
            events = [{'start_s': float(t), 'source': 'manual experiment annotation'}
                      for t in manual_starts[str(path)]]
        events = sorted(events, key=lambda e: e['start_s'])
        prefix = f'{bag_index:02d}_{path.stem}'
        manifests.append(dict(bag=str(path), sha256=digest, size_bytes=path.stat().st_size,
                              duration_s=data['duration_s'], bag_start_ns=data['start_ns'],
                              odom_frame_ids=data['frames'], topics=data['inventory'],
                              auto_candidates=auto_events, selected_events=events, logs=data['logs'],
                              parameter_snapshots=data['parameter_snapshots'],
                              parameter_updates=data['parameter_updates'],
                              parameter_warnings=data['parameter_warnings']))
        overview = output / f'{prefix}_overview.png'
        plot_overview(data, events, cfg, overview)
        generated_artifacts.add(overview)
        if on_plot:
            on_plot(overview)
        for i, event in enumerate(events, 1):
            result, values = evaluate_trial(data, event, cfg)
            result.update(bag=str(path), run=len(results)+1)
            results.append(result)
            if values is not None:
                image = output / f'{prefix}_trial_{i}.png'
                plot_trial(result, values, cfg, image)
                generated_artifacts.add(image)
                if on_plot:
                    on_plot(image)
                headers = ['ekf_header_offset_s', 'grasp_elapsed_s', *[f'ekf_{a}_m' for a in 'xyz'],
                           *[f'reference_{a}_m' for a in 'xyz'],
                           *[f'error_{a}_m' for a in 'xyz'], 'error_3d_m']
                samples_file = output / f'{prefix}_trial_{i}_samples.csv'
                np.savetxt(samples_file, values, delimiter=',',
                           header=','.join(headers), comments='')
                generated_artifacts.add(samples_file)
    valid = [r for r in results if r['data_valid']]
    summary = dict(trials=len(results), valid_trials=len(valid),
                   numeric_comparison=f'每次三维距离误差算术均值 <= {cfg.threshold_m:g} m')
    report = dict(settings=asdict(cfg), summary=summary, trials=results, bags=manifests,
                  method='mean(norm(ekf_position - controller_reference)); [start,start+10s)',
                  position_topic=ODOM, reference_topic=CONTROL,
                  source_notes=['src/flygripper_flight/launch/flight.launch: odom input /ekf/ekf_odom',
                                'src/flygripper_actuator/include/flygripper_actuator/servo/servo_control.hpp: CH9 / CH10'])
    (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    fields = ['run', 'bag', 'start_s', 'end_s', 'samples', 'mean_3d_m', 'time_mean_3d_m',
              'rmse_3d_m', 'max_3d_m', 'threshold_m', 'numeric_pass', 'data_valid', 'issues']
    write_csv(output / 'trials.csv', [{k: r.get(k) for k in fields} for r in results], fields)
    for path in old_artifacts - generated_artifacts:
        path.unlink()
    return report
