#!/usr/bin/env python3
"""Plot recorded moving-grasp gate evidence from a ROS 1 bag (offline)."""
from pathlib import Path
import json
import warnings

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
import numpy as np
from rosbags.highlevel import AnyReader

ALIGN = '/drone0/planning/grasp_alignment'
SNAPSHOT = '/drone0/planning/rosparam_snapshot'
PHASE = '/drone0/planning/grasp_phase'
STATUS = '/flygripper/flight_trajectory_status'
ODOM = '/ekf/ekf_odom'
POSE = '/vrpn_client_node/HIARM_car/pose'
TWIST = '/vrpn_client_node/HIARM_car/twist'
TRIGGER = '/triger'
LOG = '/rosout'
COMMAND = '/flygripper/actuator_command'
LIMITS = ('grasp_start_dis', 'grasp_max_relative_speed', 'grasp_max_yaw_error')
ODOM_YAW = '__odom_yaw'
TWIST_XY = '__twist_xy'


def read_gate_evidence(path):
    """Read bag record time; do not infer in-process callback or monotonic time."""
    rows = {k: [] for k in (ALIGN, SNAPSHOT, PHASE, STATUS, ODOM, POSE, TWIST, TRIGGER, LOG, COMMAND)}
    rows[ODOM_YAW] = []
    rows[TWIST_XY] = []
    with AnyReader([Path(path)]) as reader:
        bag_start = reader.start_time
        bag_end = (reader.end_time - bag_start) * 1e-9
        connections = [c for c in reader.connections if c.topic in rows]
        available = {c.topic for c in connections}
        for connection, stamp, raw in reader.messages(connections=connections):
            topic = connection.topic
            t = (stamp - bag_start) * 1e-9
            if topic in (POSE, TRIGGER):
                rows[topic].append(t)
                continue
            m = reader.deserialize(raw, connection.msgtype)
            if topic == ODOM:
                rows[topic].append(t)
                q = m.pose.pose.orientation
                quaternion = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
                norm = np.linalg.norm(quaternion)
                if np.isfinite(quaternion).all() and np.isfinite(norm) and norm > 1e-9:
                    x, y, z, w = quaternion / norm
                    yaw = np.arctan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
                    rows[ODOM_YAW].append((t, yaw))
            elif topic == TWIST:
                rows[topic].append(t)
                v, h = m.twist.linear, m.header.stamp
                stamp_ns = int(h.sec) * 1_000_000_000 + int(h.nanosec)
                if stamp_ns > 0 and np.isfinite([v.x, v.y, v.z]).all():
                    rows[TWIST_XY].append((t, (stamp_ns - bag_start)*1e-9,
                                           v.x, v.y, m.header.frame_id))
            elif topic == ALIGN:
                v = m.vector
                rows[topic].append((t, v.x, v.y, v.z))
            elif topic == SNAPSHOT:
                try:
                    data = json.loads(m.data)
                    params = data.get('parameters', {})
                    if not isinstance(params, dict):
                        raise ValueError('parameters is not an object')
                    rows[topic].append((t, params))
                except (ValueError, TypeError, AttributeError) as exc:
                    warnings.warn(f'规划参数快照无法解析，t={t:.3f}s: {exc}')
            elif topic == PHASE:
                rows[topic].append((t, m.data))
            elif topic == COMMAND:
                if np.isfinite(m.reference_position):
                    rows[topic].append((t, int(m.mode), float(m.reference_position)))
            elif topic == STATUS:
                # These fields are necessary, but cannot alone prove flightActive().
                required = ('trajectory_active', 'fcu_state_fresh', 'fcu_armed',
                            'fcu_guided_nogps', 'fsm_state', 'reference_type',
                            'FSM_TRAJ', 'REF_TRAJ')
                ok = ((m.trajectory_active and m.fcu_state_fresh and m.fcu_armed and
                       m.fcu_guided_nogps and m.fsm_state == m.FSM_TRAJ and
                       m.reference_type == m.REF_TRAJ)
                      if all(hasattr(m, field) for field in required) else None)
                rows[topic].append((t, None if ok is None else bool(ok)))
            else:
                text = m.msg.replace('\n', ' ')
                if ('[planner]' in text.lower() and any(x in text.lower() for x in
                    ('abort', 'brak', 'invalid', 'stale', 'lost', 'failed', 'cannot',
                     'closing', 'lift', 'trajectory', 'retrigger'))):
                    rows[topic].append((t, text))
    return rows, available, bag_end


def limits_at(times, snapshots, overrides=None):
    """Unknown before first recorded snapshot. Overrides are explicitly manual."""
    overrides = overrides or {}
    extra = set(overrides) - set(LIMITS)
    if extra:
        raise ValueError(f'未知手动门限: {sorted(extra)}')
    result = np.full((len(times), len(LIMITS)), np.nan)
    sources = []
    for j, key in enumerate(LIMITS):
        if key in overrides:
            value = float(overrides[key])
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'{key} 手动门限必须为有限正数')
            result[:, j] = value
            sources.append('manual')
            continue
        events = []
        for t, params in snapshots:
            try:
                value = float(params[key])
                if not np.isfinite(value) or value <= 0:
                    value = np.nan
            except (KeyError, TypeError, ValueError):
                value = np.nan
            events.append((t, value))
        if events:
            st, sv = np.asarray(events).T
            idx = np.searchsorted(st, times, side='right') - 1
            result[idx >= 0, j] = sv[idx[idx >= 0]]
            sources.append('snapshot')
        else:
            sources.append('unknown')
    return result, sources


def last_values(times, samples, max_age):
    """Return last observed record and age, with unknown before first message."""
    if not samples:
        return np.full(len(times), -1, dtype=int)
    stamps = np.asarray([x[0] for x in samples], dtype=float)
    idx = np.searchsorted(stamps, times, side='right') - 1
    out = np.full(len(times), -1, dtype=int)
    valid = idx >= 0
    j = np.flatnonzero(valid)
    fresh = times[j] - stamps[idx[j]] <= max_age
    out[j] = 0
    for k in j[fresh]:
        value = samples[idx[k]][1]
        out[k] = -1 if value is None else (1 if value else 0)
    return out


def longest_sampled_run(times, passed, max_gap):
    """Consecutive passing alignment samples, not the planner's gate timer."""
    best = 0.0
    begin = None
    previous = None
    for t, ok in zip(times, passed):
        if not ok:
            begin = previous = None
        elif begin is None or previous is None or t - previous > max_gap:
            begin = previous = t
        else:
            previous = t
            best = max(best, t - begin)
    return best


def close_command_starts(rows, gap_s):
    """Match MODE_POSITION to the latest available close target snapshot."""
    if not np.isfinite(gap_s) or gap_s <= 0:
        raise ValueError('CLOSE_BURST_GAP_S 必须是有限正数')
    snapshots = rows[SNAPSHOT]
    stamps = np.asarray([x[0] for x in snapshots], dtype=float)
    matches = []
    for t, mode, position in rows[COMMAND]:
        idx = np.searchsorted(stamps, t, side='right') - 1
        if idx < 0 or mode != 0:
            continue
        params = snapshots[idx][1]
        try:
            target = float(params['grasp_position'])
            if not np.isfinite(target):
                continue
            # Identical open/close values cannot distinguish intent.
            if 'grasp_open_position' in params and np.isclose(target, float(params['grasp_open_position']), rtol=1e-9, atol=1e-9):
                continue
            if np.isclose(position, target, rtol=1e-9, atol=1e-9):
                matches.append(t)
        except (KeyError, TypeError, ValueError):
            continue
    matches = np.asarray(matches)
    return matches[np.r_[True, np.diff(matches) > gap_s]] if len(matches) else np.array([])


def select_gate_window(triggers, close_starts, bag_end, trigger_index=0, before_s=2.0, after_s=5.0):
    """Select one trigger's mission without attributing a later retrigger's close to it."""
    if isinstance(trigger_index, bool) or not isinstance(trigger_index, (int, np.integer)) or trigger_index < 0:
        raise ValueError('GATE_TRIGGER_INDEX 必须为从 0 开始的整数')
    if not np.isfinite([bag_end, before_s, after_s]).all() or bag_end <= 0 or before_s < 0 or after_s < 0:
        raise ValueError('录包长度必须为正，前后窗口须为有限非负数')
    if not triggers:
        return 0.0, bag_end, None, None, '未记录 /triger，无法按触发时刻截取；使用可用录包范围'
    if trigger_index >= len(triggers):
        raise ValueError(f'GATE_TRIGGER_INDEX={trigger_index} 超出范围，bag 中共有 {len(triggers)} 次触发')
    trigger = float(triggers[trigger_index])
    next_trigger = float(triggers[trigger_index+1]) if trigger_index+1 < len(triggers) else None
    candidates = [float(t) for t in close_starts
                  if t >= trigger and (next_trigger is None or t < next_trigger)]
    start = max(0.0, trigger - before_s)
    if candidates:
        close = candidates[0]
        end = min(bag_end, close + after_s)
        message = f'第 {trigger_index+1}/{len(triggers)} 次触发 {trigger:.3f} s；匹配闭爪指令 {close:.3f} s'
    else:
        close = None
        end = min(bag_end, next_trigger) if next_trigger is not None else bag_end
        message = ('第 %d/%d 次触发 %.3f s；本次未识别到闭爪指令，显示至%s，'
                   '缺少指令或目标快照时不能据此断定没有闭爪' %
                   (trigger_index+1, len(triggers), trigger, '下次触发' if next_trigger is not None else '录包结束'))
    return start, end, trigger, close, message


def yaw_filter_parameters(snapshots):
    """Use recorded initial parameters; missing values are explicit assumptions."""
    params = snapshots[0][1] if snapshots else {}
    specs = (('target_velocity_cutoff_hz', 3.0, True),
             ('grasp_yaw_velocity_filter_hz', 1.0, False),
             ('target_twist_timeout', 0.20, False),
             ('grasp_yaw_speed_low', 0.05, False))
    values, assumed = [], []
    for key, fallback, allow_zero in specs:
        try:
            value = float(params[key])
            if not np.isfinite(value) or value < 0 or (not allow_zero and value == 0):
                raise ValueError(key)
        except (KeyError, TypeError, ValueError):
            value = fallback
            assumed.append(key)
        values.append(value)
    try:
        offset = float(params['grasp_heading_offset'])
        if not np.isfinite(offset):
            raise ValueError('invalid offset')
    except (KeyError, TypeError, ValueError):
        offset = 0.0
        assumed.append('grasp_heading_offset')
    return (*values, offset, assumed)


def offline_car_yaw(rows):
    """Approximate two vector filters; stage 2 uses recorded planner sample times.

    Bag record times do not reconstruct callback order or direction readiness.
    Only show direction while both relevant speed vectors exceed the noise floor.
    """
    first_hz, second_hz, timeout, min_speed, offset, assumed = yaw_filter_parameters(rows[SNAPSHOT])
    events = rows[TWIST_XY]
    stage1, accepted = [], []
    previous_header = previous_record = previous_frame = None
    filtered = None
    for t, header, vx, vy, frame in events:
        if previous_header is not None and header <= previous_header:
            continue
        vector = np.asarray([vx, vy], dtype=float)
        if (filtered is None or frame != previous_frame or first_hz == 0 or
                header - previous_header > timeout or t - previous_record > timeout):
            filtered = vector.copy()
        else:
            alpha = -np.expm1(-2*np.pi*first_hz*(header-previous_header))
            filtered = filtered + alpha*(vector-filtered)
        previous_header, previous_record, previous_frame = header, t, frame
        angle = np.arctan2(filtered[1], filtered[0]) if np.linalg.norm(filtered) >= min_speed else np.nan
        stage1.append((t, angle))
        accepted.append((t, header, filtered.copy()))
    # Current planner updates Direction only at planning checks with a new twist.
    # ALIGN is published at such checks; record order is an offline approximation.
    ticks = [row[0] for row in rows[ALIGN]]
    method = 'alignment samples' if ticks else 'twist samples (alignment unavailable)'
    if not ticks:
        ticks = [row[0] for row in accepted]
    times = np.asarray([row[0] for row in accepted])
    stage2, last_index, last_record, second = [], -1, None, None
    triggers = rows[TRIGGER]
    trigger_idx = 0
    for tick in ticks:
        while trigger_idx < len(triggers) and triggers[trigger_idx] <= tick:
            second, last_record, last_index = None, None, -1
            trigger_idx += 1
        idx = np.searchsorted(times, tick, side='right') - 1
        if idx < 0 or idx == last_index or tick - accepted[idx][0] > timeout:
            continue
        receipt, _, vector = accepted[idx]
        if second is None or receipt <= last_record or receipt-last_record > 0.20:
            second = vector.copy()
        else:
            alpha = -np.expm1(-2*np.pi*second_hz*(receipt-last_record))
            second = second + alpha*(vector-second)
        last_index, last_record = idx, receipt
        # The planner can hold its prior yaw at low speed or on direction disagreement;
        # the simplified offline result is therefore hidden instead of inventing a yaw.
        angle = (np.arctan2(second[1], second[0]) + offset
                 if np.linalg.norm(vector) >= min_speed and np.linalg.norm(second) >= min_speed
                 else np.nan)
        stage2.append((tick, angle))
    return stage1, stage2, (first_hz, second_hz, offset, assumed, method)


def plot_yaw_segments(ax, samples, start, end, max_gap, label, color, anchor=None):
    """Unwrap each valid segment and place each on the closest aircraft yaw branch."""
    data = np.asarray(samples, dtype=float).reshape(-1, 2)
    data = data[(data[:, 0] >= start) & (data[:, 0] <= end)]
    if not len(data):
        return np.empty((0, 2))
    output = np.full(len(data), np.nan)
    indices = np.flatnonzero(np.isfinite(data[:, 1]))
    if not len(indices):
        return np.column_stack((data[:, 0], output))
    split = np.flatnonzero((np.diff(indices) != 1) | (np.diff(data[indices, 0]) > max_gap)) + 1
    for group in np.split(indices, split):
        values = np.unwrap(data[group, 1])
        if anchor is not None and len(anchor):
            good = np.flatnonzero(np.isfinite(anchor[:, 1]))
            if len(good):
                near = good[np.argmin(np.abs(anchor[good, 0]-data[group[0], 0]))]
                # Avoid aligning to an unrelated aircraft sample across a data gap.
                if abs(anchor[near, 0]-data[group[0], 0]) <= max_gap:
                    values += 2*np.pi*np.round((anchor[near, 1]-values[0])/(2*np.pi))
        output[group] = values
    # NaNs break jumps across invalid samples. Explicit separators break large gaps.
    breaks = np.flatnonzero(np.diff(data[:, 0]) > max_gap) + 1
    ax.plot(np.insert(data[:, 0], breaks, np.nan),
            np.degrees(np.insert(output, breaks, np.nan)), color=color, lw=1.1, label=label)
    return np.column_stack((data[:, 0], output))


def plot_gate_conditions(path, output, start_s=0., end_s=None, overrides=None,
                         close_burst_gap_s=1.0, plot_max_gap_s=0.25, trigger_index=0):
    """Visualize sampled evidence; return summary, never assert complete gate readiness."""
    rows, available, bag_end = read_gate_evidence(path)
    close_starts = close_command_starts(rows, close_burst_gap_s)
    window_start, window_end, selected_trigger, selected_close, window_message = select_gate_window(
        rows[TRIGGER], close_starts, bag_end, trigger_index)
    # START_S and END_S are optional additional restrictions shared with the notebook.
    start = max(window_start, float(start_s), 0.0)
    end = min(window_end, float(end_s)) if end_s is not None else window_end
    if not np.isfinite([start, end, plot_max_gap_s]).all() or end <= start or plot_max_gap_s <= 0:
        raise ValueError('自动触发窗口与 START_S/END_S 没有交集，或 plot_max_gap_s 无效')
    raw = np.asarray(rows[ALIGN], dtype=float).reshape(-1, 4)
    align = raw[(raw[:, 0] >= start) & (raw[:, 0] <= end)]
    values = align[:, 1:4].copy()
    if len(values):
        # Distance, speed, and absolute yaw error cannot be negative.
        values[(~np.isfinite(values)) | (values < 0)] = np.nan
    limits, sources = limits_at(align[:, 0], rows[SNAPSHOT], overrides)
    known = np.isfinite(values) & np.isfinite(limits)
    passes = known & (values <= limits)
    all_known = known.all(axis=1)
    all_pass = all_known & passes.all(axis=1)
    phase_samples = [(t, p == 'approach') for t, p in rows[PHASE]]
    approach = last_values(align[:, 0], phase_samples, 1.0) == 1
    relevant = approach if rows[PHASE] else np.ones(len(align), dtype=bool)
    eligible = relevant & all_known
    longest = longest_sampled_run(align[:, 0], all_pass & relevant, plot_max_gap_s)
    fig, axes = plt.subplots(6, 1, figsize=(14, 13), sharex=True,
                             gridspec_kw={'height_ratios': [1, 1, 1, 1.15, .55, 1.5]},
                             constrained_layout=True)
    units = [('Distance (m)', 1.), ('Relative speed (m/s)', 1.), ('Yaw error (deg)', 180/np.pi)]
    for j, (label, factor) in enumerate(units):
        ax = axes[j]
        if len(align):
            y = values[:, j] * factor
            breaks = np.flatnonzero(np.diff(align[:, 0]) > plot_max_gap_s) + 1
            ax.plot(np.insert(align[:, 0], breaks, np.nan), np.insert(y, breaks, np.nan),
                    color='tab:blue', lw=1.2, label='Recorded planner alignment')
            ax.step(align[:, 0], limits[:, j] * factor, where='post', color='tab:red',
                    ls='--', lw=1.2, label=f'Limit ({sources[j]})')
        else:
            ax.text(.5, .5, 'No recorded grasp_alignment samples', transform=ax.transAxes, ha='center')
        ax.set_ylabel(label)
        ax.grid(alpha=.25)
        if len(align): ax.legend(loc='upper right', fontsize=8)
    # The two filtered directions are offline approximations, not logged yaw commands.
    car_stage1, car_stage2, yaw_config = offline_car_yaw(rows)
    first_hz, second_hz, offset, assumed, method = yaw_config
    anchor = plot_yaw_segments(axes[3], rows[ODOM_YAW], start, end, plot_max_gap_s,
                               'Drone EKF yaw', 'tab:blue')
    plot_yaw_segments(axes[3], car_stage1, start, end, plot_max_gap_s,
                      f'Car velocity direction {first_hz:g} Hz (offline)', 'tab:orange', anchor)
    plot_yaw_segments(axes[3], car_stage2, start, end, plot_max_gap_s,
                      f'Car direction {first_hz:g} → {second_hz:g} Hz (offline)', 'tab:green', anchor)
    axes[3].set_ylabel('Yaw / direction (deg)')
    axes[3].set_title('Car curves: offline velocity-vector filters, not recorded planner yaw', fontsize=9, loc='left')
    axes[3].grid(alpha=.25)
    if axes[3].lines:
        axes[3].legend(loc='upper right', fontsize=8)
    else:
        axes[3].text(.5, .5, 'No valid aircraft yaw or car velocity', transform=axes[3].transAxes, ha='center')
    # Sampled gate evidence is not a reconstruction of direction readiness or dwell.
    axes[4].set_facecolor('#eeeeee')
    axes[4].set_yticks([0], ['Three numeric tests'])
    if len(align):
        colors = np.where(~all_known, '#999999', np.where(all_pass, '#2ca02c', '#d62728'))
        axes[4].scatter(align[:, 0], np.zeros(len(align)), c=colors, marker='|', s=100)
    axes[4].set_ylim(-.5, .5)
    axes[4].set_title('Only sampled numeric tests: green pass / red fail / gray unknown', fontsize=9, loc='left')
    grid = np.linspace(start, end, min(12000, max(2, int(np.ceil((end-start)/0.05))+1)))
    status_age = 0.20  # Display-only record-time horizon, not in-process freshness.
    evidence = [last_values(grid, rows[STATUS], status_age),
                last_values(grid, phase_samples, 1.0)]
    for topic in (ODOM, POSE, TWIST):
        evidence.append(last_values(grid, [(t, True) for t in rows[topic]], status_age))
    cmap = ListedColormap(['#bdbdbd', '#f0a0a0', '#9ad39a'])
    axes[5].imshow(np.asarray(evidence), aspect='auto', interpolation='nearest', origin='upper',
                   extent=(start, end, len(evidence)-.5, -.5), cmap=cmap,
                   norm=BoundaryNorm([-1.5,-.5,.5,1.5], 3))
    axes[5].set_yticks(range(5), ['Flight fields', 'Phase approach (1 s)', 'EKF bag recent',
                                 'Car pose bag recent', 'Car twist bag recent'])
    axes[5].set_title('Recorded evidence (phase 1 s; others 0.20 s): green observed / red failed or not recent / gray unknown', fontsize=9, loc='left')
    axes[5].set_xlabel('Bag record time (s)')
    for ax in axes:
        for t in close_starts:
            if start <= t <= end:
                ax.axvline(t, color='purple', ls='--', lw=1.0, alpha=.9)
        for t in rows[TRIGGER]:
            if start <= t <= end:
                ax.axvline(t, color='black', ls=':', lw=.8, alpha=.7)
        ax.set_xlim(start, end)
    fig.suptitle('Moving grasp gate evidence | purple: matching close command; black: trigger', fontsize=12)
    out = Path(output)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    plt.close(fig)
    summary = {'window_start_s': start, 'window_end_s': end,
               'selected_trigger_s': selected_trigger, 'selected_close_s': selected_close,
               'alignment_samples': len(align), 'approach_samples': int(relevant.sum()),
               'all_known_approach': int(eligible.sum()),
               'numeric_pass_approach': int((all_pass & relevant).sum()),
               'longest_numeric_sampled_s': longest,
               'limit_sources': dict(zip(LIMITS, sources)),
               'missing_topics': sorted({ALIGN, SNAPSHOT, PHASE, STATUS, ODOM, POSE, TWIST, TRIGGER, COMMAND} - available)}
    print(f'闭爪条件证据图: {out}')
    print(f'小车航向离线滤波: {first_hz:g} Hz → {second_hz:g} Hz，偏置 {offset:g} rad；'
          f'第二级时间采样: {method}')
    if assumed:
        print('以下滤波参数未在快照中找到或无效，使用假定值: ' + ', '.join(assumed))
    if len(rows[SNAPSHOT]) > 1:
        print('存在多次参数快照，航向离线重算使用第一份快照；不证明运行时状态。')
    print(f'{window_message}；绘图范围 {start:.3f}–{end:.3f} s')
    print('匹配快照闭爪目标的指令分组时刻: ' + (', '.join(f'{t:.3f} s' for t in close_starts) if len(close_starts) else '未识别；缺少指令或目标快照时不能排除实际曾闭爪'))
    print(f'规划器 alignment 样本: {len(align)}; '+ ('approach 阶段' if rows[PHASE] else '未记录阶段，使用全部 alignment') +
          f'样本: {relevant.sum()}')
    for j, key in enumerate(LIMITS):
        selected = relevant & known[:, j]
        failures = int((selected & ~passes[:, j]).sum())
        print(f'  {key}: 未通过 {failures}/{int(selected.sum())} 个已知样本; 门限来源 {sources[j]}')
    dwell_values = []
    for _, params in rows[SNAPSHOT]:
        try:
            value = float(params['grasp_gate_duration'])
            if np.isfinite(value) and value > 0 and value not in dwell_values:
                dwell_values.append(value)
        except (KeyError, TypeError, ValueError):
            pass
    dwell_text = ', '.join(f'{x:g}' for x in dwell_values) if dwell_values else 'unknown'
    print(f'三项数值同时满足: {summary["numeric_pass_approach"]}/{summary["all_known_approach"]} 个已知样本; '+
          f'最长连续采样区间 {longest:.3f} s（不是规划器内部计时）；快照 gate_duration: {dwell_text} s')
    if summary['missing_topics']:
        print('未记录话题: ' + ', '.join(summary['missing_topics']))
    if not len(align):
        print('未记录规划器 alignment，无法判断这三项数值；请检查是否记录话题及是否进入相关规划分支。')
    if not len(align) or not known.all():
        print('部分门限不可用：无法做对应阈值判断；可显式提供人工门限并标注来源。')
    if rows[LOG]:
        print('相关规划日志（所选时间内最后 10 条）:')
        for t, msg in [(t, m) for t, m in rows[LOG] if start <= t <= end][-10:]:
            print(f'  {t:.3f}s {msg}')
    print('注意：参数快照是录包时参数服务器记录，不保证运行中被规划器重新读取。')
    print('限制：数值通过、bag 记录新鲜和飞行状态字段均不能证明完整闭爪门限通过；'
          '航向就绪、任务轨迹匹配、回调/时间戳新鲜度、内部连续计时和可行轨迹仍需证据。')
    return summary
