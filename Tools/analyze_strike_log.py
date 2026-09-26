#! /usr/bin/env python3
#
# Analyze striker strike accuracy from a PX4 .ulg log.
#
# Computes the closest point of approach (CPA) -- the minimum distance
# reached between the vehicle's actual flown trajectory and the commanded
# strike target -- segmented per designation_id (one segment per
# MAV_CMD_USER_1 STRIKE command found in the log). Handles multiple strikes
# in a single log.
#
# By default, writes one self-contained interactive 3D HTML plot per strike
# (rotate/zoom/pan in any browser, no X display needed -- open the file
# directly) with the CPA metrics baked into the plot itself, rather than
# printing them to the console. Pass --no-plot for a plain numeric summary
# instead (e.g. for scripting).
#
# Target position is read from the strike_target uORB topic when present
# (logged from feature/striker-v1.17 onward) and matched to the vehicle as a
# step function over time, so an EKF-reset re-projection mid-dive is picked
# up correctly instead of comparing against a stale pre-reset target. Logs
# from before strike_target was logged fall back to parsing the striker
# module's one-time "STRIKE target #N: x=... y=... z=..." console line, which
# gives a single fixed target position per designation (no reset correction).
#
# Install: pip install pyulog numpy plotly
# Usage:
#   python3 Tools/analyze_strike_log.py <path/to/log.ulg>
#   python3 Tools/analyze_strike_log.py <path/to/log.ulg> --out ./plots
#   python3 Tools/analyze_strike_log.py <path/to/log.ulg> --no-plot
#
from __future__ import print_function

import argparse
import os
import re
import sys

import numpy as np
from pyulog import ULog


def load_position(ulog):
    d = ulog.get_dataset('vehicle_local_position')
    return d.data['timestamp'], d.data['x'], d.data['y'], d.data['z']


def load_strike_targets_from_topic(ulog):
    """Preferred path: real strike_target uORB data, segmented by
    designation_id, target position tracked as a step function over time so
    EKF-reset re-projections are picked up. Returns None if the topic isn't
    in this log (older flight, before it was added to logged_topics.cpp)."""
    try:
        d = ulog.get_dataset('strike_target')
    except Exception:
        return None

    data = d.data
    ts = data['timestamp']
    des_id = data['designation_id']
    active = data['active']
    x, y, z = data['x'], data['y'], data['z']

    seen = []
    for did in des_id:
        if did != 0 and did not in seen:
            seen.append(did)

    segments = []
    for i, did in enumerate(seen):
        mask = (des_id == did) & (active == 1)
        if not np.any(mask):
            continue
        idxs = np.where(mask)[0]
        start_t = ts[idxs[0]]
        last_active_t = ts[idxs[-1]]

        # Window extends a little past the last "active" sample (the RECOVERY
        # pull-out can close further on the target after departure/abort),
        # but never into the next designation's own window.
        tail_us = 5_000_000  # 5s
        if i + 1 < len(seen):
            next_start_t = ts[np.where(des_id == seen[i + 1])[0][0]]
            end_t = min(last_active_t + tail_us, next_start_t)
        else:
            end_t = min(last_active_t + tail_us, ts[-1])

        window_mask = (ts >= start_t) & (ts <= end_t)
        segments.append({
            'designation_id': int(did),
            'start_t': start_t,
            'end_t': end_t,
            't': ts[window_mask],
            'x': x[window_mask],
            'y': y[window_mask],
            'z': z[window_mask],
        })

    return segments


_CONSOLE_TARGET_RE = re.compile(
    r'STRIKE target #(\d+): x=([-\d.]+) y=([-\d.]+) z=([-\d.]+)')


def load_strike_targets_from_console(ulog):
    """Fallback for logs predating strike_target being logged: parse the
    fixed target position out of the striker module's console PX4_INFO line.
    No EKF-reset correction (the console line is only printed once, at
    designation time), and the window boundary is just "until the next
    designation, or end of log" since there's no active-flag data to bound
    a RECOVERY tail against."""
    msgs = sorted(ulog.logged_messages, key=lambda m: m.timestamp)
    hits = []

    for i, m in enumerate(msgs):
        match = _CONSOLE_TARGET_RE.search(m.message)

        if match:
            hits.append((i, m.timestamp, match.groups()))

    if not hits:
        return None

    segments = []

    for n, (i, start_t, (des_num, x, y, z)) in enumerate(hits):
        end_t = hits[n + 1][1] if n + 1 < len(hits) else msgs[-1].timestamp
        segments.append({
            'designation_id': int(des_num),
            'start_t': start_t,
            'end_t': end_t,
            't': None,  # fixed target position, no step function
            'x': float(x),
            'y': float(y),
            'z': float(z),
        })

    return segments


def compute_cpa(pos_t, pos_x, pos_y, pos_z, seg):
    mask = (pos_t >= seg['start_t']) & (pos_t <= seg['end_t'])

    if not np.any(mask):
        return None

    t = pos_t[mask]
    x = pos_x[mask]
    y = pos_y[mask]
    z = pos_z[mask]

    if seg.get('t') is not None and len(seg['t']) > 0:
        # Step function: for each vehicle sample, use the most recent target
        # sample at or before it -- picks up EKF-reset re-projections as soon
        # as strike_manager republishes the corrected coordinates.
        idx = np.searchsorted(seg['t'], t, side='right') - 1
        idx = np.clip(idx, 0, len(seg['t']) - 1)
        tx = seg['x'][idx]
        ty = seg['y'][idx]
        tz = seg['z'][idx]
    else:
        tx = np.full_like(x, seg['x'])
        ty = np.full_like(y, seg['y'])
        tz = np.full_like(z, seg['z'])

    d3d = np.sqrt((x - tx) ** 2 + (y - ty) ** 2 + (z - tz) ** 2)
    d2d = np.sqrt((x - tx) ** 2 + (y - ty) ** 2)
    i = int(np.argmin(d3d))

    return {
        'designation_id': seg['designation_id'],
        'cpa_3d': float(d3d[i]),
        'cpa_horizontal': float(d2d[i]),
        'cpa_vertical': float(abs(z[i] - tz[i])),
        'cpa_time_s': (t[i] - seg['start_t']) / 1e6,
        'vehicle_pos': (float(x[i]), float(y[i]), float(z[i])),
        'target_pos': (float(tx[i]), float(ty[i]), float(tz[i])),
        # Full NED track for this engagement window, kept for plotting.
        'track': (x, y, z),
        'track_t': t,
    }


def plot_segment_3d(r, source, out_dir):
    """Writes one self-contained, interactive (rotate/zoom/pan) 3D HTML plot
    for a single engagement, with the CPA metrics baked into the plot itself
    (title + a fixed on-screen text box) rather than printed to console."""
    import plotly.graph_objects as go

    x, y, z = r['track']  # NED: x=North, y=East, z=Down
    # Plot as East/North/Altitude (up-positive) so it reads like a map with height.
    alt = -z
    tx, ty, tz = r['target_pos']
    talt = -tz
    vx, vy, vz = r['vehicle_pos']
    valt = -vz

    fig = go.Figure()

    # Color the track by elapsed time so direction of travel is visible even
    # without animating -- darker/lighter along the colorscale = later in time.
    t_rel = (r['track_t'] - r['track_t'][0]) / 1e6

    fig.add_trace(go.Scatter3d(
        x=y, y=x, z=alt,
        mode='lines+markers',
        line=dict(color='royalblue', width=4),
        marker=dict(size=2, color=t_rel, colorscale='Blues', showscale=False),
        name='Vehicle track',
        hovertext=['t={:.1f}s'.format(tt) for tt in t_rel],
        hoverinfo='text',
    ))

    fig.add_trace(go.Scatter3d(
        x=[ty], y=[tx], z=[talt],
        mode='markers',
        marker=dict(size=10, color='red', symbol='diamond'),
        name='Target',
        hovertext=['Target<br>N={:.1f} E={:.1f} Alt={:.1f}m'.format(tx, ty, talt)],
        hoverinfo='text',
    ))

    fig.add_trace(go.Scatter3d(
        x=[vy], y=[vx], z=[valt],
        mode='markers',
        marker=dict(size=8, color='green'),
        name='CPA ({:.2f} m)'.format(r['cpa_3d']),
        hovertext=['CPA<br>3D={:.2f}m  Horizontal={:.2f}m  Vertical={:.2f}m<br>t={:.1f}s after designation'.format(
            r['cpa_3d'], r['cpa_horizontal'], r['cpa_vertical'], r['cpa_time_s'])],
        hoverinfo='text',
    ))

    metrics_text = (
        '<b>Strike #{}</b><br>'
        'Closest approach (3D): <b>{:.2f} m</b><br>'
        'Horizontal miss: {:.2f} m<br>'
        'Vertical offset: {:.2f} m<br>'
        'Time to CPA: {:.1f} s after designation<br>'
        '<span style="font-size:11px;color:gray">Source: {}</span>'
    ).format(r['designation_id'], r['cpa_3d'], r['cpa_horizontal'],
             r['cpa_vertical'], r['cpa_time_s'], source)

    fig.update_layout(
        title='Strike #{} - CPA {:.2f} m'.format(r['designation_id'], r['cpa_3d']),
        scene=dict(
            xaxis_title='East [m]',
            yaxis_title='North [m]',
            zaxis_title='Altitude [m]',
            aspectmode='data',
        ),
        legend=dict(x=0.75, y=0.98),
    )

    # Fixed text box overlay (paper coordinates -- stays put when the 3D
    # scene is rotated/zoomed) carrying the actual numbers.
    fig.add_annotation(
        text=metrics_text,
        xref='paper', yref='paper',
        x=0.02, y=0.98,
        showarrow=False,
        align='left',
        bgcolor='rgba(255,255,255,0.85)',
        bordercolor='black',
        borderwidth=1,
        font=dict(size=13),
    )

    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'strike_{}_3d.html'.format(r['designation_id']))
    fig.write_html(out_path)
    return out_path


def main():
    parser = argparse.ArgumentParser(
        description='Compute strike closest-approach (miss) distance from a PX4 .ulg log, '
                    'and (by default) write an interactive 3D HTML plot with the metrics on it.')
    parser.add_argument('logfile', help='Path to the .ulg file')
    parser.add_argument('--no-plot', action='store_true',
                         help='Skip the interactive 3D HTML plot and print a plain numeric '
                             'summary to the console instead (e.g. for scripting)')
    parser.add_argument('--out', default='.',
                         help='Output directory for the HTML plots (default: current dir)')
    args = parser.parse_args()

    ulog = ULog(args.logfile, disable_str_exceptions=True)
    pos_t, pos_x, pos_y, pos_z = load_position(ulog)

    segments = load_strike_targets_from_topic(ulog)
    source = 'strike_target uORB topic (EKF-reset-corrected)'

    if segments is None:
        segments = load_strike_targets_from_console(ulog)
        source = 'console log fallback (strike_target topic not in this log)'

    if not segments:
        print('No strike designations found in this log.')
        sys.exit(1)

    print('Target data source: {}'.format(source))
    print('Found {} strike designation(s) in {}'.format(len(segments), args.logfile))

    results = []

    for seg in segments:
        r = compute_cpa(pos_t, pos_x, pos_y, pos_z, seg)

        if r is None:
            print('  designation_id={}: no matching vehicle_local_position samples'.format(
                seg['designation_id']))
            continue

        results.append(r)

    if not results:
        sys.exit(1)

    if args.no_plot:
        for r in results:
            print('\ndesignation_id={}:'.format(r['designation_id']))
            print('  Closest approach : {:.2f} m (3D)'.format(r['cpa_3d']))
            print('  Horizontal miss  : {:.2f} m'.format(r['cpa_horizontal']))
            print('  Vertical offset  : {:.2f} m'.format(r['cpa_vertical']))
            print('  Time to CPA      : {:.1f} s after designation'.format(r['cpa_time_s']))
    else:
        for r in results:
            out_path = plot_segment_3d(r, source, args.out)
            print('designation_id={}: CPA={:.2f}m -> {}'.format(
                r['designation_id'], r['cpa_3d'], out_path))


if __name__ == '__main__':
    main()
