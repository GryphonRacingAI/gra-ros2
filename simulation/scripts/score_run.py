#!/usr/bin/env python3
"""Score one simulator run against ground truth (recorded with gt_record.py).

usage: score_run.py EVENT run.csv [--json] [--tracks DIR]
       EVENT: acceleration | skidpad | autocross | trackdrive | mppi_track

What is measured
  contacts   cones the car's footprint came closer than CONTACT m (0.12) to. The footprint is the body and the
             four wheels as rectangles in base_link; the cone is its true position in the track .sdf (every
             colour, also the *_gap_after_* and *_fill_* cones). A cone knocked earlier is still judged where it
             stood. 0.12 m is about the half-width of a cone's base, so a contact is the car reaching the cone.
  closest    smallest clearance to any cone (m)
  laps, lap_times, finished, drive_s, vmax, vmean
  off_centre |distance to nearest blue - distance to nearest yellow| / 2 while moving: median and max (m). An
             approximate distance from the middle of the track; only meaningful where both boundaries exist.
  slam_error |SLAM pose - true pose| per lap: median and max (m), from run_slam.csv if it is there. The SLAM
             map frame is the car's start pose, so the SLAM pose is moved into the world frame with the true start pose.
"""
import argparse
import bisect
import json
import os
import re

import numpy as np

TRACK = {'acceleration': 'acceleration', 'skidpad': 'skidpad', 'autocross': 'drive1',
         'trackdrive': 'track_small', 'mppi_track': 'mppi_track'}
CONTACT = 0.12
# body, front-left, front-right, rear-left, rear-right wheel: (x0, x1, y0, y1) in base_link (ads_dv.xacro)
FOOTPRINT = [(-1.175, 1.175, -0.245, 0.245), (0.5095, 1.0245, 0.374, 0.57), (0.5095, 1.0245, -0.57, -0.374),
             (-1.0245, -0.5095, 0.47, 0.668), (-1.0245, -0.5095, -0.668, -0.47)]
DEFAULT_TRACKS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models', 'tracks')


def load_cones(event, tracks_dir):
    """{name: (x, y)} of every cone of the event's track (pool bodies dyn_* excluded)."""
    path = os.path.join(tracks_dir, TRACK[event] + '.sdf')
    if not os.path.exists(path):   # installed layout: share/simulation/models/tracks
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('simulation'), 'models', 'tracks', TRACK[event] + '.sdf')
    cones = {}
    for m in re.finditer(r'<model name=[\'"]((?:large_orange|blue|yellow|orange)_\w+)[\'"]>\s*'
                         r'(?:<static>\w+</static>\s*)?<pose>([-\d.e ]+)</pose>', open(path).read()):
        cones[m.group(1)] = np.array([float(v) for v in m.group(2).split()[:2]])
    return cones


def clearance(cone, x, y, yaw):
    """Distance from the cone centre to the car's footprint for every pose (N,)."""
    dx, dy = cone[0] - x, cone[1] - y
    c, s = np.cos(yaw), np.sin(yaw)
    lx, ly = c * dx + s * dy, -s * dx + c * dy            # cone in base_link
    best = np.full(len(lx), np.inf)
    for x0, x1, y0, y1 in FOOTPRINT:
        best = np.minimum(best, np.hypot(np.maximum(np.maximum(x0 - lx, lx - x1), 0.0),
                                         np.maximum(np.maximum(y0 - ly, ly - y1), 0.0)))
    return best


def slam_error(run_csv, t, x, y, yaw, laps):
    path = run_csv[:-4] + '_slam.csv'
    if not os.path.exists(path):
        return None
    s = np.genfromtxt(path, delimiter=',', names=True)
    if s.size < 2:
        return None
    per_lap = {}
    c0, s0 = np.cos(yaw[0]), np.sin(yaw[0])                  # SLAM's map frame is the car's start pose
    for ts, sx, sy in zip(s['t'], s['x'], s['y']):
        k = min(bisect.bisect(t, ts), len(t) - 1)
        tx, ty = x[0] + c0 * sx - s0 * sy, y[0] + s0 * sx + c0 * sy
        per_lap.setdefault(int(laps[k]) + 1, []).append(float(np.hypot(tx - x[k], ty - y[k])))
    return {lap: [round(float(np.median(e)), 2), round(max(e), 2)] for lap, e in sorted(per_lap.items())}


def score(event, run_csv, tracks_dir=DEFAULT_TRACKS):
    d = np.genfromtxt(run_csv, delimiter=',', names=True, dtype=None, encoding=None)
    d = d[np.abs(d['v']) < 30]                               # spawn transient
    x, y, yaw, v, t = d['x'], d['y'], d['yaw'], d['v'], d['t']
    cones = load_cones(event, tracks_dir)
    passes = []                                               # (clearance, name, time since start)
    for name, c in cones.items():
        near = np.hypot(x - c[0], y - c[1]) < 3.0
        if near.any():
            cl = clearance(c, x[near], y[near], yaw[near])
            i = int(cl.argmin())
            passes.append((float(cl[i]), name, float(t[near][i] - t[0])))
    passes.sort()
    moving = v > 0.3
    laps = int(d['laps'].max())
    lap_end = [float(t[np.argmax(d['laps'] >= k)]) for k in range(1, laps + 1)]
    t_go = float(t[np.argmax(moving)]) if moving.any() else float(t[0])

    off = None
    blue = np.array([c for n, c in cones.items() if n.startswith('blue')])
    yellow = np.array([c for n, c in cones.items() if n.startswith('yellow')])
    if len(blue) and len(yellow) and moving.any():
        p = np.column_stack((x[moving], y[moving]))[::5]
        d_b = np.linalg.norm(p[:, None, :] - blue[None, :, :], axis=2).min(axis=1)
        d_y = np.linalg.norm(p[:, None, :] - yellow[None, :, :], axis=2).min(axis=1)
        both = (d_b < 6.0) & (d_y < 6.0)
        if both.any():
            e = np.abs(d_b - d_y)[both] / 2.0
            off = [round(float(np.median(e)), 2), round(float(e.max()), 2)]

    return dict(
        event=event, cones=len(cones),
        finished=bool((d['state'] == 'FINISHED').any()), laps=laps,
        drive_s=round(float(t[np.where(moving)[0][-1]] - t_go), 1) if moving.any() else 0.0,
        lap_times=[round(b - a, 1) for a, b in zip([t_go] + lap_end[:-1], lap_end)],
        vmax=round(float(v.max()), 1), vmean=round(float(v[moving].mean()), 1) if moving.any() else 0.0,
        contacts=sum(p[0] < CONTACT for p in passes),
        closest=round(passes[0][0], 2) if passes else None,
        hit=[(n, round(c, 2), round(ts)) for c, n, ts in passes if c < CONTACT],
        near=[(n, round(c, 2)) for c, n, ts in passes[:3]],
        off_centre=off, slam_error=slam_error(run_csv, t, x, y, yaw, d['laps']),
        start_gap=round(float(min(np.hypot(x[0] - c[0], y[0] - c[1]) for c in cones.values())), 2))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('event', choices=sorted(TRACK))
    ap.add_argument('run_csv')
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--tracks', default=DEFAULT_TRACKS, help='directory with the track .sdf files')
    a = ap.parse_args()
    r = score(a.event, a.run_csv, a.tracks)
    if a.json:
        print(json.dumps(r))
    else:
        for k, val in r.items():
            print(f'{k:11s} {val}')


if __name__ == '__main__':
    main()
