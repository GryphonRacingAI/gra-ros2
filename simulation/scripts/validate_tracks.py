#!/usr/bin/env python3
"""Check track geometry without needing ROS or Gazebo.

Gazebo will happily load a track that is geometrically undriveable, so these
checks run on the cone CSVs directly and catch the failures a simulator
cannot: a corner tighter than the car can steer, a corridor that overlaps
itself, cones bunched or gapped, a loop that does not close.

Vehicle limits come from control/pure_pursuit.py defaults:
    wheelbase 1.534 m, max steering 0.366 rad -> min turn radius ~4.0 m

Tracks may be stored either fully closed, left open with a start/finish gate,
or (as mppi_track is) closed with the first few points repeated at the end.
All three are accepted; the repeated points are reported, because duplicated
cone models spawn on top of each other in Gazebo.

Usage:
    ./validate_tracks.py                  # every track under simulation/tracks
    ./validate_tracks.py mppi_track
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

# Vehicle limits (control/pure_pursuit.py)
WHEELBASE = 1.534
MAX_STEER = 0.366
MIN_TURN_RADIUS = WHEELBASE / math.tan(MAX_STEER)

# Track conventions (simulation/scripts/generate_tracks.py)
TRACK_WIDTH = 3.0
WIDTH_TOL = 0.05
SPACING_MIN = 1.0
SPACING_MAX = 7.0
GATE_MAX = 12.0          # widest credible start/finish gate
CLEARANCE_MIN = 4.0      # non-adjacent parts of the corridor
ADJACENCY_SKIP = 12      # points ignored either side when checking self-proximity
SEAM_TOL = 0.05          # distance under which two points count as duplicates
SEAM_MAX = 20            # most repeated seam points we will look for
GROUND_HALF_EXTENT = 50.0  # models/world/ground_plane is 100 x 100 m, origin-centred


def load(path: Path) -> list[tuple[float, float]]:
    with path.open() as fh:
        return [(float(r[0]), float(r[1])) for r in csv.reader(fh) if len(r) >= 2]


def centreline(inner, outer) -> list[tuple[float, float]]:
    return [((a[0] + b[0]) / 2, (a[1] + b[1]) / 2) for a, b in zip(inner, outer)]


def seam_overlap(points) -> int:
    """How many trailing points repeat the leading points."""
    n = len(points)
    for k in range(min(SEAM_MAX, n // 4), 0, -1):
        if all(math.dist(points[n - k + i], points[i]) < SEAM_TOL for i in range(k)):
            return k
    return 0


def corner_radii(points, wrap: bool) -> list[float]:
    n = len(points)
    out = []
    for i in (range(n) if wrap else range(1, n - 1)):
        p, q, r = points[(i - 1) % n], points[i], points[(i + 1) % n]
        a, b, c = math.dist(p, q), math.dist(q, r), math.dist(r, p)
        area2 = abs((q[0] - p[0]) * (r[1] - p[1]) - (r[0] - p[0]) * (q[1] - p[1]))
        out.append(float("inf") if area2 < 1e-12 else (a * b * c) / (2 * area2))
    return out


def segments_cross(p1, p2, p3, p4) -> bool:
    def orient(a, b, c):
        v = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])
        return 0 if abs(v) < 1e-12 else (1 if v > 0 else -1)
    d1, d2 = orient(p3, p4, p1), orient(p3, p4, p2)
    d3, d4 = orient(p1, p2, p3), orient(p1, p2, p4)
    return d1 != d2 and d3 != d4


def self_intersects(points, wrap: bool) -> tuple[bool, int | None]:
    n = len(points)
    last = n if wrap else n - 1
    for i in range(last):
        a1, a2 = points[i], points[(i + 1) % n]
        for j in range(i + 2, last):
            if wrap and i == 0 and j == n - 1:
                continue
            b1, b2 = points[j], points[(j + 1) % n]
            if segments_cross(a1, a2, b1, b2):
                return True, i
    return False, None


def min_self_clearance(points, wrap: bool) -> float:
    n = len(points)
    best = float("inf")
    for i in range(n):
        for j in range(i + ADJACENCY_SKIP, n):
            along = min(abs(i - j), n - abs(i - j)) if wrap else abs(i - j)
            if along < ADJACENCY_SKIP:
                continue
            best = min(best, math.dist(points[i], points[j]))
    return best


def check(name: str, track_dir: Path) -> bool:
    inner_p, outer_p = track_dir / "inner_cones.csv", track_dir / "outer_cones.csv"
    if not inner_p.exists() or not outer_p.exists():
        return _report(name, ["missing inner_cones.csv / outer_cones.csv"], [], [])

    inner, outer = load(inner_p), load(outer_p)
    problems: list[str] = []
    warnings: list[str] = []
    notes: list[str] = []

    if len(inner) != len(outer):
        problems.append(f"cone counts differ: inner {len(inner)} vs outer {len(outer)}")
        return _report(name, problems, warnings, notes)

    total_cones = len(inner) + len(outer)

    # 1. constant track width
    widths = [math.dist(a, b) for a, b in zip(inner, outer)]
    if abs(min(widths) - TRACK_WIDTH) > WIDTH_TOL or abs(max(widths) - TRACK_WIDTH) > WIDTH_TOL:
        problems.append(f"width {min(widths):.2f}-{max(widths):.2f} m, expected "
                        f"{TRACK_WIDTH} +/- {WIDTH_TOL}")
    notes.append(f"width {min(widths):.2f}-{max(widths):.2f} m")

    # 2. every cone must land on the ground_plane, or that part of the track
    #    floats over nothing. Cheap to get wrong: the plane is origin-centred,
    #    so a track built outward from (0, 0) drifts off one corner.
    reach = max(max(abs(x), abs(y)) for x, y in inner + outer)
    if reach > GROUND_HALF_EXTENT:
        problems.append(f"cones reach {reach:.2f} m from the origin, past the "
                        f"{GROUND_HALF_EXTENT} m edge of the ground_plane: "
                        f"that section hangs off the floor")
    notes.append(f"reach {reach:.2f} m (plane edge {GROUND_HALF_EXTENT} m)")

    centre = centreline(inner, outer)

    # 3. repeated seam points spawn duplicate cone models on top of each other
    dup = seam_overlap(centre)
    if dup:
        warnings.append(f"{dup} seam point(s) repeated: {2 * dup} cone models spawn "
                        f"on top of existing ones (points {len(centre) - dup}.."
                        f"{len(centre) - 1} duplicate 0..{dup - 1})")
        inner, outer = inner[:-dup], outer[:-dup]
        centre = centre[:-dup]

    # 3. closure: sampled up to the start, or deliberately gated
    closure = math.dist(centre[0], centre[-1])
    step = sum(math.dist(centre[i], centre[i + 1])
               for i in range(len(centre) - 1)) / (len(centre) - 1)
    if closure <= step * 2.0:
        wrap = True
        notes.append(f"closed loop (final step {closure:.2f} m)")
    elif closure <= GATE_MAX:
        wrap = False
        notes.append(f"open loop with a {closure:.2f} m start/finish gate")
    else:
        wrap = False
        problems.append(f"loop neither closes nor forms a plausible gate: "
                        f"{closure:.2f} m gap (mean step {step:.2f} m)")

    # 4. cone spacing along each edge, skipping the gate segment if present
    for label, pts in (("inner", inner), ("outer", outer)):
        n = len(pts)
        sp = [math.dist(pts[i], pts[(i + 1) % n]) for i in range(n if wrap else n - 1)]
        if min(sp) < SPACING_MIN:
            problems.append(f"{label} cones bunched: min spacing {min(sp):.2f} m")
        if max(sp) > SPACING_MAX:
            problems.append(f"{label} cone gap too wide: max spacing {max(sp):.2f} m")
        notes.append(f"{label} spacing {min(sp):.2f}-{max(sp):.2f} m")

    # 5. corner radius vs what the car can physically steer
    min_r = min(corner_radii(centre, wrap))
    if min_r < MIN_TURN_RADIUS:
        problems.append(f"corner radius {min_r:.2f} m is below the car's minimum "
                        f"turn radius {MIN_TURN_RADIUS:.2f} m")
    elif min_r < MIN_TURN_RADIUS * 1.25:
        notes.append(f"min radius {min_r:.2f} m is near the steering limit "
                     f"({MIN_TURN_RADIUS:.2f} m)")
    else:
        notes.append(f"min radius {min_r:.2f} m")

    # 6. inner edge must not invert into a cusp
    if min_r <= TRACK_WIDTH / 2:
        problems.append(f"min radius {min_r:.2f} m <= half width "
                        f"{TRACK_WIDTH / 2:.2f} m: inner edge inverts")

    # 7. the corridor must not cross itself
    crossed, where = self_intersects(centre, wrap)
    if crossed:
        problems.append(f"centreline crosses itself near point {where}")

    # 8. distant parts of the loop must stay clear of each other
    clear = min_self_clearance(centre, wrap)
    if clear < CLEARANCE_MIN:
        problems.append(f"corridor passes within {clear:.2f} m of itself "
                        f"(min {CLEARANCE_MIN} m): cone sections will interleave")
    notes.append(f"self-clearance {clear:.2f} m")

    last = len(centre) if wrap else len(centre) - 1
    length = sum(math.dist(centre[i], centre[(i + 1) % len(centre)]) for i in range(last))
    notes.append(f"lap {length:.1f} m, {total_cones} cones")

    return _report(name, problems, warnings, notes)


def _report(name, problems, warnings, notes) -> bool:
    status = "FAIL" if problems else ("warn" if warnings else "pass")
    print(f"\n{name}  [{status}]")
    for n in notes:
        print(f"    - {n}")
    for w in warnings:
        print(f"    ~  {w}")
    for p in problems:
        print(f"    !! {p}")
    return not problems


def main() -> None:
    tracks_root = Path(__file__).resolve().parent.parent / "tracks"
    wanted = sys.argv[1:]
    dirs = sorted(d for d in tracks_root.iterdir() if d.is_dir())
    if wanted:
        dirs = [d for d in dirs if d.name in wanted]
    if not dirs:
        print(f"no track directories found under {tracks_root}")
        sys.exit(1)

    print(f"vehicle min turn radius: {MIN_TURN_RADIUS:.2f} m "
          f"(wheelbase {WHEELBASE} m, max steer {MAX_STEER} rad)")
    results = [check(d.name, d) for d in dirs]
    print(f"\n{sum(results)}/{len(results)} tracks pass")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
