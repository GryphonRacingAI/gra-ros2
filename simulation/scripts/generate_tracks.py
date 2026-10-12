#!/usr/bin/env python3
"""Generate Formula Student simulation tracks as cone CSVs, xacro and SDF.

Conventions are taken from the existing ``mppi_track`` data so generated
tracks are interchangeable with the hand-made ones:

- track width is a constant 3.0 m (cone pairs are index-matched and
  perpendicular to the centreline, exactly as ``mppi_track`` is built)
- the centreline is sampled at a uniform arc-length step, so spacing along
  the inner edge tightens in corners and opens out on the outside
- inner cones are blue, outer cones are yellow, and two large-orange pairs
  straddle the start/finish line
- the car spawns on the centreline at the start line, facing along the
  tangent; the printed spawn poses go into ``dynamic_event.launch.py``

Pure standard library on purpose: this runs without ROS, numpy or scipy, so
tracks can be regenerated and checked before a workspace is built.

Usage:
    ./generate_tracks.py                 # write every track
    ./generate_tracks.py --track hairpin_chain
    ./generate_tracks.py --list
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

# Geometry conventions, derived from simulation/tracks/mppi_track/*.csv
HALF_WIDTH = 1.5        # -> 3.0 m track width
CONE_SPACING = 2.5      # m, uniform along the centreline

# models/world/ground_plane is a 100 x 100 m plane centred on the origin, so
# every cone must sit inside +/- 50 m or the track hangs off the floor.
# mppi_track reaches 49.4 m, so it was clearly built to this limit too.
# Tracks are recentred on the origin and scaled down if they still overflow.
GROUND_HALF_EXTENT = 50.0
FIT_MARGIN = 3.0        # keep cones this far inside the plane edge
CONE_Z = {"blue": 0.15, "yellow": 0.15, "orange": 0.15, "large_orange": 0.033}

# Cone mass/inertia per type, matching models/cones/cone.xacro
CONE_INERTIAL = {
    "large_orange": ("1.05", ("0.003", "0.003", "0.002")),
    "default": ("0.45", ("0.013", "0.013", "0.006")),
}


# --------------------------------------------------------------------------
# Track definitions
#
# Each entry is a closed loop of centreline waypoints in metres. A periodic
# Catmull-Rom spline is fitted through them, so the waypoints are passed
# through exactly and the curve between them stays smooth.
# --------------------------------------------------------------------------
TRACKS: dict[str, dict] = {
    # ---- competition-realistic -------------------------------------------
    "fsuk_autocross": {
        "style": "competition-realistic",
        "description": "FS-UK style autocross: start straight, fast right-hander, "
                       "an esses section, two hairpins and a back straight.",
        "waypoints": [
            (0.0, 0.0), (26.0, 0.0), (42.0, 4.0), (52.0, 16.0), (50.0, 30.0),
            (40.0, 38.0), (28.0, 36.0), (22.0, 26.0), (12.0, 24.0),
            (4.0, 32.0), (-8.0, 38.0), (-22.0, 34.0), (-28.0, 22.0),
            (-24.0, 10.0), (-12.0, 2.0),
        ],
    },
    "fsg_trackdrive": {
        "style": "competition-realistic",
        "description": "FSG style trackdrive loop: long main straight into wide "
                       "sweepers with a single tight hairpin at the far end.",
        "waypoints": [
            (0.0, 0.0), (34.0, -2.0), (56.0, 2.0), (70.0, 14.0), (72.0, 30.0),
            (62.0, 42.0), (46.0, 46.0), (30.0, 42.0), (20.0, 32.0),
            (6.0, 30.0), (-10.0, 34.0), (-24.0, 28.0), (-30.0, 14.0),
            (-22.0, 2.0),
        ],
    },
    "endurance_loop": {
        "style": "competition-realistic",
        "description": "Compact trackdrive circuit of continuous medium-radius "
                       "corners, intended for repeated 10-lap endurance runs.",
        "waypoints": [
            (0.0, 0.0), (20.0, 2.0), (34.0, 12.0), (36.0, 28.0), (26.0, 38.0),
            (10.0, 40.0), (-4.0, 34.0), (-10.0, 22.0), (-22.0, 18.0),
            (-30.0, 8.0), (-24.0, -4.0), (-10.0, -6.0),
        ],
    },
    # ---- deliberate stress tests -----------------------------------------
    "hairpin_chain": {
        "style": "stress-test",
        "description": "Five consecutive tight hairpins joined by short straights. "
                       "Stresses cone association and the steering-angle limit.",
        "waypoints": [
            (0.0, 0.0), (18.0, 0.0), (28.0, 6.0), (24.0, 15.0), (10.0, 16.0),
            (0.0, 22.0), (4.0, 32.0), (18.0, 34.0), (28.0, 40.0),
            (24.0, 50.0), (8.0, 52.0), (-6.0, 48.0), (-14.0, 38.0),
            (-12.0, 26.0), (-18.0, 16.0), (-14.0, 4.0),
        ],
    },
    "fast_sweepers": {
        "style": "stress-test",
        "description": "Long straights into high-speed sweepers and nothing tight. "
                       "Stresses target speed, lookahead tuning and path smoothness.",
        "waypoints": [
            (0.0, 0.0), (45.0, -4.0), (80.0, 4.0), (100.0, 26.0), (96.0, 52.0),
            (70.0, 68.0), (36.0, 70.0), (6.0, 60.0), (-14.0, 42.0),
            (-18.0, 20.0),
        ],
    },
}


# --------------------------------------------------------------------------
# Curve maths
# --------------------------------------------------------------------------
def catmull_rom_closed(points: list[tuple[float, float]],
                       per_segment: int = 60) -> list[tuple[float, float]]:
    """Densely sample a closed Catmull-Rom spline through ``points``."""
    n = len(points)
    out: list[tuple[float, float]] = []
    for i in range(n):
        p0 = points[(i - 1) % n]
        p1 = points[i]
        p2 = points[(i + 1) % n]
        p3 = points[(i + 2) % n]
        for s in range(per_segment):
            t = s / per_segment
            t2, t3 = t * t, t * t * t
            # Uniform Catmull-Rom basis (tension 0.5)
            x = 0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t
                       + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
                       + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3)
            y = 0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t
                       + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
                       + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
            out.append((x, y))
    return out


def resample_closed(poly: list[tuple[float, float]],
                    spacing: float) -> list[tuple[float, float]]:
    """Resample a closed polyline at a uniform step that divides it exactly."""
    seg = [math.dist(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))]
    total = sum(seg)
    count = max(8, round(total / spacing))
    step = total / count

    out: list[tuple[float, float]] = []
    target = 0.0
    walked = 0.0
    idx = 0
    for _ in range(count):
        while walked + seg[idx] < target - 1e-9:
            walked += seg[idx]
            idx = (idx + 1) % len(poly)
        a = poly[idx]
        b = poly[(idx + 1) % len(poly)]
        frac = 0.0 if seg[idx] == 0 else (target - walked) / seg[idx]
        out.append((a[0] + (b[0] - a[0]) * frac, a[1] + (b[1] - a[1]) * frac))
        target += step
    return out


def tangents(points: list[tuple[float, float]]) -> list[float]:
    """Heading at each point of a closed loop, by central difference."""
    n = len(points)
    angles = []
    for i in range(n):
        ax, ay = points[(i - 1) % n]
        bx, by = points[(i + 1) % n]
        angles.append(math.atan2(by - ay, bx - ax))
    return angles


def corner_radii(points: list[tuple[float, float]]) -> list[float]:
    """Circumscribed-circle radius at each point of a closed loop."""
    n = len(points)
    radii = []
    for i in range(n):
        p, q, r = points[(i - 1) % n], points[i], points[(i + 1) % n]
        a, b, c = math.dist(p, q), math.dist(q, r), math.dist(r, p)
        area2 = abs((q[0] - p[0]) * (r[1] - p[1]) - (r[0] - p[0]) * (q[1] - p[1]))
        radii.append(float("inf") if area2 < 1e-12 else (a * b * c) / (2 * area2))
    return radii


def fit_to_ground(centre: list[tuple[float, float]]
                  ) -> tuple[list[tuple[float, float]], float]:
    """Recentre on the origin and shrink until every cone sits on the plane.

    The centreline is transformed, never the cone positions, so re-offsetting
    by HALF_WIDTH afterwards keeps the track exactly 3 m wide at any scale.
    """
    xs = [p[0] for p in centre]
    ys = [p[1] for p in centre]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    centred = [(x - cx, y - cy) for x, y in centre]

    # cones sit HALF_WIDTH outside the centreline, so budget for that
    limit = GROUND_HALF_EXTENT - FIT_MARGIN - HALF_WIDTH
    reach = max(max(abs(x), abs(y)) for x, y in centred)
    scale = 1.0 if reach <= limit else limit / reach
    if scale < 1.0:
        centred = [(x * scale, y * scale) for x, y in centred]
    return centred, scale


def build_sides(centre: list[tuple[float, float]]
                ) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """Offset the centreline by +/- HALF_WIDTH along its normal."""
    inner, outer = [], []
    for (x, y), th in zip(centre, tangents(centre)):
        nx, ny = -math.sin(th), math.cos(th)
        inner.append((x - nx * HALF_WIDTH, y - ny * HALF_WIDTH))
        outer.append((x + nx * HALF_WIDTH, y + ny * HALF_WIDTH))
    return inner, outer


# --------------------------------------------------------------------------
# Emitters
# --------------------------------------------------------------------------
def cone_entries(inner, outer):
    """Yield (name, type, x, y) for every cone, start gate included."""
    last = len(inner) - 1
    gate = {0, last}
    bi = yi = oi = 0
    for i, (p, q) in enumerate(zip(inner, outer)):
        if i in gate:
            oi += 1
            yield (f"large_orange_{oi}", "large_orange", p[0], p[1])
            oi += 1
            yield (f"large_orange_{oi}", "large_orange", q[0], q[1])
        else:
            bi += 1
            yield (f"blue_{bi}", "blue", p[0], p[1])
            yi += 1
            yield (f"yellow_{yi}", "yellow", q[0], q[1])


def write_csvs(out_dir: Path, inner, outer) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, pts in (("inner_cones.csv", inner), ("outer_cones.csv", outer)):
        with (out_dir / name).open("w", newline="") as fh:
            # csv defaults to CRLF; the existing track CSVs are LF and
            # .gitattributes normalises text, so emit LF to keep regeneration
            # a no-op on a fresh checkout.
            writer = csv.writer(fh, lineterminator="\n")
            for x, y in pts:
                writer.writerow([f"{x:.10f}", f"{y:.10f}"])


def write_pairs(path: Path, inner, outer) -> None:
    """Cone-pair file in the format perfect_path reads for its midline.

    Mirrors config/perfect_path_mppi_track_pairs.txt, written by
    generate_mppi_track_world.py: one line per pair, inner then outer.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as fh:
        for (ix, iy), (ox, oy) in zip(inner, outer):
            fh.write(f"{ix} {iy} {ox} {oy}\n")


def _cone_xacro_lines(inner, outer) -> list[str]:
    lines = []
    for name, ctype, x, y in cone_entries(inner, outer):
        z = CONE_Z[ctype]
        lines.append(f'      <xacro:cone name="{name}" type="{ctype}" '
                     f'pose="{x:.6g} {y:.6g} {z} 0 0 0"/>')
    return lines


def write_xacro(path: Path, inner, outer) -> None:
    body = [
        '<?xml version="1.0" ?>',
        "<sdf version='1.7' xmlns:xacro=\"http://www.ros.org/wiki/xacro\">",
        "  <world name='map'>",
        "",
        '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">',
        "      </plugin>",
        '    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">',
        "    </plugin>",
        '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">',
        "      <render_engine>ogre2</render_engine>",
        "    </plugin>",
        "    <plugin",
        '        filename="gz-sim-user-commands-system"',
        '        name="gz::sim::systems::UserCommands">',
        "      </plugin>",
        "",
        '    <xacro:include filename="$(find simulation)/models/world/physics.xacro"/>',
        "    <xacro:physics_tags/>",
        "",
        "    <include>",
        "      <uri>model://simulation/models/world/sun</uri>",
        "    </include>",
        "",
        "    <include>",
        "      <uri>model://simulation/models/world/ground_plane</uri>",
        "    </include>",
        "",
        '    <xacro:include filename="$(find simulation)/models/cones/cone.xacro"/>',
        "",
        '    <model name="cones">',
    ]
    body += _cone_xacro_lines(inner, outer)
    body += [
        '      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">',
        "        <use_pose_vector_msg>true</use_pose_vector_msg>",
        "        <publish_model_pose>true</publish_model_pose>",
        "        <publish_nested_model_pose>true</publish_nested_model_pose>",
        "        <publish_link_pose>false</publish_link_pose>",
        "      </plugin>",
        "    </model>",
        "  </world>",
        "</sdf>",
        "",
    ]
    path.write_text("\n".join(body))


def _cone_sdf_lines(inner, outer) -> list[str]:
    """Hand-expanded equivalent of the cone.xacro macro."""
    lines = []
    for name, ctype, x, y in cone_entries(inner, outer):
        z = CONE_Z[ctype]
        mass, (ixx, iyy, izz) = CONE_INERTIAL.get(ctype, CONE_INERTIAL["default"])
        lines += [
            f'      <model name="{name}">',
            "        <static>true</static>",
            f"        <pose>{x:.6g} {y:.6g} {z} 0 0 0</pose>",
            f'        <link name="{name}_link">',
            '          <visual name="visual">',
            "            <geometry>",
            "              <mesh>",
            f"                <uri>model://simulation/models/cones/cone_{ctype}/mesh.dae</uri>",
            "              </mesh>",
            "            </geometry>",
            "          </visual>",
            "          <!-- Different inertial properties for different types of cones -->",
            "          <inertial>",
            f"            <mass>{mass}</mass>",
            "            <inertia>",
            f"              <ixx>{ixx}</ixx>",
            "              <ixy>0.0</ixy>",
            "              <ixz>0.0</ixz>",
            f"              <iyy>{iyy}</iyy>",
            "              <iyz>0.0</iyz>",
            f"              <izz>{izz}</izz>",
            "            </inertia>",
            "          </inertial>",
            "        </link>",
            "      </model>",
        ]
    return lines


def write_sdf(path: Path, name: str, inner, outer) -> None:
    body = [
        '<?xml version="1.0" ?>',
        "<!-- =================================================================================== -->",
        f"<!--     This document was autogenerated by generate_tracks.py from {name:<21}-->",
        "<!--     EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 -->",
        "<!-- =================================================================================== -->",
        '<sdf version="1.7">',
        '  <world name="map">',
        '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">',
        "      </plugin>",
        '    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">',
        "    </plugin>",
        '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">',
        "      <render_engine>ogre2</render_engine>",
        "    </plugin>",
        '    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands">',
        "      </plugin>",
        "    <gravity>0 0 -9.8</gravity>",
        "    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>",
        '    <atmosphere type="adiabatic"/>',
        '    <physics type="ode">',
        "      <max_step_size>0.003333</max_step_size>",
        "      <real_time_factor>1</real_time_factor>",
        "      <real_time_update_rate>50</real_time_update_rate>",
        "    </physics>",
        "    <scene>",
        "      <ambient>0.4 0.4 0.4 1</ambient>",
        "      <background>0.7 0.7 0.7 1</background>",
        "      <shadows>1</shadows>",
        "    </scene>",
        "    <wind/>",
        "    <spherical_coordinates>",
        "      <surface_model>EARTH_WGS84</surface_model>",
        "      <latitude_deg>0</latitude_deg>",
        "      <longitude_deg>0</longitude_deg>",
        "      <elevation>0</elevation>",
        "      <heading_deg>0</heading_deg>",
        "    </spherical_coordinates>",
        "    <include>",
        "      <uri>model://simulation/models/world/sun</uri>",
        "    </include>",
        "    <include>",
        "      <uri>model://simulation/models/world/ground_plane</uri>",
        "    </include>",
        '    <model name="cones">',
    ]
    body += _cone_sdf_lines(inner, outer)
    body += [
        '      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">',
        "        <use_pose_vector_msg>true</use_pose_vector_msg>",
        "        <publish_model_pose>true</publish_model_pose>",
        "        <publish_nested_model_pose>true</publish_nested_model_pose>",
        "        <publish_link_pose>false</publish_link_pose>",
        "      </plugin>",
        "    </model>",
        "  </world>",
        "</sdf>",
        "",
    ]
    path.write_text("\n".join(body))


# --------------------------------------------------------------------------
def generate(name: str, spec: dict, sim_dir: Path, quiet: bool = False) -> dict:
    dense = catmull_rom_closed(spec["waypoints"])
    centre = resample_closed(dense, CONE_SPACING)
    centre, scale = fit_to_ground(centre)
    inner, outer = build_sides(centre)

    write_csvs(sim_dir / "tracks" / name, inner, outer)
    write_xacro(sim_dir / "models" / "tracks" / f"{name}.xacro", inner, outer)
    write_sdf(sim_dir / "models" / "tracks" / f"{name}.sdf", name, inner, outer)
    write_pairs(sim_dir / "config" / f"perfect_path_{name}_pairs.txt", inner, outer)

    # Spawn pose: on the centreline at the start line, facing along the tangent.
    th = tangents(centre)[0]
    length = sum(math.dist(centre[i], centre[(i + 1) % len(centre)])
                 for i in range(len(centre)))
    radii = corner_radii(centre)
    reach = max(max(abs(x), abs(y)) for x, y in inner + outer)
    info = {
        "name": name,
        "style": spec["style"],
        "cones": 2 * len(centre),
        "length_m": length,
        "min_radius_m": min(radii),
        "scale": scale,
        "reach_m": reach,
        "spawn": (centre[0][0], centre[0][1], th),
    }
    if not quiet:
        print(f"{name:<18} {info['cones']:>4} cones  {length:6.1f} m lap  "
              f"min R {info['min_radius_m']:5.2f} m  reach {reach:5.1f} m  "
              f"scale {scale:4.2f}  "
              f"spawn x={info['spawn'][0]:8.4f} y={info['spawn'][1]:8.4f} "
              f"Y={th:9.6f}")
    return info


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--track", action="append", choices=sorted(TRACKS),
                    help="generate only this track (repeatable)")
    ap.add_argument("--list", action="store_true", help="list tracks and exit")
    args = ap.parse_args()

    if args.list:
        for name, spec in TRACKS.items():
            print(f"{name:<18} [{spec['style']}] {spec['description']}")
        return

    sim_dir = Path(__file__).resolve().parent.parent
    chosen = args.track or sorted(TRACKS)
    for name in chosen:
        generate(name, TRACKS[name], sim_dir)


if __name__ == "__main__":
    main()
