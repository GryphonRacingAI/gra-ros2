#!/usr/bin/env python3
"""Swap mppi_track blue/yellow roles and narrow width to 3 m."""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

TARGET_WIDTH = 3.0
Z_REGULAR = "0.15"
Z_LARGE_ORANGE = "0.15"


def load_csv(path: Path) -> list[tuple[float, float]]:
    points = []
    with path.open() as f:
        for row in csv.reader(f):
            if len(row) >= 2:
                points.append((float(row[0]), float(row[1])))
    return points


def write_csv(path: Path, points: list[tuple[float, float]]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        for x, y in points:
            writer.writerow([x, y])


def narrow_inner(
    inner: np.ndarray, outer: np.ndarray, width: float
) -> np.ndarray:
    out = np.empty_like(inner)
    for i in range(len(inner)):
        vec = outer[i] - inner[i]
        dist = np.linalg.norm(vec)
        if dist < 1e-9:
            out[i] = inner[i]
        else:
            out[i] = outer[i] - (vec / dist) * width
    return out


def pose_str(x: float, y: float, z: str) -> str:
    return f"{x} {y} {z} 0 0 0"


def build_xacro(
    inner_orange: tuple[float, float],
    inner_trace: list[tuple[float, float]],
    outer_orange: tuple[float, float],
    outer_trace: list[tuple[float, float]],
) -> str:
    lines = [
        '<?xml version="1.0" ?>',
        '<sdf version=\'1.7\' xmlns:xacro="http://www.ros.org/wiki/xacro">',
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

    ox, oy = inner_orange
    lines.append(
        f'      <xacro:cone name="large_orange_1" type="large_orange" '
        f'pose="{pose_str(ox, oy, Z_LARGE_ORANGE)}"/>'
    )

    for i, (x, y) in enumerate(inner_trace, start=2):
        lines.append(
            f'      <xacro:cone name="yellow_{i}" type="yellow" '
            f'pose="{pose_str(x, y, Z_REGULAR)}"/>'
        )

    ox, oy = outer_orange
    lines.append(
        f'      <xacro:cone name="large_orange_2" type="large_orange" '
        f'pose="{pose_str(ox, oy, Z_LARGE_ORANGE)}"/>'
    )

    for i, (x, y) in enumerate(outer_trace, start=2):
        lines.append(
            f'      <xacro:cone name="blue_{i}" type="blue" '
            f'pose="{pose_str(x, y, Z_REGULAR)}"/>'
        )

    lines.extend(
        [
            "",
            '      <plugin name="gz::sim::systems::PosePublisher" filename="gz-sim-pose-publisher-system">',
            "        <use_pose_vector_msg>true</use_pose_vector_msg>",
            "        <publish_model_pose>true</publish_model_pose>",
            "        <publish_nested_model_pose>true</publish_nested_model_pose>",
            "        <publish_link_pose>false</publish_link_pose>",
            "      </plugin>",
            "",
            "    </model>",
            "",
            "  </world>",
            "</sdf>",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    sim_dir = script_dir.parent
    track_dir = sim_dir / "tracks" / "mppi_track"
    inner_path = track_dir / "inner_cones.csv"
    outer_path = track_dir / "outer_cones.csv"
    xacro_path = sim_dir / "models" / "tracks" / "mppi_track.xacro"
    pairs_path = sim_dir / "config" / "perfect_path_mppi_track_pairs.txt"

    inner_raw = np.array(load_csv(inner_path))
    outer_raw = np.array(load_csv(outer_path))
    n = min(len(inner_raw), len(outer_raw))

    inner_narrow = narrow_inner(inner_raw[:n], outer_raw[:n], TARGET_WIDTH)

    write_csv(inner_path, [tuple(p) for p in inner_narrow])
    # outer boundary unchanged; rewrite for consistent formatting
    write_csv(outer_path, [tuple(p) for p in outer_raw[:n]])

    # xacro uses first point as start orange + next 145 as boundary cones
    xacro_pairs = 146
    inner_orange = tuple(inner_narrow[0])
    outer_orange = tuple(outer_raw[0])
    inner_trace = [tuple(p) for p in inner_narrow[1:xacro_pairs]]
    outer_trace = [tuple(p) for p in outer_raw[1:xacro_pairs]]

    xacro_path.write_text(
        build_xacro(inner_orange, inner_trace, outer_orange, outer_trace)
    )

    pairs_path.parent.mkdir(parents=True, exist_ok=True)
    with pairs_path.open("w") as f:
        for i in range(xacro_pairs):
            ix, iy = inner_narrow[i]
            ox, oy = outer_raw[i]
            f.write(f"{ix} {iy} {ox} {oy}\n")

    widths = np.linalg.norm(outer_raw[:xacro_pairs] - inner_narrow[:xacro_pairs], axis=1)
    print(f"Updated {xacro_path}")
    print(f"Updated {inner_path} and {outer_path}")
    print(f"Updated {pairs_path}")
    print(
        f"Track width over {xacro_pairs} pairs: "
        f"min={widths.min():.3f} max={widths.max():.3f} mean={widths.mean():.3f} m"
    )


if __name__ == "__main__":
    main()