#!/usr/bin/env python3
"""Convert an fsd_path_planning demo JSON track into a Gazebo xacro world file."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


CONE_Z = {
    "blue": "0.15",
    "yellow": "0.15",
    "large_orange": "0.033",
}


def load_cones_from_json(json_path: Path, frame: str) -> dict[str, list[tuple[float, float]]]:
    data = json.loads(json_path.read_text())
    if not data:
        raise ValueError(f"{json_path} contains no frames")

    if frame == "last":
        entry = data[-1]
    elif frame == "max":
        entry = max(data, key=lambda d: sum(len(c) for c in d["slam_cones"]))
    else:
        entry = data[int(frame)]

    slam_cones = entry["slam_cones"]
    if len(slam_cones) != 5:
        raise ValueError("Expected slam_cones to contain exactly 5 arrays")

    return {
        "yellow": [(float(x), float(y)) for x, y in slam_cones[1]],
        "blue": [(float(x), float(y)) for x, y in slam_cones[2]],
        "large_orange": [(float(x), float(y)) for x, y in slam_cones[4]],
    }


def build_xacro(track_name: str, cones: dict[str, list[tuple[float, float]]]) -> str:
    lines = [
        '<?xml version="1.0" ?>',
        '<sdf version=\'1.7\' xmlns:xacro="http://www.ros.org/wiki/xacro">',
        "  <world name='sim_world'>",
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

    for cone_type in ("blue", "yellow", "large_orange"):
        for i, (x, y) in enumerate(cones[cone_type], start=1):
            z = CONE_Z[cone_type]
            pose = f"{x} {y} {z} 0 0 0"
            lines.append(
                f'      <xacro:cone name="{cone_type}_{i}" type="{cone_type}" pose="{pose}"/>'
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
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "json_path",
        type=Path,
        help="Path to a demo JSON track (e.g. fsg_19_2_laps.json)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Output .xacro path (default: models/tracks/<json_stem>.xacro)",
    )
    parser.add_argument(
        "--frame",
        choices=("last", "max"),
        default="last",
        help="Which frame to export: last frame or frame with most cones",
    )
    args = parser.parse_args()

    json_path = args.json_path.resolve()
    output_path = args.output
    if output_path is None:
        script_dir = Path(__file__).resolve().parent
        output_path = script_dir.parent / "models" / "tracks" / f"{json_path.stem}.xacro"

    cones = load_cones_from_json(json_path, args.frame)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(build_xacro(json_path.stem, cones))

    print(f"Wrote {output_path}")
    print(
        "Cones:",
        f"blue={len(cones['blue'])},",
        f"yellow={len(cones['yellow'])},",
        f"large_orange={len(cones['large_orange'])}",
    )


if __name__ == "__main__":
    main()