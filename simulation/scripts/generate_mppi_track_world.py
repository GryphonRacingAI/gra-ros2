#!/usr/bin/env python3

import csv
from pathlib import Path


def main():
    script_dir = Path(__file__).resolve().parent
    workspace_root = script_dir.parent.parent
    control_scripts = workspace_root / "simulation" / "tracks" / "mppi_track" 
    inner_path = control_scripts / "inner_cones.csv"
    outer_path = control_scripts / "outer_cones.csv"
    output_path = script_dir.parent / "models" / "tracks" / "mppi_track.xacro"

    inner_cones = []
    outer_cones = []


    with open(inner_path) as f:
        for row in csv.reader(f):
            if len(row) >= 2:
                inner_cones.append((float(row[0]), float(row[1])))

    with open(outer_path) as f:
        for row in csv.reader(f):
            if len(row) >= 2:
                outer_cones.append((float(row[0]), float(row[1])))

    n = min(len(inner_cones), len(outer_cones))
    inner_cones = inner_cones[:n]
    outer_cones = outer_cones[:n]

    lines = [
        '<?xml version="1.0" ?>',
        '<sdf version=\'1.7\' xmlns:xacro="http://www.ros.org/wiki/xacro">',
        '  <world name=\'map\'>',
        "",
        "    <plugin filename=\"ignition-gazebo-physics-system\" name=\"ignition::gazebo::systems::Physics\">",
        "      </plugin>",
        "    <plugin filename=\"ignition-gazebo-scene-broadcaster-system\" name=\"ignition::gazebo::systems::SceneBroadcaster\">",
        "    </plugin>",
        "    <plugin filename=\"ignition-gazebo-sensors-system\" name=\"ignition::gazebo::systems::Sensors\">",
        "      <render_engine>ogre2</render_engine>",
        "    </plugin>",
        "    <plugin",
        "        filename=\"ignition-gazebo-user-commands-system\"",
        "        name=\"ignition::gazebo::systems::UserCommands\">",
        "      </plugin>",
        "",
        "    <xacro:include filename=\"$(find simulation)/models/world/physics.xacro\"/>",
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
        "    <xacro:include filename=\"$(find simulation)/models/cones/cone.xacro\"/>",
        "",
        "    <model name=\"cones\">",
    ]

    for i, (x, y) in enumerate(inner_cones):
        pose = f"{x} {y} 0.15 0 0 0"
        if i == 0:
            cone_type = "orange"
            lines.append(f'      <xacro:cone name="orange_{i+1}" type="{cone_type}" pose="{pose}"/>')
        else:
            cone_type = "blue"
            lines.append(f'      <xacro:cone name="blue_{i+1}" type="{cone_type}" pose="{pose}"/>')


    for i, (x, y) in enumerate(outer_cones):
        pose = f"{x} {y} 0.15 0 0 0"
        if i == 0:
            cone_type = "orange"
            lines.append(f'      <xacro:cone name="orange_{i+1}" type="{cone_type}" pose="{pose}"/>')
        else:
            cone_type = "yellow"
            lines.append(f'      <xacro:cone name="yellow_{i+1}" type="{cone_type}" pose="{pose}"/>')

    lines.extend([
        "",
        "      <plugin name=\"ignition::gazebo::systems::PosePublisher\" filename=\"ignition-gazebo-pose-publisher-system\">",
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
    ])

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n")

    pairs_path = script_dir.parent / "config" / "perfect_path_mppi_track_pairs.txt"
    pairs_path.parent.mkdir(parents=True, exist_ok=True)
    with open(pairs_path, "w") as f:
        for i in range(n):
            f.write(f"{inner_cones[i][0]} {inner_cones[i][1]} {outer_cones[i][0]} {outer_cones[i][1]}\n")


if __name__ == "__main__":
    main()
