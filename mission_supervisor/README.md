# Mission Supervisor

Owns **when** the car is allowed to drive. Path planning and control may run before RES go (so health checks pass), but their commands only reach `/ackermann_cmd_controller` while the VCU reports **AS_DRIVING**.

```bash
ros2 run mission_supervisor mission_supervisor --ros-args -p use_sim_time:=true
```

Sim stack: [`tmux/startup.sh`](../tmux/README.md) starts this node, `lap_counter`, and remaps the controller to `/ackermann_cmd_planner`.

Entry point: `mission_supervisor.supervisor:main` (`mission_supervisor/supervisor.py`). The older `mission_supervisor_node` executable is a leftover autocross stub — do not use it for ADS-DV.

---

## Command graph

```
controller  →  /ackermann_cmd_planner
                    ↓
         mission_supervisor
           • dynamic AMI 1–4: relay planner cmds only in AS_DRIVING
           • scripted AMI 5–7: supervisor publishes drive (sweep/ramp/brake)
                    ↓
         /ackermann_cmd_controller  →  wheel_speed_controller  →  /ackermann_cmd
                    ↓
         ackermann_can  →  CAN  →  VCU (or vcu_sim.py)
                    ↓
         /vcu2ai  →  supervisor (state, health, wheel RPM)
```

Leaving `AS_DRIVING` (or finishing a mission) publishes speed 0.

---

## AMI missions

| AMI | Name | Handler | Relays planner? |
|-----|------|---------|-----------------|
| 1 | acceleration | timed run (`acceleration_run_s`, default 15 s) | yes |
| 2 | skidpad | laps (`skidpad_laps`, default 4) | yes |
| 3 | autocross | laps (`autocross_laps`, default 1) | yes |
| 4 | trackdrive | laps (`trackdrive_laps`, default 10) | yes |
| 5 | static_inspection_A | scripted sweep / ramp / brake | no |
| 6 | static_inspection_B | scripted ramp / EBS | no |
| 7 | autonomous_demo | FSUK demo sequence | no |

`/mission` is published as soon as AMI is selected (before RES go) so the rest of the stack can see the event name.

---

## Parameters

| Parameter | Default | Role |
|-----------|---------|------|
| `controller_node` | `mppi_controller` | name that must be alive for dynamic events (tmux sets `pure_pursuit_controller` when `CONTROLLER=pp`) |
| `path_node` | `track_pathfinder` | `perfect_path` when `CONES=perfect` |
| `perception_node` | `predict_node` | skipped if `require_perception:=false` |
| `require_perception` | `true` | |
| `planner_cmd_topic` | `/ackermann_cmd_planner` | |
| `acceleration_run_s` | `15.0` | |
| `skidpad_laps` / `autocross_laps` / `trackdrive_laps` | 4 / 1 / 10 | |

Health (`/mission_supervisor/status`, 1 Hz): `/vcu2ai` freshness (0.5 s) and missing nodes for the selected AMI. Watch it before RES go.

---

## VCU state machine

Matches ADS-DV / [`vcu_sim.py`](../fsai_api/README.md). Keyboard on the sim: `a` ASMS, `t` TSMS, `1–7` AMI, `g` RES go.

```
AS_OFF --------> AS_READY
    - AS master switch ON  AND
    - TS master switch ON  AND
    - EBS not latched / SDC closed  AND
    - AMI mission selected  AND
    - AI MISSION_STATUS == SELECTED   (ackermann_can sets this when AMI != 0)

AS_READY ------> AS_DRIVING
    - 5 s hold  AND
    - torque and steer requests == 0  AND
    - abs(actual steer) < 5 deg  AND
    - DIRECTION_REQUEST == NEUTRAL  AND
    - RES Go rising edge

AS_DRIVING -----> AS_FINISHED
    - AI MISSION_STATUS == FINISHED  AND
    - all wheel speeds < 10 rpm

AS_DRIVING ----> EMERGENCY_BRAKE
    - RES Go OFF  OR  ASMS off  OR  AI estop  OR  SDC open  OR  comms/brake faults
```

ADS-DV spec (faults): [ADS-DV Software Interface Specification](https://github.com/FS-AI/FS-AI_ADS-DV_Documentation/blob/main/ADS-DV_Software_Interface_Specification_v4.0.pdf) §4.

---

## Other executables

| Command | Use |
|---------|-----|
| `ros2 run mission_supervisor lap_counter` | `/laps` from orange cones + odom (needed for AMI 2–4) |
| `ros2 run mission_supervisor autonomous_demo` | standalone demo node (tmux uses the supervisor handler instead) |
| `static_a` / `static_b` | standalone inspection nodes |

Hardware boot (real `can0`, no Gazebo): [`bringup/startup_tmux.sh`](../bringup/startup_tmux.sh).
