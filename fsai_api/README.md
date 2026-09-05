# fsai_api
Bridges communication between ROS nodes and the Vehicle Control Unit (VCU) via CAN.


## Installation
- Install CAN utilities: `sudo apt install can-utils`
- Optional: `rosdep install --from-paths src -r -y`.

## Usage

### 1. Setup CAN Interface

**For real CAN (can0):**
```bash
sudo ip link set can0 up type can bitrate 500000
```

**For virtual CAN (vcan0) - testing only:**
```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

### 2. Launch CAN Bridge

**Real CAN:**
```bash
ros2 run fsai_api ackermann_can can0
```

**Virtual CAN:**
```bash
ros2 run fsai_api ackermann_can vcan0
```

### 3. Run Speed Controller (Optional)
```bash
ros2 run fsai_api wheel_speed_controller.py
```

### 4. Cleanup (Virtual CAN only)
When done testing with virtual CAN:
```bash
sudo ip link delete vcan0
```

## Interface

| Node | Inputs | Outputs | Description |
|------|--------|---------|-------------|
| `ackermann_can` | `/ackermann_cmd` (`ackermann_msgs/msg/AckermannDrive`)<br>`/emergency_brake` (`std_msgs/msg/Bool`)<br>`/chequered_flag` (`std_msgs/msg/Bool`)<br>`/brake` (`std_msgs/msg/Bool`) | `/vcu2ai` (`fsai_api/msg/VCU2AI`) | ROS↔VCU CAN bridge for drive commands and vehicle state |
| `wheel_speed_controller` | `/vcu2ai` (`fsai_api/msg/VCU2AI`)<br>`/ackermann_cmd_controller` (`ackermann_msgs/msg/AckermannDrive`) | `/ackermann_cmd` (`ackermann_msgs/msg/AckermannDrive`) | PI controller for wheel speed regulation |

## VCU simulator (`vcu_sim.py`)

Virtual-CAN ADS-DV state machine for bench testing without the real VCU.

```bash
# terminal 1
python3 src/fsai_api/scripts/vcu_sim.py vcan0
# or: ros2 run fsai_api vcu_sim.py vcan0

# terminal 2
ros2 run fsai_api ackermann_can vcan0
```

Keys in the vcu_sim pane: `a`=ASMS, `t`=TSMS, `1-7`=mission, `0`=deselect, `g`=RES go, `e`=SDC open, `r`=power cycle, `q`=quit.

### State transitions (matches ADS-DV / mission_supervisor README)

```
AS_OFF --------> AS_READY
    - AS master switch ON  AND
    - TS master switch ON  AND
    - EBS not latched / SDC closed  AND
    - AMI mission selected (1-7)  AND
    - AI MISSION_STATUS == SELECTED   (ackermann_can sets this when AMI != 0)

AS_READY ------> AS_DRIVING
    - 5 s hold elapsed  AND
    - front/rear axle torque request == 0  AND
    - steer angle request == 0  AND
    - abs(actual steer) < 5 deg  AND
    - DIRECTION_REQUEST == NEUTRAL  AND
    - RES "Go" rising edge (OFF -> ON)

AS_DRIVING -----> AS_FINISHED
    - AI MISSION_STATUS == FINISHED  AND
    - all wheel speeds < 10 rpm
    (FINISHED while wheels still moving -> EMERGENCY / MISSION_STATUS_FAULT)

AS_DRIVING ----> EMERGENCY_BRAKE
    - RES Go == OFF  OR  ASMS off  OR  AI estop  OR  SDC open  OR
    - AI_COMMS_LOST / AUTONOMOUS_BRAKING / BRAKE_PLAUSIBILITY faults

AS_FINISHED ----[SDC open]---> EMERGENCY_BRAKE

AS_EMERGENCY_BRAKE ----> AS_OFF
    - EBS timer complete (15 s)  AND  AS master switch OFF
```

Automated check (spawns vcu_sim + ackermann_can, exercises the graph above):

```bash
source install/setup.bash
python3 test_vcu_state_machine.py --spawn
```

Or use the interactive tmux session: [`./tmux/vcu_sim.sh`](../tmux/README.md) with teleop_vcu sliders for `/ackermann_cmd` + brake flags.

Full sim (Gazebo + YOLO + pathfinder + controller) uses the same VCU sim: `./tmux/startup.sh`. Arm ASMS/TSMS/AMI/RES in the `vcu` pane. `mission_supervisor` forwards `/ackermann_cmd_planner` onto `/ackermann_cmd_controller` only in `AS_DRIVING`. `/vcu2ai` is teed through `run_logged` into `~/colcon_ws/logs/<stamp>/vcu2ai.log`.

## Notes
- `vcu_sim.py` implements the ADS-DV autonomous-system state machine on vcan0 (see above).

