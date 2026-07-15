# Mission Supervisor

### Ackermann Can
Responsible for sending AI2VCU CAN messages to VCU.
Receives VCU2AI through the C fsai_api and publishes for ROS2 to get vehicle feedback.
Subscribes to braking, ackermann_cmd, chequered_flag 

### VCU Simulator
`vcu_sim.py` - Implements the FSAI_API spec's state machine and the transitions as shown below:
```
AS_OFF --------> AS_READY
    - [AS master switch ON] AND
    - [TS master switch ON] AND
    - [EBS status == ARMED] AND
    - [MISSION_STATUS == SELECTED]

AS_READY ------> AS_DRIVING
    1.
        - [5s timer elapsed]                AND
        - [front axle torque request == 0]  AND
        - [rear axle torque request == 0]   AND
        - [steer angle request == 0]        AND
        - [abs(actual steer angle) < 5 deg] AND
        - [DIRECTION_REQUEST == NEUTRAL]    AND
    2.
        - ["Go" signal OFF -> ON]

AS_DRIVING -----> AS_FINISHED
    - [MISSION_STATUS == FINISHED]  AND
    - [all wheel speeds < 10rpm]

AS_DRIVING ----> EMERGENCY_BRAKE
    - ["Go" signal == OFF]      OR
    - Fault Conditions Met      OR
    - ... (out of our control conditions)

AS_FINISHED ----[SDC open]---> EMERGENCY_BRAKE
    1. [EBS timer complete (15s)]
    2. [AS master switch == OFF]
```

### Mission Supervisor Node
- [ ] Responsible for status inspection of nodes: *perception*, *path_planning*, *control*, *lap_counter*
- [ ] Responsible for avoid faults when in *AS_DRIVING* state: refer to section 4 of [ADS-DV_Software_Specs](https://github.com/FS-AI/FS-AI_ADS-DV_Documentation/blob/main/ADS-DV_Software_Interface_Specification_v4.0.pdf)
    - [ ] MISSION_STATUS_FAULT 
    - [ ] AUTONOMOUS_BRAKING_FAULT
    - [ ] AI_COMMS_LOST_FAULT
    - [ ] BRAKE_PLAUSIBILITY_FAULT