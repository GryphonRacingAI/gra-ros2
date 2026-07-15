#!/usr/bin/env python3

import os
import select
import socket
import struct
import sys
import termios
import time
import tty

# CAN IDs
AI2VCU_STATUS_ID = 0x510
AI2VCU_DRIVE_F_ID = 0x511
AI2VCU_DRIVE_R_ID = 0x512
AI2VCU_STEER_ID = 0x513
AI2VCU_BRAKE_ID = 0x514

VCU2AI_STATUS_ID = 0x520
VCU2AI_DRIVE_F_ID = 0x521
VCU2AI_DRIVE_R_ID = 0x522
VCU2AI_STEER_ID = 0x523
VCU2AI_BRAKE_ID = 0x524
VCU2AI_WHEEL_SPEEDS_ID = 0x525
VCU2AI_WHEEL_COUNTS_ID = 0x526

# AS states
AS_OFF = 1
AS_READY = 2
AS_DRIVING = 3
AS_EMERGENCY_BRAKE = 4
AS_FINISHED = 5
AS_STATE_NAMES = {1: 'AS_OFF', 2: 'AS_READY', 3: 'AS_DRIVING', 4: 'AS_EMERGENCY_BRAKE', 5: 'AS_FINISHED'}

# AI2VCU_Status signals
MISSION_NOT_SELECTED = 0
MISSION_SELECTED = 1
MISSION_RUNNING = 2
MISSION_FINISHED = 3
DIRECTION_NEUTRAL = 0  
DIRECTION_FORWARD = 1

# Vehicle parameters
MOTOR_RATIO = 3.5
AXLE_TORQUE_MAX_raw = 1950  # 195.0 Nm
STEER_ANGLE_MAX_raw = 272   # 27.2 deg
STEER_RATE_degps = 30.0
SPEED_TIME_CONSTANT_s = 0.5
COAST_DECEL_rpmps = 60.0
BRAKE_DECEL_rpmps = 8.0     # per % of brake pressure
EBS_DECEL_rpmps = 800.0
PULSES_PER_REV = 20.0

# State machine timing (from the ADS-DV Software Interface Specification)
READY_HOLD_s = 5.0
AI_COMMS_TIMEOUT_s = 0.5
EBS_TIMER_s = 15.0
WHEEL_STOPPED_rpm = 10.0

LOOP_PERIOD_s = 0.01
HANDSHAKE_PERIOD_s = 0.05

CAN_FRAME_FMT = '<IB3x8s'


class VcuSim:
    def __init__(self, interface):
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))
        self.sock.setblocking(False)

        # operator inputs
        self.asms = False
        self.tsms = False
        self.go_switch = False
        self.sdc_open = False
        self.ami_state = 0

        # AI2VCU data
        self.ai_handshake = 0
        self.ai_estop = 0
        self.ai_mission_status = MISSION_NOT_SELECTED
        self.ai_direction = DIRECTION_NEUTRAL
        self.ai_torque_f_raw = 0
        self.ai_torque_r_raw = 0
        self.ai_motor_speed_max_rpm = 0
        self.ai_steer_raw = 0
        self.ai_brake_f_pct = 0.0
        self.ai_brake_r_pct = 0.0
        self.ai_status_rx_time = None

        # vehicle state
        self.as_state = AS_OFF
        self.ready_time = None
        self.ebs_time = None
        self.ebs_latched = False
        self.res_go = False
        self.prev_go_switch = False
        self.handshake = 0
        self.handshake_time = 0.0
        self.wheel_rpm = 0.0
        self.steer_raw = 0.0
        self.brake_f_pct = 0.0
        self.brake_r_pct = 0.0
        self.pulses = 0.0

    def send(self, can_id, data):
        frame = struct.pack(CAN_FRAME_FMT, can_id, len(data), bytes(data).ljust(8, b'\x00'))
        try:
            self.sock.send(frame)
        except OSError:
            pass  # e.g. tx queue full

    def receive(self, now):
        while True:
            try:
                frame = self.sock.recv(16)
            except BlockingIOError:
                return
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
            can_id &= socket.CAN_EFF_MASK

            if can_id == AI2VCU_STATUS_ID:
                self.ai_handshake = data[0] & 0x01
                self.ai_estop = data[1] & 0x01
                self.ai_mission_status = (data[1] >> 4) & 0x03
                self.ai_direction = (data[1] >> 6) & 0x03
                self.ai_status_rx_time = now
            elif can_id == AI2VCU_DRIVE_F_ID:
                self.ai_torque_f_raw = data[0] + (data[1] << 8)
                self.ai_motor_speed_max_rpm = data[2] + (data[3] << 8)
            elif can_id == AI2VCU_DRIVE_R_ID:
                self.ai_torque_r_raw = data[0] + (data[1] << 8)
            elif can_id == AI2VCU_STEER_ID:
                self.ai_steer_raw = struct.unpack_from('<h', data)[0]
            elif can_id == AI2VCU_BRAKE_ID:
                self.ai_brake_f_pct = 0.5 * data[0]
                self.ai_brake_r_pct = 0.5 * data[1]

    def handle_key(self, key):
        if key == 'a':
            self.asms = not self.asms
        elif key == 't':
            self.tsms = not self.tsms
        elif key in '01234567':
            self.ami_state = int(key)
        elif key == 'g':
            self.go_switch = not self.go_switch
        elif key == 'e':
            self.sdc_open = True
        elif key == 'r':  # power cycle
            self.asms = False
            self.tsms = False
            self.go_switch = False
            self.sdc_open = False
            self.ebs_latched = False
            self.ami_state = 0
            self.as_state = AS_OFF
            self.res_go = False
            self.wheel_rpm = 0.0
            self.steer_raw = 0.0

    def enter_emergency(self, now, cause):
        self.as_state = AS_EMERGENCY_BRAKE
        self.ebs_time = now
        self.ebs_latched = True
        self.res_go = False
        print(f"\r\nEMERGENCY_BRAKE: {cause}\r")

    def step_state_machine(self, now):
        go_rising = self.go_switch and not self.prev_go_switch
        self.prev_go_switch = self.go_switch

        if self.as_state == AS_OFF:
            if (self.asms and self.tsms and not self.sdc_open and not self.ebs_latched
                    and self.ami_state != 0 and self.ai_mission_status == MISSION_SELECTED):
                self.as_state = AS_READY
                self.ready_time = now

        elif self.as_state == AS_READY:
            if self.sdc_open:
                self.enter_emergency(now, 'shutdown circuit open')
            elif self.ai_estop:
                self.enter_emergency(now, 'AI estop request')
            elif not self.asms:
                self.as_state = AS_OFF
            elif (go_rising and (now - self.ready_time) >= READY_HOLD_s
                    and self.ai_torque_f_raw == 0 and self.ai_torque_r_raw == 0
                    and self.ai_steer_raw == 0 and self.ai_direction == DIRECTION_NEUTRAL
                    and abs(self.steer_raw) < 50):
                self.as_state = AS_DRIVING
                self.res_go = True

        elif self.as_state == AS_DRIVING:
            wheels_moving = self.wheel_rpm > WHEEL_STOPPED_rpm
            comms_lost = (self.ai_status_rx_time is None
                          or (now - self.ai_status_rx_time) > AI_COMMS_TIMEOUT_s)
            if self.sdc_open:
                self.enter_emergency(now, 'shutdown circuit open')
            elif self.ai_estop:
                self.enter_emergency(now, 'AI estop request')
            elif not self.go_switch:
                self.enter_emergency(now, 'RES go signal off')
            elif not self.asms:
                self.enter_emergency(now, 'ASMS off')
            elif comms_lost:
                self.enter_emergency(now, 'AI comms lost')
            elif self.ai_mission_status == MISSION_FINISHED and wheels_moving:
                self.enter_emergency(now, 'MISSION_STATUS_FAULT')
            elif self.ai_direction == DIRECTION_NEUTRAL and wheels_moving:
                self.enter_emergency(now, 'AUTONOMOUS_BRAKING_FAULT')
            elif ((self.ai_torque_f_raw > 0 or self.ai_torque_r_raw > 0)
                    and (self.ai_brake_f_pct > 0 or self.ai_brake_r_pct > 0)):
                self.enter_emergency(now, 'BRAKE_PLAUSIBILITY_FAULT')
            elif self.ai_mission_status == MISSION_FINISHED:
                self.as_state = AS_FINISHED
                self.res_go = False

        elif self.as_state == AS_FINISHED:
            if self.sdc_open:
                self.enter_emergency(now, 'shutdown circuit open')
            elif not self.asms:
                self.as_state = AS_OFF

        elif self.as_state == AS_EMERGENCY_BRAKE:
            if (now - self.ebs_time) >= EBS_TIMER_s and not self.asms:
                self.as_state = AS_OFF

    def step_vehicle(self, dt):
        if self.as_state == AS_DRIVING:
            self.brake_f_pct = self.ai_brake_f_pct
            self.brake_r_pct = self.ai_brake_r_pct

            if self.brake_f_pct > 0 or self.brake_r_pct > 0:
                decel = BRAKE_DECEL_rpmps * max(self.brake_f_pct, self.brake_r_pct)
                self.wheel_rpm = max(0.0, self.wheel_rpm - decel * dt)
            elif (self.ai_direction == DIRECTION_FORWARD
                    and (self.ai_torque_f_raw > 0 or self.ai_torque_r_raw > 0)):
                target_rpm = self.ai_motor_speed_max_rpm / MOTOR_RATIO
                self.wheel_rpm += (target_rpm - self.wheel_rpm) * min(1.0, dt / SPEED_TIME_CONSTANT_s)
            else:
                self.wheel_rpm = max(0.0, self.wheel_rpm - COAST_DECEL_rpmps * dt)

            steer_target = max(-STEER_ANGLE_MAX_raw, min(STEER_ANGLE_MAX_raw, self.ai_steer_raw))
            steer_step = 10.0 * STEER_RATE_degps * dt
            self.steer_raw += max(-steer_step, min(steer_step, steer_target - self.steer_raw))
        elif self.as_state == AS_EMERGENCY_BRAKE:
            self.brake_f_pct = 100.0
            self.brake_r_pct = 100.0
            self.wheel_rpm = max(0.0, self.wheel_rpm - EBS_DECEL_rpmps * dt)
        else:
            self.brake_f_pct = 0.0
            self.brake_r_pct = 0.0
            self.wheel_rpm = max(0.0, self.wheel_rpm - COAST_DECEL_rpmps * dt)

        self.pulses += self.wheel_rpm / 60.0 * PULSES_PER_REV * dt

    def transmit(self, now):
        if (now - self.handshake_time) >= HANDSHAKE_PERIOD_s:
            self.handshake ^= 1
            self.handshake_time = now

        rpm = int(self.wheel_rpm)
        pulse_count = int(self.pulses) & 0xFFFF
        steer_raw = int(self.steer_raw)

        self.send(VCU2AI_STATUS_ID, [
            self.handshake,
            (self.asms << 1) | (self.tsms << 2) | (self.res_go << 3),
            (self.as_state & 0x0F) | ((self.ami_state & 0x0F) << 4),
            0, 0, 0, 0, 0])
        self.send(VCU2AI_DRIVE_F_ID, struct.pack('<3H', 0, self.ai_torque_f_raw, AXLE_TORQUE_MAX_raw))
        self.send(VCU2AI_DRIVE_R_ID, struct.pack('<3H', 0, self.ai_torque_r_raw, AXLE_TORQUE_MAX_raw))
        self.send(VCU2AI_STEER_ID, struct.pack('<hHh', steer_raw, STEER_ANGLE_MAX_raw, self.ai_steer_raw))
        self.send(VCU2AI_BRAKE_ID, [
            int(2.0 * self.brake_f_pct), int(2.0 * self.ai_brake_f_pct),
            int(2.0 * self.brake_r_pct), int(2.0 * self.ai_brake_r_pct), 0])
        self.send(VCU2AI_WHEEL_SPEEDS_ID, struct.pack('<4H', rpm, rpm, rpm, rpm))
        self.send(VCU2AI_WHEEL_COUNTS_ID, struct.pack('<4H', *(4 * [pulse_count])))

    def print_status(self):
        line = (f"{AS_STATE_NAMES[self.as_state]:<18s}"
                f" AMI={self.ami_state} ASMS={int(self.asms)} TSMS={int(self.tsms)} GO={int(self.go_switch)}"
                f" | AI: mission={self.ai_mission_status} dir={self.ai_direction} estop={self.ai_estop}"
                f" trq={0.1 * self.ai_torque_f_raw:5.1f}Nm brake={self.ai_brake_f_pct:3.0f}%"
                f" | rpm={self.wheel_rpm:4.0f} steer={0.1 * self.steer_raw:+5.1f}deg")
        end = '\r' if sys.stdout.isatty() else '\n'
        print(f"{line}   ", end=end, flush=True)


def main():
    interface = sys.argv[1] if len(sys.argv) > 1 else 'vcan0'

    try:
        sim = VcuSim(interface)
    except OSError as e:
        print(f"Can't open [{interface}]: {e}")
        return 1

    print(f"VCU simulator on {interface}")
    print("keys: a=ASMS  t=TSMS  1-7=select mission  0=deselect  g=RES go  e=E-stop  r=power cycle  q=quit")

    stdin_fd = sys.stdin.fileno()
    old_termios = None
    if sys.stdin.isatty():
        old_termios = termios.tcgetattr(stdin_fd)
        tty.setcbreak(stdin_fd)

    last = time.monotonic()
    last_print = 0.0
    try:
        while True:
            now = time.monotonic()
            dt = now - last
            last = now

            if select.select([stdin_fd], [], [], 0)[0]:
                key = os.read(stdin_fd, 1).decode(errors='ignore')
                if key == 'q' or key == '':
                    break
                sim.handle_key(key)

            sim.receive(now)
            sim.step_state_machine(now)
            sim.step_vehicle(dt)
            sim.transmit(now)

            if (now - last_print) >= 0.2:
                sim.print_status()
                last_print = now

            time.sleep(LOOP_PERIOD_s)
    except KeyboardInterrupt:
        pass
    finally:
        if old_termios is not None:
            termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_termios)
        print()

    return 0


if __name__ == '__main__':
    sys.exit(main())
