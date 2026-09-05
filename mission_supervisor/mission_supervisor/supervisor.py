#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt16
from ackermann_msgs.msg import AckermannDrive
from fsai_api.msg import VCU2AI


def linear_interpolate(value, a1, a2, b1, b2):
    if value < a1:
        return b1
    elif value > a2:
        return b2
    else:
        left_span = a2 - a1
        right_span = b2 - b1
        value_scaled = float(value - a1) / float(left_span)
        return b1 + (value_scaled * right_span)


class MissionHandler:
    """Started when AS_DRIVING is entered with the matching AMI mission.
    tick() is called at 10Hz from the supervisor timer and must not block,
    so /vcu2ai data stays live during the mission."""
    name = 'base'
    # Dynamic events: supervisor forwards /ackermann_cmd_planner.
    # Scripted events publish drive themselves and must not relay.
    relays_planner = False

    def __init__(self, sup):
        self.sup = sup
        self.done = False

    def start(self, now):
        pass

    def tick(self, now):
        raise NotImplementedError


class ScriptedHandler(MissionHandler):
    """Runs a fixed sequence of phases, based on the autonomous_demo sequence
    driven at FSUK. Each phase is (name, duration); duration None means the
    phase decides itself when it is finished."""
    PHASES = []
    SWEEP_ANGLES = [-0.7, 0.7, 0.0]
    TARGET_SPEED_mps = 0.0

    def start(self, now):
        self.phase_i = 0
        self.phase_start = now
        self.sup.get_logger().info(f"{self.name}: starting phase '{self.PHASES[0][0]}'")

    def tick(self, now):
        if self.done:
            return
        name, duration = self.PHASES[self.phase_i]
        t = now - self.phase_start
        finished = getattr(self, 'phase_' + name)(t)
        if finished or (duration is not None and t >= duration):
            if name == 'brake':
                self.sup.publish_brake(False)
            self.phase_i += 1
            self.phase_start = now
            if self.phase_i >= len(self.PHASES):
                self.done = True
            else:
                self.sup.get_logger().info(f"{self.name}: starting phase '{self.PHASES[self.phase_i][0]}'")

    def phase_wait(self, t):
        return False

    def phase_pause(self, t):
        return False

    def phase_sweep(self, t):
        a = self.SWEEP_ANGLES
        if t < 2:
            angle = linear_interpolate(t, 0, 2, 0, a[0])
        elif t <= 3:
            angle = a[0]
        elif t < 5:
            angle = linear_interpolate(t, 3, 5, a[0], a[1])
        elif t <= 6:
            angle = a[1]
        elif t < 8.5:
            angle = linear_interpolate(t, 6, 8.5, a[1], a[2])
        else:
            angle = a[2]
        self.sup.publish_drive(0.0, angle)
        return False

    def phase_ramp(self, t):
        duration = self.PHASES[self.phase_i][1]
        self.sup.publish_drive(linear_interpolate(t, 0, duration, 0.0, self.TARGET_SPEED_mps))
        return False

    def phase_hold(self, t):
        self.sup.publish_drive(self.TARGET_SPEED_mps)
        return False

    def phase_brake(self, t):
        self.sup.publish_drive(0.0)
        self.sup.publish_brake(True)
        avg = self.sup.avg_rear_wheel_rpm()
        if avg is not None:
            return avg <= 5.0
        return t >= 12.0  # no wheel speed data: brake for a fixed time
        # the phase duration acts as the safety timeout

    def phase_finish(self, t):
        self.sup.publish_drive(0.0)
        self.sup.publish_chequered_flag()
        return True

    def phase_finish_estop(self, t):
        self.sup.publish_drive(0.0)
        self.sup.publish_emergency_brake(True)
        self.sup.publish_chequered_flag()
        return True


class AutonomousDemoHandler(ScriptedHandler):
    # Sequence and values ported from autonomous_demo.py (time-based branch)
    name = 'autonomous_demo'
    ACCELERATION = 1.1  # m/s^2

    PHASES = [
        ('wait', 5.0),
        ('sweep', 9.0),
        ('pause', 1.0),
        ('accel', 2.5),
        ('pause', 2.5),
        ('brake', 20.0),
        ('pause', 2.5),
        ('accel', 2.5),
        ('pause', 2.5),
        ('finish_estop', None),
    ]

    def phase_accel(self, t):
        self.sup.publish_drive(t * self.ACCELERATION)
        return False


class StaticInspectionAHandler(ScriptedHandler):
    # TODO: verify sequence and values against the current FS-AI rulebook
    name = 'static_inspection_A'
    TARGET_SPEED_mps = 2.0

    PHASES = [
        ('wait', 2.0),
        ('sweep', 9.0),
        ('pause', 1.0),
        ('ramp', 10.0),
        ('hold', 5.0),
        ('brake', 20.0),
        ('finish', None),
    ]


class StaticInspectionBHandler(ScriptedHandler):
    # Ends with an EBS test. TODO: verify sequence and values against the
    # current FS-AI rulebook
    name = 'static_inspection_B'
    TARGET_SPEED_mps = 3.0

    PHASES = [
        ('wait', 2.0),
        ('ramp', 10.0),
        ('hold', 5.0),
        ('finish_estop', None),
    ]


class LapMissionHandler(MissionHandler):
    """Dynamic events: finish after the target lap count from /laps."""
    relays_planner = True

    def __init__(self, sup, event, target_laps):
        super().__init__(sup)
        self.name = event
        self.event = event
        self.target_laps = target_laps

    def start(self, now):
        self.start_laps = self.sup.laps
        self.sup.get_logger().info(f"{self.event}: running until {self.target_laps} lap(s)")

    def tick(self, now):
        if self.done:
            return
        if self.sup.laps - self.start_laps >= self.target_laps:
            self.sup.get_logger().info(f"{self.event}: lap target reached")
            self.sup.publish_chequered_flag()
            self.done = True


class TimedRunHandler(MissionHandler):
    """Dynamic events without a usable lap count: run for a fixed time."""
    relays_planner = True

    def __init__(self, sup, event, duration):
        super().__init__(sup)
        self.name = event
        self.event = event
        self.duration = duration

    def start(self, now):
        self.start_time = now
        self.sup.get_logger().info(f"{self.event}: running for {self.duration}s")

    def tick(self, now):
        if self.done:
            return
        if now - self.start_time >= self.duration:
            self.sup.get_logger().info(f"{self.event}: time elapsed")
            self.sup.publish_chequered_flag()
            self.done = True


class MissionSupervisor(Node):
    # Constants from VCU2AI message
    AS_OFF = 1
    AS_READY = 2
    AS_DRIVING = 3
    AS_EMERGENCY_BRAKE = 4
    AS_FINISHED = 5

    AMI_NOT_SELECTED = 0
    AMI_ACCELERATION = 1
    AMI_SKIDPAD = 2
    AMI_AUTOCROSS = 3
    AMI_TRACK_DRIVE = 4
    AMI_STATIC_INSPECTION_A = 5
    AMI_STATIC_INSPECTION_B = 6
    AMI_AUTONOMOUS_DEMO = 7

    AMI_EVENT_NAMES = {
        AMI_ACCELERATION: 'acceleration',
        AMI_SKIDPAD: 'skidpad',
        AMI_AUTOCROSS: 'autocross',
        AMI_TRACK_DRIVE: 'trackdrive',
        AMI_STATIC_INSPECTION_A: 'static_inspection_A',
        AMI_STATIC_INSPECTION_B: 'static_inspection_B',
        AMI_AUTONOMOUS_DEMO: 'autonomous_demo',
    }

    DYNAMIC_AMI = (
        AMI_ACCELERATION,
        AMI_SKIDPAD,
        AMI_AUTOCROSS,
        AMI_TRACK_DRIVE,
    )

    VCU2AI_TIMEOUT_s = 0.5

    def __init__(self):
        super().__init__('mission_supervisor')

        self.declare_parameter('acceleration_run_s', 15.0)  # TODO: replace with 75m distance once odometry/pulse counts are reliable
        self.declare_parameter('skidpad_laps', 4)
        self.declare_parameter('autocross_laps', 1)
        self.declare_parameter('trackdrive_laps', 10)
        self.declare_parameter('controller_node', 'mppi_controller')
        self.declare_parameter('path_node', 'track_pathfinder')
        self.declare_parameter('perception_node', 'predict_node')
        self.declare_parameter('require_perception', True)
        self.declare_parameter('planner_cmd_topic', '/ackermann_cmd_planner')

        # Publishers (same interface as autonomous_demo.py)
        self.ackermann_publisher = self.create_publisher(AckermannDrive, '/ackermann_cmd_controller', 1)
        self.brake_publisher = self.create_publisher(Bool, '/brake', 1)
        self.emergency_brake_publisher = self.create_publisher(Bool, '/emergency_brake', 1)
        self.chequered_flag_publisher = self.create_publisher(Bool, '/chequered_flag', 1)
        self.mission_publisher = self.create_publisher(String, '/mission', 1)
        self.status_publisher = self.create_publisher(String, '/mission_supervisor/status', 1)

        # Subscribers
        self.create_subscription(VCU2AI, '/vcu2ai', self.vcu2ai_callback, 1)
        self.create_subscription(UInt16, '/laps', self.laps_callback, 1)
        planner_topic = self.get_parameter('planner_cmd_topic').value
        self.create_subscription(AckermannDrive, planner_topic, self.planner_cmd_callback, 10)

        # State from VCU2AI
        self.as_state = None
        self.ami_state = None
        self.rl_wheel_speed_rpm = None
        self.rr_wheel_speed_rpm = None
        self.laps = 0

        # Mission dispatch state
        self.active = None
        self.mission_done = False
        self.brake_engaged = False
        self.last_vcu2ai_time = None
        self.last_planner_cmd = None
        self.cmd_enabled = False

        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.health_tick)

        self.get_logger().info(
            "MissionSupervisor initialized, waiting for AS_DRIVING and a mission from the AMI. "
            f"Planner cmds on {planner_topic} are forwarded only while AS_DRIVING "
            f"(controller={self.get_parameter('controller_node').value})."
        )

    def make_handler(self, ami_state):
        if ami_state == self.AMI_ACCELERATION:
            return TimedRunHandler(self, 'acceleration', self.get_parameter('acceleration_run_s').value)
        if ami_state == self.AMI_SKIDPAD:
            return LapMissionHandler(self, 'skidpad', self.get_parameter('skidpad_laps').value)
        if ami_state == self.AMI_AUTOCROSS:
            return LapMissionHandler(self, 'autocross', self.get_parameter('autocross_laps').value)
        if ami_state == self.AMI_TRACK_DRIVE:
            return LapMissionHandler(self, 'trackdrive', self.get_parameter('trackdrive_laps').value)
        if ami_state == self.AMI_STATIC_INSPECTION_A:
            return StaticInspectionAHandler(self)
        if ami_state == self.AMI_STATIC_INSPECTION_B:
            return StaticInspectionBHandler(self)
        if ami_state == self.AMI_AUTONOMOUS_DEMO:
            return AutonomousDemoHandler(self)
        return None

    def mission_nodes(self, ami_state):
        if ami_state not in self.DYNAMIC_AMI:
            return []
        nodes = []
        if self.get_parameter('require_perception').value:
            nodes.append(self.get_parameter('perception_node').value)
        path_node = self.get_parameter('path_node').value
        if path_node:
            nodes.append(path_node)
        nodes.append(self.get_parameter('controller_node').value)
        if ami_state in (self.AMI_SKIDPAD, self.AMI_AUTOCROSS, self.AMI_TRACK_DRIVE):
            nodes.append('lap_counter')
        return nodes

    def vcu2ai_callback(self, msg):
        if self.as_state is not None and self.as_state != msg.as_state:
            self.get_logger().info(f"AS State changed to: {msg.as_state}")
        if self.ami_state is not None and self.ami_state != msg.ami_state:
            self.get_logger().info(f"AMI State changed to: {msg.ami_state}")
        self.as_state = msg.as_state
        self.ami_state = msg.ami_state
        self.rl_wheel_speed_rpm = msg.rl_wheel_speed_rpm
        self.rr_wheel_speed_rpm = msg.rr_wheel_speed_rpm
        self.last_vcu2ai_time = self.get_clock().now().nanoseconds * 1e-9

    def laps_callback(self, msg):
        self.laps = msg.data

    def planner_cmd_callback(self, msg):
        self.last_planner_cmd = msg

    def disable_drive(self):
        if self.cmd_enabled:
            self.publish_drive(0.0, 0.0)
            self.cmd_enabled = False

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # announce the selected mission as soon as it appears on the AMI, so the
        # planning/control stack knows the event before the RES go is given
        if self.ami_state in self.AMI_EVENT_NAMES:
            msg = String()
            msg.data = self.AMI_EVENT_NAMES[self.ami_state]
            self.mission_publisher.publish(msg)

        if self.as_state in (self.AS_EMERGENCY_BRAKE, self.AS_FINISHED):
            if self.active is not None:
                self.get_logger().info(f"{self.active.name}: stopped (AS state {self.as_state})")
                self.active = None
            self.disable_drive()
            self.mission_done = False  # ready for the next run after a power cycle
            return

        if self.as_state != self.AS_DRIVING:
            if self.active is not None:
                self.get_logger().warn(f"{self.active.name}: aborted (left AS_DRIVING)")
                self.active = None
            self.disable_drive()
            return

        if self.active is None and not self.mission_done:
            handler = self.make_handler(self.ami_state)
            if handler is None:
                self.disable_drive()
                return
            missing = self.missing_nodes(self.ami_state)
            if missing:
                self.get_logger().error(
                    f"not starting '{handler.name}': missing nodes: {', '.join(missing)}"
                )
                return
            self.active = handler
            self.get_logger().info(f"AS_DRIVING with AMI {self.ami_state}: starting mission '{handler.name}'")
            self.active.start(now)

        if self.active is not None:
            self.cmd_enabled = True
            self.active.tick(now)
            if self.active.done:
                self.get_logger().info(f"{self.active.name}: mission complete")
                self.active = None
                self.mission_done = True
                self.disable_drive()
            elif self.active.relays_planner and self.last_planner_cmd is not None:
                self.ackermann_publisher.publish(self.last_planner_cmd)
        else:
            self.disable_drive()

    def missing_nodes(self, ami_state):
        required = self.mission_nodes(ami_state)
        alive = set(self.get_node_names())
        return [n for n in required if n not in alive]

    def health_tick(self):
        """Status inspection: watch the CAN bridge and the nodes the selected
        mission needs, so problems are visible before the RES go is given."""
        now = self.get_clock().now().nanoseconds * 1e-9
        problems = []

        if self.last_vcu2ai_time is None:
            problems.append('no /vcu2ai received (is ackermann_can running?)')
        elif now - self.last_vcu2ai_time > self.VCU2AI_TIMEOUT_s:
            problems.append(f'/vcu2ai stale for {now - self.last_vcu2ai_time:.1f}s (AI_COMMS_LOST risk)')

        if self.ami_state:
            missing = self.missing_nodes(self.ami_state)
            if missing:
                problems.append('missing nodes: ' + ', '.join(missing))

        status = 'OK' if not problems else '; '.join(problems)
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)
        if problems:
            self.get_logger().warn(status, throttle_duration_sec=5.0)

    def avg_rear_wheel_rpm(self):
        if self.rl_wheel_speed_rpm is None or self.rr_wheel_speed_rpm is None:
            return None
        return (self.rl_wheel_speed_rpm + self.rr_wheel_speed_rpm) / 2

    def publish_drive(self, speed, steering_angle=0.0):
        # never request speed while braking (BRAKE_PLAUSIBILITY_FAULT)
        if self.brake_engaged and speed > 0.0:
            self.get_logger().warn('speed request suppressed while braking', throttle_duration_sec=5.0)
            speed = 0.0
        msg = AckermannDrive()
        msg.speed = float(speed)
        msg.steering_angle = float(steering_angle)
        self.ackermann_publisher.publish(msg)

    def publish_brake(self, on):
        self.brake_engaged = on
        msg = Bool()
        msg.data = on
        self.brake_publisher.publish(msg)

    def publish_emergency_brake(self, on):
        msg = Bool()
        msg.data = on
        self.emergency_brake_publisher.publish(msg)

    def publish_chequered_flag(self):
        msg = Bool()
        msg.data = True
        self.chequered_flag_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = MissionSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
