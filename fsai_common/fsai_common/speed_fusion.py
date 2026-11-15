"""Longitudinal speed from wheel speed and IMU acceleration (no ROS imports).

All four wheels are driven and braked, so the wheel speed is not the car's speed while a tyre spins
up or locks. The fused speed is the IMU's longitudinal acceleration integrated and pulled towards
the wheel speed; how hard it is pulled depends on how far the two are apart.
"""
import math


class SpeedFusion:
    """update(v_wheels, a_long, dt) once per IMU sample; the result is in .v (m/s) and .slipping.

    speed_tc          s, pull towards the wheels while they agree with the IMU
    slip_threshold    m/s, |wheels - fused| above this: the wheels are slipping
    slip_tc           s, the (slow) pull towards the wheels while they slip
    max_imu_gap       s, a longer gap between samples is not integrated: the speed is the wheels'
    locked_min_speed  m/s, wheels at 0 while the fused speed is above this: locked wheels, car sliding
    locked_tc         s, pull towards 0 while the wheels are locked and the car feels stopped
    stopped_accel     m/s^2, |a_long| below this: the car feels no deceleration any more
    """

    def __init__(self, speed_tc=0.3, slip_threshold=0.5, slip_tc=5.0, max_imu_gap=0.1,
                 locked_min_speed=0.5, locked_tc=2.0, stopped_accel=0.5):
        self.speed_tc, self.slip_threshold, self.slip_tc = speed_tc, slip_threshold, slip_tc
        self.max_imu_gap = max_imu_gap
        self.locked_min_speed, self.locked_tc, self.stopped_accel = locked_min_speed, locked_tc, stopped_accel
        self.v = 0.0
        self.slipping = False

    def update(self, v_wheels, a_long, dt):
        """v_wheels: rear axle wheel speed (m/s). a_long: bias-corrected longitudinal acceleration of
        the rear axle (m/s^2). dt: time since the previous IMU sample (s).
        Returns what happened, for logging: None, 'gap', 'locked' or 'slip' (only on the first step
        of a locked / slipping phase)."""
        if dt > self.max_imu_gap:
            # One IMU sample cannot stand for a long gap: integrating a single braking sample over a
            # 1 s stall of the data took 3.5 m/s off the fused speed, the slip logic then distrusted
            # the (correct) wheels for seconds and SLAM ended 7-11 m off. Take the wheels as they are.
            self.v, self.slipping = v_wheels, False
            return 'gap'
        if abs(v_wheels) < 1e-3:
            if self.v > self.locked_min_speed:
                # Wheels read 0 but the car was moving a moment ago: locked wheels, the car slides.
                # While it still feels deceleration the IMU alone carries the speed (an extra decay
                # here would brake on top of what the IMU already measures and cut the slide short).
                # Once it feels stopped, pull towards 0: accelerometer bias must not keep a speed.
                was_slipping, self.slipping = self.slipping, True
                self.v += a_long * dt
                if abs(a_long) < self.stopped_accel:
                    self.v *= math.exp(-dt / self.locked_tc)
                self.v = max(0.0, self.v)
                return None if was_slipping else 'locked'
            self.v, self.slipping = 0.0, False      # wheels stopped: standing
            return None
        self.v += a_long * dt
        innovation = v_wheels - self.v
        if abs(innovation) > self.slip_threshold:
            was_slipping, self.slipping = self.slipping, True
            self.v += (1.0 - math.exp(-dt / self.slip_tc)) * innovation
            return None if was_slipping else 'slip'
        self.slipping = False
        self.v += (1.0 - math.exp(-dt / self.speed_tc)) * innovation
        return None
