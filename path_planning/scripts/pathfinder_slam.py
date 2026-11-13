#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Header, ColorRGBA, UInt16
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from common_msgs.msg import ConeArray


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def to_local(points, x, y, yaw):
    """(N, 2) points from a world frame (map or odom) into the frame of the pose (x, y, yaw) given
    in that world frame: +x forward, +y left. Same rotation as fastslam_node._process_cones."""
    if len(points) == 0:
        return np.empty((0, 2))
    pts = np.asarray(points, dtype=float)
    dx = pts[:, 0] - x
    dy = pts[:, 1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((dx * c + dy * s, -dx * s + dy * c))


def to_world(points, x, y, yaw):
    """Inverse of to_local: (N, 2) points in the frame of the pose (x, y, yaw) into the world frame
    (map or odom) the pose is given in."""
    if len(points) == 0:
        return np.empty((0, 2))
    pts = np.asarray(points, dtype=float)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((x + pts[:, 0] * c - pts[:, 1] * s, y + pts[:, 0] * s + pts[:, 1] * c))


class CentrelineAlgorithm:
    """
    Centreline Pathfinder.
    geometric midpoint with path smoothing.
    """
    def __init__(self, logger, lookahead_distance: float = 20.0):
        self.logger = logger
        self.lookahead_distance = lookahead_distance
        
        self.min_x_lookahead = 0.5
        self.max_gate_distance = 8.0
        self.smoothing_window = 3
        self.cone_vertical_tolerance = 1.5 
        self.default_track_width = 3.0   # m, used until two real gates are in view
        self.lone_cone_factor = 1.35     # nearest other-colour cone further than this x track width: partner missing
        self.lone_cone_bias = 0.45       # m a one-boundary waypoint is moved from the centre towards its (mapped) boundary
        self.lone_partner_max = 14.0     # m, a far "partner" beyond this does not even tell which side the track is on
        self.lone_cone_range = 10.0      # m, cones without a partner in max_gate_distance are only used this close to the car
        self.min_cone_clearance = 0.3    # x track width: a one-boundary waypoint closer than this to a mapped cone is dropped
        self.chain_max_step = 8.0        # m between consecutive waypoints
        self.chain_max_turn = math.radians(75.0)   # largest heading change between consecutive waypoints
        
        # Last 2 Previously published /path values
        self.path_buffer : np.ndarray

    def calculate_path(self, yellow_cones, blue_cones):
        """
        Calculate centreline using geometric midpoint + smoothing
        Input: yellow_cones, blue_cones as [[x, y], ...] in base_link (car-local)
        Output: smoothed waypoints [[x, y], ...] in base_link or None
        
        Algorithm:
        1. Filter cones ahead of car
        2. Sort by forward distance
        3. Greedily pair nearest Yellow-Blue cones
        4. Compute midpoints as centreline candidates
        5. Smooth waypoints *need to check if needed*
        """
        
        if len(yellow_cones) == 0 and len(blue_cones) == 0:
            return None
        
        y_np = np.array(yellow_cones) if len(yellow_cones) > 0 else np.empty((0, 2))
        b_np = np.array(blue_cones) if len(blue_cones) > 0 else np.empty((0, 2))
        
        # Filter & sort cones ahead of car
        y_filtered = self._filter_and_sort_cones(y_np)
        b_filtered = self._filter_and_sort_cones(b_np)

        if len(y_filtered) == 0 or len(b_filtered) == 0:
            return None

        # greedy cone sorting - for each sorted yellow, find closest blue
        midpoints, gates, lone_flags = self._pair_and_midpoint(y_filtered, b_filtered)

        if len(midpoints) == 0:
            return None

        # Build waypoint list starting from car. A single gate ahead is enough to move towards:
        # on widely spaced tracks (acceleration: 5 m) the second gate is often not seen from
        # standstill, and requiring two kept the car from ever starting.
        waypoints = [np.array([0.0, 0.0])]
        waypoints.extend(self._chain_from_car(midpoints, gates))
        # One-boundary waypoints follow a single cone line, so they do not zig-zag like gate
        # midpoints do, and averaging them with their neighbours only pulls a tight corner's path
        # inwards (0.4 m in the mppi_track hairpin, onto the unseen inner cones): keep them as is.
        lone = {tuple(np.round(m, 6)) for m, l in zip(midpoints, lone_flags) if l}
        keep = [tuple(np.round(w, 6)) in lone for w in waypoints]
        if len(waypoints) < 2 or (len(waypoints) == 2 and waypoints[1][0] < 1.0):
            return None

        waypoints = self._smooth_path(waypoints, keep=keep)
        waypoints = [wp for wp in waypoints if wp[0] >= 0.0] # ahead of the car
        
        if len(waypoints) < 2:
            return None

        return np.array(waypoints)

    def _filter_and_sort_cones(self, cones):
        """Filter cones ahead of car and sort by forward distance"""
        if len(cones) == 0:
            return np.empty((0, 2))
        
        # x > min_lookahead and within lookahead distance
        mask = (cones[:, 0] > self.min_x_lookahead) & \
               (np.linalg.norm(cones, axis=1) < self.lookahead_distance)
        filtered = cones[mask]
        
        if len(filtered) == 0:
            return np.empty((0, 2))

        order = np.argsort(filtered[:, 0])
        return filtered[order]

    def _predict_next_midpoint(self) -> np.ndarray:
        """
        Takes last 2 midpoints gives next midpoint, assuming no directional change
        """
        if len(self.path_buffer) < 2:
            return errno.EINVAL
        p0 = self.path_buffer[0]
        p1 = self.path_buffer[1]
        return 2*p0 - p1
            

    def _pair_and_midpoint(self, yellow_cones, blue_cones):
        """
        Pair yellow and blue cones to form gates.
        Strategy: each yellow cone with its closest blue cone AND each blue cone with its
        closest yellow cone, by plain distance (cones may be reused), then drop duplicates.
        Returns (midpoints, gates, lone): the waypoints, each one's yellow -> blue gate vector (for
        the direction check in _chain_from_car) and whether it was placed from one boundary only.

        The old version paired each yellow with the closest *unused* blue at the same forward
        x (|dx| < cone_vertical_tolerance). In corners the gate cones are not side by side in
        the car frame, so it paired nothing (or zig-zagged, leaving the outer cones unused) and
        the car stopped mid-corner.
        """
        midpoints = []
        gates = []      # (yellow -> blue) vector of each midpoint's gate
        lone_mids = []  # per midpoint: placed from one boundary only (see below)
        width = self._track_width(yellow_cones, blue_cones)
        for side, other, is_yellow in ((yellow_cones, blue_cones, True), (blue_cones, yellow_cones, False)):
            if len(other) == 0:
                continue
            for c in side:
                dists = np.linalg.norm(other - c, axis=1)
                j = int(np.argmin(dists))
                # no gate partner within max_gate_distance: still usable as a one-boundary waypoint
                # (below) if it is near the car; the far cone then only tells which side the track is on
                far = dists[j] >= self.max_gate_distance
                if far and (dists[j] > self.lone_partner_max or np.linalg.norm(c) > self.lone_cone_range):
                    continue
                across = other[j] - c
                lone = False
                if dists[j] > self.lone_cone_factor * width:
                    # The closest cone of the other colour is much further than the track is wide:
                    # this cone's partner is not in the map (inner cones of a hairpin are outside
                    # the camera's and the lidar's view when the car passes them). The midpoint of
                    # such a skewed "gate" lies on the unseen boundary, so place the waypoint half
                    # a track width from this cone, square to its own boundary line instead.
                    normal = self._boundary_normal(c, side, across)
                    if normal is not None:
                        # stay a little closer to the boundary that is mapped (the controller's cone
                        # cost guards that one; nothing guards the unseen one)
                        across = normal * (width - 2.0 * self.lone_cone_bias)
                        # a waypoint this close to a mapped cone is not on the track: the
                        # "boundary line" was bent by a ghost or duplicate landmark
                        mid = c + across / 2.0
                        nearest = min(np.linalg.norm(side - mid, axis=1).min(), np.linalg.norm(other - mid, axis=1).min())
                        if nearest < self.min_cone_clearance * width:
                            continue
                        lone = True
                    elif far:
                        continue
                mid = c + across / 2.0
                if all(np.linalg.norm(mid - m) > 0.5 for m in midpoints):
                    midpoints.append(mid)
                    gates.append(across if is_yellow else -across)
                    lone_mids.append(lone)

        return midpoints, gates, lone_mids

    def _track_width(self, yellow_cones, blue_cones):
        """Track width from the cones in view: the lower quartile of every cone's distance to the
        closest cone of the other colour (2.5-6 m), else the default.

        Cones whose partner is missing or that pair diagonally are further from the other colour
        than the track is wide, never closer, so the low end of the distribution is the width (the
        median of the mutually-closest pairs gave 5.5 m at the hairpin and put one-boundary
        waypoints 2.5 m from the cone line, next to the other boundary)."""
        if len(yellow_cones) + len(blue_cones) < 6 or len(yellow_cones) == 0 or len(blue_cones) == 0:
            return self.default_track_width
        d = np.linalg.norm(yellow_cones[:, None, :] - blue_cones[None, :, :], axis=2)
        nearest = np.concatenate([d.min(axis=1), d.min(axis=0)])
        return float(np.clip(np.percentile(nearest, 25), 2.5, 6.0))

    def _boundary_normal(self, cone, same, towards, max_neighbour=6.0):
        """Unit vector square to the boundary line through `cone` (direction from its nearest
        same-colour neighbours), on the side `towards` points to. None if the cone has no neighbour."""
        d = np.linalg.norm(same - cone, axis=1)
        near = [i for i in np.argsort(d) if 1.0 < d[i] < max_neighbour][:2]   # > 1 m: not a duplicate of this cone
        if not near:
            return None
        if len(near) == 2 and np.dot(same[near[0]] - cone, same[near[1]] - cone) < 0:
            tangent = same[near[1]] - same[near[0]]      # one neighbour on each side
        else:
            tangent = same[near[0]] - cone
        n = np.array([-tangent[1], tangent[0]]) / (np.linalg.norm(tangent) + 1e-9)
        return n if np.dot(n, towards) >= 0 else -n

    def _chain_from_car(self, midpoints, gates):
        """Order midpoints by walking from the car to the nearest next one.

        Sorting by forward x breaks in hairpins (the track comes back towards the car). Each step
        must be shorter than max_step and turn less than max_turn, so midpoints from another part
        of the track are not joined in. `gates` holds each midpoint's yellow -> blue vector.
        """
        max_step, max_turn = self.chain_max_step, self.chain_max_turn
        remaining = [(np.asarray(m, dtype=float), np.asarray(g, dtype=float)) for m, g in zip(midpoints, gates)]
        pos, heading = np.zeros(2), 0.0
        chain = []
        while remaining:
            best, best_d = None, max_step
            for i, (m, gate) in enumerate(remaining):
                d = np.linalg.norm(m - pos)
                if d < 1e-3 or d >= best_d:
                    continue
                turn = math.atan2(m[1] - pos[1], m[0] - pos[0]) - heading
                if abs(math.atan2(math.sin(turn), math.cos(turn))) >= max_turn:
                    continue
                # Driving direction: blue on the left, yellow on the right. The step must cross the
                # gate that way (sin of the angle from step to yellow->blue > 0.1), which rules out
                # driving the track backwards and "gates" made across the infield. (0.5 rejected
                # valid steps near hairpin apexes, where the next gate is crossed at a shallow angle.)
                step = m - pos
                if (step[0] * gate[1] - step[1] * gate[0]) / (np.linalg.norm(step) * np.linalg.norm(gate) + 1e-9) > 0.1:
                    best, best_d = i, d
            if best is None:
                break
            m, _ = remaining.pop(best)
            heading = math.atan2(m[1] - pos[1], m[0] - pos[0])
            pos = m
            chain.append(m)
        return chain

    def _smooth_path(self, waypoints, window_size=None, keep=None):
        """
        Smooth waypoints using moving average filter (waypoints flagged in `keep` are not moved)
        """
        if window_size is None:
            window_size = self.smoothing_window
        
        if len(waypoints) <= window_size:
            return waypoints
        
        waypoints = np.array(waypoints)
        smoothed = [waypoints[0]]
        
        for i in range(1, len(waypoints) - 1):
            start = max(0, i - window_size // 2)
            end = min(len(waypoints), i + window_size // 2 + 1)
            
            window = waypoints[start:end]
            smoothed_point = waypoints[i] if keep is not None and keep[i] else np.mean(window, axis=0)
            smoothed.append(smoothed_point)
        
        smoothed.append(waypoints[-1])
        return smoothed

class SkidpadPlanner:
    """Skidpad: entry, right circle x2, left circle x2, exit (FS rules), anchored on the cones.

    Layout from the rules: circle centres 9.125 m either side of the timing line centre, inner
    cone radius 7.625 m, outer 10.625 m. The timing line centre is first the mean of the large
    orange cones in the SLAM map, then refined by fitting all mapped blue/yellow cones to the two
    circles. The driving direction is the car's initial heading (map +x): the car starts on the
    entry lane facing the timing line. Progress along the path only moves forward (searched in a
    small window), so the crossing in the middle of the figure 8 can't make it skip or repeat.
    """
    R_CENTRE, R_INNER, R_OUTER = 9.125, 7.625, 10.625

    def __init__(self, logger, start_distance=15.0, spacing=0.5, window=20.0, exit_length=25.0):
        self.logger = logger
        self.spacing = spacing
        self.window = int(window / spacing)
        self.centre = np.array([start_distance, 0.0])  # timing line centre (map), until cones are mapped
        self.anchored = False
        self.frozen = False
        self.idx = 0
        # path in the skidpad frame: origin = timing line centre, +x = driving direction, +y = left
        pts = [np.array([x, 0.0]) for x in np.arange(-start_distance - 5.0, 0.0, spacing)]
        n = int(2 * math.pi * self.R_CENTRE / spacing)
        for lap in range(2):  # right circle (clockwise), centre (0, -R)
            for k in range(n):
                phi = math.pi / 2 - 2 * math.pi * k / n
                pts.append(np.array([self.R_CENTRE * math.cos(phi), -self.R_CENTRE + self.R_CENTRE * math.sin(phi)]))
        for lap in range(2):  # left circle (counter-clockwise), centre (0, +R)
            for k in range(n):
                phi = -math.pi / 2 + 2 * math.pi * k / n
                pts.append(np.array([self.R_CENTRE * math.cos(phi), self.R_CENTRE + self.R_CENTRE * math.sin(phi)]))
        pts += [np.array([x, 0.0]) for x in np.arange(0.0, exit_length, spacing)]
        self.path = np.array(pts)
        self.entry_end = int((start_distance + 5.0) / spacing)

    def update_anchor(self, orange, blue, yellow):
        """Locate the timing line centre from the mapped cones (map frame)."""
        cones = np.array(list(blue) + list(yellow)) if len(blue) + len(yellow) else np.empty((0, 2))
        if self.frozen or len(cones) < 10:
            return
        # Start from the large orange cones if there are some, but let the circles decide: the
        # orange mean alone was 6 m off (small entry-lane cones taken for large ones), so search
        # +-8 m along the driving direction and +-2 m across.
        guess = np.mean(orange, axis=0) if len(orange) >= 2 else self.centre.copy()
        best = guess
        if len(cones) >= 10:
            best_err = float('inf')
            for dx in np.arange(-8.0, 8.01, 0.25):
                for dy in np.arange(-2.0, 2.01, 0.25):
                    c = guess + [dx, dy]
                    d = np.minimum(np.linalg.norm(cones - (c + [0, -self.R_CENTRE]), axis=1),
                                   np.linalg.norm(cones - (c + [0, self.R_CENTRE]), axis=1))
                    r = np.minimum(np.abs(d - self.R_INNER), np.abs(d - self.R_OUTER))
                    err = np.sum(np.minimum(r, 1.0) ** 2)  # robust to stray cones
                    if err < best_err:
                        best, best_err = c, err
        self.centre = best if not self.anchored else 0.8 * self.centre + 0.2 * best
        if not self.anchored:
            self.logger.info(f"[SKIDPAD] timing line found at map ({best[0]:.2f}, {best[1]:.2f})")
        self.anchored = True

    def plan(self, car_map_xy):
        """Path window ahead of the car in the map frame, and whether the skidpad is done."""
        local = self.path + self.centre
        lo, hi = max(0, self.idx - 4), min(len(local), self.idx + self.window)
        self.idx = lo + int(np.argmin(np.linalg.norm(local[lo:hi] - car_map_xy, axis=1)))
        if self.idx > self.entry_end and self.anchored and not self.frozen:
            self.frozen = True  # don't move the circles under the car
            self.logger.info("[SKIDPAD] entered the circles, layout frozen")
        done = self.idx >= len(local) - 4
        return local[self.idx:self.idx + self.window], done


class CentrelineTrackPathfinder(Node):
    def __init__(self):
        super().__init__('centreline_track_pathfinder')

        self.output_frame = self.declare_parameter('output_frame', 'odom').value
        self.cone_map_topic = self.declare_parameter('cone_map_topic', '/slam/cone_map').value
        lookahead = self.declare_parameter('lookahead_distance', 20.0).value
        self.test_mode = bool(self.declare_parameter('test_mode', True).value)
        self.viz_frame = self.declare_parameter('viz_frame', 'map').value
        self.marker_scale = float(self.declare_parameter('marker_scale', 0.25).value)
        self.line_width = float(self.declare_parameter('line_width', 0.15).value)

        self.slam_pose = None
        self.odom_pose = None
        self.odom_hist = deque(maxlen=300)
        self.last_good = None       # (time, path in odom) of the last planned path
        self.correction = None      # smoothed map -> odom correction (x, y, yaw)
        self.correction_t = None
        self.correction_tau = float(self.declare_parameter('correction_tau', 1.0).value)  # s
        self.last_odom_stamp = None

        self.create_subscription(ConeArray, self.cone_map_topic, self.cone_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/slam/pose', self.slam_pose_callback, 10)

        # Remembered lap (opt-in, remember_lap:=true; closed circuits only). The line the car drove on
        # lap 1 is stored in the map frame; from lap 2 on it is the path, as long as the car is on it
        # and it stays clear of every cone the car can see now. Otherwise the live planner takes over
        # for that update (track changed, a cone was moved, or the car is not where it thinks it is).
        self.remember_lap = bool(self.declare_parameter('remember_lap', False).value)
        self.remember_min_clear = float(self.declare_parameter('remember_min_clear', 0.75).value)   # m, path centre to a cone
        self.remember_max_offset = float(self.declare_parameter('remember_max_offset', 1.5).value)  # m, car to the stored line
        self.laps = 0
        self.lap_trace = []         # map-frame points driven so far (lap 1)
        self.lap_loop = None        # (N, 2) closed loop in the map frame, 0.5 m spacing
        self.lap_idx = 0
        self.lap_mode = None        # 'remembered' / 'live', for the log
        self.lidar_cones_odom = np.empty((0, 2))
        if self.remember_lap:
            self.create_subscription(UInt16, '/laps', self.laps_callback, 10)
            self.create_subscription(PointCloud2, '/lidar_cones', self.lidar_cones_callback, 1)
            self.get_logger().info("PATHFINDER: remember_lap on: lap 1 is recorded and followed from lap 2")

        self.get_logger().info(
            f"PATHFINDER: map cones -> base_link planning -> {self.output_frame} path"
        )
        if self.test_mode:
            self.get_logger().info(
                f"PATHFINDER: test_mode enabled, publishing /viz/path and "
                f"/viz/path_markers in {self.viz_frame}"
            )

        self.centreline_planner = CentrelineAlgorithm(self.get_logger(), lookahead_distance=lookahead)
        # tuning of the centreline planner (defaults = the values in CentrelineAlgorithm) and of the recovery arc
        cp = self.centreline_planner
        for name in ('max_gate_distance', 'default_track_width', 'lone_cone_factor', 'lone_cone_bias',
                     'lone_partner_max', 'lone_cone_range', 'min_cone_clearance', 'chain_max_step'):
            setattr(cp, name, float(self.declare_parameter(name, getattr(cp, name)).value))
        cp.chain_max_turn = math.radians(float(self.declare_parameter('chain_max_turn_deg', math.degrees(cp.chain_max_turn)).value))
        self.recovery_radius = float(self.declare_parameter('recovery_radius', 4.5).value)   # m
        self.recovery_length = float(self.declare_parameter('recovery_length', 5.0).value)   # m
        self.event = self.declare_parameter('event', 'trackdrive').value
        self.skidpad = SkidpadPlanner(self.get_logger()) if self.event == 'skidpad' else None
        self.done_pub = self.create_publisher(Bool, '/mission/path_complete', 1)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.viz_path_pub = self.create_publisher(Path, '/viz/path', 10) if self.test_mode else None
        self.marker_pub = self.create_publisher(MarkerArray, '/viz/path_markers', 1) if self.test_mode else None

    def slam_pose_callback(self, msg):
        """Update the (smoothed) map -> odom correction from a SLAM pose.

        The particle filter's pose estimate jumps by tens of cm between updates. Planning from it
        directly moved the whole path by that much every 0.1 s and the controller chased it. Like
        REP 105's map -> odom, the correction (SLAM pose relative to odometry at the same time) is
        low-pass filtered and the car is placed in the map with smooth odometry on top of it.
        """
        q = msg.pose.orientation
        slam = (msg.pose.position.x, msg.pose.position.y, quaternion_to_yaw(q.x, q.y, q.z, q.w))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.odom_hist:
            return
        ts = [h[0] for h in self.odom_hist]
        odom = self.odom_hist[min(range(len(ts)), key=lambda i: abs(ts[i] - t))][1:]
        # correction T with slam = T (+) odom
        dyaw = slam[2] - odom[2]
        c, s = math.cos(dyaw), math.sin(dyaw)
        corr = np.array([slam[0] - (c * odom[0] - s * odom[1]), slam[1] - (s * odom[0] + c * odom[1]), dyaw])
        if self.correction is None:
            self.correction = corr
        else:
            a = min(1.0, max(0.0, t - self.correction_t) / self.correction_tau) if self.correction_t else 1.0
            d = corr - self.correction
            d[2] = math.atan2(math.sin(d[2]), math.cos(d[2]))
            self.correction = self.correction + a * d
        self.correction_t = t
        self.slam_pose = slam

    def laps_callback(self, msg):
        self.laps = int(msg.data)

    def lidar_cones_callback(self, msg):
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y'), skip_nans=True).reshape(-1, 2)
        self.lidar_cones_odom = pts.astype(float)

    def record_lap(self, rx, ry):
        """Lap 1: store where the car drives (map frame, a point every 0.5 m). When the first lap is
        counted, close the loop where the trace comes back to its own beginning."""
        if self.lap_loop is not None or self.lap_trace is None:
            return
        if not self.lap_trace or math.hypot(rx - self.lap_trace[-1][0], ry - self.lap_trace[-1][1]) >= 0.5:
            self.lap_trace.append((rx, ry))
        if self.laps < 1 or len(self.lap_trace) < 60:
            return
        pts = np.array(self.lap_trace)
        head = pts[:max(20, len(pts) // 5)]                       # where the lap began
        d = np.hypot(*(head - pts[-1]).T); j = int(np.argmin(d))
        if d[j] > 2.0:
            if len(pts) > 4000:                                    # never came back: give up
                self.get_logger().warn("[PATHFINDER] remember_lap: the lap did not close, staying with live planning")
                self.lap_trace = None
            return
        loop = pts[j:]
        # light smoothing of the driven line (closed loop, +-1 m), it is followed as it is otherwise
        k = 2; pad = np.vstack([loop[-k:], loop, loop[:k]])
        self.lap_loop = np.array([pad[i:i + 2 * k + 1].mean(axis=0) for i in range(len(loop))])
        self.lap_idx = 0
        self.get_logger().info(f"[PATHFINDER] remember_lap: lap 1 stored, {len(loop)} points, "
                               f"{np.hypot(*np.diff(loop, axis=0).T).sum():.0f} m, closes within {d[j]:.2f} m")

    def remembered_path(self, rx, ry, r_yaw, cones_local, length=20.0):
        """The stored lap ahead of the car in base_link, or None if it cannot be trusted right now."""
        if self.lap_loop is None:
            return None
        n = len(self.lap_loop)
        win = (self.lap_idx + np.arange(-6, 40)) % n               # only look around the last position
        d = np.hypot(self.lap_loop[win, 0] - rx, self.lap_loop[win, 1] - ry)
        if d.min() > self.remember_max_offset:                    # lost it: search the whole loop once
            d_all = np.hypot(self.lap_loop[:, 0] - rx, self.lap_loop[:, 1] - ry)
            if d_all.min() > self.remember_max_offset:
                return self._lap_mode('live', f"car is {d_all.min():.1f} m from the stored lap")
            self.lap_idx = int(np.argmin(d_all))
        else:
            self.lap_idx = int(win[int(np.argmin(d))])
        ahead = self.lap_loop[(self.lap_idx + 2 + np.arange(int(length / 0.5))) % n]
        local = to_local(ahead, rx, ry, r_yaw)
        gap = np.where(np.hypot(*np.diff(local, axis=0).T) > 2.0)[0]
        if len(gap):
            local = local[:gap[0] + 1]                           # never bridge a hole in the stored line
        if len(local) < 8 or local[0][0] < -0.5:
            return self._lap_mode('live', "stored lap is not ahead of the car")
        cones = [np.asarray(c, dtype=float).reshape(-1, 2) for c in cones_local if len(c)]
        if len(self.lidar_cones_odom) and self.odom_pose is not None:
            cones.append(to_local(self.lidar_cones_odom, *self.odom_pose))   # odom -> base_link
        if cones:
            c = np.vstack(cones); c = c[(c[:, 0] > -1.0) & (np.hypot(c[:, 0], c[:, 1]) < length + 2.0)]
            if len(c):
                clear = np.hypot(local[:, None, 0] - c[None, :, 0], local[:, None, 1] - c[None, :, 1]).min()
                if clear < self.remember_min_clear:
                    return self._lap_mode('live', f"a cone is {clear:.2f} m from the stored lap")
        self._lap_mode('remembered', "following the stored lap")
        return np.vstack([[0.0, 0.0], local])

    def _lap_mode(self, mode, why):
        if mode != self.lap_mode:
            self.lap_mode = mode
            self.get_logger().info(f"[PATHFINDER] remember_lap -> {mode}: {why}")
        return None

    def car_map_pose(self):
        """Car pose in the map: smoothed correction (+) latest odometry."""
        tx, ty, tyaw = self.correction
        ox, oy, oyaw = self.odom_pose
        c, s = math.cos(tyaw), math.sin(tyaw)
        return (tx + c * ox - s * oy, ty + s * ox + c * oy, math.atan2(math.sin(tyaw + oyaw), math.cos(tyaw + oyaw)))

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )
        self.last_odom_stamp = msg.header.stamp
        self.odom_hist.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,) + self.odom_pose)

    def cone_callback(self, msg):
        if self.correction is None or self.odom_pose is None:
            self.get_logger().warn("[PATHFINDER] Waiting for /slam/pose and /odom", throttle_duration_sec=5.0)
            return

        if msg.header.frame_id != 'map':
            self.get_logger().warn(
                f"[PATHFINDER] Expected cone_map in map frame, got '{msg.header.frame_id}'",
                throttle_duration_sec=5.0,
            )

        yellow_map = [[c.position.x, c.position.y] for c in msg.yellow_cones]
        blue_map = [[c.position.x, c.position.y] for c in msg.blue_cones]

        rx, ry, r_yaw = self.car_map_pose()
        if self.skidpad is not None:
            self.plan_skidpad(msg, yellow_map, blue_map)
            return
        yellow_local = to_local(yellow_map, rx, ry, r_yaw)
        blue_local = to_local(blue_map, rx, ry, r_yaw)
        # Orange cones (start/finish line, acceleration braking zone) mark the lane on both sides
        # and only where it is straight: put each on the side of the car it is on. Without them the
        # path ended at the last blue/yellow gate before the acceleration finish line.
        orange_map = [[c.position.x, c.position.y] for c in list(msg.orange_cones) + list(msg.large_orange_cones)]
        if orange_map:
            orange_local = to_local(orange_map, rx, ry, r_yaw)
            yellow_local = np.vstack([yellow_local.reshape(-1, 2), orange_local[orange_local[:, 1] < 0]])
            blue_local = np.vstack([blue_local.reshape(-1, 2), orange_local[orange_local[:, 1] >= 0]])

        self.get_logger().debug(
            f"[PATHFINDER INPUT] Yellow: {len(yellow_local)}, Blue: {len(blue_local)}, "
            f"cone_map frame: {msg.header.frame_id}"
        )
        if len(yellow_local) > 0:
            self.get_logger().debug(
                f"  Yellow local - X: [{yellow_local[:, 0].min():.2f}, {yellow_local[:, 0].max():.2f}], "
                f"Y: [{yellow_local[:, 1].min():.2f}, {yellow_local[:, 1].max():.2f}]"
            )
        if len(blue_local) > 0:
            self.get_logger().debug(
                f"  Blue local - X: [{blue_local[:, 0].min():.2f}, {blue_local[:, 0].max():.2f}], "
                f"Y: [{blue_local[:, 1].min():.2f}, {blue_local[:, 1].max():.2f}]"
            )

        try:
            path_local = None
            if self.remember_lap:
                self.record_lap(rx, ry)
                path_local = self.remembered_path(rx, ry, r_yaw, (yellow_local, blue_local))
            if path_local is None:
                path_local = self.centreline_planner.calculate_path(
                    yellow_local.tolist(), blue_local.tolist()
                )
            ox, oy, o_yaw = self.odom_pose
            now = self.get_clock().now().nanoseconds * 1e-9
            path_out = None
            if path_local is not None:
                path_out = to_world(path_local, ox, oy, o_yaw)
                self.last_good = (now, path_out)
            else:
                # One map update without a gate pairing (a ghost or mislabelled cone breaks the chain
                # for a moment): keep driving the last good path, which is in odom and so still
                # valid, as long as it is recent and the car is on it. Before, the 5 m recovery arc
                # was published at once and the controller slowed to a crawl on it.
                path_out = self.held_path(ox, oy, now)
                if path_out is None:
                    path_local = self.recovery_arc(yellow_local, blue_local)
                    if path_local is not None:
                        path_out = to_world(path_local, ox, oy, o_yaw)

            if path_out is not None:
                self.get_logger().debug(
                    f"[PATHFINDER OUTPUT] {len(path_out)} waypoints in {self.output_frame}"
                )
                self.get_logger().debug(
                    f"  Range - X: [{path_out[:, 0].min():.2f}, {path_out[:, 0].max():.2f}], "
                    f"Y: [{path_out[:, 1].min():.2f}, {path_out[:, 1].max():.2f}]"
                )
                self.get_logger().debug(
                    f"  First 5: {[f'({p[0]:.2f}, {p[1]:.2f})' for p in path_out[:5]]}"
                )
                self.publish_path(path_out, self.output_frame, msg.header.stamp)
                if self.test_mode:
                    # path_out is in odom; back to the car frame, then into the map for RViz
                    path_viz = to_world(to_local(path_out, ox, oy, o_yaw), rx, ry, r_yaw)
                    self.publish_viz(path_viz, self.viz_frame, msg.header.stamp)
            else:
                self.get_logger().warn(
                    "[PATHFINDER OUTPUT] No valid path generated (insufficient cones or waypoints)",
                    throttle_duration_sec=2.0,
                )
        except Exception as e:
            self.get_logger().error(f"Planning Error: {e}")
    
    def held_path(self, ox, oy, now, max_age=2.0, max_offset=1.5, min_ahead=4.0):
        """The last good path from the car's nearest point on, if it is recent, the car is on it
        and at least min_ahead m of it are left; otherwise None."""
        if self.last_good is None or now - self.last_good[0] > max_age:
            return None
        pts = self.last_good[1]
        d = np.hypot(pts[:, 0] - ox, pts[:, 1] - oy)
        i = int(np.argmin(d))
        if d[i] > max_offset:
            return None
        rest = np.vstack([[ox, oy], pts[i + 1:]])
        if np.sum(np.hypot(*np.diff(rest, axis=0).T)) < min_ahead:
            return None
        self.get_logger().info("[PATHFINDER] no gate this update: holding the last path", throttle_duration_sec=2.0)
        return rest

    def recovery_arc(self, yellow_local, blue_local):
        """No gate ahead, but one boundary is: turn towards the other one.

        Blue cones are the left edge, so if only blue is ahead the track lies to the right (and to
        the left if only yellow is). Seen in sim: the car under-turned in a corner on lap 1 and
        stopped facing the outer cones; with only blue in view no gate could ever be formed.
        """
        def ahead(c):
            c = np.asarray(c).reshape(-1, 2)
            return int(np.sum((c[:, 0] > 0.5) & (np.linalg.norm(c, axis=1) < 12.0)))
        n_y, n_b = ahead(yellow_local), ahead(blue_local)
        if (n_y == 0) == (n_b == 0):
            return None  # nothing ahead, or both colours (then the planner's failure is something else)
        side = -1.0 if n_b > 0 else 1.0  # right if we only see blue
        radius, length = self.recovery_radius, self.recovery_length
        s = np.arange(0.0, length + 1e-9, 0.5)
        self.get_logger().warn(
            f"[PATHFINDER] only {'blue' if n_b else 'yellow'} cones ahead: turning {'right' if side < 0 else 'left'} to find the track",
            throttle_duration_sec=2.0)
        return np.column_stack((radius * np.sin(s / radius), side * radius * (1.0 - np.cos(s / radius))))

    def plan_skidpad(self, msg, yellow_map, blue_map):
        orange_map = [[c.position.x, c.position.y] for c in msg.large_orange_cones]
        self.skidpad.update_anchor(orange_map, blue_map, yellow_map)
        rx, ry, r_yaw = self.car_map_pose()
        window, done = self.skidpad.plan(np.array([rx, ry]))
        if done:
            self.done_pub.publish(Bool(data=True))
        if len(window) < 2:
            return
        # same map -> base_link -> odom chain as the centreline path
        path_local = to_local(window, rx, ry, r_yaw)
        ox, oy, o_yaw = self.odom_pose
        self.publish_path(to_world(path_local, ox, oy, o_yaw), self.output_frame, msg.header.stamp)
        if self.test_mode:
            self.publish_viz(window, self.viz_frame, msg.header.stamp)

    def _path_stamp(self, preferred_stamp):
        if preferred_stamp.sec != 0 or preferred_stamp.nanosec != 0:
            return preferred_stamp
        if self.last_odom_stamp is not None:
            return self.last_odom_stamp
        return self.get_clock().now().to_msg()

    def _make_path_msg(self, waypoints, frame_id, stamp):
        ros_path = Path()
        ros_path.header.stamp = self._path_stamp(stamp)
        ros_path.header.frame_id = frame_id
        for point in waypoints:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            ros_path.poses.append(pose)
        return ros_path

    def publish_path(self, waypoints, frame_id, stamp):
        self.path_pub.publish(self._make_path_msg(waypoints, frame_id, stamp))

    def publish_viz(self, waypoints, frame_id, stamp):
        ros_path = self._make_path_msg(waypoints, frame_id, stamp)
        self.viz_path_pub.publish(ros_path)
        self.publish_path_markers(ros_path)

    def publish_path_markers(self, ros_path: Path):
        stamp = ros_path.header.stamp
        frame_id = ros_path.header.frame_id
        arr = MarkerArray()

        delete_all = Marker()
        delete_all.header.frame_id = frame_id
        delete_all.header.stamp = stamp
        delete_all.ns = 'pathfinder'
        delete_all.id = 0
        delete_all.action = Marker.DELETEALL
        arr.markers.append(delete_all)

        points = Marker()
        points.header.frame_id = frame_id
        points.header.stamp = stamp
        points.ns = 'pathfinder'
        points.id = 1
        points.type = Marker.SPHERE_LIST
        points.action = Marker.ADD
        points.scale.x = self.marker_scale
        points.scale.y = self.marker_scale
        points.scale.z = self.marker_scale
        points.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.9)
        for pose in ros_path.poses:
            pt = Point()
            pt.x = pose.pose.position.x
            pt.y = pose.pose.position.y
            pt.z = 0.1
            points.points.append(pt)
        arr.markers.append(points)

        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = stamp
        line.ns = 'pathfinder'
        line.id = 2
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = self.line_width
        line.color = ColorRGBA(r=0.9, g=0.2, b=0.9, a=0.9)
        for pose in ros_path.poses:
            pt = Point()
            pt.x = pose.pose.position.x
            pt.y = pose.pose.position.y
            pt.z = 0.1
            line.points.append(pt)
        arr.markers.append(line)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = CentrelineTrackPathfinder()
    if not node.get_parameter('use_sim_time').value:
        node.get_logger().warn(
            "use_sim_time is false; in Gazebo set --ros-args -p use_sim_time:=true "
            "and launch RViz the same way to avoid TF timestamp errors"
        )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()