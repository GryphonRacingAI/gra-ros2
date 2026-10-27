#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from common_msgs.msg import ConeArray


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def map_to_base_link(points, rx, ry, yaw):
    """Transform (N, 2) points from map frame to base_link (car-local).

    Car pose (rx, ry, yaw) must be in map. +x is forward, +y is left.
    Uses the same rotation as fastslam_node._process_cones.
    """
    if len(points) == 0:
        return np.empty((0, 2))
    pts = np.asarray(points, dtype=float)
    dx = pts[:, 0] - rx
    dy = pts[:, 1] - ry
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((dx * c + dy * s, -dx * s + dy * c))


def base_link_to_odom(points, ox, oy, yaw):
    """Transform (N, 2) points from base_link to odom frame.

    Car pose (ox, oy, yaw) is from /odom. Output matches MPPI vehicle_state frame.
    """
    if len(points) == 0:
        return np.empty((0, 2))
    pts = np.asarray(points, dtype=float)
    c, s = math.cos(yaw), math.sin(yaw)
    out_x = ox + pts[:, 0] * c - pts[:, 1] * s
    out_y = oy + pts[:, 0] * s + pts[:, 1] * c
    return np.column_stack((out_x, out_y))


def base_link_to_map(points, rx, ry, yaw):
    """Transform (N, 2) points from base_link to map frame using /slam/pose."""
    if len(points) == 0:
        return np.empty((0, 2))
    pts = np.asarray(points, dtype=float)
    c, s = math.cos(yaw), math.sin(yaw)
    out_x = rx + pts[:, 0] * c - pts[:, 1] * s
    out_y = ry + pts[:, 0] * s + pts[:, 1] * c
    return np.column_stack((out_x, out_y))


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
        midpoints = self._pair_and_midpoint(y_filtered, b_filtered)

        if len(midpoints) == 0:
            return None

        # Build waypoint list starting from car. A single gate ahead is enough to move towards:
        # on widely spaced tracks (acceleration: 5 m) the second gate is often not seen from
        # standstill, and requiring two kept the car from ever starting.
        waypoints = [np.array([0.0, 0.0])]
        waypoints.extend(self._chain_from_car(midpoints))
        if len(waypoints) < 2 or (len(waypoints) == 2 and waypoints[1][0] < 1.0):
            return None

        waypoints = self._smooth_path(waypoints)
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
        Returns list of midpoint waypoints.

        The old version paired each yellow with the closest *unused* blue at the same forward
        x (|dx| < cone_vertical_tolerance). In corners the gate cones are not side by side in
        the car frame, so it paired nothing (or zig-zagged, leaving the outer cones unused) and
        the car stopped mid-corner.
        """
        midpoints = []
        self.gates = []  # (yellow -> blue) vector of each midpoint's gate, for the direction check
        for side, other, is_yellow in ((yellow_cones, blue_cones, True), (blue_cones, yellow_cones, False)):
            if len(other) == 0:
                continue
            for c in side:
                dists = np.linalg.norm(other - c, axis=1)
                j = int(np.argmin(dists))
                # Accept pairing if within track width
                if dists[j] < self.max_gate_distance:
                    mid = (c + other[j]) / 2.0
                    if all(np.linalg.norm(mid - m) > 0.5 for m in midpoints):
                        midpoints.append(mid)
                        self.gates.append(other[j] - c if is_yellow else c - other[j])

        return midpoints

    def _chain_from_car(self, midpoints, max_step=8.0, max_turn=math.radians(75)):
        """Order midpoints by walking from the car to the nearest next one.

        Sorting by forward x breaks in hairpins (the track comes back towards the car). Each step
        must be shorter than max_step and turn less than max_turn, so midpoints from another part
        of the track are not joined in.
        """
        remaining = [(np.asarray(m, dtype=float), np.asarray(g, dtype=float)) for m, g in zip(midpoints, self.gates)]
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
                # gate that way (sin of the angle from step to yellow->blue > 0.5), which rules out
                # driving the track backwards and "gates" made across the infield.
                step = m - pos
                if (step[0] * gate[1] - step[1] * gate[0]) / (np.linalg.norm(step) * np.linalg.norm(gate) + 1e-9) > 0.5:
                    best, best_d = i, d
            if best is None:
                break
            m, _ = remaining.pop(best)
            heading = math.atan2(m[1] - pos[1], m[0] - pos[0])
            pos = m
            chain.append(m)
        return chain

    def _smooth_path(self, waypoints, window_size=None):
        """
        Smooth waypoints using moving average filter
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
            smoothed_point = np.mean(window, axis=0)
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
        if self.frozen or len(orange) < 2:
            return
        guess = np.mean(orange, axis=0)
        cones = np.array(list(blue) + list(yellow)) if len(blue) + len(yellow) else np.empty((0, 2))
        best = guess
        if len(cones) >= 6:
            best_err = float('inf')
            for dx in np.arange(-1.0, 1.01, 0.1):
                for dy in np.arange(-1.0, 1.01, 0.1):
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
        self.correction = None      # smoothed map -> odom correction (x, y, yaw)
        self.correction_t = None
        self.correction_tau = float(self.declare_parameter('correction_tau', 1.0).value)  # s
        self.last_odom_stamp = None

        self.create_subscription(ConeArray, self.cone_map_topic, self.cone_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/slam/pose', self.slam_pose_callback, 10)

        self.get_logger().info(
            f"PATHFINDER: map cones -> base_link planning -> {self.output_frame} path"
        )
        if self.test_mode:
            self.get_logger().info(
                f"PATHFINDER: test_mode enabled, publishing /viz/path and "
                f"/viz/path_markers in {self.viz_frame}"
            )

        self.centreline_planner = CentrelineAlgorithm(self.get_logger(), lookahead_distance=lookahead)
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
        yellow_local = map_to_base_link(yellow_map, rx, ry, r_yaw)
        blue_local = map_to_base_link(blue_map, rx, ry, r_yaw)

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
            path_local = self.centreline_planner.calculate_path(
                yellow_local.tolist(), blue_local.tolist()
            )

            if path_local is not None:
                ox, oy, o_yaw = self.odom_pose
                path_out = base_link_to_odom(path_local, ox, oy, o_yaw)
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
                    path_viz = base_link_to_map(path_local, rx, ry, r_yaw)
                    self.publish_viz(path_viz, self.viz_frame, msg.header.stamp)
            else:
                self.get_logger().warn(
                    "[PATHFINDER OUTPUT] No valid path generated (insufficient cones or waypoints)",
                    throttle_duration_sec=2.0,
                )
        except Exception as e:
            self.get_logger().error(f"Planning Error: {e}")
    
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
        path_local = map_to_base_link(window, rx, ry, r_yaw)
        ox, oy, o_yaw = self.odom_pose
        self.publish_path(base_link_to_odom(path_local, ox, oy, o_yaw), self.output_frame, msg.header.stamp)
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