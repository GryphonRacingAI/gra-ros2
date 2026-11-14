#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, ColorRGBA, UInt16
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from common_msgs.msg import ConeArray

from path_planning.centreline import CentrelineAlgorithm
from path_planning.geometry import quaternion_to_yaw, to_local, to_world
from path_planning.remember_lap import RememberedLap
from path_planning.skidpad import SkidpadPlanner


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
        self.last_good = None       # (time, path in odom) of the last planned path
        self.correction = None      # smoothed map -> odom correction (x, y, yaw)
        self.correction_t = None
        self.correction_tau = float(self.declare_parameter('correction_tau', 1.0).value)  # s
        self.last_odom_stamp = None

        self.create_subscription(ConeArray, self.cone_map_topic, self.cone_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # /odom poses as received here, to pair each SLAM pose with the odometry sample nearest to its
        # stamp. A tf2 lookup at the stamp gives the same pose (checked: identical to /odom) but made
        # lap 1 of trackdrive slow in 6 of 10 runs, so the pairing stays with this node's own samples.
        self.odom_hist = deque(maxlen=300)
        self.create_subscription(PoseStamped, '/slam/pose', self.slam_pose_callback, 10)

        # Remembered lap (opt-in, remember_lap:=true; closed circuits only), see remember_lap.py
        self.remember_lap = bool(self.declare_parameter('remember_lap', False).value)
        self.remembered = RememberedLap(
            self.get_logger(),
            min_clear=float(self.declare_parameter('remember_min_clear', 0.75).value),
            max_offset=float(self.declare_parameter('remember_max_offset', 1.5).value))
        self.laps = 0
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
        odom = self.odom_pose_at(msg.header.stamp)
        if odom is None:
            return
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

    def odom_pose_at(self, stamp):
        """(x, y, yaw) from the /odom sample nearest in time to `stamp`, or None before the first one."""
        if not self.odom_hist:
            return None
        t = stamp.sec + stamp.nanosec * 1e-9
        return min(self.odom_hist, key=lambda h: abs(h[0] - t))[1:]

    def laps_callback(self, msg):
        self.laps = int(msg.data)

    def lidar_cones_callback(self, msg):
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y'), skip_nans=True).reshape(-1, 2)
        self.lidar_cones_odom = pts.astype(float)

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
                self.remembered.record(rx, ry, self.laps)
                visible = [yellow_local, blue_local]
                if len(self.lidar_cones_odom):
                    visible.append(to_local(self.lidar_cones_odom, *self.odom_pose))   # odom -> base_link
                path_local = self.remembered.path(rx, ry, r_yaw, visible)
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
                # Not on the acceleration straight: there the cones of one colour can run out first near
                # the end, and the arc steered the car off the lane (the controller extends the last
                # path straight ahead on that event instead).
                if path_out is None and self.event != 'acceleration':
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