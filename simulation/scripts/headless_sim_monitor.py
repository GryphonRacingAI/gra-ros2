#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import json
import time
from datetime import datetime
from pathlib import Path

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point

class HeadlessSimMonitor(Node):
    def __init__(self):
        super().__init__('headless_sim_monitor')
        
        self.declare_parameter('results_dir', '/tmp/mppi_results')
        self.declare_parameter('inner_cones_csv', '')
        self.declare_parameter('outer_cones_csv', '')
        self.declare_parameter('track_name', 'mppi_track')
        self.declare_parameter('velocity_timeout', 5.0)
        self.declare_parameter('min_velocity_threshold', 0.1)
        
        self.results_dir = Path(self.get_parameter('results_dir').value)
        self.inner_cones_csv = self.get_parameter('inner_cones_csv').value
        self.outer_cones_csv = self.get_parameter('outer_cones_csv').value
        self.track_name = self.get_parameter('track_name').value
        self.velocity_timeout = self.get_parameter('velocity_timeout').value
        self.min_velocity_threshold = self.get_parameter('min_velocity_threshold').value
        
        self.results_dir.mkdir(parents=True, exist_ok=True)
        
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.sub_mppi_params = self.create_subscription(
            String, '/mppi/parameters', self.mppi_params_callback, 10)
        
        self.pub_shutdown = self.create_publisher(
            Bool, '/sim/shutdown_request', 10)
        
        self.pub_status = self.create_publisher(
            String, '/sim/status', 10)
        
        self.trajectory = []
        self.velocity_history = []
        self.collision_count = 0
        self.laps_completed = 0.0
        
        self.start_time = None
        self.last_velocity_time = None
        self.zero_velocity_start = None
        
        self.max_speed = 0.0
        self.total_distance = 0.0
        self.last_position = None
        
        self.inner_cones = None
        self.outer_cones = None
        self.finish_line_crossed = False
        self.last_cross_product_sign = None
        
        self.run_active = False
        self.failure_reason = None
        self.mppi_params = {}
        
        self._load_track()
        
        self.timer = self.create_timer(0.1, self.monitor_loop)
        
        self.get_logger().info(f"Headless Sim Monitor initialized. Results dir: {self.results_dir}")
    
    def _load_track(self):
        """Load track cone positions for lap detection"""
        try:
            if self.inner_cones_csv and self.outer_cones_csv:
                self.inner_cones = np.loadtxt(self.inner_cones_csv, delimiter=',')
                self.outer_cones = np.loadtxt(self.outer_cones_csv, delimiter=',')
                self.get_logger().info(f"Loaded track: {len(self.inner_cones)} inner, {len(self.outer_cones)} outer cones")
                
                n = min(len(self.inner_cones), len(self.outer_cones))
                self.centerline = (self.inner_cones[:n] + self.outer_cones[:n]) / 2.0
                
                self.finish_line_start = self.centerline[0]
                self.finish_line_end = self.centerline[1]
                self.get_logger().info(f"Finish line: {self.finish_line_start} -> {self.finish_line_end}")
        except Exception as e:
            self.get_logger().error(f"Failed to load track: {e}")
    
    def mppi_params_callback(self, msg):
        """Store MPPI parameters for results logging"""
        try:
            self.mppi_params = json.loads(msg.data)
            self.get_logger().info("Received MPPI parameters")
        except Exception as e:
            self.get_logger().error(f"Failed to parse MPPI parameters: {e}")
    
    def odom_callback(self, msg):
        """Process odometry data"""
        if not self.run_active:
            self.run_active = True
            self.start_time = self.get_clock().now()
            self.get_logger().info("Run started!")
        
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        
        current_pos = np.array([p.x, p.y])
        speed = np.sqrt(v.x**2 + v.y**2)
        
        self.trajectory.append({
            'x': p.x,
            'y': p.y,
            'z': p.z,
            'speed': speed,
            'time': self.get_clock().now().nanoseconds / 1e9
        })
        
        self.velocity_history.append(speed)
        
        if speed > self.max_speed:
            self.max_speed = speed
        
        if self.last_position is not None:
            self.total_distance += np.linalg.norm(current_pos - self.last_position)
        
        self.last_position = current_pos
        
        if speed < self.min_velocity_threshold:
            if self.zero_velocity_start is None:
                self.zero_velocity_start = self.get_clock().now()
        else:
            self.zero_velocity_start = None
        
        self._check_lap_crossing(current_pos)
    
    def _check_lap_crossing(self, current_pos):
        """Detect if vehicle crossed the finish line"""
        if self.centerline is None or len(self.trajectory) < 2:
            return
        
        prev_pos = np.array([self.trajectory[-2]['x'], self.trajectory[-2]['y']])
        
        line_vec = self.finish_line_end - self.finish_line_start
        
        prev_vec = prev_pos - self.finish_line_start
        curr_vec = current_pos - self.finish_line_start
        
        prev_cross = np.cross(line_vec, prev_vec)
        curr_cross = np.cross(line_vec, curr_vec)
        
        if prev_cross * curr_cross < 0:
            t_num = np.cross(prev_pos - self.finish_line_start, prev_pos - current_pos)
            t_den = np.cross(line_vec, prev_pos - current_pos)
            
            if abs(t_den) > 1e-6:
                t = t_num / t_den
                if 0 <= t <= 1:
                    if self.finish_line_crossed:
                        self.laps_completed += 1.0
                        self.get_logger().info(f"Lap completed! Total laps: {self.laps_completed}")
                    else:
                        self.finish_line_crossed = True
                        self.get_logger().info("First finish line crossing detected")
    
    def monitor_loop(self):
        """Main monitoring loop"""
        if not self.run_active:
            return
        
        if self.zero_velocity_start is not None:
            elapsed = (self.get_clock().now() - self.zero_velocity_start).nanoseconds / 1e9
            if elapsed >= self.velocity_timeout:
                self.get_logger().warn(f"Velocity timeout reached ({elapsed:.1f}s)")
                self.failure_reason = "velocity_zero_timeout"
                self._end_simulation()
    
    def _end_simulation(self):
        """End simulation and save results"""
        if not self.run_active:
            return
        
        self.run_active = False
        
        duration = 0.0
        if self.start_time is not None:
            duration = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        avg_speed = np.mean(self.velocity_history) if self.velocity_history else 0.0
        
        final_pos = [0.0, 0.0]
        if self.trajectory:
            final_pos = [self.trajectory[-1]['x'], self.trajectory[-1]['y']]
        
        run_id = f"mppi_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        results = {
            'run_id': run_id,
            'timestamp': datetime.now().isoformat(),
            'track': self.track_name,
            'mppi_params': self.mppi_params,
            'results': {
                'collisions': self.collision_count,
                'laps_completed': self.laps_completed,
                'avg_speed_mps': float(avg_speed),
                'max_speed_mps': float(self.max_speed),
                'total_distance_m': float(self.total_distance),
                'duration_s': float(duration),
                'failure_reason': self.failure_reason or 'unknown',
                'final_position': final_pos
            },
            'trajectory_file': f"{run_id}_trajectory.png"
        }
        
        results_file = self.results_dir / f"{run_id}.json"
        with open(results_file, 'w') as f:
            json.dump(results, f, indent=2)
        
        self.get_logger().info(f"Results saved to {results_file}")
        self.get_logger().info(f"Stats - Laps: {self.laps_completed}, Collisions: {self.collision_count}, "
                              f"Avg Speed: {avg_speed:.2f} m/s, Distance: {self.total_distance:.1f} m")
        
        self._generate_visualization(run_id)
        
        shutdown_msg = Bool()
        shutdown_msg.data = True
        self.pub_shutdown.publish(shutdown_msg)
        
        status_msg = String()
        status_msg.data = json.dumps(results)
        self.pub_status.publish(status_msg)
    
    def _generate_visualization(self, run_id):
        """Generate trajectory visualization"""
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            
            fig, ax = plt.subplots(figsize=(12, 10))
            
            if self.inner_cones is not None and self.outer_cones is not None:
                ax.scatter(self.inner_cones[:, 0], self.inner_cones[:, 1], 
                          c='blue', marker='o', s=30, label='Inner Cones', alpha=0.6)
                ax.scatter(self.outer_cones[:, 0], self.outer_cones[:, 1], 
                          c='orange', marker='o', s=30, label='Outer Cones', alpha=0.6)
            
            if self.trajectory:
                traj_x = [p['x'] for p in self.trajectory]
                traj_y = [p['y'] for p in self.trajectory]
                speeds = [p['speed'] for p in self.trajectory]
                
                scatter = ax.scatter(traj_x, traj_y, c=speeds, cmap='viridis', 
                                   s=10, alpha=0.7, label='Trajectory')
                plt.colorbar(scatter, ax=ax, label='Speed (m/s)')
                
                ax.plot(traj_x[0], traj_y[0], 'go', markersize=15, 
                       label='Start', markeredgecolor='black', markeredgewidth=2)
                ax.plot(traj_x[-1], traj_y[-1], 'rs', markersize=15, 
                       label='End', markeredgecolor='black', markeredgewidth=2)
            
            ax.set_xlabel('X (m)', fontsize=12)
            ax.set_ylabel('Y (m)', fontsize=12)
            ax.set_title(f'MPPI Test Run - {run_id}\n'
                        f'Laps: {self.laps_completed:.1f}, Distance: {self.total_distance:.1f}m, '
                        f'Avg Speed: {np.mean(self.velocity_history):.2f}m/s', fontsize=14)
            ax.legend(loc='best')
            ax.grid(True, alpha=0.3)
            ax.axis('equal')
            
            viz_file = self.results_dir / f"{run_id}_trajectory.png"
            plt.savefig(viz_file, dpi=150, bbox_inches='tight')
            plt.close()
            
            self.get_logger().info(f"Visualization saved to {viz_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to generate visualization: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HeadlessSimMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
