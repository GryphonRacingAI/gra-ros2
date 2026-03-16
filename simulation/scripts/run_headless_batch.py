#!/usr/bin/env python3
import subprocess
import time
import json
import sys
import signal
from pathlib import Path
from datetime import datetime
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BatchRunner(Node):
    def __init__(self, num_runs, results_dir):
        super().__init__('batch_runner')
        self.num_runs = num_runs
        self.results_dir = Path(results_dir)
        self.results_dir.mkdir(parents=True, exist_ok=True)
        
        self.current_run = 0
        self.run_ids = []
        self.run_active = False
        self.launch_process = None
        
        self.sub_status = self.create_subscription(
            String, '/sim/status', self.status_callback, 10)
        
        self.get_logger().info(f"Batch Runner initialized for {num_runs} runs")
    
    def status_callback(self, msg):
        """Handle simulation status updates"""
        try:
            status = json.loads(msg.data)
            run_id = status.get('run_id')
            if run_id:
                self.run_ids.append(run_id)
                self.get_logger().info(f"Run {self.current_run + 1}/{self.num_runs} completed: {run_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse status: {e}")
    
    def start_simulation(self, run_number, sim_speed=2.0, track='mppi_track'):
        """Start a single simulation run"""
        self.get_logger().info(f"Starting run {run_number}/{self.num_runs}")
        
        cmd = [
            'ros2', 'launch', 'simulation', 'headless_mppi_test.launch.py',
            f'sim_speed:={sim_speed}',
            f'results_dir:={self.results_dir}',
            f'track_name:={track}',
            'headless:=true',
            'autostart:=true'
        ]
        
        self.launch_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        self.run_active = True
        return self.launch_process
    
    def stop_simulation(self):
        """Stop the current simulation"""
        if self.launch_process:
            self.get_logger().info("Stopping simulation...")
            self.launch_process.terminate()
            try:
                self.launch_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Simulation did not terminate, killing...")
                self.launch_process.kill()
            self.launch_process = None
        self.run_active = False
    
    def run_batch(self, sim_speed=2.0, track='mppi_track', delay_between_runs=5.0):
        """Execute batch of simulation runs"""
        self.get_logger().info(f"Starting batch of {self.num_runs} runs")
        
        for run_num in range(1, self.num_runs + 1):
            self.current_run = run_num
            
            self.start_simulation(run_num, sim_speed, track)
            
            while self.run_active and self.launch_process:
                rclpy.spin_once(self, timeout_sec=0.5)
                
                if self.launch_process.poll() is not None:
                    self.run_active = False
                    break
            
            self.stop_simulation()
            
            if run_num < self.num_runs:
                self.get_logger().info(f"Waiting {delay_between_runs}s before next run...")
                time.sleep(delay_between_runs)
        
        self.get_logger().info("Batch completed!")
        self._generate_summary()
    
    def _generate_summary(self):
        """Generate summary report across all runs"""
        self.get_logger().info("Generating summary report...")
        
        all_results = []
        for run_id in self.run_ids:
            try:
                results_file = self.results_dir / f"{run_id}.json"
                if results_file.exists():
                    with open(results_file, 'r') as f:
                        data = json.load(f)
                        all_results.append(data['results'])
            except Exception as e:
                self.get_logger().error(f"Failed to load results for {run_id}: {e}")
        
        if not all_results:
            self.get_logger().warn("No results to summarize")
            return
        
        import numpy as np
        
        summary = {
            'batch_timestamp': datetime.now().isoformat(),
            'num_runs': len(all_results),
            'statistics': {
                'avg_laps_completed': float(np.mean([r['laps_completed'] for r in all_results])),
                'std_laps_completed': float(np.std([r['laps_completed'] for r in all_results])),
                'avg_collisions': float(np.mean([r['collisions'] for r in all_results])),
                'std_collisions': float(np.std([r['collisions'] for r in all_results])),
                'avg_distance_m': float(np.mean([r['total_distance_m'] for r in all_results])),
                'std_distance_m': float(np.std([r['total_distance_m'] for r in all_results])),
                'avg_speed_mps': float(np.mean([r['avg_speed_mps'] for r in all_results])),
                'max_speed_mps': float(np.max([r['max_speed_mps'] for r in all_results])),
                'avg_duration_s': float(np.mean([r['duration_s'] for r in all_results])),
            },
            'failure_reasons': {},
            'run_ids': self.run_ids
        }
        
        for r in all_results:
            reason = r.get('failure_reason', 'unknown')
            summary['failure_reasons'][reason] = summary['failure_reasons'].get(reason, 0) + 1
        
        summary_file = self.results_dir / f"batch_summary_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(summary_file, 'w') as f:
            json.dump(summary, f, indent=2)
        
        self.get_logger().info(f"Summary saved to {summary_file}")
        
        print("\n" + "="*60)
        print("BATCH SUMMARY")
        print("="*60)
        print(f"Total Runs: {summary['num_runs']}")
        print(f"\nAverage Laps: {summary['statistics']['avg_laps_completed']:.2f} ± {summary['statistics']['std_laps_completed']:.2f}")
        print(f"Average Collisions: {summary['statistics']['avg_collisions']:.2f} ± {summary['statistics']['std_collisions']:.2f}")
        print(f"Average Distance: {summary['statistics']['avg_distance_m']:.1f}m ± {summary['statistics']['std_distance_m']:.1f}m")
        print(f"Average Speed: {summary['statistics']['avg_speed_mps']:.2f} m/s")
        print(f"Max Speed: {summary['statistics']['max_speed_mps']:.2f} m/s")
        print(f"\nFailure Reasons:")
        for reason, count in summary['failure_reasons'].items():
            print(f"  {reason}: {count}")
        print("="*60 + "\n")


def main():
    parser = argparse.ArgumentParser(description='Run batch of headless MPPI simulations')
    parser.add_argument('--num-runs', type=int, default=1, help='Number of sequential runs')
    parser.add_argument('--sim-speed', type=float, default=2.0, help='Simulation speed multiplier')
    parser.add_argument('--results-dir', type=str, default='/tmp/mppi_results', help='Results directory')
    parser.add_argument('--track', type=str, default='mppi_track', help='Track name')
    parser.add_argument('--delay', type=float, default=5.0, help='Delay between runs (seconds)')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    runner = BatchRunner(args.num_runs, args.results_dir)
    
    def signal_handler(sig, frame):
        print("\nInterrupted! Stopping simulation...")
        runner.stop_simulation()
        runner.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        runner.run_batch(args.sim_speed, args.track, args.delay)
    except Exception as e:
        runner.get_logger().error(f"Batch run failed: {e}")
    finally:
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
