#!/usr/bin/env python3
import json
import numpy as np
from pathlib import Path
from datetime import datetime
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class ResultsManager:
    """Utility class for managing simulation results and visualizations"""
    
    def __init__(self, results_dir='/tmp/mppi_results'):
        self.results_dir = Path(results_dir)
        self.results_dir.mkdir(parents=True, exist_ok=True)
    
    def save_results(self, run_data, mppi_params=None):
        """
        Save run results to JSON file
        
        Args:
            run_data: Dictionary containing run statistics
            mppi_params: Dictionary of MPPI controller parameters
        
        Returns:
            Path to saved results file
        """
        run_id = run_data.get('run_id', f"mppi_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        
        results = {
            'run_id': run_id,
            'timestamp': datetime.now().isoformat(),
            'track': run_data.get('track', 'unknown'),
            'mppi_params': mppi_params or {},
            'results': run_data.get('results', {}),
            'trajectory_file': f"{run_id}_trajectory.png"
        }
        
        results_file = self.results_dir / f"{run_id}.json"
        with open(results_file, 'w') as f:
            json.dump(results, f, indent=2)
        
        return results_file
    
    def generate_trajectory_visualization(self, trajectory, inner_cones, outer_cones, 
                                         run_id, stats=None, collision_points=None):
        """
        Generate top-down trajectory visualization
        
        Args:
            trajectory: List of dicts with 'x', 'y', 'speed' keys
            inner_cones: Nx2 array of inner cone positions
            outer_cones: Nx2 array of outer cone positions
            run_id: Unique run identifier
            stats: Dictionary of statistics to display
            collision_points: List of collision positions
        
        Returns:
            Path to saved visualization file
        """
        fig, ax = plt.subplots(figsize=(14, 12))
        
        if inner_cones is not None and len(inner_cones) > 0:
            ax.scatter(inner_cones[:, 0], inner_cones[:, 1], 
                      c='blue', marker='o', s=40, label='Inner Cones', 
                      alpha=0.7, edgecolors='darkblue', linewidths=1)
        
        if outer_cones is not None and len(outer_cones) > 0:
            ax.scatter(outer_cones[:, 0], outer_cones[:, 1], 
                      c='orange', marker='o', s=40, label='Outer Cones', 
                      alpha=0.7, edgecolors='darkorange', linewidths=1)
        
        if trajectory and len(trajectory) > 0:
            traj_x = [p['x'] for p in trajectory]
            traj_y = [p['y'] for p in trajectory]
            speeds = [p['speed'] for p in trajectory]
            
            scatter = ax.scatter(traj_x, traj_y, c=speeds, cmap='viridis', 
                               s=15, alpha=0.8, label='Trajectory', zorder=5)
            cbar = plt.colorbar(scatter, ax=ax, label='Speed (m/s)', pad=0.02)
            cbar.ax.tick_params(labelsize=10)
            
            ax.plot(traj_x[0], traj_y[0], 'go', markersize=18, 
                   label='Start Position', markeredgecolor='black', 
                   markeredgewidth=2.5, zorder=10)
            
            ax.plot(traj_x[-1], traj_y[-1], 'rs', markersize=18, 
                   label='Final Position', markeredgecolor='black', 
                   markeredgewidth=2.5, zorder=10)
        
        if collision_points and len(collision_points) > 0:
            coll_x = [p[0] for p in collision_points]
            coll_y = [p[1] for p in collision_points]
            ax.scatter(coll_x, coll_y, c='red', marker='x', s=200, 
                      linewidths=3, label='Collisions', zorder=15)
        
        ax.set_xlabel('X Position (m)', fontsize=13, fontweight='bold')
        ax.set_ylabel('Y Position (m)', fontsize=13, fontweight='bold')
        
        title = f'MPPI Control Test - {run_id}'
        if stats:
            title += f"\nLaps: {stats.get('laps_completed', 0):.1f} | "
            title += f"Distance: {stats.get('total_distance_m', 0):.1f}m | "
            title += f"Avg Speed: {stats.get('avg_speed_mps', 0):.2f}m/s | "
            title += f"Collisions: {stats.get('collisions', 0)}"
        
        ax.set_title(title, fontsize=14, fontweight='bold', pad=15)
        ax.legend(loc='best', fontsize=11, framealpha=0.9)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.axis('equal')
        
        ax.tick_params(labelsize=10)
        
        viz_file = self.results_dir / f"{run_id}_trajectory.png"
        plt.savefig(viz_file, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()
        
        return viz_file
    
    def load_results(self, run_id):
        """Load results from JSON file"""
        results_file = self.results_dir / f"{run_id}.json"
        if not results_file.exists():
            raise FileNotFoundError(f"Results file not found: {results_file}")
        
        with open(results_file, 'r') as f:
            return json.load(f)
    
    def aggregate_results(self, run_ids):
        """
        Aggregate statistics across multiple runs
        
        Args:
            run_ids: List of run identifiers
        
        Returns:
            Dictionary with aggregated statistics
        """
        all_results = []
        for run_id in run_ids:
            try:
                results = self.load_results(run_id)
                all_results.append(results['results'])
            except Exception as e:
                print(f"Warning: Could not load results for {run_id}: {e}")
        
        if not all_results:
            return {}
        
        aggregate = {
            'num_runs': len(all_results),
            'avg_laps_completed': np.mean([r['laps_completed'] for r in all_results]),
            'avg_collisions': np.mean([r['collisions'] for r in all_results]),
            'avg_distance_m': np.mean([r['total_distance_m'] for r in all_results]),
            'avg_speed_mps': np.mean([r['avg_speed_mps'] for r in all_results]),
            'max_speed_mps': np.max([r['max_speed_mps'] for r in all_results]),
            'avg_duration_s': np.mean([r['duration_s'] for r in all_results]),
            'failure_reasons': {}
        }
        
        for r in all_results:
            reason = r.get('failure_reason', 'unknown')
            aggregate['failure_reasons'][reason] = aggregate['failure_reasons'].get(reason, 0) + 1
        
        return aggregate
    
    def save_aggregate_report(self, run_ids, output_file='aggregate_report.json'):
        """Save aggregated results report"""
        aggregate = self.aggregate_results(run_ids)
        
        report_file = self.results_dir / output_file
        with open(report_file, 'w') as f:
            json.dump(aggregate, f, indent=2)
        
        return report_file

if __name__ == '__main__':
    manager = ResultsManager()
    
    dummy_trajectory = [
        {'x': i, 'y': np.sin(i/10)*5, 'speed': 3.0 + np.random.rand()} 
        for i in range(100)
    ]
    
    inner_cones = np.array([[i, -3] for i in range(0, 100, 5)])
    outer_cones = np.array([[i, 3] for i in range(0, 100, 5)])
    
    stats = {
        'laps_completed': 1.5,
        'total_distance_m': 150.0,
        'avg_speed_mps': 3.5,
        'collisions': 2
    }
    
    viz_file = manager.generate_trajectory_visualization(
        dummy_trajectory, inner_cones, outer_cones, 
        'test_run', stats, [[25, 0], [75, 1]]
    )
    
    print(f"Test visualization saved to: {viz_file}")
