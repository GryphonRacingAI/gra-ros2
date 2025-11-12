import rclpy
from rclpy.node import Node
import numpy as np
import math
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
# Assuming you place the fastslam64 directory in your package and modify imports
from fastslam64.lib.particle3 import FlatParticle
from fastslam64.lib.common import CUDAMemory, rescale, get_pose_estimate, resample
from fastslam64.cuda.fastslam import load_cuda_modules
# Import your custom cone array message
# from fsai_msgs.msg import ConeArray 

# --- Placeholder Functions for the FastSLAM Core ---
# NOTE: You MUST reorganize the original repo's run_SLAM into a class method.
class FastSLAMCore:
    def __init__(self, config):
        self.config = config
        # 1. Initialize CUDA/GPU resources
        # 2. Load CUDA kernels
        # 3. Initialize particles and CUDAMemory
        # ... (Similar to the beginning of run_SLAM)

    def execute_step(self, odometry_input, cone_measurements_rb):
        """
        Executes one SLAM prediction and update step on the GPU.
        :param odometry_input: (v, omega) tuple.
        :param cone_measurements_rb: A NumPy array of (r, beta) measurements.
        :return: Estimated (x, y, theta) and the best particle's landmark map.
        """
        # 1. Prediction (GPU kernel call with v, omega)
        # 2. Update (GPU kernel call with r, beta measurements)
        # 3. Rescale/Resample check
        # 4. Get pose estimate
        # ... (Core logic from the original repo)
        
        # --- Placeholder return values ---
        estimated_pose = np.array([0.0, 0.0, 0.0]) # x, y, theta
        best_particle_map = [] # Landmark estimates (x, y, cov)
        return estimated_pose, best_particle_map

class FastSLAMNode(Node):
    def __init__(self):
        super().__init__('fastslam_node')
        self.get_logger().info('Initializing FastSLAM Node...')

        # Load configuration (You'll need a config file for your particle/sensor settings)
        # from fastslam64.config_simulation import config # Or create your own
        # self.slam_core = FastSLAMCore(config)
        
        # Initialize SLAM core placeholder
        self.slam_core = None 

        # ROS 2 Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/slam/pose', 10)
        self.map_pub = self.create_publisher(MarkerArray, '/slam/cone_map', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ROS 2 Subscribers and Time Synchronization
        # Assuming Odometry is published as nav_msgs/Odometry and Cones as your custom msg
        self.odom_sub = Subscriber(self, Odometry, '/fused_odom')
        # self.cone_sub = Subscriber(self, ConeArray, '/perception/cone_array')
        
        # Placeholder for cone subscriber until exact msg is known:
        # We will use a timer for demonstration simplicity instead of message_filters
        self.create_timer(0.02, self.timer_callback) # Run at 50 Hz


    def cone_to_range_bearing(self, cone_array_msg):
        """Converts cone X,Y positions (relative to robot) to Range and Bearing (r, beta)."""
        measurements = []
        
        # Iterate through all cone lists (yellow, blue, etc.)
        for cone_list in [cone_array_msg.yellow_cones, cone_array_msg.blue_cones]: # Assuming structure
            for cone in cone_list:
                # X and Y are relative to the robot's base_link (required for SLAM)
                x = cone.position.x
                y = cone.position.y
                
                # Range (r)
                r = math.sqrt(x**2 + y**2)
                # Bearing (beta)
                beta = math.atan2(y, x)
                
                # We need to assign a temporary ID for data association if the perception ID isn't used
                # For this step, we just collect r, beta pairs
                measurements.append([r, beta]) 
                
        return np.array(measurements, dtype=np.float64)

    def timer_callback(self):
        # --- 1. Get Synchronized Data (simplified for demo) ---
        # In a real node, this logic belongs in the synchronized callback
        
        # Placeholder: Assume we have the latest odometry (v, omega) and cones (r, beta)
        odom_input = (0.5, 0.0) # (v, omega) - example: 0.5 m/s forward
        
        # Placeholder: Assume we read the latest cone message
        # cone_data_msg = self.cone_sub.get_latest() # In a real system
        
        # --- 2. Pre-processing & SLAM Execution ---
        # cones_rb = self.cone_to_range_bearing(cone_data_msg)
        
        # Placeholder: 2x2 array of r, beta pairs
        cones_rb = np.array([
            [5.0, 0.1], # Cone 1: 5m range, 0.1 rad bearing
            [6.2, -0.5] # Cone 2: 6.2m range, -0.5 rad bearing
        ])
        
        if self.slam_core:
            estimated_pose, best_map = self.slam_core.execute_step(odom_input, cones_rb)
            
            # --- 3. ROS 2 Output ---
            self.publish_pose(estimated_pose)
            self.publish_map(best_map)


    def publish_pose(self, pose):
        """Publishes the robot's estimated pose (x, y, theta) in the map frame."""
        x, y, theta = pose
        now = self.get_clock().now().to_msg()
        
        # 1. Publish PoseStamped (Optional but useful for visualization)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        # Convert yaw (theta) to quaternion for orientation (omitted for brevity)
        self.pose_pub.publish(pose_msg)

        # 2. Publish map -> odom TF (MANDATORY for localization visualization)
        # This transform tells the system where the ODOM frame is located *within* the MAP frame.
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom' # Assuming your odometry publishes the odom -> base_link TF
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        # Convert yaw to quaternion and set t.transform.rotation (omitted for brevity)
        self.tf_broadcaster.sendTransform(t)


    def publish_map(self, best_map):
        """Publishes the estimated cone locations from the best particle."""
        # This would iterate through the landmark estimates (mean x, y) in best_map
        # and create a Marker for each, publishing a MarkerArray.
        marker_array = MarkerArray()
        # ... (implementation needed to convert EKF means to Marker positions)
        self.map_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    fastslam_node = FastSLAMNode()
    rclpy.spin(fastslam_node)
    fastslam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
