import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # QoS settings
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscribe encoder angles and IMU angles from the car
        self.subscription = self.create_subscription(Float32MultiArray, '/car_angles', self.car_angles_callback, qos_profile)
        self.imu_subscription = self.create_subscription(Float32MultiArray, '/imu_euler', self.imu_listener_callback, qos_profile)

        # Publish current position, initialize car position and odometry
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initial car state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  
        self.theta_odom = 0.0
        self.theta_imu = 0.0
        self.use_imu = True
        self.orientation_change = 0.0
        self.prev_right_angle = None
        self.prev_left_angle = None
        
        # Robot parameters 
        self.circumference = 20.892
        self.turning_circumference = 34.558
        self.temp_time = self.get_clock().now().to_msg()

        # IMU reference
        self.last_heading = None
        self.initial_imu_heading = None

        self.publish_initial_movement()

    def publish_initial_movement(self):
        """Publishes the initial movement command."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_publisher.publish(twist_msg)
        self.get_logger().info('Published initial twist command: [0.0, 0.0]')

    def car_angles_callback(self, msg):
        right_angle, left_angle = msg.data
        
        if self.prev_right_angle is None or self.prev_left_angle is None:
            self.prev_right_angle = right_angle
            self.prev_left_angle = left_angle
            return
        
        # Compute delta angles
        delta_theta_right = right_angle - self.prev_right_angle
        delta_theta_left = left_angle - self.prev_left_angle
        
        # Update previous angles
        self.prev_right_angle = right_angle
        self.prev_left_angle = left_angle
        
        # Compute displacement and orientation change
        displacement = ((delta_theta_right + delta_theta_left) / (2 * 360)) * self.circumference
    

        if not self.use_imu:
            orientation_change = ((delta_theta_left - delta_theta_right) / 2) * (self.circumference / self.turning_circumference)
        
            # Update odometry theta
            self.theta_odom += orientation_change
            self.theta_odom %= 360
        
        # use imu or encoder to calculate theta
        if self.use_imu:
            self.theta = self.theta_imu
        else:
            self.theta = self.theta_odom

        # Update position using fused theta
        self.x += displacement * np.cos(np.radians(self.theta + self.orientation_change / 2))
        self.y += displacement * np.sin(np.radians(self.theta + self.orientation_change / 2))
        
        # Update raw odometry theta
        self.get_logger().info(f'Original Encoder Orientation: {self.theta_odom:.2f}')

    def imu_listener_callback(self, msg):
        
        self.temp_time = self.get_clock().now().to_msg()
        # Extract heading value
        current_heading = msg.data[0]  

        if self.initial_imu_heading is None:
            self.initial_imu_heading = current_heading  
            self.last_heading = current_heading
            self.get_logger().info(f'Storing initial IMU heading: {self.initial_imu_heading:.2f}')

        heading_change = current_heading - self.last_heading
        # Ignore small drifts (≤ 0.1) in heading calculation, but still publish odometry
        if abs(heading_change) >= 0.1:
            # ignore the imu noise around 8.0 and current_heading around 248
            if current_heading >= 360:
                current_heading = self.last_heading
            else:
                self.theta_imu += (current_heading - self.last_heading)
                if self.use_imu:
                    self.orientation_change = heading_change
                
        self.last_heading = current_heading
        #self.get_logger().info(f'real angle:{current_heading:.2f}')
        self.theta_imu %= 360
        self.get_logger().info(f'IMU heading: {self.theta_imu:.2f}')
        
        if self.use_imu:
            self.theta = self.theta_imu
        else:
            self.theta = self.theta_odom
        
        # Publish odometry
        # only when going straight publish odometry 
        #if abs(self.orientation_change) <= 3:
        self.publish_odometry()

    def publish_odometry(self):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.temp_time
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"

        quat = quaternion_from_euler(0, 0, np.radians(self.theta))
        
        # Broadcast TF transformation
        tf_msg.transform.translation.x = self.x / 100
        tf_msg.transform.translation.y = self.y / 100
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(tf_msg)

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x / 100
        odom.pose.pose.position.y = self.y / 100
        odom.pose.pose.position.z = 0.0 # 2D plane

        odom.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )
        self.odom_publisher.publish(odom)
        
        self.get_logger().info(f'Odometry published: x={self.x:.2f}, y={self.y:.2f}, θ={self.theta:.2f}°')



def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
