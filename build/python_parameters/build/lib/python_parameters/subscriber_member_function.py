import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Float32, Int16
import time

class PubSubNode(Node):
    def __init__(self):
        super().__init__('communication_center')
        self.imu_subscription = self.create_subscription(Float32MultiArray, 'imu_euler', self.imu_listener_callback, 10)
        self.imu_subscription  # prevent unused variable warning
        self.angle_subscription = self.create_subscription(Int16, '/car/angle', self.angle_listener_callback, 10)
        self.angle_subscription  # prevent unused variable warning
        self.angle = -1
        self.direction_publisher_ = self.create_publisher(Float32, '/car/direction', 10)
    
    def imu_listener_callback(self, msg):
        imu_message = msg.data[0]
        if imu_message <= 360:
            self.get_logger().info('I heard: "%s"' % imu_message)
            if self.angle != -1:
                self.corr_direction(imu_message)
    
    def angle_listener_callback(self, msg):
        if -1 <= msg.data <= 360:
            self.angle = msg.data

    def corr_direction(self, imu):
        real_angle = imu
        if 30 >= real_angle - self.angle >=1 or -330 >= real_angle - self.angle >= -359:
            msg = Float32()
            msg.data = -0.1
            self.direction_publisher_.publish(msg)         # turn left
            self.get_logger().info('turn left')
        elif 30 >= self.angle - real_angle >=1 or -330 >= self.angle - real_angle >= -359:
            msg = Float32()
            msg.data = 0.1
            self.direction_publisher_.publish(msg)          # turn right
            self.get_logger().info('turn right')

        return 1

def main(args=None):
    rclpy.init(args=args)

    node = PubSubNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
