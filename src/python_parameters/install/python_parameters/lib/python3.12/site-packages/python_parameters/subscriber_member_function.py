import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import serial
import time

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'car', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        arduino_port = '/dev/ttyACM0'
        baud_rate = 115200  # Match this with the baud rate set in your Arduino code
        self.arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
        time.sleep(2)

    def listener_callback(self, msg):
        message = msg.data
        self.get_logger().info('I heard: "%s"' % message)
        self.send_data_serial(message)
    
    def send_data_serial(self, message):
        arduino_port = '/dev/ttyACM0'
        baud_rate = 115200  # Match this with the baud rate set in your Arduino code

        try:
            self.arduino.write(message.encode())  # Encode the string into bytes
            print(f"Sent: {message}")

        except serial.SerialException as e:
            print(f"Error: {e}")

#        finally:
#            if 'arduino' in locals() and arduino.is_open:
#                arduino.close()
#                print("Connection closed.")

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
