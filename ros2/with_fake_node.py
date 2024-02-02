import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class Talker1(Node):

    def __init__(self):
        super().__init__('talker')  # same name
        # type, topic
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        # topic msg
        msg = String()
        msg.data = f'Hello World: {self.counter}, Time: {time.strftime("%H:%M:%S")}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    talker1 = Talker1()
    rclpy.spin(talker1)
    talker1.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
