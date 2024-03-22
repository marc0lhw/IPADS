import rclpy
from rclpy.node import Node
from can_msgs.msg import ControlHardware

class WMSubscriberNode(Node):

    def __init__(self):
        super().__init__('WmMotionControllerNode_attack')
        self.subscription = self.create_subscription(
            ControlHardware,
            '/can/control_hardware',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("Received ControlHardware message:")
        self.get_logger().info("Horn: {}".format(msg.horn))
        self.get_logger().info("Head Light: {}".format(msg.head_light))
        self.get_logger().info("Left Light: {}".format(msg.left_light))
        self.get_logger().info("Right Light: {}".format(msg.right_light))

def main(args=None):
    rclpy.init(args=args)
    wm_subscriber_node = WMSubscriberNode()
    rclpy.spin(wm_subscriber_node)
    wm_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
