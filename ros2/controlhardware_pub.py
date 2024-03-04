import rclpy
from rclpy.node import Node
from can_msgs.msg import ControlHardware  # Import the ControlHardware message type

class WMMotionControllerNode(Node):

    def __init__(self):
        super().__init__('wm_motion_controller_node')
        self.publisher_ = self.create_publisher(ControlHardware, '/can/control_hardware', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = ControlHardware()
        msg.horn = False  # ON (1) or OFF (0)
        msg.head_light = False  # ON (1) or OFF (0)
        msg.left_light = False  # ON (1) or OFF (0)
        msg.right_light = False  # ON (1) or OFF (0)
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing ControlHardware message')

def main(args=None):
    rclpy.init(args=args)
    wm_motion_controller_node = WMMotionControllerNode()
    rclpy.spin(wm_motion_controller_node)
    wm_motion_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

