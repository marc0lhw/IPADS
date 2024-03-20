import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ChatterSubscriber(Node):

    def __init__(self, node_id):
        super().__init__('listener_' + str(node_id))
        self.subscription = self.create_subscription(
            String,
            '/chatter',
            self.listener_callback,
            10)
        self.msg_received = False

    def listener_callback(self, msg):
        self.msg_received = True
        self.get_logger().info('Received message: %s' % msg.data)

def main():
    rclpy.init()
    nodes = []
    for i in range(1, 101):
        node = ChatterSubscriber(i)
        nodes.append(node)
        rclpy.spin_once(node, timeout_sec=0.001)  # 각 노드를 한 번씩 spin_once() 호출

    while rclpy.ok():
        rclpy.spin_once(nodes[0], timeout_sec=0.1)
        if sum(node.msg_received for node in nodes) == len(nodes):
            break

    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

