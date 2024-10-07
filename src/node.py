import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Float64


class Publisher(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher_ = self.create_publisher(
            Float64,
            '/aquabot/thrusters/left/thrust',
            5)
        self.get_logger().info('Subscriber started')
        self.run()

    def run(self):
        while(1):
            time.sleep(1)
            msg = Float64()
            msg.data = 500.0
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing new thrust')


def main(args=None):
    rclpy.init(args=args)

    pub = Publisher()

    rclpy.spin(pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()