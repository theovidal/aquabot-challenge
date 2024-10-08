import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Float64, Float64MultiArray, Int64, String
from sensor_msgs.msg import NavSatFix

class Aquabot(Node):
    # Note: this function shouldn't block or call another one,
    # otherwise callbacks are never triggered, so we don't receive any message
    def __init__(self):
        super().__init__('aquabot')
        self.left_thrust = self.create_publisher(
            Float64,
            '/aquabot/thrusters/left/thrust',
            5)
        self.acoustics = self.create_subscription(
            NavSatFix,
            '/aquabot/sensors/gps/gps/fix',
            self.acoustics_callback,
            10
        )

        self.get_logger().info('Script started')

        # Timer to publish thrust every 1 second
        self.timer = self.create_timer(1.0, self.publish_thrust)

    def acoustics_callback(self, msg):
        self.get_logger().info(f'Latitude: {msg.latitude} ; Longitude : {msg.longitude} ; Altitude = {msg.altitude}')

    def publish_thrust(self):
        time.sleep(1)
        msg = Float64()
        msg.data = 500.0
        self.left_thrust.publish(msg)
        self.get_logger().info('Publishing new thrust')

    # Reset the parameters conveniently to avoid any problem with the bot
    def shutdown(self):
        msg = Float64()
        msg.data = 0.0
        self.left_thrust.publish(msg)
        self.get_logger().info('Kindly shutting down the bot...')



def main(args=None):
    rclpy.init(args=args)

    bot = Aquabot()

    # Interrupt the execution nicely when pressing Ctrl+C
    try:
        rclpy.spin(bot)
    except KeyboardInterrupt:
        bot.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()