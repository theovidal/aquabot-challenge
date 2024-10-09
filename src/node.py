import rclpy
from rclpy.node import Node
import time
import matplotlib.pyplot as plt
import numpy as np

from std_msgs.msg import Float64, Float64MultiArray, Int64, String
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import PoseArray

class Aquabot(Node):
    # Note: this function shouldn't block or call another one,
    # otherwise callbacks are never triggered, so we don't receive any message
    def __init__(self):
        super().__init__('aquabot')

        # Creating a published to update the thrust command for the left engine
        self.left_thrust = self.create_publisher(
            Float64,
            '/aquabot/thrusters/left/thrust',
            5)

        # Subscribing to the topic where GPS information is sent
        self.gps = self.create_subscription(
            NavSatFix, # This type is obtained after executing the command "ros2 topic info /aquabot/sensors/gps/gps/fix"
            '/aquabot/sensors/gps/gps/fix',
            self.acoustics_callback, # The callback is a function that is called when a message is received
            10
        )
        self.camera = self.create_subscription(
            Image,
            '/aquabot/sensors/cameras/main_camera_sensor/image_raw',
            self.camera_callback,
            10
        )
        # Creating a matplotlib frame for the camera image
        self.camera_frame = plt.subplot(1, 1, 1)
        self.camera_image = None

        # Position is acquired 10 seconds after the node has started
        self.windturbines_position = self.create_subscription(
            PoseArray,
            '/aquabot/ais_sensor/windturbines_positions',
            self.windturbines_position_callback,
            10
        )

        plt.ion()

        self.get_logger().info('Script started')

        # Setting a timer to command engine thrust every 1 second
        self.timer = self.create_timer(1.0, self.publish_thrust)

    def windturbines_position_callback(self, msg):
        self.get_logger().info(f'{msg}')

    def acoustics_callback(self, msg):
        self.get_logger().info(f'Latitude: {msg.latitude} ; Longitude : {msg.longitude} ; Altitude = {msg.altitude}')

    def camera_callback(self, msg):
        self.get_logger().info(f'Got image data {msg.is_bigendian}')

        # Manipulating the image, as its received as a flat array that needs to be reshaped in (width, height, channels)
        # channels = Red, Green and Blue
        image = np.array(msg.data).reshape(msg.width, msg.height, 3)
        if self.camera_image is None:
            self.camera_image = self.camera_frame.imshow(image)
            plt.show()
        else:
            self.camera_image.set_data(image)
            plt.pause(0.2)

    def publish_thrust(self):
        # Creating the message and setting the data
        msg = Float64()
        msg.data = 500.0
        # Publishing this message to the topic (with the published we created in the __init__ function)
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
        # "Spin" executes the node, i.e.:
        # - listening to publications in topics so our node gets them
        # - executing functions defined in timers at the defined intervals
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