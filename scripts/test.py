# import ROS 2 libraries and the Node
import rclpy
from rclpy.node import Node
# Import XELA Message type for the stream
from xela_server_ros2.msg import SensStream

# Make a minimal subscriber node
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # Create a subscription to the xServTopic topic
        self.subscription = self.create_subscription(
            SensStream,
            # 'xPalmTopic',
            'xFingersTopic',
            self.listener_callback,
            10)
    # Callback function for the subscription
    def listener_callback(self, msg: SensStream):
        sensors = msg.sensors
        # Here we will just print out some details about the sensor, 
        # checking how many taxels has it reported and if it has forces included.
        log = self.get_logger()
        log.info("-------------------------")
        log.info(f"Broadcast: {len(sensors)} sensor(s)")
        for sensor in sensors:
            log.info(f"Model `{sensor.model}` at message `{sensor.message}` with " + 
		             f"{int(len(sensor.taxels)/3)} taxels" +
                     f" and {'with' if sensor.forces else 'without'} calibration")

# Define the main function, based on the ROS2 tutorial
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()