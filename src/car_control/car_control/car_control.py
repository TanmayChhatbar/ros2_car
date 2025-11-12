import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Header


class CarControlSubscriber(Node):
    def __init__(self):
        super().__init__('car_control_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            50)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('steering: %f, throttle: %f, brake: %f, handbrake: %f' % (msg.axes[0], msg.axes[1], msg.axes[2], msg.buttons[0]))


def main(args=None):
    rclpy.init(args=args)

    car_control_subscriber = CarControlSubscriber()

    rclpy.spin(car_control_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    car_control_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
