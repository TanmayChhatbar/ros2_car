import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# from joystick
class controlCMD:
    def __init__(self):
        self.throttle = 0.0
        self.steering = 0.0
        self.brake = 0.0
        self.handbrake = 0.0

class hardwareCMD:
    def __init__(self):
        self.steering = None
        self.esc = None

        # limits
        self.steering_range = 60  # degrees
        self.steering_center = 90  # degrees
        self.esc_range = 70  # degrees
        self.esc_center = 90  # degrees

    def control2hw(self, control: controlCMD):
        self.steering.angle = (control.steering / 2 + 0.5) * self.steering_range + self.steering_center - self.steering_range / 2
        self.esc.angle = ((control.brake - control.throttle) / 2 + 0.5) * self.esc_range + self.esc_center - self.esc_range / 2

class simulatedServo:
    def __init__(self):
        self.angle = 0.0

class CarControlSubscriber(Node):
    def __init__(self):
        super().__init__('car_control_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            50)
        self.subscription  # prevent unused variable warning
        self.publisher_exists = True

        # safety reset commands if no publishers
        self.timer = self.create_timer(0.1, self.check_publishers)

        # control commands from /joy
        self.control = controlCMD()

        # to hardware
        i2c = board.I2C()
        try:
            self.pca = PCA9685(i2c)
            self.pca.frequency = 100

            # commands translated to hw (steering, esc)
            self.hw = hardwareCMD()
            self.hw.steering = servo.Servo(self.pca.channels[7], min_pulse=500, max_pulse=2500)
            self.hw.esc = servo.Servo(self.pca.channels[8], min_pulse=500, max_pulse=2500)

        except Exception as e:
            self.get_logger().warning('Failed to initialize PCA9685: %s\nSimulating output' % str(e))
            self.hw = hardwareCMD()
            self.hw.steering = simulatedServo()
            self.hw.esc = simulatedServo()


        #TODO hall sensor closed loop feedback
        #TODO use handbrake

    def listener_callback(self, msg):
        # parse joystick message and write to hardware
        self.control.steering = msg.axes[0]
        self.control.throttle = msg.axes[1]
        self.control.brake = msg.axes[2]
        self.control.handbrake = msg.buttons[0]

        # self.get_logger().info('steering: %f, throttle: %f, brake: %f, handbrake: %f' % \
        #  (self.control.steering, self.control.throttle, self.control.brake, self.control.handbrake))

        # write to hardware
        self.hw.control2hw(self.control)

    def check_publishers(self):
        # if no publishers on /joy, reset control commands
        if self.subscription.get_publisher_count() == 0:
            if self.publisher_exists:
                self.get_logger().warning('No publishers connected to "joy" topic. Resetting control commands to zero.')
                self.publisher_exists = False
            self.control.steering = 0.0
            self.control.throttle = 0.0
            self.hw.control2hw(self.control)
        else:
            if not self.publisher_exists:
                self.get_logger().info('Joystick command found.')
                self.publisher_exists = True

def main(args=None):
    rclpy.init(args=args)

    car_control_subscriber = CarControlSubscriber()
    try:
        rclpy.spin(car_control_subscriber)

    except KeyboardInterrupt:
        pass

    finally:
        car_control_subscriber.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()
