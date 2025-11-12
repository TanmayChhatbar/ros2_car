#!/usr/bin/env python3

# joystick stuff
import pyglet
from racecar_simwheel import RacecarSimwheel


# ros stuff
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

joystick = RacecarSimwheel()


class ControlPublisher(Node):
    def __init__(self):
        super().__init__('control_publisher')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # Publish at 100 Hz
        self.get_logger().info("Control publisher started.")
        self.joystick = RacecarSimwheel()
        self.new_time = self.get_clock().now()

    def timer_callback(self):
        self.joystick.update()  # Update joystick/pedal values
        new_publish = False
        if self.joystick.isNewInput():
            self.new_time = self.get_clock().now()
            new_publish = True
        elif self.new_time - self.get_clock().now() > rclpy.duration.Duration(seconds=1):
            # If no new input for 1 second, zero out values
            self.joystick.steering = 0.0
            self.joystick.throttle = 0.0
            self.joystick.brake = 0.0
            new_publish = True

        if new_publish:
            # Create and publish the Joy message
            joy_msg = Joy()
            joy_msg.header = Header()
            joy_msg.header.stamp = self.new_time.to_msg()
            joy_msg.axes = [self.joystick.steering, self.joystick.throttle, self.joystick.brake]
            joy_msg.buttons = [self.joystick.handbrake]  # Add button states if needed
            self.publisher_.publish(joy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlPublisher()

    # Set up pyglet for non-blocking device updates (at 60 Hz)
    pyglet.clock.schedule_interval(node.joystick.update, 1/60.0)
    
    try:
        joystick.window.clear()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Control publisher stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
