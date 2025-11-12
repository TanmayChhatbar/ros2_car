#!/usr/bin/env python3

# joystick stuff
import pygame
from command_control.racecar_joystick import RacecarJoystick
import time # test

# ros stuff
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

joystick = RacecarJoystick()

class ControlPublisher(Node):
    def __init__(self):
        super().__init__('control_publisher')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        self.get_logger().info("Control publisher started.")
        self.joystick = RacecarJoystick()
        self.new_time = self.get_clock().now()

    def timer_callback(self):
        self.joystick.update()
        new_publish = False
        if self.joystick.isNewInput():
            self.new_time = self.get_clock().now()
            new_publish = True
        elif self.new_time - self.get_clock().now() > rclpy.duration.Duration(seconds=1):
            self.joystick.steering = 0.0
            self.joystick.throttle = 0.0
            self.joystick.brake = 0.0
            new_publish = True

        if new_publish:
            joy_msg = Joy()
            joy_msg.header = Header()
            joy_msg.header.stamp = self.new_time.to_msg()
            joy_msg.axes = [self.joystick.steering, self.joystick.throttle, self.joystick.brake]
            joy_msg.buttons = [self.joystick.handbrake]  # Add button states if needed
            self.publisher_.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Control publisher stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
