#!/usr/bin/env python3

import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import ColorRGBA


class LedSimpleDemoPublisher(Node):

    def __init__(self):
        super().__init__('led_simple_demo_publisher')
        self.publisher_ = self.create_publisher(ColorRGBA, '/backlight/all', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):

        rand = random.randint(0, 100)/100
        g = 1.0 - rand
        b = 0.0

        led = ColorRGBA(r = rand, g = g, b = b, a = 1.0)
               # r = random.randint(0, 100)/100, 
               # g = random.randint(0, 100)/100, 
               # b = random.randint(0, 100)/100, a = 1.0)

        self.publisher_.publish(led)


def main(args=None):
    rclpy.init(args=args)

    leddemo_publisher = LedSimpleDemoPublisher()

    rclpy.spin(leddemo_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    leddemo_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
