#!/usr/bin/env python3

import rclpy
import random
from typing import Optional

from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

from std_msgs.msg import ColorRGBA


class LedsDemoLifecycle(Node):

    def __init__(self, node_name, **kwargs):

        self.timer_period = 1  # seconds

        self._pub: Optional[Publisher] = None
        self._timer: Optional[Timer] = None

        super().__init__(node_name, **kwargs)
        
        self.get_logger().info('Node started')

    def timer_callback(self):

        if self._pub is None or not self._pub.is_activated:
            pass
        else:
            rand = random.randint(0, 100)/100
            g = 1.0 - rand
            b = 0.0

            led = ColorRGBA(r = rand, g = g, b = b, a = 1.0)
            self._pub.publish(led)        


    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._pub = self.create_lifecycle_publisher(ColorRGBA, '/backlight/all', 10)
        self.get_logger().info('on_configure() is called.')

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('on_activate() is called.')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self._timer)

        # clear all leds
        led_msg = ColorRGBA()
        self._pub.publish(led_msg)    

        self.get_logger().info('on_deactivate() is called.')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:

        self.destroy_timer(self._timer)
        self.destroy_publisher(self._pub)

        self.red_iter = None
        self.green_iter = None
        self.blue_iter = None

        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:

        self.destroy_timer(self._timer)
        self.destroy_publisher(self._pub)

        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS


def main():
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = LedsDemoLifecycle('leds_lifecycle')
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
