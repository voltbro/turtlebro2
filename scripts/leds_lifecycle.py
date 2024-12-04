#!/usr/bin/env python3

import rclpy
from itertools import cycle
from typing import Optional

from collections.abc import Iterable

from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

from std_msgs.msg import ColorRGBA
from turtlebro.msg import ColorRGBAArray 


class LedsDemoLifecycle(Node):

    def __init__(self, node_name, **kwargs):

        self.timer_period = 0.05  # seconds

        self.led_msg = ColorRGBAArray()
        self._pub: Optional[Publisher] = None
        self._timer: Optional[Timer] = None

        self.red_iter: Iterable[int] = None
        self.green_iter: Iterable[int] = None
        self.blue_iter: Iterable[int] = None

        super().__init__(node_name, **kwargs)
        
        self.get_logger().info('Node started')

    def timer_callback(self):

        if self._pub is None or not self._pub.is_activated:
            pass
        else:
            for key, msg in enumerate(self.led_msg.array):
                if msg.b > 0: self.led_msg.array[key].b = msg.b * 0.80
            
            self.led_msg.array[next(self.red_iter)]   = ColorRGBA(r = 1.0, g = 0.0, b = 0.0, a = 1.0)
            self.led_msg.array[next(self.green_iter)] = ColorRGBA(r = 0.0, g = 1.0, b = 0.0, a = 1.0)
            self.led_msg.array[next(self.blue_iter)]  = ColorRGBA(r = 0.0, g = 0.0, b = 1.0, a = 1.0)
            
            self._pub.publish(self.led_msg)        


    def on_configure(self, state: State) -> TransitionCallbackReturn:


        self._pub = self.create_lifecycle_publisher(ColorRGBAArray, '/backlight/array', 10)

        leds = list(range(24))
        self.red_iter = cycle(leds[2:] + leds[:2]) # +2 led cycle
        self.green_iter = cycle(leds[1:] + leds[:1]) # +1 led cycle
        self.blue_iter = cycle(leds)

        self.get_logger().info('on_configure() is called.')

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('on_activate() is called.')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self._timer)

        # clear all leds
        self.led_msg = ColorRGBAArray()
        self._pub.publish(self.led_msg)    

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