import rclpy
from itertools import cycle

from rclpy.node import Node

from std_msgs.msg import ColorRGBA
from turtlebro.msg import ColorRGBAArray 


class LedDemoPublisher(Node):

    def __init__(self):
        super().__init__('leddemo_publisher')
        self.publisher_ = self.create_publisher(ColorRGBAArray, '/backlight/array', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        leds = list(range(24))
        self.red_iter = cycle(leds[2:] + leds[:2]) # + 2 led cycle
        self.green_iter = cycle(leds[1:] + leds[:1]) # +1 led cycle
        self.blue_iter = cycle(leds)
        

    def timer_callback(self):
        led_msg = ColorRGBAArray()

        led_msg.array[next(self.red_iter)]   = ColorRGBA(r = 1.0, g = 0.0, b = 0.0, a = 0.0)
        led_msg.array[next(self.green_iter)] = ColorRGBA(r = 0.0, g = 1.0, b = 0.0, a = 0.0)
        led_msg.array[next(self.blue_iter)]  = ColorRGBA(r = 0.0, g = 0.0, b = 1.0, a = 0.0)
        

        self.publisher_.publish(led_msg)


def main(args=None):
    rclpy.init(args=args)

    leddemo_publisher = LedDemoPublisher()

    rclpy.spin(leddemo_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    leddemo_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()