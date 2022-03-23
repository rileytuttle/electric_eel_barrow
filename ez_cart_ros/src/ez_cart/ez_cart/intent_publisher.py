import rclpy
from rclpy.node import Node

import pdb

from .ds4_parser import DS4Controller

from ez_cart_interfaces.msg import Intent, WheelVels

class IntentPublisher(Node):
    def __init__(self):
        super().__init__('intent_publisher')
        self.publisher_ = self.create_publisher(Intent, 'intent', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.controller = DS4Controller(interface="/dev/input/js0", connecting_using_ds4drv=False)
        self.i = 0
    def timer_callback(self):
        msg = self.controller.intent
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    intent_publisher = IntentPublisher()
    rclpy.spin(intent_publisher)

    intent_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
