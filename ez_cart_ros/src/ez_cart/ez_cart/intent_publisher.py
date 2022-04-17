import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from .controller import Controller
from ez_cart_interfaces.msg import Intent, WheelVels
import re
import os

PS4_CODE_MAP = {
    'ABS_X': 0,
    'ABS_Y': 1,
    'ABS_Z': 2,
    'ABS_RX': 3,
    'ABS_RY': 4,
    'ABS_RZ': 5,
    'ABS_HAT0X': 6,
    'ABS_HAT0Y': 7,
    'BTN_EAST': 0,
    'BTN_SOUTH': 1,
    'BTN_NORTH': 2,
    'BTN_WEST': 3,
    'BTN_TL': 4,
    'BTN_TR': 5,
    'BTN_Z': 6,
    'BTN_TL2': 7,
    'BTN_TR2': 8,
    'BTN_MODE': 9,
    'BTN_SELECT': 10,
    'BTN_START': 11,
    'BTN_THUMBL': 12,
    'BTN_THUMBR': 13
}

PRO_CODE_MAP = {
    'ABS_X': 0,
    'ABS_Y': 1,
    'ABS_RX': 2,
    'ABS_RY': 3,
    'ABS_HAT0X': 4,
    'ABS_HAT0Y': 5,
    'BTN_SOUTH': 0,
    'BTN_EAST': 1,
    'BTN_WEST': 2,
    'BTN_NORTH': 3,
    'BTN_TL': 4,
    'BTN_TR': 5,
    'BTN_TL2': 6,
    'BTN_TR2': 7,
    'BTN_SELECT': 8,
    'BTN_START': 9,
    'BTN_THUMBL': 10,
    'BTN_THUMBR': 11,
    'BTN_HOME': 12,
    'BTN_SHARE': 13
}

controllers = {"Pro Controller": Controller("Pro Controller", PRO_CODE_MAP, max_joystick_factor=1.0),
               "Sony Wireless Controller": Controller("Sony Wireless Controller", PS4_CODE_MAP, has_abs_triggers=True)}

class IntentPublisher(Node):
    def __init__(self):
        super().__init__('intent_publisher')
        self.publisher_ = self.create_publisher(Intent, 'intent', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.controller_type = self.get_controller_type()
        if self.controller_type is None:
            # do something idk
            raise NotImplementedError("not sure what to do with no controller")
        self.controller = controllers[self.controller_type] 

    def get_controller_type(self):
        for line in os.popen('dmesg').readlines():
            rematch = re.search("input: (.*) as .*", line)
            if rematch is not None:
                if rematch.group(1) in controllers.keys():
                    print(f'using {rematch.group(1)}')
                    return rematch.group(1)
        return None

    def timer_callback(self):
        msg = Intent()
        msg.wheel_vels.left = self.controller.state.left_stick.y
        msg.wheel_vels.right = self.controller.state.right_stick.y
        msg.brake = self.controller.state.brake.state
        msg.gear = self.controller.state.gear
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        self.controller.update(msg)

def main(args=None):
    rclpy.init(args=args)

    intent_publisher = IntentPublisher()
    rclpy.spin(intent_publisher)

    intent_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
