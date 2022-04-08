import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from .ds4_state import DS4ControllerState
from ez_cart_interfaces.msg import Intent, WheelVels

# PS4_CODE_MAP = {
#     'ABS_X': 0,
#     'ABS_Y': 1,
#     'ABS_Z': 2,
#     'ABS_RX': 3,
#     'ABS_RY': 4,
#     'ABS_RZ': 5,
#     'ABS_HAT0X': 6,
#     'ABS_HAT0Y': 7,
#     'BTN_EAST': 0,
#     'BTN_SOUTH': 1,
#     'BTN_NORTH': 2,
#     'BTN_WEST': 3,
#     'BTN_Z': 4,
#     'BTN_TL': 5,
#     'BTN_TR': 6,
#     'BTN_TL2': 7,
#     'BTN_TR2': 8,
#     'BTN_MODE': 9,
#     'BTN_SELECT': 10,
#     'BTN_START': 11,
#     'BTN_THUMBL': 12,
#     'BTN_THUMBR': 13
# }

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
        self.controller_state = DS4ControllerState()
    def timer_callback(self):
        msg = Intent()
        msg.wheel_vels.left = self.controller_state.left_stick.y
        msg.wheel_vels.right = self.controller_state.right_stick.y
        msg.brake = self.controller_state.square.state
        msg.gear = self.controller_state.gear
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        # set the left joystick
        self.controller_state.left_stick.x = msg.axes[PS4_CODE_MAP['ABS_X']]
        self.controller_state.left_stick.y = msg.axes[PS4_CODE_MAP['ABS_Y']]
        self.controller_state.right_stick.x = msg.axes[PS4_CODE_MAP['ABS_RX']]
        self.controller_state.right_stick.y = msg.axes[PS4_CODE_MAP['ABS_RY']]
        self.controller_state.left_trigger.value = msg.axes[PS4_CODE_MAP['ABS_Z']]
        self.controller_state.right_trigger.value = msg.axes[PS4_CODE_MAP['ABS_RZ']]
        self.controller_state.square.state = True if msg.buttons[PS4_CODE_MAP['BTN_WEST']] != 0 else False

        # only count rising edges of shoulder buttons
        if msg.buttons[PS4_CODE_MAP['BTN_TL']] != 0 and self.controller_state.l1.state = False:
            self.controller_state.l1.state = True
            self.controller_state.gear -= 1
        elif msg.buttons[PS4_CODE_MAP['BTN_TL']] == 0:
            self.controller_state.l1.state = False
        if msg.buttons[PS4_CODE_MAP['BTN_TR']] != 0 and self.controller_state.r1.state = False:
            self.controller_state.r1.state = True
            self.controller_state.gear += 1
        elif msg.buttons[PS4_CODE_MAP['BTN_TR']] == 0:
            self.controller_state.r1.state = False
        if self.controller_state.gear < 1:
            self.controller_state.gear = 1
        if self.controller_state.gear > 10:
            self.controller_state.gear = 10

def main(args=None):
    rclpy.init(args=args)

    intent_publisher = IntentPublisher()
    rclpy.spin(intent_publisher)

    intent_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
