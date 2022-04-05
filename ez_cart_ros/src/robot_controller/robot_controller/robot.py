import rclpy
from rclpy.node import Node

from pysabertooth import Sabertooth

from ez_cart_interfaces.msg import Intent, WheelVels, CommandedVels

class DebugWheelController():
    def __init__(self):
        pass
    def drive(self, wheel, vel):
        pass

class RobotController(Node):
    class Wheel():
        def __init__(self, pin, wheel_controller):
            self.pin = pin
            self.vel = 0.0
            self.wheel_controller = wheel_controller
        def set_vel(self, new_vel):
            self.vel = new_vel
            print(f'setting wheel to {new_vel}')
            self.wheel_controller.drive(self.pin, new_vel)
        def __str__(self):
            return f'wheel: {self.pin}, vel: {self.vel}'
    def __init__(self):
        super().__init__('ez_cart')
        # setup subscriber
        self.subscription = self.create_subscription(
            Intent,
            'intent',
            self.listener_callback,
            10)
        # setup publisher
        self.publisher = self.create_publisher(CommandedVels, "commanded_vels", 10)
        timer_period = 1 # 1 second because this doesn't need to be super fast (is not consumed except for debug)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # setup other stuff
        self.wheels = {"left": [], "right": []}
        self.vel_multiplier = 50
        self.wheel_controller = Sabertooth("/dev/ttyACM0")
        self.left_wheels = [1]
        self.right_wheels = [0]
        for left_pin, right_pin in zip(self.left_wheels, self.right_wheels):
            self.add_wheel(pin=left_pin, dir="left")
            self.add_wheel(pin=right_pin, dir="right")
    def timer_callback(self):
        msg = CommandedVels()
        msg.left = [wheel.vel for wheel in self.wheels['left']]
        msg.right = [wheel.vel for wheel in self.wheels['right']]
        self.publisher.publish(msg)

    def add_wheel(self, dir, pin=None):
        """ add a wheel to the wheel lists
            requires a pin that will control the wheel
        """
        try:
            self.wheels[dir].append(self.Wheel(pin, self.wheel_controller))
        except ValueError as ve:
            raise ValueError("invalid wheel side")

    def set_vels(self, left_vel, right_vel):
        for wheel in self.wheels["left"]:
            wheel.set_vel(left_vel)
        for wheel in self.wheels["right"]:
            wheel.set_vel(right_vel)

    def zero_vels(self):
        """ zero out both wheels
            for stopping situations
        """
        self.set_vels(0.0, 0.0)

    def print_vels(self):
        """ print out the left and right wheel vels
            assuming for the moment the wheel vels are the same per side
        """
        print(self.wheels['left'][0])
        print(self.wheels['right'][0])

    def listener_callback(self, msg):
        """ process controller Intent into something that the robot can understand
        """
        # stop robot if we have lost the controller
        # if not controller.is_connected:
        #     self.zero_vels()
        if msg.brake:
            """ because there can be some issues with latching the last joystick values
                we sometimes need a specific button to hit the brakes
            """
            self.zero_vels()
        else:
            # this chunk doesn't use the assumption but I am assuming that
            # the events only send for a change in the state
            if msg.gear == 0:
                # first gear
                self.vel_multiplier = 50
            elif msg.gear == 1:
                # second gear
                self.vel_multiplier = 100

            left_vel = msg.wheel_vels.left * self.vel_multiplier
            right_vel = msg.wheel_vels.right * self.vel_multiplier
            self.set_vels(left_vel, right_vel)
def main(args=None):
    rclpy.init(args=args)

    robot_node = RobotController()
    rclpy.spin(robot_node)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()