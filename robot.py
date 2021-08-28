import pdb
from pysabertooth import Sabertooth

class Robot():
    class Wheel():
        def __init__(self, pin, wheel_controller):
            self.pin = pin
            self.vel = 0.0
            self.wheel_controller = wheel_controller
        def set_vel(self, new_vel):
            self.vel = new_vel
            self.wheel_controller.drive(self.pin, new_vel)
    def __init__(self):
        self.wheels = {"left": [], "right", []}
        self.vel_multiplier = 50
        self.wheel_controller = Sabertooth("/dev/ttyACM0")
    def add_wheel(self, dir, pin=None):
        """ add a wheel to the wheel lists
            requires a pin that will control the wheel
        """
        try:
            self.wheels[dir].append(self.Wheel(pin, self.wheel_controller))
        except ValueError as ve:
            raise ValueError("invalid wheel side")
    def zero_vels(self):
        """ zero out both wheels
            for stopping situations
        """
        set_vels(0.0, 0.0)

    def set_vels(self, left_vel, right_vel):
        for wheel in self.wheels["left"]:
            wheel.set_vel(left_vel)
        for wheel in self.wheels["right"]:
            wheel.set_vel(right_vel)

    def process_controller_input(self, controller_state, controller):
        """ process controller state into something that the robot can understand
            right now that is just to convert the
        """
        # stop robot if we have lost the controller
        if not controller.is_connected:
            controller_state.zero_sticks()
            self.zero_vels()
        elif controller_state.square == "down":
            """ because there can be some issues with latching the last joystick values
                we sometimes need a specific button to hit the brakes
            """
            controller_state.zero_sticks()
            self.zero_vels()
        else:
            # this chunk doesn't use the assumption but I am assuming that
            # the events only send for a change in the state
            if (controller_state.l1 == "down"):
                self.vel_multiplier = 50
            else if (controller_state.r1 == "down"):
                self.vel_multiplier = 100

            left_vel = (controller_state.left_stick.x / -32768.0) * self.vel_multiplier
            right_vel = (controller_state.right_stick.x / -32768.0) * self.vel_multiplier
            self.set_vels(left_vel, right_vel)
    def print_vels(self):
        """ print out the left and right wheel vels
            assuming for the moment the wheel vels are the same per side
        """
        print(f'left_vels: {self.wheels["left"][0].vel}')
        print(f'right_vels: {self.wheels["right"][0].vel}')

