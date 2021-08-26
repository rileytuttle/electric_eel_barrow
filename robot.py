import pdb
from pysabertooth import Sabertooth

class Robot():
    class Wheel():
        def __init__(self, pin):
            self.pin = pin
            self.vel = 0.0
            self.saber = Sabertooth("/dev/ttyACM0")
        def set_vel(self, new_vel):
            self.vel = new_vel
            self.saber.drive(self.pin, new_vel)
    def __init__(self):
        self.left_wheels = []
        self.right_wheels = []
        self.vel_multiplier = 50
    def add_wheel(self, dir, pin=None):
        """ add a wheel to the wheel lists
            requires a pin that will control the wheel
        """
        if dir == "left":
            self.left_wheels.append(self.Wheel(pin))
        elif dir == "right":
            self.right_wheels.append(self.Wheel(pin))
        else:
            raise ValueError("invalid wheel side")
    def set_vels(self, left_vel, right_vel):
        # print(f'new left vel {left_vel}')
        # print(f'new right vel {right_vel}')
        for wheel in self.left_wheels:
            wheel.set_vel(left_vel)
        for wheel in self.right_wheels:
            wheel.set_vel(right_vel)

    def process_controller_input(self, controller_state):
        """ process controller state into something that the robot can understand
            right now that is just to convert the
        """
        # this chunk doesn't use the assumption but I am assuming that
        # the events only send for a change in the state
        if (controller_state.l1 == "down"):
            self.vel_multiplier = 50
        else if (controller_state.r1 == "down"):
            self.vel_multiplier = 100
            
        left_vel = (controller_state.left_stick.x / -32768.0) * self.vel_multiplier
        right_vel = (controller_state.right_stick.x / -32768.0) * self.vel_multiplier
        self.set_vels(left_vel, right_vel)
        if controller_state.square == "down":
            """ because there can be some issues with latching the last joystick values
                we sometimes need a specific button to hit the brakes
            """
            controller_state.zero_sticks()
            self.set_vels(0.0, 0.0)
    def print_vels(self):
        """ print out the left and right wheel vels
            assuming for the moment the wheel vels are the same per side
        """
        print(f'left_vels: {self.left_wheels[0].vel}')
        print(f'right_vels: {self.right_wheels[0].vel}')

