import pdb
class Robot():
    class Wheel():
        def __init__(self, pin):
            self.pin = 0
            self.vel = 0
        def set_vel(self, new_vel):
            self.vel = new_vel
    def __init__(self):
        self.left_wheels = []
        self.right_wheels = []
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
        left_vel = controller_state.left_stick.x / -32768
        right_vel = controller_state.right_stick.x / -32768
        self.set_vels(left_vel, right_vel)
    def print_vels(self):
        """ print out the left and right wheel vels
            assuming for the moment the wheel vels are the same per side
        """
        print(f'left_vels: {self.left_wheels[0].vel}')
        print(f'right_vels: {self.right_wheels[0].vel}')

