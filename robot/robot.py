import pdb
from pysabertooth import Sabertooth

class DebugWheelController():
    def __init__(self):
        pass
    def drive(self, wheel, vel):
        pass

class Robot():
    class Wheel():
        def __init__(self, pin, wheel_controller, is_turnable=False):
            self.pin = pin
            self.vel = 0.0
            self.wheel_controller = wheel_controller
            self.is_turnable = is_turnable
            self.angle = 0.0 # angle can be between -180 and 180. 0 is straight forward -180/180 are straight back. assume start at 0. TODO: add in calibration logic
        def set_vel(self, new_vel):
            self.vel = new_vel
            self.wheel_controller.drive(self.pin, new_vel)
        def __str__(self):
            return f'wheel: {self.pin}, vel: {self.vel}, angle: {self.angle}'
        def set_angle(self, requested_angle):
            self.angle = requested_angle
            # do whatever needs to be done to turn the wheels to that angle
    def __init__(self, has_turnable_wheels=False):
        self.wheels = {"left": [], "right": []}
        self.vel_multiplier = 50
        self.wheel_controller = Sabertooth("/dev/ttyACM0")
        # self.wheel_controller = DebugWheelController()
        self.has_turnable_wheels = has_turnable_wheels
    def add_wheel(self, dir, pin=None):
        """ add a wheel to the wheel lists
            requires a pin that will control the wheel
        """
        try:
            self.wheels[dir].append(self.Wheel(pin, self.wheel_controller, self.has_turnable_wheels))
        except ValueError as ve:
            raise ValueError("invalid wheel side")
    def set_vels(self, left_vel, right_vel):
        for wheel in self.wheels["left"]:
            wheel.set_vel(left_vel)
        for wheel in self.wheels["right"]:
            wheel.set_vel(right_vel)
    def set_angles(self, angles):
        # at the moment we are assuming that there are 4 wheels set up in a skid steer arrangement
        self.wheels["left"][0] = angles["left_front"] # until some abstraction is done we are assuming that the 0 wheel is the front
        self.wheels["left"][1] = angles["left_rear"]
        self.wheels["right"][0] = angles["right_front"]
        self.wheels["right"][1] = angles["right_rear"]
    def generate_angles(self, controller):
        max_angle = 45.0
        # angles are assumed to be calculated for the left side
        # right side wheels are mirrored. so really it should be thought of as angle towards the cart and away from the cart
        motion_type = get_motion_type(controller)
        angles = {"left_front": 0.0, "left_rear": 0.0, "right_front": 0.0, "right_rear": 0.0}
        if motion_type == "arc": # both wheels going same direction but one is faster
            # wheels should turn proportionate to the difference in the wheel speeds
            proportion = (controller.intent.wheel_vels["left"] - controller.intent.wheel_vels["right"])
            # say left is greater than right we should be turning right. so then front left would be angled toward the cart which is positive
            # rear left would be angled away from the cart or negative
            # and the right side wheels would be mirrored
            angle["left_front"] = proportion * max_angle
            angle["left_rear"] = proportion * -max_angle
            angle["right_front"] = proportion * -max_angle
            angle["right_rear"] = proportion * max_angle
        elif motion_type == "tight arc": # wheels going different directions different speeds
            # need to think about this one more
            raise NotImplementedError("need to think this through and implement")
        elif motion_type == "straight": # wheels going same speed same direction
            angle["left_front"] = 0.0
            angle["left_rear"] = 0.0
            angle["right_front"] = 0.0
            angle["right_rear"] = 0.0
        elif motion_type == "turn in place": # wheels going same speed different direction
            # should be cranked to the max angle is assumed to be 45 degrees
            # max angle for this and max angle the wheels can turn do not have to be the same
            # other behaviors may need a different angle. say we add a strafe function we would want everything to turn straight to the side
            angle["left_front"] = max_angle
            angle["left_rear"] = -max_angle
            angle["right_front"] = max_angle
            angle["right_rear"] = -max_angle
        else:
            raise ValueError(f"invalid motion type {motion_type}")
        return angles
    def zero_vels(self):
      
        """ zero out both wheels
            for stopping situations
        """
        self.set_vels(0.0, 0.0)
    def process_controller_input(self, controller):
        """ process controller state into something that the robot can understand
            right now that is just to convert the
        """
        # stop robot if we have lost the controller
        if not controller.is_connected:
            self.zero_vels()
        elif controller.intent.brake:
            """ because there can be some issues with latching the last joystick values
                we sometimes need a specific button to hit the brakes
            """
            self.zero_vels()
        else:
            # this chunk doesn't use the assumption but I am assuming that
            # the events only send for a change in the state
            if controller.intent.gear == 0:
                # first gear
                self.vel_multiplier = 50
            elif controller.intent.gear == 1:
                # second gear
                self.vel_multiplier = 100

            if self.has_turnable_wheels:
                angles = self.generate_angles(controller)
                self.set_angles(angles)
            left_vel = controller.intent.wheel_vels["left"] * self.vel_multiplier
            right_vel = controller.intent.wheel_vels["right"] * self.vel_multiplier
            self.set_vels(left_vel, right_vel)
    def print_vels(self):
        """ print out the left and right wheel vels
            assuming for the moment the wheel vels are the same per side
        """
        print(self.wheels['left'][0])
        print(self.wheels['right'][0])

