from enum import Enum

class ControllerState():
    class Button():
        def __init__(self):
            self.state = False
    class JoyStick():
        def __init__(self, max_abs_factor=1.0):
            self.max_abs_factor = max_abs_factor
            self.x = 0.0
            self.y = 0.0
        def zero(self):
            self.x = 0
            self.y = 0
        def set_val(self, valx, valy):
            self.x = valx / self.max_abs_factor
            self.y = valy / self.max_abs_factor

    class Trigger():
        def __init__(self, max_abs_factor=1.0):
            self.max_abs_factor = max_abs_factor
            self.value = 0
        def set_val(self, value):
            self.value = value / self.max_abs_factor
        def zero(self):
            self.value = 0
    def __init__(self, max_trigger_factor, max_joystick_factor):
        self.right_stick = self.JoyStick(max_joystick_factor)
        self.left_stick = self.JoyStick(max_joystick_factor)
        self.right_trigger = self.Trigger(max_trigger_factor)
        self.left_trigger = self.Trigger(max_joystick_factor)
        self.brake = self.Button()
        self.l1 = self.Button()
        self.r1 = self.Button()
        self.gear = 0
    def set_sticks(self, leftx, lefty, rightx, righty):
        self.left_stick.set_val(leftx, lefty);
        self.right_stick.set_val(rightx, righty);
    def zero_sticks(self):
        """ sometimes the sticks can be stuck away from their real value
            I think because the pyPS4Controller module misses some events
            if they are too fast. for instance if the events are in too
            quick of a succession
        """
        self.right_stick.zero()
        self.left_stick.zero()

class Controller():
    def __init__(self, name, controller_map, max_trigger_factor=1.0, has_abs_triggers=False, max_joystick_factor=1.0):
        self.name = name
        self.max_trigger_factor = max_trigger_factor
        self.has_abs_triggers = has_abs_triggers
        self.max_joystick_factor = max_joystick_factor
        self.map = controller_map
        self.state = ControllerState(max_trigger_factor, max_joystick_factor)

    def update(self, joy_msg):
        self.state.set_sticks(
            joy_msg.axes[self.map["ABS_X"]],
            joy_msg.axes[self.map["ABS_Y"]],
            joy_msg.axes[self.map["ABS_RX"]],
            joy_msg.axes[self.map["ABS_RY"]])
        if self.has_abs_triggers:
            self.state.set_triggers(joy_msg.axes["ABS_Z"], joy_msg.axes["ABS_RZ"])
        self.state.brake.state = True if joy_msg.buttons[self.map['BTN_WEST']] != 0 else False
        # only count rising edges of shoulder buttons
        if joy_msg.buttons[self.map['BTN_TL']] != 0 and not self.state.l1.state:
            self.state.l1.state = True
            self.state.gear -= 1
        elif joy_msg.buttons[self.map['BTN_TL']] == 0:
            self.state.l1.state = False
        if joy_msg.buttons[self.map['BTN_TR']] != 0 and not self.state.r1.state:
            self.state.r1.state = True
            self.state.gear += 1
        elif joy_msg.buttons[self.map['BTN_TR']] == 0:
            self.state.r1.state = False
        if self.state.gear < 1:
            self.state.gear = 1
        if self.state.gear > 10:
            self.state.gear = 10

