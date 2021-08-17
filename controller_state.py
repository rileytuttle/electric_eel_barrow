class ControllerState():
    class Button():
        def __init__(self):
            self.state = "up"
    class JoyStick():
        def __init__(self):
            self.x = 0
            self.y = 0
        def zero(self):
            self.x = 0
            self.y = 0
    class Trigger():
        def __init__(self, shift=True):
            """ values from -(2**(16-1)) to (2**(16-1)-1) -> [-32768, 32767]
                this represents full released to full press respectively
                need to shift into [0, 65535]
                we should take all values and add 32768
            """
            self.MIN = -(2**(16-1))
            if shift:
                self.value = 0
            else:
                self.value = self.MIN
            self.shift = shift
        def set(self, value):
            if self.shift:
                self.value = value - self.MIN
            else:
                self.value = value
        def zero(self):
            if self.shift:
                self.value = 0
            else:
                self.value = self.MIN
    def __init__(self):
        self.right_stick = self.JoyStick()
        self.left_stick = self.JoyStick()
        self.right_trigger = self.Trigger()
        self.left_trigger = self.Trigger()
        self.square = self.Button()
    def zero_sticks(self):
        self.right_stick.zero()
        self.left_stick.zero()
