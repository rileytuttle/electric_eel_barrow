class ControllerState():
    class Button():
        def __init__(self):
            self.state = False
    class JoyStick():
        def __init__(self):
            self.x = 0
            self.y = 0
        def zero(self):
            self.x = 0
            self.y = 0
    class Trigger():
        def __init__(self, shift=True):
            """ values from -(2**(16-1)) to (2**(16-1)-1) -> [-32768, 32767] (16 bit signed integer)
                this represents full released to full press respectively
                need to shift into [0, 65535]
                we should take all values and subtract -32768
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
        self.l1 = self.Button()
        self.r1 = self.Button()
        self.share = self.Button()
        self.options = self.Button()
    def zero_sticks(self):
        """ sometimes the sticks can be stuck away from their real value
            I think because the pyPS4Controller module misses some events
            if they are too fast. for instance if the events are in too
            quick of a succession
        """
        self.right_stick.zero()
        self.left_stick.zero()
