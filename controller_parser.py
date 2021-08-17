from pyPS4Controller.controller import Controller

class MyController(Controller):
    def __init__(self, controller_state, **kwargs):
        Controller.__init__(self, **kwargs)
        self.controller_state = controller_state

    def on_R3_y_at_rest(self):
        self.controller_state.right_stick.y = 0

    def on_R3_x_at_rest(self):
        self.controller_state.right_stick.x = 0 

    def on_R3_up(self, amount):
        self.controller_state.right_stick.x = amount

    def on_R3_down(self, amount):
        self.controller_state.right_stick.x = amount

    def on_R3_right(self, amount):
        self.controller_state.right_stick.y = amount

    def on_R3_left(self, amount):
        self.controller_state.right_stick.y = amount

    def on_L3_y_at_rest(self):
        self.controller_state.left_stick.y = 0

    def on_L3_x_at_rest(self):
        self.controller_state.left_stick.x = 0 

    def on_L3_up(self, amount):
        self.controller_state.left_stick.x = amount

    def on_L3_down(self, amount):
        self.controller_state.left_stick.x = amount

    def on_L3_right(self, amount):
        self.controller_state.left_stick.y = amount

    def on_L3_left(self, amount):
        self.controller_state.left_stick.y = amount

    def on_R2_press(self, amount):
        self.controller_state.right_trigger.set(amount)

    def on_R2_release(self):
        self.controller_state.right_trigger.zero()
    
    def on_L2_press(self, amount):
        self.controller_state.left_trigger.set(amount)

    def on_L2_release(self):
        self.controller_state.left_trigger.zero()

    def on_x_press(self):
        pass

    def on_x_release(self):
        pass

    def on_triangle_press(self):
        pass

    def on_triangle_release(self):
        pass

    def on_circle_press(self):
        pass

    def on_circle_release(self):
        pass

    def on_square_press(self):
        pass

    def on_square_release(self):
        pass

    def on_R3_press(self):
        pass

    def on_R3_release(self):
        pass

    def on_L3_press(self):
        pass

    def on_L3_release(self):
        pass

    def on_R1_press(self):
        pass

    def on_R1_release(self):
        pass

    def on_L1_press(self):
        pass

    def on_L1_release(self):
        pass

    def on_share_press(self):
        pass

    def on_share_release(self):
        pass

    def on_options_press(self):
        pass

    def on_options_release(self):
        pass

    def on_left_arrow_press(self):
        pass

    def on_left_right_arrow_release(self):
        pass

    def on_up_arrow_press(self):
        pass

    def on_up_down_arrow_release(self):
        pass

    def on_right_arrow_press(self):
        pass

    def on_down_arrow_press(self):
        pass

    def on_playstation_button_press(self):
        pass

    def on_playstation_button_release(self):
        pass

