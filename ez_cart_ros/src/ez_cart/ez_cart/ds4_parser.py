from pyPS4Controller.controller import Controller
import os
import time
from .ds4_state import DS4ControllerState
from ez_cart_interfaces.msg import Intent, WheelVels

class DS4Controller(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.controller_state = DS4ControllerState()
        self.intent = Intent()

    def listen(self, timeout=30, on_connect=None, on_disconnect=None, on_sequence=None):
        """
        Start listening for events on a given self.interface
        :param timeout: INT, seconds. How long you want to wait for the self.interface.
                        This allows you to start listening and connect your controller after the fact.
                        If self.interface does not become available in N seconds, the script will exit with exit code 1.
        :param on_connect: function object, allows to register a call back when connection is established
        :param on_disconnect: function object, allows to register a call back when connection is lost
        :param on_sequence: list, allows to register a call back on specific input sequence.
                        e.g [{"inputs": ['up', 'up', 'down', 'down', 'left', 'right,
                                         'left', 'right, 'start', 'options'],
                              "callback": () -> None)}]
        :return: None
        """
        def on_disconnect_callback():
            self.is_connected = False
            if on_disconnect is not None:
                on_disconnect()

        def on_connect_callback():
                self.is_connected = True
                if on_connect is not None:
                    on_connect()

        def wait_for_interface():
            print("Waiting for interface: {} to become available . . .".format(self.interface))
            for i in range(timeout):
                if os.path.exists(self.interface):
                    print("Successfully bound to: {}.".format(self.interface))
                    on_connect_callback()
                    return
                time.sleep(1)
                print("Timeout({} sec). Interface not available.".format(timeout))
                self.connection_failed = True

        def read_events():
            try:
                return _file.read(self.event_size)
            except IOError:
                print("Interface lost. Device disconnected?")
                on_disconnect_callback()
                exit(1)

        def check_for(sub, full, start_index):
            return [start for start in range(start_index, len(full) - len(sub) + 1) if
                    sub == full[start:start + len(sub)]]

        def unpack():
            __event = struct.unpack(self.event_format, event)
            return (__event[3:], __event[2], __event[1], __event[0])

        wait_for_interface()
        try:
            _file = open(self.interface, "rb")
            event = read_events()
            if on_sequence is None:
                on_sequence = []
            special_inputs_indexes = [0] * len(on_sequence)
            while not self.stop and event:
                (overflow, value, button_type, button_id) = unpack()
                if button_id not in self.black_listed_buttons:
                    self.__handle_event(button_id=button_id, button_type=button_type, value=value, overflow=overflow,
                                        debug=self.debug)
                for i, special_input in enumerate(on_sequence):
                    check = check_for(special_input["inputs"], self.event_history, special_inputs_indexes[i])
                    if len(check) != 0:
                        special_inputs_indexes[i] = check[0] + 1
                        special_input["callback"]()
                event = read_events()
        except KeyboardInterrupt:
            print("\nExiting (Ctrl + C)")
            on_disconnect_callback()
            exit(1)

    def on_R3_y_at_rest(self):
        self.controller_state.right_stick.y = 0
        self.intent.wheel_vels.right = 0.0

    def on_R3_x_at_rest(self):
        self.controller_state.right_stick.x = 0 
        self.intent.wheel_vels.right = 0.0

    def on_R3_up(self, amount):
        self.controller_state.right_stick.x = amount
        self.intent.wheel_vels.right = amount / -32768.0

    def on_R3_down(self, amount):
        self.controller_state.right_stick.x = amount
        self.intent.wheel_vels.right = amount / -32768.0

    def on_R3_right(self, amount):
        self.controller_state.right_stick.y = amount
        self.intent.wheel_vels.right = amount / -32768.0

    def on_R3_left(self, amount):
        self.controller_state.right_stick.y = amount
        self.intent.wheel_vels.right = amount / -32768.0

    def on_L3_y_at_rest(self):
        self.controller_state.left_stick.y = 0
        self.intent.wheel_vels.left = 0.0

    def on_L3_x_at_rest(self):
        self.controller_state.left_stick.x = 0 
        self.intent.wheel_vels.left = 0.0

    def on_L3_up(self, amount):
        self.controller_state.left_stick.x = amount
        self.intent.wheel_vels.left = amount / -32768.0

    def on_L3_down(self, amount):
        self.controller_state.left_stick.x = amount
        self.intent.wheel_vels.left = amount / -32768.0

    def on_L3_right(self, amount):
        self.controller_state.left_stick.y = amount
        self.intent.wheel_vels.left = amount / -32768.0

    def on_L3_left(self, amount):
        self.controller_state.left_stick.y = amount
        self.intent.wheel_vels.left = amount / -32768.0

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
        self.controller_state.square = True
        self.intent.brake = True;

    def on_square_release(self):
        self.controller_state.square = False
        self.intent.brake = False;

    def on_R3_press(self):
        pass

    def on_R3_release(self):
        pass

    def on_L3_press(self):
        pass

    def on_L3_release(self):
        pass

    def on_R1_press(self):
        self.controller_state.r1 = True
        self.intent.gear = 1

    def on_R1_release(self):
        self.controller_state.r1 = False

    def on_L1_press(self):
        self.controller_state.l1 = True
        self.intent.gear = 0

    def on_L1_release(self):
        self.controller_state.l1 = False

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

