#!/usr/bin/env python3

from controller_parser import MyController
from controller_state import ControllerState


cs = ControllerState()
controller = MyController(controller_state=cs, interface="/dev/input/js0")


controller.listen()
