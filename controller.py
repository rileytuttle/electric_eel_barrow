#!/usr/bin/env python3

""" this is a general purpose controller intent class
    it will capture the intent of whatever controller we
    are using. ds4 or phsyical or whatever
"""

class ControllerIntent():
    """ the controller should know about the following
        wheel velocities
        brake button
        connection state
    """
    def __init__(self, thread):
        self.wheel_vels = {"left": [0], "right":[0]}
        self.brake = False
        self.gear = 0
        self.controller = DS4Controller(interface="/dev/input/js0", connecting_using_ds4drv=False)
