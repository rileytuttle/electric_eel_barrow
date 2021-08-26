#!/usr/bin/env python3

from controller_state import ControllerState
from controller_parser import MyController
from robot import Robot
from time import sleep

import threading

import pdb

class Params():
    def __init__(self):
        self.robot = Robot()
        self.left_wheels = [1]
        self.right_wheels = [0]
        for left_pin, right_pin in zip(self.left_wheels, self.right_wheels):
            self.robot.add_wheel(pin=left_pin, dir="left")
            self.robot.add_wheel(pin=right_pin, dir="right")
        self.controller_state = ControllerState()
        self.controller = MyController(controller_state=self.controller_state, interface="/dev/input/js0", connecting_using_ds4drv=False)

        self.thread = threading.Thread(target=self.controller.listen)
        self.thread.start()
        self.update_rate = 0.1

def try_to_reconnect(params):
    while not params.controller.is_connected:
        params.controller_state = ControllerState()
        params.controller = MyController(controller_state=params.controller_state, interface="/dev/input/js0", connecting_using_ds4drv=False)
        print(f'trying to reconnect to /dev/input/js0')
        sleep(1)

def setup():
    params = Params()
    return params

def main(params):
    while True:
        # print(f'right stick x,y: ({cs.right_stick.x},{cs.right_stick.y}) ')
        # print(f'left stick x,y: ({cs.left_stick.x},{cs.left_stick.y}) ')
        # print(f'right trigger: {cs.right_trigger.value}')
        # print(f'left trigger: {cs.left_trigger.value}')
        #
        params.robot.process_controller_input(params.controller_state)
        params.robot.print_vels()
        sleep(params.update_rate)

if __name__ == "__main__":
    params = setup()
    main(params)
