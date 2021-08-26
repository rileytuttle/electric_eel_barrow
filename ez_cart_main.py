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

        self.thread = threading.Thread(target=self.controller.listen, kwargs={"timeout": 5})
        self.thread.start()
        self.update_rate = 0.1

def connect_controller(params):
    while not params.controller.is_connected:
        params.controller_state = ControllerState()
        params.controller = MyController(controller_state=params.controller_state, interface="/dev/input/js0", connecting_using_ds4drv=False)
        print(f'trying to reconnect to /dev/input/js0')
        sleep(1)

def setup():
    params = Params()
    return params

def runner():
    try:
        params = setup()
        application(params)
    except Exception as e:
        # pdb.set_trace()
        # params.thread.join()
        # sleep(2) # sleep for 2 seconds before trying again
        # runner()
        pass
        
def application(params):
    while True:
        if not params.thread.is_alive():
            raise Exception("thread quit")
        if params.controller.connection_failed:
            raise Exception("controller connection failed")
        params.robot.process_controller_input(params.controller_state)
        sleep(params.update_rate)

if __name__ == "__main__":
    runner()
