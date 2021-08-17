#!/usr/bin/env python3

from controller_state import ControllerState
from controller_parser import MyController
from robot import Robot
from time import sleep

import threading

import pdb


robot = Robot()
left_wheels = [0, 1]
right_wheels = [2, 3]
for wheel in left_wheels:
    robot.add_wheel(pin=wheel, dir="left")
for wheel in right_wheels:
    robot.add_wheel(pin=wheel, dir="right")
cs = ControllerState()

controller = MyController(controller_state=cs, interface="/dev/input/js0", connecting_using_ds4drv=False, on_disconnect=on_disconnect)
# you can start listening before controller is paired, as long as you pair it within the timeout window

# controller.listen(timeout=60)

t = threading.Thread(target=controller.listen)
t.start()
update_rate = 1

while True:
    # print(f'right stick x,y: ({cs.right_stick.x},{cs.right_stick.y}) ')
    # print(f'left stick x,y: ({cs.left_stick.x},{cs.left_stick.y}) ')
    # print(f'right trigger: {cs.right_trigger.value}')
    # print(f'left trigger: {cs.left_trigger.value}')
    robot.process_controller_input(cs)
    robot.print_vels()
    sleep(update_rate)
