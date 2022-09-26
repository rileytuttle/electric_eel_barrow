# from pynput.mouse import Listener

# last_position = None

# def on_move(x, y):
#     global last_position
#     if last_position:
#         if x > last_position:
#             print('mouse moved right')
#         elif x < last_position:
#             print('mouse moved left')
#         last_position = x

# with Listener(on_move=on_move) as listener:
#     listener.join()

import pyautogui
import time

while True:
    prev_x, prev_y = pyautogui.position()
    time.sleep(0.3)
    curr_x, curr_y = pyautogui.position()
    if (curr_x - prev_x) > 0:
        print("move right")
    elif (prev_x - curr_x) > 0:
        print("move left")
    if (curr_y - prev_y) > 0:
        print("move down")
    elif (prev_y - curr_y) > 0:
        print("move up")

