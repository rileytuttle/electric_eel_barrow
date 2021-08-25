#!/usr/bin/env python3

from pysabertooth import Sabertooth
from time import sleep

saber = Sabertooth("/dev/ttyACM1")

while True:
    saber.drive(1, 50)
    sleep(3)
    saber.drive(1, 0)
    sleep(3)
    saber.drive(1, -50)
    sleep(3)
    saber.drive(1, 0)
    sleep(3)



                                    
