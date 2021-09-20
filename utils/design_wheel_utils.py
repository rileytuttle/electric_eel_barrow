#!/usr/bin/env python3
from math import *
import numpy as np

def mph_to_fps(mph):
    """ convert mph to feet per second
    """
    return (mph * 5280 / (60 * 60))

def fps_to_mph(fps):
    """ convert fps to mph
    """
    return(fps / 5280 * (60 * 60))

def mph_to_rpm(mph, diam):
    """ if given the mph and wheel diameter
        return the rpms required to reach that speed
        diam in feet
    """
    fps = mph_to_fps(mph)
    feet_per_minute = fps * 60
    rpm = feet_per_minute / (diam * 2 * pi)
    return rpm

def rpm_to_mph(rpm, diam):
    """ given an rpm and diameter of wheel
        in feet, find the speed we can expect in mph
    """
    feet_per_min = rpm * diam * 2 *pi
    fps = feet_per_min / 60
    return (fps_to_mph(fps))

def gear_ratio(rpm, diam, mph):
    """ given a motor output rpm
        a proposed wheel diameter in feet
        and a target mph find the gearing required
    """
    post_gear_rpm = mph_to_rpm(mph, diam)
    return rpm / post_gear_rpm
