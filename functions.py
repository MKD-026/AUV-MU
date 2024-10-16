import numpy as np
import sys
import serial

def diagonal_L(check):
    if check == 0:
        val = [0, 1900, 1800, 1800, 1200]
    else:
        val = [1, 1900, 1800, 1800, 1200]
    return val

def diagonal_R(check):
    if check == 0:
        val = [0, 1700, 1900, 1900, 1100]
    else:
        val = [1, 1700, 1900, 1900, 1100]
    return val


def straight(check):
    if check == 0:
        val = [0, 1900, 1900, 1900, 1100]
    else:
        val = [1, 1900, 1900, 1900, 1100]

    return val

def anticlockwise(check):
    if check == 0:
        val = [0, 1100, 1100, 1900, 1100]
    else:
        val = [1, 1100, 1100, 1900, 1100]

    return val

def clockwise(check):
    if check == 0:
        val = [0, 1200, 1800, 1200, 1200]
    else:
        val = [1, 1200, 1800, 1200, 1200]

    return val

