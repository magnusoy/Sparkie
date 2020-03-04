import numpy as np
import time


def get_position(velocity=0.0, acceleration=0.0, time=0.0):
    return (velocity * time) - (0.5 * acceleration * (time ** 2))

def millis():
        """Returns the current time in milliseconds
        Returns
        -------
        current time in milliseconds
        """

        return time.time()

def update_value(new, condition):
    if abs(new) > condition:
        return new
    else:
        return 0.0
    

def save_to_csv(x, y, z):
    with open('slam.csv', 'w') as f:
        f.write("X,,Y,Z\n")
        for i, ii, iii in zip(x, y, z):
            f.write(f'{i},{ii},{iii}\n')