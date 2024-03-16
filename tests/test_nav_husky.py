#!/usr/bin/env python3
import numpy as np
import math
from matplotlib import pyplot as plt

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True

def f(x):
       # Calc which of the 12 direction classes the desired theta is in
    shift = math.pi / 4
    shifted_theta = x + shift
    desired_class = math.floor(shifted_theta /(2 * math.pi / 12)) % 12 
    return desired_class

def g(x):
    return min(1.1*max((1 - abs(x)), 0), 1)

def h(distance_to_destination):
    VMAX = 1
    return max(max(0, min(VMAX*distance_to_destination, VMAX)), -(VMAX*(distance_to_destination-1))**2 + VMAX)

def i(minDist):
    OBSTACLE_DIST_THRESH = 7
    return 15**((OBSTACLE_DIST_THRESH - minDist)/(OBSTACLE_DIST_THRESH)-1)



#x = np.linspace(-3.14, 3.14, 100)
#x = np.linspace(-1, 1, 200)

x = np.linspace(0, 7, 200)

y = []
for value in x:
    y.append(i(value))


plt.plot(x, y, color='red')

plt.show()

"""
import matplotlib.pyplot as plt
import numpy as np
import math

def test(desired_theta):
    # Calc which of the 12 direction classes the desired theta is in
    shift = 2 * math.pi / 12 / 2
    shifted_theta = desired_theta + shift
    desired_class = int(shifted_theta /(2 * math.pi) * 12) % 12
    return desired_class


# 100 linearly spaced numbers
x = np.linspace(-3.14,3.14,100)

plt.plot(x, test(x), color='red')

plt.show()

"""