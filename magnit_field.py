import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import math





def calculete_B_in_decard(r, theta):
    B0 = 3.12e-5
    RE = 6.370
    alpha = np.radians(9.6)

    fac = B0 * (RE / r)**3
    B_in_sphere = -2 * fac * np.cos(theta + alpha), -fac * np.sin(theta + alpha)
    phi = 0
    B_x = B_in_sphere[0] * math.sin(B_in_sphere[1]) * math.cos(phi)
    B_y = B_in_sphere[0] * math.sin(B_in_sphere[1]) * math.sin(phi)
    B_z = B_in_sphere[0] * math.cos(B_in_sphere[1])
    B_dec = np.array([B_x, B_y, B_z])
    return B_dec
