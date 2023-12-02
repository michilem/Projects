import numpy as np
from matplotlib import pyplot as plt

def compute_range(rd):
    vmax = 5 * rd / (60000 + rd)
    vlow = 5 * rd / (120000 + rd) 
    return vmax, vlow, vmax - vlow

def find_rd():
    rd = np.arange(10000, 200000, 0.05)
    vmax, vlow, r = compute_range(rd)
    ix = np.argmax(r)
    print('rd =', rd[ix], 'range =', r[ix], "vmax =", vmax[ix], "vlow =", vlow[ix])
    plt.plot(rd, r)
    plt.plot(rd[ix], r[ix], 'ro')
    plt.xlabel('rd')
    plt.ylabel('range')
    plt.show()

find_rd()
