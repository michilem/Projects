# draw arc in 3d space
# https://www.codementor.io/hirengadhiya/python-matplotlib-plotting-an-arc-in-3d-plot-wor3d4gzg
# Callin Switzer
# 16 July 2019
# 
# run this first: pip install --upgrade matplotlib

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from itertools import product

import sys
print("Python Version:", sys.version)

# Tested and used parameters in the prior tasks:
# 1. L = 2, T = pi, P = pi/4
# 2. L = 5, T = pi/2, P = pi/2
# 3. L = 5, T = pi/4, P = 0

d = 0.3 # changed from 0.1 (in previous tasks) to 0.3 for better visualization 
STEPS = 500 # Animation 'speed', increase to slow it down
s = 6 # size of scatter plot points

# Specify the desired configuration you want to visualize
L_DESIRED = 5.0
PHI_DESIRED = np.pi/4
THETA_DESIRED = np.pi

# define starting values of the above parameters - set the starting value as desired value to keep the parameter constant
L_START = 2.0
PHI_START = 0
THETA_START = np.pi/2

# discretize for plotting
Ls = np.linspace(L_START, L_DESIRED, STEPS)
Phis = np.linspace(PHI_START, PHI_DESIRED, STEPS)
Thetas = np.linspace(THETA_START, THETA_DESIRED, STEPS)

def compute_lenghts(L, phi, theta, radius):
    i = np.array(range(1, 5))
    Phis = (2 * i - 1) * np.pi / 4 - phi
    Ls = L - theta * d * np.cos(Phis)
    return Ls, Phis

def plot_arc(L, theta, phi, phi_i, ax, label, color):
    radius = L / theta
    X, Y, Z = sphere2cart(radius, np.linspace(0, theta, num = 100), phi)
    x_offset, y_offset = pol2cart(radius, phi)
    x_offset_2, y_offset_2 = pol2cart(d, phi_i+phi)
    X += x_offset + x_offset_2
    Y += y_offset + y_offset_2
    ax.plot(X, Y, Z, label=label, color = color)
    ax.scatter(X[-1], Y[-1], Z[-1], label, color = color, s=s)
    return X[-1], Y[-1], Z[-1]

def cart2sphere(x, y, z):
    r = np.sqrt(x**2 + y**2 + z**2)
    theta = np.arccos(z, r)
    phi = np.arctan2(y, x)
    return r, theta, phi

def sphere2cart(r, theta, phi):
    theta = theta - np.pi/2
    x = r * np.sin(theta)* np.cos(phi)
    y = r * np.sin(theta)* np.sin(phi)
    z = r * np.cos(theta)
    return x, y, z

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return x, y
    

fig = plt.figure()
ax = fig.add_subplot(projection="3d")


# create a list with all the 'sampling' points which we want to plot
vals = []
for i in range(len(Ls)):
    vals.append((Ls[i], Phis[i], Thetas[i]))
    

def update_frame(i):
    ax.clear()

    # Get the point from the points list at index i
    L, phi, theta = vals[i]
    
    # Compute radius of circle and k
    radius = L / theta    # radius of circle
    k = 1/radius   # if you want to use k instead of radius

    Ls, Phis = compute_lenghts(L, phi, theta, radius)
    #print("Ls: ", Ls)
    #print("Phis: ", Phis)

    # discretize for plotting
    arcIndex = np.linspace(0, theta, num = 100)
    X, Y, Z, = sphere2cart(radius, arcIndex, phi)

    # move arc origin to center of xy plane
    x1, y1 = pol2cart(radius, phi)
    X += x1
    Y += y1

    # plot arc
    #ax.plot(X, Y, Z, label='arc')

    colors = ['red', 'green', 'blue', 'orange']
    endpoints = []
    for i in range(len(Ls)):
        endpoint = plot_arc(Ls[i], theta, phi, Phis[i], ax, label=f'L{i+1}', color=colors[i])
        endpoints.append(endpoint)

    for endpoint in endpoints:
        plt.plot([X[-1], endpoint[0]], [Y[-1], endpoint[1]], [Z[-1], endpoint[2]], color = "blue", alpha=0.7)

    endpoints.append(endpoints[0])
    for p1, p2 in zip(endpoints[:-1], endpoints[1:]):
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color = "black")

    # plot axes
    ax.plot(np.zeros(100), np.zeros(100), np.linspace(-np.max(np.abs(Z)), np.max(np.abs(Z)), 100), c= "black", alpha = 0.2)
    ax.plot(np.zeros(100), np.linspace(-np.max(np.abs(Z)), np.max(np.abs(Z)), 100), np.zeros(100),  c= "black", alpha = 0.2)
    ax.plot(np.linspace(-np.max(np.abs(Z)), np.max(np.abs(Z)), 100), np.zeros(100), np.zeros(100),  c= "black", alpha = 0.2)

    # plot center of circle
    ax.scatter(np.array([x1]), np.array([y1]), np.array([0]), c = 'purple', label = "center", s=s)

    # plot endpoint
    ax.scatter(X[-1], Y[-1], Z[-1], c = 'black', label = "endpoint", s=s)

    # plot projection on each axis
    ax.plot(X, np.zeros(len(X)), np.zeros(len(X)), color = "black", label = "X projection")
    ax.plot(np.zeros(len(X)), Y, np.zeros(len(X)), color = "black", label = "Y projection")
    ax.plot(np.zeros(len(X)), np.zeros(len(X)), Z, color = "black", label = "Z projection")

    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)

    ax.legend()    


ani = FuncAnimation(fig, update_frame, frames=len(vals),
                interval=50, repeat=False)
plt.show()

