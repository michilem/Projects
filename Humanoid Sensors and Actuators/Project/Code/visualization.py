import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
import numpy as np
import serial
import time
import threading
import queue

ser = serial.Serial('/dev/ttyACM0', 9600)

time.sleep(2)

data_queue = queue.Queue(maxsize=1)

"""
This dictionary maps the MUX id and the selection pins
to a predefined global grid coordinate. We can visualize the e-textile mat
as a matrix such that these coordinates map to it.
"""
coords_map = {
    1: {
        (0, 0, 0, 0): (0, 0),
        (0, 0, 0, 1): (0, 1),
        (0, 0, 1, 0): (0, 2),
        (0, 0, 1, 1): (0, 3),
        (0, 1, 0, 0): (0, 4),
        (0, 1, 0, 1): (1, 0),
        (0, 1, 1, 0): (1, 1),
        (0, 1, 1, 1): (1, 2),
        (1, 0, 0, 0): (1, 3),
        (1, 0, 0, 1): (1, 4),
    },
    2: {
        (0, 0, 0, 0): (2, 0),
        (0, 0, 0, 1): (2, 1),
        (0, 0, 1, 0): (2, 2),
        (0, 0, 1, 1): (2, 3),
        (0, 1, 0, 0): (2, 4),
        (0, 1, 0, 1): (3, 0),
        (0, 1, 1, 0): (3, 1),
        (0, 1, 1, 1): (3, 2),
        (1, 0, 0, 0): (3, 3),
        (1, 0, 0, 1): (3, 4),
    },
    3: {
        (0, 0, 0, 0): (0, 9),
        (0, 0, 0, 1): (0, 8),
        (0, 0, 1, 0): (0, 7),
        (0, 0, 1, 1): (0, 6),
        (0, 1, 0, 0): (0, 5),
        (0, 1, 0, 1): (1, 9),
        (0, 1, 1, 0): (1, 8),
        (0, 1, 1, 1): (1, 7),
        (1, 0, 0, 0): (1, 6),
        (1, 0, 0, 1): (1, 5),
    },
    4: {
        (0, 0, 0, 0): (2, 9),
        (0, 0, 0, 1): (2, 8),
        (0, 0, 1, 0): (2, 7),
        (0, 0, 1, 1): (2, 6),
        (0, 1, 0, 0): (2, 5),
        (0, 1, 0, 1): (3, 9),
        (0, 1, 1, 0): (3, 8),
        (0, 1, 1, 1): (3, 7),
        (1, 0, 0, 0): (3, 6),
        (1, 0, 0, 1): (3, 5),
    },
}

def read_serial_data():
    ser.reset_input_buffer()
    grid = np.empty(shape=(10, 4))
    while True:
        line = ser.readline().decode('utf-8').strip()
        values = [int(x) for x in line.split()]

        if len(values) != 8:
            continue

        muxes, bits = values[:4], tuple(values[4:])
        for mux_id, mux_value in enumerate(muxes, 1):
            grid_coord = coords_map[mux_id]
            j, i = grid_coord[bits]
            grid[i, j] = mux_value

        with data_queue.mutex:
            data_queue.queue.clear()

        data_queue.put(grid)

"""
We run the data acquisition in a separate thread and plotting on the main thread
"""
thread1 = threading.Thread(target=read_serial_data)
thread1.start()


circle_radius = 0.5
x_spacing = 3.0
y_spacing = 1.0

fig, ax = plt.subplots()
norm = Normalize(vmin=0, vmax=1023)
sm = ScalarMappable(norm=norm, cmap=plt.cm.viridis)
fake_image = sm.get_array()

first = True
while True:
    matrix = data_queue.get()
    ax.clear()
    for i in range(10):
        for j in range(4):
            color = plt.cm.viridis(matrix[i][j] / 1023)
            circle = patches.Circle((j * x_spacing, i * y_spacing), radius=circle_radius, color=color)
            ax.add_patch(circle)
    plt.xlim(-x_spacing, 4 * x_spacing)
    plt.ylim(-y_spacing, 10 * y_spacing)
    plt.gca().invert_yaxis() 
    ax.set_aspect('equal', adjustable='box')
    plt.pause(0.01)

    if first:
        # We just want the colorbar on the first iteration
        cbar = plt.colorbar(sm, ax=ax)
        cbar.set_ticks([0, 512, 1023])
        cbar.set_ticklabels(['High force', 'Medium force', 'Low force'])
        first = False
