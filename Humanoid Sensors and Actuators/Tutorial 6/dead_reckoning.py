from imu_kalman import stepImpl
import imu_common
import time
import quaternion
import numpy as np
import scipy.io
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R
from framerotation import solveQ2E
from ecompass import ecompass
from math import pi

def dead_reckon_orientation(angular_acc, angular_vel, initial_rotation):
    Fs = 200.
    delta_t = 1/Fs
    time = [x/Fs for x in range(len(angular_acc))]

    trajectory = []
    integrated_state = initial_rotation
    for i, t in enumerate(time):
        # delta_quat_acc = quaternion.from_euler_angles(angular_acc[i] * (delta_t ** 2) / 2)
        delta_quat_vel = quaternion.from_euler_angles(angular_vel[i] * delta_t)
        # integrated_state = integrated_state * delta_quat_vel * delta_quat_acc
        integrated_state = integrated_state * delta_quat_vel
        trajectory.append(integrated_state)
    return trajectory
    
def fusion():
    fuse = imu_common.IMUFusionCommon()
    mat_data = scipy.io.loadmat('rpy_9axis.mat')
    
    # TODO: implement dead reckoning
    accIn = mat_data['sensorData']['Acceleration'][0][0]
    gyroIn = mat_data['sensorData']['AngularVelocity'][0][0]

    first_a = accIn[0]
    first_m = np.array([1, 0, 0])
    initial_rotation = ecompass(first_a, first_m)


    # Plot the Euler angles
    Fs = 200.
    time = [x/Fs for x in range(len(accIn))]

    traj = dead_reckon_orientation(accIn, gyroIn, initial_rotation)
    traj_as_euler = solveQ2E(traj)
    z, y, x = traj_as_euler

    plt.plot(time, z*180/pi, label='Z-axis', color='blue')
    plt.plot(time, y*180/pi, label='Y-axis', color='green')
    plt.plot(time, x*180/pi, label='X-axis', color='red')

    # Set plot title, legend, and labels
    plt.title('Rotation Estimate')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Rotation (degrees)')

    # Display the plot
    plt.show()

if __name__=="__main__":
    fusion()