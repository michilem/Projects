from imu_kalman import stepImpl
import imu_common
import scipy.io
import matplotlib.pyplot as plt
from framerotation import solveQ2E
from math import pi

def fusion():
    fuse = imu_common.IMUFusionCommon()
    mat_data = scipy.io.loadmat('imu_9axis.mat')
    
    # Access variables from the loaded .mat file
    accIn = mat_data['sensorData']['Acceleration'][0][0]
    gyroIn = mat_data['sensorData']['AngularVelocity'][0][0]
    orientation = stepImpl(fuse, accIn, gyroIn)
    # Convert quaternion to Euler angles

    z,y,x = solveQ2E(orientation)

    # Plot the Euler angles
    Fs = 200.
    time = [x/Fs for x in range(len(accIn))]
    plt.plot(time, z*180/pi, label='Z-axis')
    plt.plot(time, y*180/pi, label='Y-axis')
    plt.plot(time, x*180/pi, label='X-axis')

    # Set plot title, legend, and labels
    plt.title('Rotation Estimate')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Rotation (degrees)')

    # Display the plot
    plt.show()


if __name__=="__main__":
    fusion()