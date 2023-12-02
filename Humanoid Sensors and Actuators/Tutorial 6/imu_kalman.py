import numpy as np
import quaternion
from ecompass import ecompass

def stepImpl(obj, accelIn, gyroIn):
    # Fuse the sensor readings from the accelerometer and gyroscope.
    # accelIn - Nx3 matrix of accel samples in m/s^2
    # gyroIn - Nx3 matrix of gyro samples in rad/s

    accelIn = np.reshape(np.transpose(accelIn), (3, 1, -1))
    gyroIn = np.reshape(np.transpose(gyroIn), (3, 1, -1))
    accelIn = np.transpose(accelIn, (1, 0, 2))
    gyroIn = np.transpose(gyroIn, (1, 0, 2))
    ref = obj.pRefSys
    Fs = 200. # taken from dead_reckoning

    numiters = accelIn.shape[2]

    # Allocate output
    orientOut = obj.allocateOutputs(numiters)

    # Loop through each frame.
    for iter in range(numiters):
        # Indirect Kalman filter. Track the *errors* in the estimates of the
        # the orientation (as a 3-element rotation vector) gyroscope bias, linear acceleration
        #
        # The Kalman filter tracks these errors. The actual orientation, gyroscope bias and linear acceleration are
        # updated with the Kalman filter states after predict & correct phases.

        angVel = gyroIn[:, :, iter]
        accel = accelIn[:, :, iter]

        if obj.pFirstTime:
            # Basic tilt corrected ecompass orientation algorithm.
            # Do this in the first iteration only. 

            # TODO: Magnetometer not in use - assume the device is pointing north
            # TODO: Calculate the initial orientation with ecompass as posterior orientation estimation of the first iteration
            m = np.array([1, 0, 0])

            obj.pOrientPost = ecompass(accel, m)
            obj.pFirstTime = False
        
        # Update the orientation quaternion based on the gyroscope readings.
        # TODO: complete the predictOrientation function in imu_common.py
        obj.pOrientPrior = obj.predictOrientation(angVel, obj.pGyroOffset, obj.pOrientPost)

        # TODO: Convert the prior estimation back to Rotation matrix
        Rprior = quaternion.as_rotation_matrix(obj.pOrientPrior)

        # The Kalman filter measurement:
        # Accel estimate of gravity - Gyro estimate of gravity
        # Gyro: Rprior is from the gyro measurements (above).

        # Gravity vector is the last column of that matrix.
        # TODO: complete the rotmat2gravity function in imu_common.py
        gravityGyroMeas = obj.rotmat2gravity(Rprior).T
        
        # Accel: Decay the estimate of the linear acceleration and
        # subtract it from the accelerometer reading.
        obj.pLinAccelPrior = obj.LinearAccelerationDecayFactor * obj.pLinAccelPost
        gravityAccelMeas = ref.GravitySign * accel + obj.pLinAccelPrior
        gravityAccelGyroDiff = gravityAccelMeas - gravityGyroMeas
        # TODO: Compute the Observation Matrix H from gravityGyroMeas
        # Measurement matrix H 3-by-9
        k = 1/Fs
        H = np.array([[0, gravityGyroMeas[0,2], -gravityGyroMeas[0,1], 0, -k*gravityGyroMeas[0,2], k*gravityGyroMeas[0,1], 1, 0, 0],
                      [-gravityGyroMeas[0,2], 0, gravityGyroMeas[0,0], k*gravityGyroMeas[0,2], 0, -k*gravityGyroMeas[0,0], 0, 1, 0],
                      [gravityGyroMeas[0,1], -gravityGyroMeas[0,0], 0, -k*gravityGyroMeas[0,1], k*gravityGyroMeas[0,0], 0, 0, 0, 1]])
        
        Qv = obj.pQv # Covariance of the observation model noise
        Qw = obj.pQw # Covariance of prior error estimation
        # TODO: Calculate the Kalman Gain K
        S = Qv + H.dot(Qw.dot(H.T))
        K = Qw.dot(H.T.dot(np.linalg.inv(S)))
        
        # ze = gravityAccelGyroDiff.reshape(-1, 1)
        ze = gravityAccelGyroDiff.T #  

        # TODO: Update a posteriori error using the Kalman gain
        xe_post =  K.dot(ze)
        
        # Corrected error estimates
        orientErr = xe_post[0:3,:].T
        gyroOffsetErr = xe_post[3:6,:].T
        linAccelErr = xe_post[6:,:].T

        # Estimate estimates based on the Kalman filtered error estimates.
        #
        # TODO: Convert orientation error into a quaternion.
        # Update a posteriori orientation with the conjugate of orientErr
        qerr = quaternion.from_euler_angles(orientErr)
        obj.pOrientPost = obj.pOrientPrior * np.conjugate(qerr)
        obj.pOrientPost = obj.pOrientPost / np.linalg.norm(quaternion.as_float_array(obj.pOrientPost))

        # Force rotation angle to be positive
        if np.all(quaternion.as_float_array(obj.pOrientPost)<0):
            obj.pOrientPost = -obj.pOrientPost
        
        # Update gyro bias and linear acceleration by subtracting estimated errors 
        obj.pGyroOffset = obj.pGyroOffset - gyroOffsetErr
        obj.pLinAccelPost = obj.pLinAccelPost - linAccelErr

        # Update Error Estimate Covariance
        Ppost = Qw - K.dot(H.dot(Qw))

        # Update Qw - the process noise. Qw is a function of Ppost
        Qw = np.zeros((9, 9))

        diags_1_to_3 = np.arange(3)
        diags_4_to_6 = np.arange(3, 6)
        diags_7_to_9 = np.arange(6, 9)
        Qw[diags_1_to_3, diags_1_to_3] = Ppost[diags_1_to_3, diags_1_to_3] + \
            (obj.pKalmanPeriod**2) * (Ppost[diags_4_to_6, diags_4_to_6] + \
            (obj.GyroscopeDriftNoise + obj.GyroscopeNoise))

        Qw[diags_4_to_6, diags_4_to_6] = Ppost[diags_4_to_6, diags_4_to_6] + obj.GyroscopeDriftNoise

        offDiag = -obj.pKalmanPeriod * Qw[diags_4_to_6, diags_4_to_6]
        Qw[diags_4_to_6, diags_1_to_3] = offDiag
        Qw[diags_1_to_3, diags_4_to_6] = offDiag

        
        Qw[diags_7_to_9, diags_7_to_9] = (obj.LinearAccelerationDecayFactor**2) * \
            Ppost[diags_7_to_9, diags_7_to_9] + obj.LinearAccelerationNoise

        obj.pQw = Qw

        # Populate output arguments
        if obj.OrientationFormat.lower() == 'quaternion':
            orientOut[iter] = obj.pOrientPost[0]
        else:
            pass

    return orientOut

