import numpy as np
import quaternion 

class IMUFusionCommon:
    def __init__(self):
        self.AccelerometerNoise = 2e-6 * (gms2(1) ** 2)
        self.GyroscopeNoise = 9.1385e-5
        self.GyroscopeDriftNoise = 3.0462e-13
        self.LinearAccelerationNoise = 1e-4 * (gms2(1) ** 2)
        self.LinearAccelerationDecayFactor = 0.5
        self.OrientationFormat = 'quaternion'
        self.ReferenceFrame = 'NED'
        self.SampleRate = 200

        # Hidden properties
        self.pFirstTime = True
        self.pQw = self.getInitialProcCov(empirical=True)
        self.pOrientPost = None
        self.pOrientPrior = None
        self.pRefSys = ReferenceFrame.getMathObject(self.ReferenceFrame)
        self.pSensorPeriod = 1.0 / self.SampleRate
        self.pKalmanPeriod = self.pSensorPeriod
        self.pGyroOffset = np.zeros((1,3))
        self.pLinAccelPrior = np.zeros((1,3))
        self.pLinAccelPost = np.zeros((1,3))
        self.pInputPrototype = None
        self.pQv = self.updateMeasurementErrCov()

    def set_AccelerometerNoise(self, val):
        self.AccelerometerNoise = val

    def set_GyroscopeNoise(self, val):
        self.GyroscopeNoise = val

    def set_GyroscopeDriftNoise(self, val):
        self.GyroscopeDriftNoise = val

    def set_LinearAccelerationNoise(self, val):
        self.LinearAccelerationNoise = val

    def set_LinearAccelerationDecayFactor(self, val):
        self.LinearAccelerationDecayFactor = val

    def saveObjectImpl(self):
        s = {
            'pQw': self.pQw,
            'pQv': self.pQv,
            'pOrientPost': self.pOrientPost,
            'pOrientPrior': self.pOrientPrior,
            'pFirstTime': self.pFirstTime,
            'pRefSys': self.pRefSys,
            'pSensorPeriod': self.pSensorPeriod,
            'pKalmanPeriod': self.pKalmanPeriod,
            'pGyroOffset': self.pGyroOffset,
            'pLinAccelPrior': self.pLinAccelPrior,
            'pLinAccelPost': self.pLinAccelPost,
            'pInputPrototype': self.pInputPrototype
        }
        return s

    def loadObjectImpl(self, s):
        self.pQw = s['pQw']
        self.pQv = s['pQv']
        self.pOrientPost = s['pOrientPost']
        self.pOrientPrior = s['pOrientPrior']
        self.pFirstTime = s['pFirstTime']
        self.pRefSys = s['pRefSys']
        self.pSensorPeriod = s['pSensorPeriod']
        self.pKalmanPeriod = s['pKalmanPeriod']
        self.pGyroOffset = s['pGyroOffset']
        self.pLinAccelPrior = s['pLinAccelPrior']
        self.pLinAccelPost = s['pLinAccelPost']
        self.pInputPrototype = s['pInputPrototype']

    def processTunedPropertiesImpl(self):
        self.setupPeriods()

    def setupImpl(self, accelIn, *args):
        self.pInputPrototype = accelIn
        self.setupPeriods()
        self.pRefSys = ReferenceFrame.getMathObject(self.ReferenceFrame)

    def setupPeriods(self):
        self.pSensorPeriod = 1.0 / self.SampleRate
        self.pKalmanPeriod = 1 * self.pSensorPeriod

    def validateFrameSize(self, x):
        nrows = x[0]
        assert nrows % self.DecimationFactor == 0, \
            'DecimationFactor must divide the frame size.'
 
    def allocateOutputs(self, numiters):
        if self.OrientationFormat.lower() == 'quaternion':
            orientOut = np.zeros((numiters,), dtype=quaternion.quaternion)
        else:
            orientOut = np.zeros((3, 3, numiters))
        return orientOut

    
    def buildHPart(self, v):
        h = np.zeros((3,3), dtype=v.dtype)
        if v.shape[0] == 3:
            h[0, 1] = v[2]
            h[0, 2] = -v[1]
            h[1, 2] = v[0]
        else:
            h[0, 1] = v[0][2]
            h[0, 2] = -v[0][1]
            h[1, 2] = v[0][0]
        h -= h.T
        return h

    def predictOrientation(self, gyro, offset, qorient):
    # TODO: complete the predictOrientation function that calculates 
    # orientation change (delta q) from gyro reading and gyro offset
    # Return in quaternion representation
        Fs = 200. # taken from dead_reckoning
        dphi = (gyro - offset)/Fs
        
        dq = quaternion.from_euler_angles(dphi)
        qorient = qorient * dq

        # Two quaternions negate each other are equivalent
        if np.all(quaternion.as_float_array(qorient)<0):
            qorient = -qorient
        return qorient
    
    def rotmat2gravity(self, R):
    # TODO: complete the rotmat2gravity function that calculates 
    # gravatational acceleration vector from orientation in rotation matrix form.
        gravity = gms2(1) # gravatational acceleration in m/s^2
        ref = self.pRefSys # NED frame
        g = gravity * np.transpose(R[:, ref.GravityIndex])
        return g
    
    def updateMeasurementErrCov(self):
        accelMeasNoiseVar = self.AccelerometerNoise +\
            self.LinearAccelerationNoise +\
            (self.pKalmanPeriod ** 2) * (self.GyroscopeDriftNoise + self.GyroscopeNoise)
        return accelMeasNoiseVar * np.eye(3)    

    def getInitialProcCov(self, empirical = False):

        diags_1_to_3 = np.arange(3)
        diags_4_to_6 = np.arange(3, 6)
        diags_7_to_9 = np.arange(6, 9)
        if empirical:
            covinit = np.zeros((9,9))
            covinit[diags_1_to_3,diags_1_to_3] = 0.000006092348396
            covinit[diags_4_to_6,diags_4_to_6] = 0.000076154354947
            covinit[diags_7_to_9,diags_7_to_9] = 0.00962361
        else:
            cOrientErrVar = (1 * (3.14159 / 180) * 1 * (3.14159 / 180) * 2000e-5)  # var in init orientation error estim.
            cGyroBiasErrVar = (1 * (3.14159 / 180) * 1 * (3.14159 / 180) * 250e-3)  # var in init gyro bias error estim
            cOrientGyroBiasErrVar = 0  # covar orient - gyro bias error estim
            cAccErrVar = 10e-5 * (gms2(1) ** 2)  # var in linear accel drift error estim
            covinit = np.zeros((9, 9))
            covinit[diags_1_to_3, diags_1_to_3] = cOrientErrVar 
            covinit[diags_4_to_6, diags_4_to_6] = cGyroBiasErrVar 
            covinit[diags_1_to_3, diags_4_to_6] = cOrientGyroBiasErrVar 
            covinit[diags_4_to_6, diags_1_to_3] = covinit[diags_1_to_3, diags_4_to_6]
            covinit[diags_7_to_9, diags_7_to_9] = cAccErrVar 
            covinit = covinit * self.getInitialProcCovMask()
        return covinit

    def getInitialProcCovMask(self):
        msk = np.zeros((9, 9), dtype=bool)
        msk[np.arange(3), np.arange(3)] = True
        msk[np.arange(3, 6), np.arange(3, 6)] = True
        msk[np.arange(3), np.arange(3, 6)] = True
        msk[np.arange(3, 6), np.arange(3)] = msk[np.arange(3), np.arange(3, 6)]
        msk[np.arange(6, 9), np.arange(6, 9)] = True
        return msk





class ReferenceFrame:
    @staticmethod
    def getMathObject(ref_frame):
        if ref_frame == 'ENU':
            return ENUReferenceFrame()
        elif ref_frame == 'NED':
            return NEDReferenceFrame()
        else:
            raise ValueError("Invalid reference frame. Supported options: 'ENU', 'NED'.")


class ENUReferenceFrame:
    NorthIndex = 1
    EastIndex = 0
    GravityIndex = 2
    NorthAxisSign = 1
    GravityAxisSign = -1
    GravitySign = -1
    ZAxisUpSign = 1
    LinAccelSign = -1


class NEDReferenceFrame:
    NorthIndex = 0
    EastIndex = 1
    GravityIndex = 2
    NorthAxisSign = 1
    GravityAxisSign = 1
    GravitySign = 1
    ZAxisUpSign = -1
    LinAccelSign = -1


def gms2(x):
    gravity = 9.81
    y = x * gravity
    return y


