import numpy as np
import quaternion

def solveQ2R(q):
    
    q_array = quaternion.as_float_array(q)
    q_array = q_array/np.linalg.norm(q_array)
    if type(q) == quaternion.quaternion:
        qa = q_array[0]
        qb = q_array[1]
        qc = q_array[2]
        qd = q_array[3]
    else:
        qa = q_array[0][0]
        qb = q_array[0][1]
        qc = q_array[0][2]
        qd = q_array[0][3]
    the2 = 2
    the1 = 1
    ab2 = qa * qb * the2
    ac2 = qa * qc * the2
    ad2 = qa * qd * the2
    bc2 = qb * qc * the2
    bd2 = qb * qd * the2
    cd2 = qc * qd * the2
    aasq = qa * qa * the2 - the1
    bbsq = qb * qb * the2
    ccsq = qc * qc * the2
    ddsq = qd * qd * the2
    rmat = np.zeros((3, 3))
    aasqi = aasq
    bc2i = bc2
    bd2i = bd2
    ac2i = ac2
    ad2i = ad2
    cd2i = cd2
    ab2i = ab2
    r= np.array([[aasqi + bbsq, bc2i + ad2i, bd2i - ac2i],
                                [bc2i - ad2i, aasqi + ccsq, cd2i + ab2i],
                                [bd2i + ac2i, cd2i - ab2i, aasqi + ddsq]])
    return r

def solveQ2E(q):
    q_array = quaternion.as_float_array(q)
    q_array = list(zip(*q_array))
    qa = np.reshape(q_array[0], (-1, 1))
    qb = np.reshape(q_array[1], (-1, 1))
    qc = np.reshape(q_array[2], (-1, 1))
    qd = np.reshape(q_array[3], (-1, 1))
    the1 = np.ones(1, dtype=qa.dtype)
    the2 = 2 * the1
    a = np.zeros_like(qa)
    b = np.zeros_like(qb)
    c = np.zeros_like(qc)
    tmp = qb * qd * the2 - qa * qc * the2
    tmp[tmp > the1] = the1
    tmp[tmp < -the1] = -the1
    tolA = np.cast['float'](0.5 * np.pi - 10 * np.finfo(the1.dtype).eps)
    tolB = np.cast['float'](-0.5 * np.pi + 10 * np.finfo(the1.dtype).eps)
    
    for ii in np.ndindex(tmp.shape):
        b = -np.arcsin(tmp)
        if np.all(b >= tolA):
            a = -2 * np.arctan2(qb, qa)
            c = 0
        elif np.all(b <= tolB):
            a = 2 * np.arctan2(qb, qa)
            c = 0
        else:
            a = np.arctan2((qa * qd * the2 + qb * qc * the2), (qa**2 * the2 - the1 + qb**2 * the2))
            c = np.arctan2((qa * qb * the2 + qc * qd * the2), (qa**2 * the2 - the1 + qd**2 * the2))
    
    return a, b, c

def solveR2Q(R):
    tr = np.trace(R)
    dd = np.hstack((tr, np.diag(R)))
    psquared = 1 + 2 * dd - np.trace(R)
    pmax = np.max(psquared)
    idx = np.argmax(psquared)

    if idx == 0:
        pa = np.sqrt(pmax)
        a = 0.5 * pa
        invpa = 0.5 / pa
        b = invpa * (R[1, 2] - R[2, 1])
        c = invpa * (R[2, 0] - R[0, 2])
        d = invpa * (R[0, 1] - R[1, 0])
    elif idx == 1:
        pb = np.sqrt(pmax)
        b = 0.5 * pb
        invpb = 0.5 / pb
        a = invpb * (R[1, 2] - R[2, 1])
        c = invpb * (R[0, 1] + R[1, 0])
        d = invpb * (R[2, 0] + R[0, 2])
    elif idx == 2:
        pc = np.sqrt(pmax)
        c = 0.5 * pc
        invpc = 0.5 / pc
        a = invpc * (R[2, 0] - R[0, 2])
        b = invpc * (R[0, 1] + R[1, 0])
        d = invpc * (R[1, 2] + R[2, 1])
    else:
        pd = np.sqrt(pmax)
        d = 0.5 * pd
        invpd = 0.5 / pd
        a = invpd * (R[0, 1] - R[1, 0])
        b = invpd * (R[2, 0] + R[0, 2])
        c = invpd * (R[1, 2] + R[2, 1])

    if a < 0:
        a = -a
        b = -b
        c = -c
        d = -d

    return a, b, c, d

def solveV2Q(r):
    
    if len(r.shape) > 1 and r.shape[0] > 1:
        if r.shape[1]>3:
            n = r.shape[0]
            theta = np.sqrt(np.sum(r**2, axis=1))
            a = np.ones((n, 1), dtype=r.dtype)
            b = np.zeros((n, 1), dtype=r.dtype)
            c = np.zeros((n, 1), dtype=r.dtype)
            d = np.zeros((n, 1), dtype=r.dtype)
    elif len(r.shape) > 1 and r.shape[0] == 1:
        r = r[0]
        theta = np.sqrt(np.sum(r**2))
    else:
        theta = np.sqrt(np.sum(r**2))
    ct = np.cos(theta/2)
    st = np.sin(theta/2)
    if len(r.shape) > 1 and r.shape[0] > 1:
        for i in range(n):
            if theta[i] != 0:
                qimag = (r[i] / theta[i]) * st[i]
                a[i] = ct[i]
                b[i] = qimag[0]
                c[i] = qimag[1]
                d[i] = qimag[2]
    else:
        if theta != 0:
            qimag = (r / theta) * st
            q_array = [ct, qimag[0], qimag[1], qimag[2]]
            q_array = q_array/np.linalg.norm(q_array)
            q = quaternion.as_quat_array(q_array)
    return q