import numpy as np
import scipy as sp
# lengths
L5 = 1      # gripper
L3 = 0.6    # forearm
L2 = 1.05   # upper arm

lengths = {
    0: 0,
    1: L2,
    2: L2 + L3,
    3: L2 + L3 + L5
}

origin = np.array([0, 0, 0, 1])

wrist = np.array([
        [0, 0, 1, 0],
        [0, 0, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, 0, 0]
    ])


# @Deprecated
def sph2rect(rho, phi, theta):
    r = rho*np.sin(phi)
    z = rho*np.cos(phi)
    x = r*np.cos(theta)
    y = r*np.sin(theta)
    return np.array([x, y, z])


# @Deprecated
def gripper_rotation(x_g, y_g, z_g, phi, theta):
    x_temp = x_g*np.cos(phi)+z_g*np.sin(phi)
    z = -x_g*np.sin(phi)+z_g*np.cos(phi)

    x = x_temp*np.cos(theta)-y_g*np.sin(theta)
    y = x_temp*np.sin(theta)+y_g*np.cos(theta)

    return np.array([x, y, z])


# @Deprecated
# dof is a list of angles [-pi/2, pi/2]
def armPoints(dof):
    p1 = sph2rect(L2, dof[0], dof[4]) + origin
    p2 = sph2rect(L3, dof[1]+dof[0], dof[4]) + p1

    phi = dof[1]+dof[0]         # rotations to the gripper axis
    theta = dof[4]

    g = sph2rect(L5, dof[3], dof[2])

    p3 = gripper_rotation(g[0], g[1], g[2], phi, theta) + p2
    p4 = gripper_rotation(-g[0], -g[1], g[2], phi, theta) + p2
    return [origin, p1, p2, p3, p4]


def s3m(n):
    return np.array([
        [0, -1, 0, lengths[n]],
        [1, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0]
    ])


def M(n):
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, lengths[n]],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])


def armPointsCoolerVersion(dof):
    t0 = sp.linalg.expm(wrist*dof[3])
    t1 = t0 @ sp.linalg.expm(s3m(0)*dof[0])
    t2 = t1 @ sp.linalg.expm(s3m(1)*dof[1])
    wrist_twist = t2 @ sp.linalg.expm(wrist*dof[2])
    t3 = wrist_twist @ sp.linalg.expm(s3m(2)*dof[4])
    t4 = wrist_twist @ sp.linalg.expm(s3m(2)*-dof[4])

    p4 = t4 @ M(3) @ origin
    p3 = t3 @ M(3) @ origin
    p2 = t2 @ M(2) @ origin
    p1 = t1 @ M(1) @ origin

    # print("coords:    ", wrist_twist @ M(3) @ origin)
    # print("dof:       ", list(float(num) for num in dof))
    return [origin, p1, p2, p3, p4]


def fKin(dof):
    return sp.linalg.expm(wrist*dof[2]) @ \
            sp.linalg.expm(s3m(0)*dof[0]) @ \
            sp.linalg.expm(s3m(1)*dof[1]) @ \
            M(3)


def six_vec(m):
    return np.array([m[2][1], m[0][2], m[1][0], m[0][3], m[1][3], m[2][3]])


def skew(m):
    return np.array([
        [0, -m[1], m[0]],
        [m[1], 0, -m[2]],
        [-m[0], m[2], 0]
    ])


def Ad(m):
    R = m[:3, :3]
    p = m[:3, 3]

    Adm = np.zeros((6, 6))
    Adm[:3, :3] = R
    Adm[3:6, 3:6] = R
    Adm[3:6, :3] = skew(p) @ R

    return Adm


# dof 0 = shoulder, dof 1 = elbow, dof 2 = base
def inverseKin(dof_guess, desired):
    it = 0
    maxiter = 10000

    Tsb = fKin(dof_guess)
    Vm = sp.linalg.logm(np.linalg.inv(Tsb) @ desired)
    V = six_vec(Vm)

    e_w = np.linalg.norm(V[3:])
    e_v = np.linalg.norm(V[:3])

    while ((e_v > 0.001 or e_w > 0.001) and it < maxiter):
        Js = np.zeros((6, 3))
        Js[:, 0] = six_vec(s3m(0))
        Js[:, 1] = Ad(sp.linalg.expm(s3m(0) * dof_guess[0]) @
                      sp.linalg.expm(s3m(1) * dof_guess[1])) @ six_vec(s3m(1))
        Js[:, 2] = Ad(sp.linalg.expm(s3m(0) * dof_guess[0]) @
                      sp.linalg.expm(s3m(1) * dof_guess[1]) @
                      sp.linalg.expm(wrist * dof_guess[2])) @ six_vec(wrist)

        Jb = Ad(np.linalg.inv(Tsb)) @ Js
        delta_theta = np.linalg.pinv(Jb) @ V
        dof_guess += delta_theta

        Tsb = fKin(dof_guess)

        Vm = sp.linalg.logm(np.linalg.inv(Tsb) @ desired)
        V = six_vec(Vm)

        e_w = np.linalg.norm(V[3:])
        e_v = np.linalg.norm(V[:3])

        print("errors", e_v, e_w)
        print("delta_theta", delta_theta)
        print("dof", dof_guess)
        it += 1

    if it >= maxiter:
        print("Max Iterations Reached - Quitting")

    dof_guess = np.array(list(dof % (2*np.pi) for dof in dof_guess))
    for i in range(len(dof_guess)):
        if (dof_guess[i] > np.pi):
            dof_guess[i] -= 2*np.pi
        if (dof_guess[i] < -np.pi):
            dof_guess[i] += 2*np.pi

    return dof_guess
