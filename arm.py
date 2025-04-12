import numpy as np

# lengths
L5 = 1      # gripper
L3 = 0.6    # forearm
L2 = 1.05   # upper arm

origin = np.array([0, 0, 0.8])


def sph2rect(rho, phi, theta):
    r = rho*np.sin(phi)
    z = rho*np.cos(phi)
    x = r*np.cos(theta)
    y = r*np.sin(theta)
    return np.array([x, y, z])


def gripper_rotation(x_g, y_g, z_g, phi, theta):
    x_temp = x_g*np.cos(phi)+z_g*np.sin(phi)
    z = -x_g*np.sin(phi)+z_g*np.cos(phi)

    x = x_temp*np.cos(theta)-y_g*np.sin(theta)
    y = x_temp*np.sin(theta)+y_g*np.cos(theta)

    return np.array([x, y, z])


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
