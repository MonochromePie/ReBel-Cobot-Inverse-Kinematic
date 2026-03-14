import numpy as np

#offset in mm.
link_params = np.array([252.2, 241.519, 246.8, 62])
h, l1, l2, l3 = link_params

def inverse_kinematics(x, y, z, theta):
    r = np.sqrt(x**2 + y**2)
    s1 = y / r
    c1 = x / r
    t1 = np.arctan2(s1, c1)

    px3 = x/c1- l3 * np.cos(theta)
    pz3 = z - l3 * np.sin(theta)

    c3 = (px3**2 + (pz3-h)**2 - l1**2 - l2**2) / (2*l1*l2)
    s3 = -np.sqrt(1 - c3**2)
    t3 = np.arctan2(s3, c3)

    a = l1 + l2*c3
    b = l2*s3
    c2 = (px3*a + (pz3-h)*b) / (a**2 + b**2)
    s2 = np.sqrt(1 - c2**2)
    t2 = np.arctan2(s2, c2)

    t4 = theta  - t2 - t3
    return np.array([t1, t2, t3, t4])