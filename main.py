import numpy as np
import sympy as sp
import ik

tt1, tt2, tt3, tt4 = sp.symbols('tt1 tt2 tt3 tt4')
h , l1, l2, l3 = sp.symbols('h l1 l2 l3')

dh_parameters = sp.Matrix([
    [0, sp.pi/2, h, tt1],
    [l1, 0, 0, tt2],
    [l2, 0, 0, tt3],
    [l3, 0, 0, tt4]
])


def dh_to_transformation(a, alpha, d, theta):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0, sp.sin(alpha), sp.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def dh_to_fk(dh_params):
    T = sp.eye(4)
    for i in range(dh_params.shape[0]):
        a, alpha, d, theta = dh_params[i, :]
        T_i = dh_to_transformation(a, alpha, d, theta)
        T = T * T_i
    return T

if __name__ == "__main__":
    ### Compute the forward kinematics transformation matrix ###
    FK = sp.simplify(dh_to_fk(dh_parameters))
    print("Forward Kinematics Transformation Matrix:")
    sp.pprint(FK)
    FK_lambda = sp.lambdify((tt1, tt2, tt3, tt4, h, l1, l2, l3), FK, "numpy")

    #### INPUT TARGET END EFFECTOR POSITION AND ORIENTATION (THETA) ####
    state_input = np.array([5, 450, 262, -np.pi/2]) # (x, y, z in mm, theta in radians)
    x, y, z, theta = state_input
    print("Input Target Position (XYZ) and Orientation:")
    print( np.round(state_input[:3],2), "mm, Theta:", np.round(np.degrees(state_input[3]),2), "degrees" )
 
    ### Compute the inverse kinematics to get joint angles ###
    tt1, tt2, tt3, tt4 = ik.inverse_kinematics(x, y, z, theta)
    print("Joint Angles (degrees):")
    print( np.round(np.degrees([tt1, tt2, tt3, tt4]),2) )

    ### Verify the forward kinematics with the computed joint angles ###
    ef_pos = FK_lambda( tt1, tt2, tt3, tt4, ik.link_params[0], ik.link_params[1], ik.link_params[2], ik.link_params[3])[0:3, 3]
    print("Output End Effector Position:")
    print( np.round(ef_pos,2) )