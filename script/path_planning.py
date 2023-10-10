import numpy as np


# start and end points
P0 = np.array([-200, 0, -220])
P3 = np.array([200, 0, -220])

# via points
z_offset = 200
P1 = np.array([P0[0], 0, P0[2]+z_offset])
P2 = np.array([P3[0], 0, P3[2]+z_offset])

# time taken
T = 2

# time scaling function parameters
a3 = 10 / pow(T, 3)
a4 = -(15 / pow(T, 4))
a5 = 6 / pow(T, 5)

# resolution of parameterization
t_intervals = 30

# time of execution (only for simulations)
time_delay = T/t_intervals


def Path_B_spline(s):
    # time scaling
    s = a3*pow(s, 3) + a4*pow(s, 4) + a5*pow(s, 5)
    # Bezier curve polinomial
    x_des = pow(1-s, 3)*P0 + 3*pow(1-s, 2)*s*P1 + 3*(1-s)*pow(s, 2)*P2 + pow(s, 3)*P3
    # offset of end effector
    x_des[0] -= (45/2)

    return x_des


