import numpy as np

# in: s -> [0, 1]   out: X
def bezier_curve(s, P):
    pos = pow(1-s, 3)*P[0] + 3*pow(1-s, 2)*s*P[1] + 3*(1-s)*pow(s, 2)*P[2] + pow(s, 3)*P[3]
    return pos



def time_optimal_bang_bang_profile(x_next, x_acc_flag, x_total, t_acc_flag, t_total, velocity, acceleration):
    # acceleration profile
    if x_next < x_acc_flag:
        t_next = np.sqrt((2*x_next)/acceleration)

    # constant velocity profile
    if x_next >= x_acc_flag and x_next < (x_total-x_acc_flag):
        t_next = t_acc_flag + ((x_next-x_acc_flag)/velocity)

    # deceleration profile
    if x_next >= (x_total-x_acc_flag):
        t_next = t_total - np.sqrt((2*abs(x_total-x_next)/acceleration))
    
    return t_next