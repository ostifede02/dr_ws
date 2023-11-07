import numpy as np

import path_planning_conf 
from path_planning_conf import max_acc, max_vel


def bezier_curve_via_points(pos_start, pos_end, z_offset):
    # start and end point
    P0 = pos_start
    P3 = pos_end
    # via points
    P1 = np.array([P0[0], 0, P0[2]+z_offset])
    P2 = np.array([P3[0], 0, P3[2]+z_offset])

    return np.array([P0, P1, P2, P3])


# in: s -> [0, 1]   out: X
def bezier_curve(s, P):
    s = max(s, 0)
    s = min(s, 1)

    x_next = pow(1-s, 3)*P[0] + 3*pow(1-s, 2)*s*P[1] + 3*(1-s)*pow(s, 2)*P[2] + pow(s, 3)*P[3]
    return x_next

# in: s -> [0, 1]   out: X
def straight_line(s, P):
    s = max(s, 0)
    s = min(s, 1)
    x_next = (1-s)*P[0] + s*P[3]
    return x_next


def time_scaling_profile_subsections(P):
    delta_s = 0.02

    # acceleration
    x_max_acc = (max_vel*max_vel) / (2*max_acc)
    x_acc_travelled = 0
    delta_x_acc_travelled = 0
    s_acc = 0

    # constant velocity
    x_vel_travelled = 0

    # deceleration
    x_max_dec = x_max_acc
    x_dec_travelled = 0
    delta_x_dec_travelled = 0
    s_dec = 1

    while True:
        # acceleration
        delta_x_acc_travelled = np.linalg.norm(bezier_curve(s_acc+delta_s, P)-bezier_curve(s_acc, P))
        x_acc_travelled += delta_x_acc_travelled
        # deceleration
        delta_x_dec_travelled = np.linalg.norm(bezier_curve(s_dec, P)-bezier_curve(s_dec-delta_s, P))
        x_dec_travelled += delta_x_dec_travelled


        # break without constant velocity
        if s_acc >= s_dec:
            x_acc_travelled -= delta_x_acc_travelled
            x_dec_travelled -= delta_x_dec_travelled
            x_vel_travelled = 0
            break

        # break with constant velocity
        if x_acc_travelled > x_max_acc and x_dec_travelled > x_max_dec:
            x_acc_travelled -= delta_x_acc_travelled
            x_dec_travelled -= delta_x_dec_travelled

            # calculate length of velocity profile 
            s = s_acc
            while s < s_dec:
                x_vel_travelled += np.linalg.norm(bezier_curve(s+delta_s, P)-bezier_curve(s, P))
                s += delta_s
            
            break

        # acceleration via points
        if x_acc_travelled <= x_max_acc:
            s_acc += delta_s

        # deceleration via points
        if x_dec_travelled <= x_max_dec:
            s_dec -= delta_s

    x_acc_flag = x_acc_travelled
    x_dec_flag = x_acc_travelled + x_vel_travelled
    x_total = x_acc_travelled + x_vel_travelled + x_dec_travelled

    return x_acc_flag, x_dec_flag, x_total


def time_scaling_profile(x_next, x_acc_flag, x_dec_flag, x_total):
    t_next = 0

    # acceleration profile
    if x_next < x_acc_flag:
        t_next = np.sqrt((2*x_next)/max_acc)

    # constant velocity profile
    if x_next >= x_acc_flag and x_next < x_dec_flag:
        t_1 = np.sqrt((2*x_acc_flag)/max_acc)       # constant -> should be optimized
        t_next = t_1 + ((x_next-x_acc_flag)/max_vel)

    # deceleration profile
    if x_next >= x_dec_flag:
        t_2 = np.sqrt((2*x_acc_flag)/max_acc) + ((x_dec_flag-x_acc_flag)/max_vel) + np.sqrt((2*abs(x_total-x_dec_flag)/max_acc))    # constant -> should be optimized
        t_next = t_2 - np.sqrt((2*abs(x_total-x_next)/max_acc))
    
    return t_next


def define_delta_s(x_total):
    if x_total <= 5:
        delta_s = 1
    elif x_total > 5 and x_total <= 15:
        max_delta_x = 1
        delta_s = round(max_delta_x / x_total, 3)
    elif x_total > 15 and x_total <= 50:
        max_delta_x = 2
        delta_s = round(max_delta_x / x_total, 3)
    else:
        max_delta_x = 5
        delta_s = round(max_delta_x / x_total, 3)

    return delta_s
