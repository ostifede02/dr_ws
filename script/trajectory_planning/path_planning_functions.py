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
    x_total = 0

    pos_current = bezier_curve(0, P)

    # compute curve's length
    s_instance = np.linspace(0, 1-delta_s, int(1/delta_s))
    for s in s_instance:
        pos_next = bezier_curve(s+delta_s, P)
        x_total += np.linalg.norm(pos_next-pos_current)
        pos_current = pos_next
    
    x_acc_flag = (max_vel*max_vel) / (2*max_acc)
    
    # if there isn't a constant velocity profile
    if 2*x_acc_flag >= x_total:
        x_acc_flag = x_total * 0.5

    x_dec_flag = x_total-x_acc_flag

    # time constants flags -> for graph scaling
    t_acc_flag = np.sqrt((2*x_acc_flag)/max_acc)
    t_dec_flag = np.sqrt((2*x_acc_flag)/max_acc) + ((x_dec_flag-x_acc_flag)/max_vel) + np.sqrt((2*abs(x_total-x_dec_flag)/max_acc))

    return x_acc_flag, x_dec_flag, x_total, t_acc_flag, t_dec_flag


def time_scaling_profile(x_next, x_acc_flag, x_dec_flag, x_total, t_acc_flag, t_dec_flag):
    t_next = 0

    # acceleration profile
    if x_next < x_acc_flag:
        t_next = np.sqrt((2*x_next)/max_acc)

    # constant velocity profile
    if x_next >= x_acc_flag and x_next < x_dec_flag:
        t_next = t_acc_flag + ((x_next-x_acc_flag)/max_vel)

    # deceleration profile
    if x_next >= x_dec_flag:
        t_next = t_dec_flag - np.sqrt((2*abs(x_total-x_next)/max_acc))
    
    return t_next


def define_delta_s(x_total):
    # if the travel distance is to short -> go directly there
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
