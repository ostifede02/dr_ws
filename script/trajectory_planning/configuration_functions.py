import configuration_data as conf
from trajectory_functions import bezier_curve
import numpy as np


def time_optimal_bang_bang_profile_subsections(x_total, velocity, acceleration):
  
    x_acc_flag = pow(velocity,2) / (2*acceleration)
    
    # if there isn't a constant velocity profile
    if 2*x_acc_flag >= x_total:
        x_acc_flag = x_total * 0.5

    # time flags
    t_acc_flag = np.sqrt((2*x_acc_flag)/acceleration)
    t_total = np.sqrt((2*x_acc_flag)/acceleration) + ((x_total-2*x_acc_flag)/velocity) + np.sqrt((2*abs(x_acc_flag)/acceleration))

    return x_acc_flag, t_acc_flag, t_total


def get_max_const_velocity(T, x_total, acceleration):
    if T < 0:
        return conf.max_velocity
    
    # check if is physically possible
    delta = pow(T*acceleration, 2)-4*x_total*acceleration
    if delta < 0:
        return None
    
    velocity = (T*acceleration-np.sqrt(pow(T*acceleration, 2)-4*x_total*acceleration))/2
    return velocity


def get_curve_length(P):
    delta_s = conf.delta_s_high_resolution
    x_total = 0
    pos_current = bezier_curve(0, P)

    # compute curve's length
    s_instance = np.linspace(0, 1-delta_s, int(1/delta_s))
    for s in s_instance:
        pos_next = bezier_curve(s+delta_s, P)
        x_total += np.linalg.norm(pos_next-pos_current)
        pos_current = pos_next

    return x_total


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


def configure_routine_path_via_points(path_routine, start_pos, end_pos):

    if path_routine == "quick":
        x1_offset = (end_pos[0] - start_pos[0])*0.333
        z1_offset = (end_pos[2] - start_pos[2])*0.333
        x2_offset = (start_pos[0] - end_pos[0])*0.333
        z2_offset = (start_pos[2] - end_pos[2])*0.333

    elif path_routine == "pick":
        x2_offset = 0
        z2_offset = (start_pos[2] - end_pos[2])*0.35
        x1_offset = ((end_pos[0]+x2_offset)-start_pos[0])*0.5
        z1_offset = ((end_pos[2]+z2_offset)-start_pos[2])*0.5

    elif path_routine == "place":
        x1_offset = 0
        z1_offset = min(abs(start_pos[0] - end_pos[0])*0.25, 70)
        x2_offset = 0
        z2_offset = min(abs(start_pos[0] - end_pos[0])*0.25, 70)
    else:
        return -1
    

    P1 = np.array([start_pos[0]+x1_offset, 0, start_pos[2]+z1_offset])
    P2 = np.array([end_pos[0]+x2_offset, 0, end_pos[2]+z2_offset])

    bezier_via_points = np.array([start_pos, P1, P2, end_pos])

    return bezier_via_points


def trajectory_configuration_block(path_routine, start_pos, end_pos, t_taken=-1):
    # calculate bezier curve via points based on routine
    bezier_via_points = configure_routine_path_via_points(path_routine, start_pos, end_pos)
    
    # get curve length
    x_total = get_curve_length(bezier_via_points)

    # define delta_s
    delta_s = define_delta_s(x_total)
    
    # if the trajectory is time constrained -> set new max velocity, else default max velocity
    max_const_velocity = get_max_const_velocity(t_taken, x_total, conf.max_acceleration)
    
    # time scaling profile flags
    x_acc_flag, t_acc_flag, t_total = time_optimal_bang_bang_profile_subsections(x_total, max_const_velocity, conf.max_acceleration)

    trajectory_config = {
        "via_points": bezier_via_points,
        "delta_s": delta_s,
        "x_total": x_total,
        "t_total": t_total,
        "x_acc_flag": x_acc_flag,
        "t_acc_flag": t_acc_flag,
        "vel": max_const_velocity,
        "acc": conf.max_acceleration
    }

    return trajectory_config

