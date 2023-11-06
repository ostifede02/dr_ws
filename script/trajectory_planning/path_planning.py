import numpy as np
import time

'''
 __     __         _       _     _           
 \ \   / /_ _ _ __(_) __ _| |__ | | ___  ___ 
  \ \ / / _` | '__| |/ _` | '_ \| |/ _ \/ __|
   \ V / (_| | |  | | (_| | |_) | |  __/\__ \
    \_/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/
                                             
'''

z_offset = 0
# start and end point
P0 = np.array([-200, 0, -280])
P3 = np.array([200, 0, -280])
# via points
P1 = np.array([P0[0], 0, P0[2]+z_offset])
P2 = np.array([P3[0], 0, P3[2]+z_offset])


# max acceleration and velocity
max_acc = 50           # [ mm / s2 ]
max_vel = 100          # [ mm / s ]

x_acc = 0
x_dec = 0

# states time scaling
STATE_ACC = "state const acc"
STATE_VEL = "state const vel"
STATE_DEC = "state const dec"
state = STATE_ACC   # initial state







'''
  _____                 _   _                 
 |  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
 | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
 |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
 |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
                                              
'''

# in: s -> [0, 1]   out: X
def bezier_curve(s):
    s = max(s, 0)
    s = min(s, 1)
    x_next = pow(1-s, 3)*P0 + 3*pow(1-s, 2)*s*P1 + 3*(1-s)*pow(s, 2)*P2 + pow(s, 3)*P3
    # x_next = (1-s)*P0 + s*P3
    return x_next


def vel_profile_subsections():
    delta_s = 0.001

    # acceleration
    x_max_acc = (max_vel*max_vel) / (2*max_acc)
    x_acc_travelled = 0
    s_acc = 0

    # constant velocity
    x_vel_travelled = 0

    # deceleration
    x_max_dec = x_max_acc
    x_dec_travelled = 0
    s_dec = 1

    while True:
        x_acc_travelled += np.linalg.norm(bezier_curve(s_acc+delta_s)-bezier_curve(s_acc))
        x_dec_travelled += np.linalg.norm(bezier_curve(s_dec)-bezier_curve(s_dec-delta_s))

        # end without constant velocity
        if s_acc >= s_dec:
            x_acc_travelled -= np.linalg.norm(bezier_curve(s_acc+delta_s)-bezier_curve(s_acc))
            x_dec_travelled -= np.linalg.norm(bezier_curve(s_dec)-bezier_curve(s_dec-delta_s))
            x_vel_travelled = 0
            break

        # end with constant velocity
        if x_acc_travelled > x_max_acc and x_dec_travelled > x_max_dec:
            x_acc_travelled -= np.linalg.norm(bezier_curve(s_acc+delta_s)-bezier_curve(s_acc))
            x_dec_travelled -= np.linalg.norm(bezier_curve(s_dec)-bezier_curve(s_dec-delta_s))

            # calculate length of velocity profile 
            s = s_acc
            while s < s_dec:
                x_vel_travelled += np.linalg.norm(bezier_curve(s+delta_s)-bezier_curve(s))
                s += delta_s
            
            break

        # acceleration via points
        if x_acc_travelled <= x_max_acc:
            s_acc += delta_s

        # deceleration via points
        if x_dec_travelled <= x_max_dec:
            s_dec -= delta_s

    return x_acc_travelled, x_vel_travelled, x_dec_travelled


def time_scaling_profile(x_next, x_acc_flag, x_dec_flag):

    # acceleration profile
    if x_next < x_acc_flag:
        t_next = np.sqrt((2*x_next)/max_acc)

    # constant velocity profile
    if x_next >= x_acc_flag and x_next < x_dec_flag:
        pass

    # deceleration profile
    if x_next >= x_dec_flag and x_next < 1:
        pass
    
    return t_next









x_acc_len, x_vel_len, x_dec_len = vel_profile_subsections()
x_acc_flag = x_acc_len
x_dec_flag = x_acc_len + x_vel_len

delta_s = 0.01

x_travelled = 0

pos_current = bezier_curve(0)
t_current = 0

s_instance = np.linspace(0,1-delta_s, int(1/delta_s))

for s in s_instance:
    ## position
    pos_next = bezier_curve(s+delta_s)
    delta_x = np.linalg.norm(pos_next-pos_current)
    x_travelled += delta_x

    ## time
    s_next = s+delta_s
    t_next = time_scaling_profile(x_travelled, x_acc_flag, x_dec_flag)
    delta_t = t_next - t_current

    t_current = t_next
    pos_current = pos_next

print(x_acc_len)
print(x_vel_len)
print(x_dec_len)
print(x_acc_len + x_vel_len + x_dec_len)
