import numpy as np
import time

'''
 __     __         _       _     _           
 \ \   / /_ _ _ __(_) __ _| |__ | | ___  ___ 
  \ \ / / _` | '__| |/ _` | '_ \| |/ _ \/ __|
   \ V / (_| | |  | | (_| | |_) | |  __/\__ \
    \_/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/
                                             
'''

z_offset = 100
# start and end point
P0 = np.array([-200, 0, -280])
P3 = np.array([10, 0, -100])
# via points
P1 = np.array([P0[0], 0, P0[2]+z_offset])
P2 = np.array([P3[0], 0, P3[2]+z_offset])


# max acceleration and velocity
max_acc = 50           # [ mm / s2 ]
max_vel = 100          # [ mm / s ]


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

    # deceleration
    x_max_dec = x_max_acc
    x_dec_travelled = 0
    s_dec = 1

    while True:
        x_acc_travelled += np.linalg.norm(bezier_curve(s_acc+delta_s)-bezier_curve(s_acc))
        x_dec_travelled += np.linalg.norm(bezier_curve(s_dec)-bezier_curve(s_dec-delta_s))

        # end without constant velocity
        if s_acc >= s_dec:
            break

        # end with constant velocity
        if x_acc_travelled > x_max_acc and x_dec_travelled > x_max_dec:
            print(f"x_acc {x_acc_travelled} > x_max {x_max_acc}")
            print(f"x_dec {x_dec_travelled} > x_max {x_max_dec}")
            break

        # acceleration via points
        if x_acc_travelled <= x_max_acc:
            s_acc += delta_s

        # deceleration via points
        if x_dec_travelled <= x_max_dec:
            s_dec -= delta_s
    
    s_acc = round(s_acc, 3)
    s_dec = round(s_dec, 3)

    return s_acc, s_dec



s1, s2 = vel_profile_subsections()


print(s1)
print(s2)