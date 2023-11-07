import numpy as np
import matplotlib.pyplot as plt


'''
 __     __         _       _     _           
 \ \   / /_ _ _ __(_) __ _| |__ | | ___  ___ 
  \ \ / / _` | '__| |/ _` | '_ \| |/ _ \/ __|
   \ V / (_| | |  | | (_| | |_) | |  __/\__ \
    \_/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/
                                             
'''

z_offset = 0
# start and end point
P0 = np.array([-10, 0, -280])
P3 = np.array([0, 0, -280])
# via points
P1 = np.array([P0[0], 0, P0[2]+z_offset])
P2 = np.array([P3[0], 0, P3[2]+z_offset])


# max acceleration and velocity
max_acc = 30                # [ mm / s2 ]
max_vel = 100         # [ mm / s ]

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


def time_scaling_profile_subsections():
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
        delta_x_acc_travelled = np.linalg.norm(bezier_curve(s_acc+delta_s)-bezier_curve(s_acc))
        x_acc_travelled += delta_x_acc_travelled
        # deceleration
        delta_x_dec_travelled = np.linalg.norm(bezier_curve(s_dec)-bezier_curve(s_dec-delta_s))
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
                x_vel_travelled += np.linalg.norm(bezier_curve(s+delta_s)-bezier_curve(s))
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
        delta_s = round(max_delta_x / x_total, 4)
    elif x_total > 15 and x_total <= 50:
        max_delta_x = 2
        delta_s = round(max_delta_x / x_total, 4)
    else:
        max_delta_x = 5
        delta_s = round(max_delta_x / x_total, 4)

    return delta_s





# path profile subsection
x_acc_flag, x_dec_flag, x_total = time_scaling_profile_subsections()
delta_s = define_delta_s(x_total)


pos_current = bezier_curve(0)
x_travelled = 0

t_current = 0

s_instance = np.linspace(0,1-delta_s, int(1/delta_s))

# PLOT data
pos_profile_data = np.empty((2, int(1/delta_s)+1))
pos_profile_data[:,0] = np.array([0,0])
plot_data_index = 1

for s_current in s_instance:
    s_next = s_current + delta_s

    ## position
    pos_next = bezier_curve(s_next)                     # via point for inverse geometry
    delta_x = np.linalg.norm(pos_next-pos_current)
    x_travelled += delta_x

    print(delta_x)

    ## time
    t_travelled = time_scaling_profile(x_travelled, x_acc_flag, x_dec_flag, x_total)
    delta_t = t_travelled - t_current                   # steppers: n and m steps in this time frame


    # update current values
    t_current = t_travelled
    pos_current = pos_next

    # plot data
    pos_profile_data[:, plot_data_index] = np.array([x_travelled, t_travelled])
    plot_data_index += 1


print(f"x acc flag: {x_acc_flag}")
print(f"x dec flag: {x_dec_flag}")
print(f"x total: {x_total}")
print(f"x travelled: {x_travelled}")
print(f"pos end: {pos_next}\n")
print(f"s end: {s_next}")
print("s instance len", int(1/delta_s))



## PLOT stuff

plot_pos_profile = plt.figure("position profile")
plot_pos_profile = plt.plot(pos_profile_data[0, :], pos_profile_data[1, :], ".")

plot_pos_profile = plt.vlines(x_acc_flag, 0, t_travelled, "r", "--")
plot_pos_profile = plt.vlines(x_dec_flag, 0, t_travelled, "r", "--")

plot_pos_profile = plt.xlabel("position [ mm ]")
plot_pos_profile = plt.ylabel("time [ s ]")

plt.show()
