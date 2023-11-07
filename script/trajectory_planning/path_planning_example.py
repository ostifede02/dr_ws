import numpy as np
import matplotlib.pyplot as plt

from path_planning_conf import *
from path_planning_functions import *


pos_start = np.array([-200, 0, -280])       # at the end of cycle pos_start_new = pos_end_prev
pos_end = np.array([200, 0, -280])          # should be input to function goto()

z_offset = np.linalg.norm(pos_end-pos_start) / 10

P = bezier_curve_via_points(pos_start, pos_end, z_offset)

# path profile subsection
x_acc_flag, x_dec_flag, x_total = time_scaling_profile_subsections(P)
delta_s = define_delta_s(x_total)

# initialization of variables
pos_current = pos_start

x_travelled = 0
t_current = 0



# PLOT data
pos_profile_data = np.empty((2, int(1/delta_s)+1))
pos_profile_data[:,0] = np.array([0,0])
plot_data_index = 1

s_instance = np.linspace(0,1-delta_s, int(1/delta_s))

for s_current in s_instance:
    #############   START   #############

    s_next = s_current + delta_s

    ## next via point
    pos_next = bezier_curve(s_next, P)

    ##
    ##  inverse geometry -> collisions? -> n,m steps
    ##

    delta_x = np.linalg.norm(pos_next-pos_current)
    x_travelled += delta_x

    ## time
    t_travelled = time_scaling_profile(x_travelled, x_acc_flag, x_dec_flag, x_total)
    delta_t = t_travelled - t_current

    ##
    ##  send n,m steps and delta_t to microcontroller
    ##

    # update current values
    t_current = t_travelled
    pos_current = pos_next

    #############   END   #############

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
