import numpy as np
import matplotlib.pyplot as plt

from configuration_functions import trajectory_configuration_block
from trajectory_functions import bezier_curve, time_optimal_bang_bang_profile


pos_start = np.array([-20, 0, -280])       # at the end of cycle pos_start_new = pos_end_prev
pos_end = np.array([200, 0, -280])          # should be input to function goto()


trj_data = trajectory_configuration_block("quick", pos_start, pos_end)


# initialization of variables
pos_current = pos_start

delta_s = trj_data["delta_s"]
x_acc_flag = trj_data["x_acc_flag"]
x_total = trj_data["x_total"]
t_acc_flag = trj_data["t_acc_flag"]
t_total = trj_data["t_total"]
velocity = trj_data["vel"]
acceleration = trj_data["acc"]

x_travelled = 0
t_current = 0


# PLOT data
pos_profile_data = np.empty((2, int(1/delta_s)+1))
pos_profile_data[:,0] = np.array([0,0])
plot_data_index = 1

s_instance = np.linspace(delta_s, 1, int(1/delta_s))

for s_next in s_instance:
    #############   START   #############

    ## next via point
    pos_next = bezier_curve(s_next, trj_data["via_points"])

    ##
    ##  inverse geometry -> collisions? -> n,m steps
    ##

    delta_x = np.linalg.norm(pos_next-pos_current)
    x_travelled += delta_x

    ## time
    t_travelled = time_optimal_bang_bang_profile(x_travelled, x_acc_flag, x_total, t_acc_flag, t_total, velocity, acceleration)
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

if t_total-t_acc_flag == 0:
    print("NO constant velocity profile")
else:
    mean_vel = (x_total-x_acc_flag)/(t_total-t_acc_flag)
    print(f"mean vel: {mean_vel}")

print(f"x acc flag: {x_acc_flag}")
print(f"x dec flag: {x_total-x_acc_flag}")
print(f"x total: {x_total}")
print(f"x travelled: {x_travelled}")
print(f"T: {t_travelled}")
print(f"pos end: {pos_next}\n")
print(f"s end: {s_next}")
print("s instance len", int(1/delta_s))



## PLOT stuff

plot_pos_profile = plt.figure("position profile")
plot_pos_profile = plt.plot(pos_profile_data[0, :], pos_profile_data[1, :])

plot_pos_profile = plt.vlines(x_acc_flag, 0, t_travelled, "r", "--", label="pos start of const velocity")
plot_pos_profile = plt.vlines(x_total-x_acc_flag, 0, t_travelled, "r", "--", label="pos end of const velocity")


plot_pos_profile = plt.hlines(t_acc_flag, 0, x_travelled, "g", "--", label="time start of const velocity")
plot_pos_profile = plt.hlines(t_total-t_acc_flag, 0, x_travelled, "g", "--", label="time end of const velocity")


plot_pos_profile = plt.title("time scaling position profile")
plot_pos_profile = plt.legend()
plot_pos_profile = plt.xlabel("position [ mm ]")
plot_pos_profile = plt.ylabel("time [ s ]")

plt.show()
