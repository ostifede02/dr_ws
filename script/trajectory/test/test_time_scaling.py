import numpy as np
import matplotlib.pyplot as plt

from script.trajectory.trajectory import Trajectory
from script import configuration as conf


pos_start = np.array([-200, 0, -280])       # at the end of cycle pos_start_new = pos_end_prev
pos_end = np.array([200, 0, -280])          # should be input to function goto()


trj = Trajectory()


# initialization of variables
pos_current = pos_start
# delta_s = conf.configuration["trajectory"]["delta_s_high_resolution"]
delta_s = 0.001


trj.set_trajectory_routine("quick", pos_start, pos_end, 5)

x_next = 0
t_current = 0


# PLOT data
pos_profile_data = np.empty((2, int(1/delta_s)+1))
pos_profile_data[:,0] = np.array([0,0])
plot_data_index = 1

s_instance = np.linspace(delta_s, 1, int(1/delta_s))

for s_next in s_instance:
    #############   START   #############

    ## next via point
    pos_next = trj.get_position_bezier_poly(s_next)

    delta_x = np.linalg.norm(pos_next-pos_current)
    x_next += delta_x

    ## time
    t_travelled = trj.get_t_next(x_next)
    delta_t = t_travelled - t_current

    # update current values
    t_current = t_travelled
    pos_current = pos_next

    #############   END   #############

    # plot data
    pos_profile_data[:, plot_data_index] = np.array([x_next, t_travelled])
    plot_data_index += 1



## print some data
if trj.t_total-trj.t_acc_flag == 0:
    print("NO constant velocity profile")
else:
    mean_vel = (trj.x_total-trj.x_acc_flag)/(trj.t_total-trj.t_acc_flag)
    print(f"mean vel: {mean_vel}")

print(f"x acc flag: {trj.x_acc_flag}")
print(f"x dec flag: {trj.x_total-trj.x_acc_flag}")
print(f"x total: {trj.x_total}")
print(f"x travelled: {x_next}")
print(f"t total: {t_travelled}")
print(f"pos end: {pos_next}\n")
print(f"s end: {s_next}")
print("s instance len", int(1/delta_s))



## PLOT stuff
plot_pos_profile = plt.figure("position profile")
plot_pos_profile = plt.plot(pos_profile_data[0, :], pos_profile_data[1, :])

plot_pos_profile = plt.vlines(trj.x_acc_flag, 0, t_travelled, "r", "--", label="position flag")
plot_pos_profile = plt.vlines(trj.x_total-trj.x_acc_flag, 0, t_travelled, "r", "--")


plot_pos_profile = plt.hlines(trj.t_acc_flag, 0, x_next, "g", "--", label="time flag")
plot_pos_profile = plt.hlines(trj.t_total-trj.t_acc_flag, 0, x_next, "g", "--")


plot_pos_profile = plt.title("time scaling position profile")
plot_pos_profile = plt.legend()
plot_pos_profile = plt.xlabel("distance traveled [ mm ]")
plot_pos_profile = plt.ylabel("time [ s ]")

plt.show()
