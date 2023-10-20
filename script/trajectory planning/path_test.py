import path_planning as pp
from path_planning import *

import matplotlib.pyplot as plt
import numpy as np



t = 0
s = 0
s_index = 0
s_iteration_filter = 1

N = 100000
time_scaling_plot_data = np.empty((2,N))
x_des_plot_data = np.empty((2, N))
vel_plot_data = np.empty(N)
vel = 0
plot_index = 0

x_des = bezier_curve(0)
x_prev = np.empty(3)

while True:
    x_prev = x_des

    # s = time_scaling(t)
    s = t
    if s > 1:
        break

    if s is None:
        break

    if s_index % s_iteration_filter != 0:
        s_index += 1
        t = round(t + delta_t, 3)
        continue
    else:
        s_index += 1
        t = round(t + delta_t, 3)

    x_des = bezier_curve(s)

    # save data for plotime_scaling_plot_data
    time_scaling_plot_data[:, plot_index]                  = np.array([t, s])
    x_des_plot_data[:, plot_index]     = np.array([x_des[0], x_des[2]])

    vel = np.linalg.norm(x_des-x_prev) / (pp.delta_t*s_iteration_filter)
    vel_plot_data[plot_index] = vel
    plot_index += 1

print(f"s={s}\tt1={pp.t_1}\tt2={pp.T-pp.t_2}\tT={pp.T}")




# PLOT STUFF
plot_time_scale_fn = plt.figure("position time scaling profile")
plot_path = plt.axis("equal")
plot_path = plt.hlines(0, 0, pp.T, 'k', '--')
plot_path = plt.hlines(1, 0, pp.T, 'k', '--')
plot_path = plt.hlines(pp.s_1, 0, pp.T, 'r', '--')
plot_path = plt.hlines(1-pp.s_1, 0, pp.T, 'r', '--')
plot_path = plt.plot(time_scaling_plot_data[0, 0:plot_index], time_scaling_plot_data[1, 0:plot_index])

# position with continuos position
plot_path = plt.figure("end effector trajectorty")
plot_path = plt.axis("equal")
plot_path = plt.plot(x_des_plot_data[0, 0:plot_index], x_des_plot_data[1, 0:plot_index], '.',label="continuous position")

# position with continuos position
plot_vel = plt.figure("veloity time scaling profile")
plot_vel = plt.plot(np.linspace(0, t, plot_index+1), vel_plot_data[0:plot_index+1])
plot_path = plt.hlines(max_vel, 0, t, 'r', '--')


plt.show()