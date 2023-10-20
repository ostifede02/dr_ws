import path_planning as pp
from path_planning import *

import matplotlib.pyplot as plt
import numpy as np


N = 300000
plot_index = 0

t = 0
s = 0
s_index = 0

ts = np.empty((2,N))
x_des_plot_data = np.empty((2, N))
vel_plot_data = np.empty(N)
vel = 0

x_des = np.empty(3)
x_prev = np.empty(3)

while True:
    x_prev = x_des

    s = time_scaling(t)
    if s is None:
        break

    if s_index % 10 != 0:
        s_index += 1
        t = round(t + delta_t, 3)
        continue
    else:
        s_index += 1
        t = round(t + delta_t, 3)

    x_des = bezier_curve(s)

    if plot_index > N:
        break
    
    # save data for plots
    ts[:, plot_index]                  = np.array([t, s])
    x_des_plot_data[:, plot_index]     = np.array([x_des[0], x_des[2]])

    vel = np.linalg.norm(x_des-x_prev) / (delta_t*10)
    vel_plot_data[plot_index] = vel

    plot_index += 1

print(f"t1={pp.t_1}\tt2={pp.T-pp.t_2}\tT={pp.T}\ts={s}")




# PLOT STUFF
plot_time_scale_fn = plt.figure("time scaling fn")
plot_path = plt.axis("equal")
plot_path = plt.hlines(0, 0, t, 'k', '--')
plot_path = plt.hlines(1, 0, t, 'k', '--')
plot_path = plt.hlines(pp.s_1, 0, t, 'r', '--')
plot_path = plt.hlines(1-pp.s_1, 0, t, 'r', '--')
plot_path = plt.plot(ts[0, 0:plot_index], ts[1, 0:plot_index])

# position with continuos position
plot_path = plt.figure("path continuous")
plot_path = plt.axis("equal")
plot_path = plt.plot(x_des_plot_data[0, 0:plot_index], x_des_plot_data[1, 0:plot_index], '.',label="continuous position")

# position with continuos position
plot_vel = plt.figure("veloity")
plot_vel = plt.plot(np.linspace(0, pp.T, plot_index), vel_plot_data[1:plot_index+1])


plt.show()