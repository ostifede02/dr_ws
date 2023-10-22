import path_planning_V2 as pp

import matplotlib.pyplot as plt
import numpy as np



s = 0
delta_s = 0.01      # time increment in seconds
s_index = 0
s_iteration_filter = 1

delta_t = 0
T = 0

N = 100000
time_scaling_plot_data = np.empty((2,N))
x_des_plot_data = np.empty((2, N))
vel_plot_data = np.empty(N)
delta_t_plot_data = np.empty(N)
vel = 0
plot_index = 1

vel_plot_data[plot_index] = 0
delta_t_plot_data[plot_index] = 0

x_des = pp.bezier_curve(0)
x_prev = np.empty(3)

while True:
    # **********  START of code  ************
    x_prev = x_des

    # print(f"vel={vel}\tdelta_t={delta_t}")
    delta_t, vel = pp.time_taken_interval(s, delta_s)
    # print("hi")

    if delta_t is None:
        break
    
    x_des = pp.bezier_curve(s)
    print(x_des)
    print(s)
    T += delta_t
    s = s + delta_s
    
    # **********  END of code  ************

    # **********  save data for plots  ************

    # save data for plotime_scaling_plot_data
    # time_scaling_plot_data[:, plot_index]   = np.array([s, s]) -> WARNING s,s
    x_des_plot_data[:, plot_index]          = np.array([x_des[0], x_des[2]])
    
    # vel = pp.get_velocity_bezier_curve(s, delta_s, delta_t)
    vel_plot_data[plot_index] = vel
    delta_t_plot_data[plot_index] = delta_t

    plot_index += 1

print(f"t1={pp.s_1}\tT={T}\ts={s}")
print(f"e={np.linalg.norm(x_des-pp.bezier_curve(1))}")




# PLOT STUFF
# plot_time_scale_fn = plt.figure("position time scaling profile")
# plot_path = plt.axis("equal")
# plot_path = plt.hlines(0, 0, 1, 'k', '--')
# plot_path = plt.hlines(1, 0, 1, 'k', '--')
# plot_path = plt.hlines(pp.s_1, 0, s, 'r', '--')
# plot_path = plt.hlines(1-pp.s_1, 0, s, 'r', '--')
# plot_path = plt.plot(time_scaling_plot_data[0, 0:plot_index], time_scaling_plot_data[1, 0:plot_index])

# position with continuos position
plot_path = plt.figure("end effector trajectorty")
plot_path = plt.axis("equal")
plot_path = plt.plot(x_des_plot_data[0, 1:plot_index], x_des_plot_data[1, 1:plot_index], '.',label="continuous position")

# velocity profile
plot_vel = plt.figure("veloity time scaling profile")
plot_vel = plt.plot(np.linspace(0, s, plot_index-1), vel_plot_data[0:plot_index-1])
plot_vel = plt.vlines(1, 0, pp.max_vel, 'r', '--')
plot_vel = plt.vlines(pp.s_1, 0, pp.max_vel, 'r', '--')

# time taken
plot_delta_t = plt.figure("delta t")
plot_delta_t = plt.plot(np.linspace(0, s, plot_index), delta_t_plot_data[0:plot_index])


plt.show()