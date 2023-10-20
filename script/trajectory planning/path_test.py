from path_planning import *
import matplotlib.pyplot as plt
import numpy as np


N = 300000
counter = 0

t = 0
s = 0
ts = np.empty((2,N))

x_des_plot_data = np.empty((2, N))
x_des = np.empty(2)

while True:
    # s = time_scaling(t)
    s = t
    if t > 1:
        s = None

    if s is not None:
        x_des = bezier_curve(s)
        print(current_velocity(t))
    else:
        break
        
    # print(f"s = {s}\tt = {t}\tx = {x_des}")

    if counter > N:
        break

    # print(f"t = {t}\t\ts = {s}")
    ts[:, counter]                  = np.array([t, s])
    x_des_plot_data[:, counter]     = np.array([x_des[0], x_des[2]])

    counter += 1
    t = round(t + delta_t, 3)

print(f"t1={t_1}\tt2={T-t_2}\tT={T}\ts={s}")

plot_time_scale_fn = plt.figure("time scaling fn")
plot_path = plt.axis("equal")
plot_path = plt.hlines(0, 0, t, 'k', '-')
plot_path = plt.hlines(1, 0, t, 'k', '-')
plot_path = plt.hlines(s_1, 0, t, 'r', '--')
plot_path = plt.hlines(1-s_1, 0, t, 'r', '--')
plot_path = plt.plot(ts[0, 0:counter], ts[1, 0:counter])

# position with continuos position
plot_path = plt.figure("path continuous")
plot_path = plt.axis("equal")
plot_path = plt.plot(x_des_plot_data[0, 0:counter], x_des_plot_data[1, 0:counter], '.',label="continuous position")

plt.show()