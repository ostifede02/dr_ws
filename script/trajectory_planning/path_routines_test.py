import numpy as np
import matplotlib.pyplot as plt

import configuration_data as conf
from configuration_functions import trajectory_configuration_block
from trajectory_functions import bezier_curve


# delta_s = 0.05

pos_start = np.array([0, 0, -100])       # at the end of cycle pos_start_new = pos_end_prev
pos_end = np.array([50, 0, -280])          # should be input to function goto()

trajectory_data = trajectory_configuration_block("quick", pos_start, pos_end)
delta_s = 0.05

# PLOT data
pos_data = np.empty((3, int(1/delta_s)))
plot_data_index = 0

s_instance = np.linspace(0, 1, int(1/delta_s))

for s in s_instance:
    ## next via point
    pos = bezier_curve(s, trajectory_data["via_points"])
    pos_next = bezier_curve(s+delta_s, trajectory_data["via_points"])
    delta_x = np.linalg.norm(pos_next-pos)
    print(delta_x)

    # plot data
    pos_data[:, plot_data_index] = pos
    plot_data_index += 1



## PLOT stuff
pos_plot = plt.figure("path routine")
pos_plot = plt.axis("equal")
pos_plot = plt.plot(pos_data[0, :], pos_data[2, :],".")


plt.show()
