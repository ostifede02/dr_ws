import numpy as np
import matplotlib.pyplot as plt

from path_planning_conf import *
from path_planning_functions import *


delta_s = 0.05

pos_start = np.array([-500, 0, -280])       # at the end of cycle pos_start_new = pos_end_prev
pos_end = np.array([0, 0, -100])          # should be input to function goto()

# pick routine
x_offset_2 = 0
z_offset_2 = (pos_start[2] - pos_end[2])*0.4
x_offset_1 = ((pos_end[0]+x_offset_2)-pos_start[0])*0.5
z_offset_1 = ((pos_end[2]+z_offset_2)-pos_start[2])*0.5

# place routine
x_offset_1 = 0
z_offset_1 = (pos_end[0] - pos_start[0])*0.25
x_offset_2 = 0
z_offset_2 = (pos_end[0] - pos_start[0])*0.25

# straight path routine
x_offset_1 = (pos_end[0] - pos_start[0])*0.333
z_offset_1 = (pos_end[2] - pos_start[2])*0.333
x_offset_2 = (pos_start[0] - pos_end[0])*0.333
z_offset_2 = (pos_start[2] - pos_end[2])*0.333

offsets = np.array([x_offset_1, z_offset_1, x_offset_2, z_offset_2])

P = bezier_curve_via_points(pos_start, pos_end, offsets)

# PLOT data
pos_data = np.empty((3, int(1/delta_s)))
plot_data_index = 0

s_instance = np.linspace(0, 1, int(1/delta_s))

for s in s_instance:
    ## next via point
    pos = bezier_curve(s, P)
    pos_next = bezier_curve(s+delta_s, P)
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
