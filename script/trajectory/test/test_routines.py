import numpy as np
import matplotlib.pyplot as plt

from script.trajectory.trajectory import Trajectory
from script import configuration as conf


# pick
pos_start_1 = np.array([0, 0, -100])
pos_end_1 = np.array([80, 0, -280])

# place
pos_start_2 = pos_end_1
pos_end_2 = np.array([-120, 0, -300])

# quick
pos_start_3 = pos_end_2
pos_end_3 = pos_start_1

trj = Trajectory()



################    PICK    ####################
trj.set_trajectory_routine("pick", pos_start_1, pos_end_1)

pos_data_1 = np.empty((3, int(1/trj.delta_s)))
plot_data_index = 0
s_instance = np.linspace(0, 1, int(1/trj.delta_s))

for s in s_instance:
    ## next via point
    pos = trj.get_position_bezier_poly(s)

    # plot data
    pos_data_1[:, plot_data_index] = pos
    plot_data_index += 1




################    PLACE    ####################
trj.set_trajectory_routine("place", pos_start_2, pos_end_2)

pos_data_2 = np.empty((3, int(1/trj.delta_s)))
plot_data_index = 0
s_instance = np.linspace(0, 1, int(1/trj.delta_s))

for s in s_instance:
    ## next via point
    pos = trj.get_position_bezier_poly(s)

    # plot data
    pos_data_2[:, plot_data_index] = pos
    plot_data_index += 1



################    RETURN    ####################
trj.set_trajectory_routine("quick", pos_start_3, pos_end_3)

pos_data_3 = np.empty((3, int(1/trj.delta_s)))
plot_data_index = 0
s_instance = np.linspace(0, 1, int(1/trj.delta_s))

for s in s_instance:
    ## next via point
    pos = trj.get_position_bezier_poly(s)

    # plot data
    pos_data_3[:, plot_data_index] = pos
    plot_data_index += 1



## PLOT stuff
pos_plot = plt.figure("path routine")

pos_plot = plt.subplot(1,2,1)
pos_plot = plt.axis("equal")
pos_plot = plt.title("path routines")
pos_plot = plt.xlabel("x-axis")
pos_plot = plt.ylabel("z-axis")

pos_plot = plt.plot(pos_data_1[0, :], pos_data_1[2, :], "r", label="pick")
pos_plot = plt.plot(pos_data_2[0, :], pos_data_2[2, :], "g", label="place")
pos_plot = plt.plot(pos_data_3[0, :], pos_data_3[2, :], "b", label="return")

pos_plot = plt.text(pos_start_1[0]+5, pos_start_1[2], "neutral position")
pos_plot = plt.text(pos_start_2[0]-30, pos_start_2[2], "pick object")
pos_plot = plt.text(pos_start_3[0]+5, pos_start_3[2], "place object")
pos_plot = plt.legend()



pos_plot = plt.subplot(1,2,2)
pos_plot = plt.axis("equal")
pos_plot = plt.title("via points of path routines")
pos_plot = plt.xlabel("x-axis")
pos_plot = plt.ylabel("z-axis")


pos_plot = plt.plot(pos_data_1[0, :], pos_data_1[2, :], ".r", label="pick")
pos_plot = plt.plot(pos_data_2[0, :], pos_data_2[2, :], ".g", label="place")
pos_plot = plt.plot(pos_data_3[0, :], pos_data_3[2, :], ".b", label="return")

pos_plot = plt.text(pos_start_1[0]+5, pos_start_1[2], "neutral position")
pos_plot = plt.text(pos_start_2[0]-30, pos_start_2[2], "pick object")
pos_plot = plt.text(pos_start_3[0]+5, pos_start_3[2], "place object")
pos_plot = plt.legend()


plt.show()
