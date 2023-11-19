import numpy as np

from delta_robot import DeltaRobot

dr = DeltaRobot(viewer=True)


pos_start = np.array([0, 0, 0])
pos_end = np.array([0, 0, 100])

dr.trajectory.set_trajectory_routine("quick", pos_start, pos_end)
print(dr.trajectory.t_total)

q = np.array([0, 0, 0, 0, 0, 0])
dr.display(q)

