## test the algorithm
from ..deltarobot import configuration as conf
from ..deltarobot.trajectory_generator import TrajectoryGenerator

import matplotlib.pyplot as plt
import numpy as np
import time


def main():
    trajectory_generator = TrajectoryGenerator()
    
    pos_start = np.array([0, 0, 0])
    pos_end = np.array([100, 0, 0])
    task_time = -1
    task_type = conf.PLACE_TRAJECTORY_ROUTINE
    # task_type = conf.PICK_TRAJECTORY_ROUTINE
    # task_type = conf.DIRECT_TRAJECTORY_ROUTINE

    t_start = time.time()
    trajectory_vector = trajectory_generator.generate_trajectory(pos_start, pos_end, task_time, task_type)
    t_end = time.time()
    print(f"Computed time: {round((t_end - t_start)*1e3, 3)} [ ms ]")


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory_vector[:,0], trajectory_vector[:,1],trajectory_vector[:,2], marker="o")
    

    # Label axes
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')

    ax.grid(True)
    plt.show()



if __name__ == "__main__":
    main()