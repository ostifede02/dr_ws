import pinocchio as pin
from pinocchio import RobotWrapper
import numpy as np
import matplotlib.pyplot as plt

from os.path import dirname, join, abspath

import kinematics.fk_analytic_sol
from kinematics.IK_solver import IK_solver

from trajectory_planning.trajectory_functions import bezier_curve



# Load the mesh files
pinocchio_model_dir = join(dirname(str(abspath(__file__))),"delta_robot_description")
mesh_dir = join(pinocchio_model_dir,"meshes")

# Load the URDF model
urdf_filename = "delta_robot.urdf"
urdf_dir = join(pinocchio_model_dir,"urdf")
urdf_file_path = join(urdf_dir, urdf_filename)

# Initialize the model
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_file_path, mesh_dir, geometry_types=[pin.GeometryType.COLLISION,pin.GeometryType.VISUAL])
data, collision_data, visual_data = pin.createDatas(model,collision_model,visual_model)
robot = RobotWrapper(model, collision_model, visual_model)
robot.collision_model.addAllCollisionPairs()
robot.collision_data = pin.GeometryData(robot.collision_model)


# NOTE: the coordinate system used in the URDF file is as follows
#       since the robot has only 2 DOF, the y-axis is not to be taken into account   
#
#           ^ z
#           |
#           |   ^ y
#           |  /
#           | /
#           |/___________> x
#           O
# 

# stepper and pulley parameters
n_teeth = 20
module = 3
c_pulley = n_teeth * module
d_pulley = c_pulley / np.pi

# steps per revolution
n_steps = 800                               # possible halfsteps [200, 400, 800, 1600, 3200]
min_displacement = c_pulley / n_steps       # minimum carriage displacement


# start and end position
P0 = np.array([-20, 0, 0])
P1 = np.array([-8, 0, 0])
P2 = np.array([8, 0, 0])
P3 = np.array([20, 0, 0])

P = np.array([P0, P1, P2, P3])


# instance of inverse kinematic solver
solver_1 = IK_solver(robot, frame_id=8)     # solver_1 solves the ik for the red left chain aka chain 1
solver_2 = IK_solver(robot, frame_id=14)    # solver_2 solves the ik for the green right chain aka chain 2

# defining intial neutral joint position
q_1 = pin.neutral(model)
q_2 = pin.neutral(model)

# defining intial joint position
x_des = bezier_curve(0, P)
q_1 = solver_1.solve_GN(q_1, x_des)
q_2 = solver_2.solve_GN(q_2, x_des)

# defining discrete initial joint position
stepper_1_steps = q_1[0] // min_displacement
q_1_reminder = q_1[0] % min_displacement
q_1_discrete = stepper_1_steps * min_displacement

stepper_2_steps = q_2[3] // min_displacement
q_2_reminder = q_2[3] % min_displacement
q_2_discrete = stepper_2_steps * min_displacement

delta_s = 0.1

# stuff for plots
x_des_plot_data = np.empty((2, int(1/delta_s)))
x_discrete_plot_data = np.empty((2, int(1/delta_s)))
error_plot_data = np.empty(int(1/delta_s))
# t_delay_per_step_plot_data = np.empty((2, int(1/delta_s)))
n_steps_plot_data = np.empty((2, int(1/delta_s)))
plot_index = 0

s_instance = np.linspace(0, 1, int(1/delta_s))

for s in s_instance:
    # calculate the position of end-effector
    x_des = bezier_curve(s, P)
    print(x_des)

    q_1_prev = q_1[0]       # position of carriage 1
    q_2_prev = q_2[3]       # position of carriage 2

    # solving the inverse geometry
    q_1 = solver_1.solve_GN(q_1, x_des)
    q_2 = solver_2.solve_GN(q_2, x_des)


    # calculating the number of steps to do (further to send to the motorcontroller)
    delta_q_1 = q_1[0] - q_1_prev
    stepper_1_steps = (delta_q_1 + q_1_reminder) // min_displacement    # number of steps to do
    q_1_reminder = (delta_q_1 + q_1_reminder) % min_displacement        # the decimal part of steps

    delta_q_2 = q_2[3] - q_2_prev
    stepper_2_steps = (delta_q_2 + q_2_reminder) // min_displacement    # number of steps to do
    q_2_reminder = (delta_q_2 + q_2_reminder) % min_displacement        # the decimal part of steps


    # discretized joint position
    q_1_discrete_displacement = stepper_1_steps * min_displacement
    q_1_discrete += q_1_discrete_displacement
    
    q_2_discrete_displacement = stepper_2_steps * min_displacement
    q_2_discrete += q_2_discrete_displacement

    # calculating the actual ee position
    x_discrete = kinematics.fk_analytic_sol.EE_position_geometrically(q_1_discrete, q_2_discrete)

    # save data for plots
    x_des_plot_data[:, plot_index]              = np.array([x_des[0], x_des[2]])
    x_discrete_plot_data[:, plot_index]         = np.array([x_discrete[0], x_discrete[2]])
    error_plot_data[plot_index]                 = np.linalg.norm(x_des-x_discrete)
    n_steps_plot_data[:, plot_index]            = np.array([stepper_1_steps, stepper_2_steps])
    # t_delay_per_step_plot_data[:, plot_index]   = np.array([path.time_delay / stepper_1_steps, path.time_delay / stepper_2_steps])*1e6
    plot_index += 1


iteration_step = np.linspace(0, int(1/delta_s), int(1/delta_s))

### plot stuff
# position with continuos vs discrete joint position
plot_path = plt.figure("path continuous and dicrete analysis")
plot_path = plt.axis("equal")
plot_path = plt.plot(x_des_plot_data[0, :], x_des_plot_data[1, :], 'x',label="continuous position")
plot_path = plt.plot(x_discrete_plot_data[0, :], x_des_plot_data[1, :], 'd', label="discrete position")
plot_path = plt.legend(title=f"steps/rev: {n_steps}")

# position error at each iteration
plot_error = plt.figure("position error")
plot_error = plt.xlabel("via point of path")
plot_error = plt.ylabel("error [mm]")
plot_error = plt.step(iteration_step, error_plot_data[:], where="mid", label="diplacement error")
plot_error = plt.legend(title=f"steps/rev: {n_steps}")
# Add numerical values next to the data points
for i, (x, y) in enumerate(zip(iteration_step, error_plot_data)):
    plt.text(x, y*1.01, f'{y:.2f} [mm]', ha='center', va='bottom', fontsize=10, color='black')

# plot_error = plt.hlines(np.mean(error_plot_data), 0, int(1/delta_s), 'r', '--', label="mean error")

# # steps per iteration
# plot_steps = plt.figure("steps")
# plot_steps = plt.xlabel("iteration")
# plot_steps = plt.ylabel(f"number of steps in {round(path.time_delay*1e3, 1)} [milliseconds]")
# plot_steps = plt.step(iteration_step, n_steps_plot_data[0,:], 'r', where="mid", label="stepper 1")
# plot_steps = plt.step(iteration_step, n_steps_plot_data[1,:], 'g', where="mid", label="stepper 2")
# plot_steps = plt.legend(title=f"steps/rev: {n_steps}")
# plot_error = plt.hlines(0, 0, int(1/delta_s), colors='k', linestyles='dashed')

# # time delay between each step
# plot_error = plt.figure("time delay per step")
# plot_error = plt.xlabel("iteration")
# plot_error = plt.ylabel("delay [microseconds]")
# plot_error = plt.ylim(top=800, bottom=-800)
# plot_error = plt.step(iteration_step, t_delay_per_step_plot_data[0,:], 'r', where="mid", label="delay stepper 1")
# plot_error = plt.step(iteration_step, t_delay_per_step_plot_data[1,:], 'g', where="mid", label="delay stepper 2")
# plot_error = plt.hlines(60, 0, int(1/delta_s), colors='r', linestyles='dashed')
# plot_error = plt.hlines(-60, 0, int(1/delta_s), colors='r', linestyles='dashed')
# plot_error = plt.legend(title=f"steps/rev: {n_steps}")

plt.show()