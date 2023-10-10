import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.visualize import GepettoVisualizer
import numpy as np
import matplotlib.pyplot as plt

from os.path import dirname, join, abspath

from IK_solver import IK_solver
import path_planning as path
import fk_analytic_sol



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
n_steps = 1600           # possible halfsteps [200, 400, 800, 1600, 3200]

# minimum carriage displacement
min_displacement = c_pulley / n_steps



# instance of inverse kinematic solver
solver_1 = IK_solver(robot, frame_id=8)     # solver_1 solves the ik for the red left chain aka chain 1
solver_2 = IK_solver(robot, frame_id=14)    # solver_2 solves the ik for the green right chain aka chain 2

# defining intial neutral joint position
q_1 = pin.neutral(model)
q_2 = pin.neutral(model)

# defining intial joint position
x_des = path.Path_B_spline(0)
q_1 = solver_1.solve_GN(q_1, x_des)
q_2 = solver_2.solve_GN(q_2, x_des)

# discrete carriage position
stepper_1_steps = q_1[0] // min_displacement
q_1_reminder = q_1[0] % min_displacement
q_1_discrete = stepper_1_steps * min_displacement

stepper_2_steps = q_2[3] // min_displacement
q_2_reminder = q_2[3] % min_displacement
q_2_discrete = stepper_2_steps * min_displacement


# stepper variable initialization
q_1_reminder = 0
q_2_reminder = 0

# stuff for plots
x_des_plot_data = np.empty((2, path.t_intervals))
x_discrete_plot_data = np.empty((2, path.t_intervals))
error_plot_data = np.empty(path.t_intervals)
n_steps_plot_data = np.empty((2, path.t_intervals))
plot_index = 0

t_instance = np.linspace(0, path.T, path.t_intervals)

for t in t_instance:
    # calculate the position of end-effector in time
    x_des = path.Path_B_spline(t)

    q_1_prev = q_1[0]
    q_2_prev = q_2[3]

    # solving the inverse geometry
    q_1 = solver_1.solve_GN(q_1, x_des)
    q_2 = solver_2.solve_GN(q_2, x_des)


    # steps per motor
    delta_q_1 = q_1[0] - q_1_prev
    stepper_1_steps = (delta_q_1 + q_1_reminder) // min_displacement
    q_1_reminder = (delta_q_1 + q_1_reminder) % min_displacement

    delta_q_2 = q_2[3] - q_2_prev
    stepper_2_steps = (delta_q_2 + q_2_reminder) // min_displacement
    q_2_reminder = (delta_q_2 + q_2_reminder) % min_displacement


    # discretized joint position
    q_1_discrete_displacement = stepper_1_steps * min_displacement
    q_1_discrete += q_1_discrete_displacement
    
    q_2_discrete_displacement = stepper_2_steps * min_displacement
    q_2_discrete += q_2_discrete_displacement

    # calculating the actual ee position
    x_discrete = fk_analytic_sol.EE_position_geometrically(q_1_discrete, q_2_discrete)

    # save data for plots
    x_des_plot_data[:, plot_index] = np.array([x_des[0], x_des[2]])
    x_discrete_plot_data[:, plot_index] = np.array([x_discrete[0], x_discrete[2]])
    error_plot_data[plot_index] = np.linalg.norm(x_des-x_discrete)
    n_steps_plot_data[:, plot_index] = np.array([stepper_1_steps, stepper_2_steps])
    plot_index += 1


iteration_step = np.linspace(0, path.t_intervals, path.t_intervals)

# plot stuff
plot_path = plt.figure("path continuous and dicrete analysis")
plot_path = plt.axis("equal")
plot_path = plt.plot(x_des_plot_data[0, :], x_des_plot_data[1, :], 'x',label="continuous position")
plot_path = plt.plot(x_discrete_plot_data[0, :], x_des_plot_data[1, :], 'd', label="discrete position")
plot_path = plt.legend(title=f"steps/rev: {n_steps}")

# position error at each iteration
plot_error = plt.figure("position error")
plot_error = plt.xlabel("iteration")
plot_error = plt.ylabel("error [mm]")
plot_error = plt.step(iteration_step, error_plot_data[:], where="mid", label="diplacement error")
plot_error = plt.hlines(np.mean(error_plot_data), 0, path.t_intervals, 'r', '--', label="mean error")
plot_error = plt.legend(title=f"steps/rev: {n_steps}")

# steps per iteration
plot_steps = plt.figure("steps")
plot_steps = plt.xlabel("iteration")
plot_steps = plt.ylabel(f"number of steps in {round(path.time_delay*1e3, 3)} milliseconds")
plot_steps = plt.step(iteration_step, n_steps_plot_data[0,:], 'r', where="mid", label="stepper 1")
plot_steps = plt.step(iteration_step, n_steps_plot_data[1,:], 'g', where="mid", label="stepper 2")
plot_steps = plt.legend(title=f"steps/rev: {n_steps}")
plot_error = plt.hlines(0, 0, path.t_intervals, colors='k', linestyles='dashed')


plt.show()






















# print(f"x_des: {x_des}")
# print(f"delta_q_1: {delta_q_1}\tdelta_q_1 with reminder: {delta_q_1 + q_1_reminder}")
# print(f"stepper 1: {stepper_1_steps}\tq_1_reminder: {q_1_reminder}\n\n")

# print(f"x_des: {x_des}")
# print(f"delta_q_2: {delta_q_2}\tdelta_q_2 with reminder: {delta_q_2 + q_2_reminder}")
# print(f"q_2_reminder: {q_2_reminder}")
# print(f"stepper 2: {stepper_2} to do in: {path.time_delay} seconds.")
# print(f"1 step each {(path.time_delay/stepper_2)*1e6} microseconds.\n")


