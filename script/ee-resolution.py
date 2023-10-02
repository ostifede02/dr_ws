import pinocchio as pin
from pinocchio import RobotWrapper
import numpy as np
from numpy.linalg import norm

from os.path import dirname, join, abspath

from IK_solver import IK_solver


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




# stepper and pulley parameters
n_teeth = 20
module = 3
c_pulley = n_teeth * module
d_pulley = c_pulley / np.pi
print(f"d_pulley = {d_pulley}")

# steps per revolution
n_steps = np.array([200, 400, 800, 1600, 3200])
# minimum carriage displacement
min_displacement = c_pulley / n_steps[1]
print(f"n_steps = {n_steps[1]}")
print(f"min_displacement = {min_displacement}\n")



# instance of inverse kinematic solver
solver_1 = IK_solver(robot, frame_id=8)     # solver_1 solves the ik for the red left chain aka chain 1
solver_2 = IK_solver(robot, frame_id=14)    # solver_2 solves the ik for the green right chain aka chain 2

# defining intial neutral joint position
q_1 = pin.neutral(model)
q_2 = pin.neutral(model)
# start point
x0 = np.array([0, 0, -200])

# joint configuration at start point
q_1 = solver_1.solve_GN(q_1, x0)
q_2 = solver_2.solve_GN(q_2, x0)
q0 = np.concatenate((q_1[0:2], q_2[2:4]))

t_range = np.linspace(0, min_displacement+1, 10000)

for t in t_range:
    x_des = x0 + np.array([t, 0, 0])

    q_1 = solver_1.solve_GN(q_1, x_des)
    q_2 = solver_2.solve_GN(q_2, x_des)
    q = np.concatenate([q_1[0:2], q_2[2:4]])


    carriage_1_displacement = abs(q[0] - q0[0])
    carriage_2_displacement = abs(q[2] - q0[2])
    ee_displacement = norm(x_des - x0)

    if(carriage_1_displacement >= min_displacement):
        print(f"The carriage 1 moved {carriage_1_displacement} [mm], while the end effector moved {ee_displacement} [mm]")
        break

    if(carriage_2_displacement >= min_displacement):
        print(f"The carriage 2 moved {carriage_2_displacement} [mm], while the end effector moved {ee_displacement} [mm]")
        break



