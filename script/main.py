import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.visualize import GepettoVisualizer
import numpy as np
import matplotlib.pyplot as plt

from os.path import dirname, join, abspath
import sys
import time

from IK_solver import IK_solver
import path_planning as path



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


# Initialize the viewer.
viz = GepettoVisualizer(model, collision_model, visual_model)
try:
    viz.initViewer()
except ImportError as err:
    print("Error while initializing the viewer. It seems you should install gepetto-viewer")
    print(err)
    sys.exit(0)

try:
    viz.loadViewerModel("pinocchio")
except AttributeError as err:
    print("Error while loading the viewer model. It seems you should start gepetto-viewer")
    print(err)
    sys.exit(0)


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



# instance of inverse kinematic solver
solver_1 = IK_solver(robot, frame_id=8)     # solver_1 solves the ik for the red left chain aka chain 1
solver_2 = IK_solver(robot, frame_id=14)    # solver_2 solves the ik for the green right chain aka chain 2

# defining intial neutral joint position
q_1 = pin.neutral(model)
q_2 = pin.neutral(model)
# desired ee position
x_des = np.empty(3)

time.sleep(1)
t_instance = np.linspace(0, path.T, path.t_intervals)

for t in t_instance:
    # calculate the position of ee in time
    x_des = path.Path_B_spline(t)

    # solving the inverse geometry
    q_1 = solver_1.solve_GN(q_1, x_des)
    q_2 = solver_2.solve_GN(q_2, x_des)

    # configuring the joints for visualization
    # keeps the ee parallel to ground
    q_1[2] = q_1[1]
    # keeps rod_3 parallel to rod_2
    q_2[5] = q_2[4]
    q = np.concatenate((q_1[0:3], q_2[3:6]))


    # checking for collisions
    pin.computeCollisions(robot.model, robot.data, robot.collision_model, robot.collision_data, q, False)

    if(robot.collision_data.collisionResults[0].isCollision()):
        print("\n\nCollision between:")
        print(robot.collision_model.collisionPairs[0])
        print("at ", t)
        break
    elif(robot.collision_data.collisionResults[1].isCollision()):
        print("\n\nCollision between:")
        print(robot.collision_model.collisionPairs[1])
        print("at ", t)
        break

    viz.display(q)
    time.sleep(path.time_delay)

















# for i in range(100):
#     if(i%2 == 0):
#         s_linspace = np.linspace(0, path.T, path.s_steps)
#     else:
#         s_linspace = np.linspace(path.T, 0, path.s_steps)
#     time.sleep(0.2)
    
#     for s in s_linspace:
#         x_des = path.Path_B_spline(s)

#         # actual code
#         q_1 = solver_1.solve_GN(q_1, x_des)
#         q_2 = solver_2.solve_GN(q_2, x_des)

#         q_1[2] = q_1[1]
#         q_2[5] = q_2[4]
#         q = np.concatenate((q_1[0:3], q_2[3:6]))

#         # checking for collisions
#         robot.collision_model.addAllCollisionPairs()
#         robot.collision_data = pin.GeometryData(robot.collision_model)
#         pin.computeCollisions(robot.model, robot.data, robot.collision_model, robot.collision_data, q, False)

#         cr = 0
#         cp = 0
#         for k in range(len(robot.collision_model.collisionPairs)):
#             cr = robot.collision_data.collisionResults[k]
#             cp = robot.collision_model.collisionPairs[k]
#             if(cr.isCollision()):
#                 if(int(cp.first) == 0 or int(cp.second) == 0):
#                     print("collision pair:",cp.first,",",cp.second) 
#                     is_collision = 1
                    
#         if(is_collision):
#             print("break")
#             break

#         viz.display(q)
#         time.sleep(path.time_delay)