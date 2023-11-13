import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.visualize import GepettoVisualizer
import numpy as np
import matplotlib.pyplot as plt

from os.path import dirname, join, abspath
import sys
import time

from kinematics.IK_solver import IK_solver
from collision.collision_check import is_Collision


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
    # calculate the position of end-effector in time
    x_des = path.Path_B_spline(t)

    # solving the inverse geometry
    q_1 = solver_1.solve_GN(q_1, x_des)
    q_2 = solver_2.solve_GN(q_2, x_des)

    # applying joint constraints
    q_1[2] = q_1[1]         # keeps the ee parallel to ground
    q_2[5] = q_2[4]         # keeps rod_3 parallel to rod_2
    q = np.concatenate((q_1[0:3], q_2[3:6]))

    if is_Collision(robot, q, (0, 1)):
        break

    viz.display(q)
    time.sleep(path.time_delay)