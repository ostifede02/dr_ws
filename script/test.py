import pinocchio as pin
from pinocchio import RobotWrapper
import numpy as np
import time
from conf import *

from os.path import dirname, join, abspath


# NOTE: the coordinate system used in the URDF file is as follows
# 
#           ^ z
#           |
#           |   ^ y
#           |  /
#           | /
#           |/___________> x
#           O
# 


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

q_1 = np.array([0, 0, 0, 0])
J1_full = robot.computeFrameJacobian(q_1, ROD1_FRAME_INDEX)

H1 = robot.framePlacement(q_1, ROD1_FRAME_INDEX)
J1_full = robot.computeFrameJacobian(q_1, ROD1_FRAME_INDEX)
J1 = J1_full[[0,2],:]
nv = J1.shape[0]

x_1 = np.array([H1.translation[0], H1.translation[2]])  # take the X-Z position of the end-effector



print(f"J1.shape[1]\n{nv}\n\n")
print(f"J1_full\n{J1_full}\n\n")
print(f"J1\n{J1}\n\n")
print(f"x_1\n{x_1}\n\n")



