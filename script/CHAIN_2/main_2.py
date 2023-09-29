import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.visualize import GepettoVisualizer
import numpy as np
import time
import matplotlib.pyplot as plt

from inverse_geometry_2 import inverse_geometry_step

from conf import *
ROD2_FRAME_INDEX = 8

from os.path import dirname, join, abspath
import sys


# NOTE: the coordinate system used in the URDF file is as follows
# 
#           ^ z
#           |
#           |   ^ y
#           |  /
#           | /
#        __O|/___________> x
#          /|
# 


# Load the mesh files
pinocchio_model_dir = join(dirname(str(abspath(__file__))),"../delta_robot_description")
mesh_dir = join(pinocchio_model_dir,"meshes")

# Load the URDF model
urdf_filename = "delta_robot_2.urdf"
urdf_dir = join(pinocchio_model_dir,"urdf")
urdf_file_path = join(urdf_dir, urdf_filename)

# Initialize the model
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_file_path, mesh_dir, geometry_types=[pin.GeometryType.COLLISION,pin.GeometryType.VISUAL])
data, collision_data, visual_data = pin.createDatas(model,collision_model,visual_model)
robot = RobotWrapper(model, collision_model, visual_model)


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



# inverse geometry for chain 1
q_2             = np.empty((robot.nq, N+1))  # joint angles
q_2[:,0]        = np.array([-450, 3*np.pi/4])
q_2_viz         = np.array([0, 0])
x_2             = np.empty((2, N))           # frame position
cost_2          = np.empty(N)
grad_norm_2     = np.empty(N)  


x_des = np.empty(2)
time_linspace = np.linspace(0, 100, 201)


viz.display(q_2[:,0])
time.sleep(3)


for t in time_linspace:

    x_des[0] = t
    x_des[1] = -2*t

    for i in range(N):

        # CHAIN 1
        H2 = robot.framePlacement(q_2[:,i], ROD2_FRAME_INDEX)
        x_2[:,i] = np.array([H2.translation[0], H2.translation[2]])  # take the X-Z position of the end-effector
        
        J2_full = robot.computeFrameJacobian(q_2[:,i], ROD2_FRAME_INDEX)
        J2 = J2_full[[0,2],:]

        q_2_next = inverse_geometry_step(q_2[:,i], x_2[:,i], x_des, J2, i, N, robot, ROD2_FRAME_INDEX)
        q_2[:,i+1] = q_2_next

        q_2_viz = q_2[:,i]
        if(q_2_next is None):
            break

        q_2[:,i+1] = q_2_next
        # CHAIN 1 END
        
        viz.display(q_2_next)

    q_2[:,0] = q_2[:,i-1]
    time.sleep(0.05)