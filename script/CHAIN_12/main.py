import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.visualize import GepettoVisualizer
import numpy as np
import time

from inverse_geometry import inverse_geometry_step

from conf import *

from os.path import dirname, join, abspath
import sys


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
pinocchio_model_dir = join(dirname(str(abspath(__file__))),"../delta_robot_description")
mesh_dir = join(pinocchio_model_dir,"meshes")

# Load the URDF model
urdf_filename = "delta_robot.urdf"
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
q_1             = np.empty((robot.nq, N+1))  # joint angles
q_1[:,0]        = np.array([450, 0, 450, 0])
q_1_viz         = np.array([0, 0, 0, 0])
x_1             = np.empty((2, N))           # frame position
cost_1          = np.empty(N)
grad_norm_1     = np.empty(N)             # gradient norm
    
# inverse geometry for chain 2
q_2             = np.empty((robot.nq, N+1))  # joint angles
q_2[:,0]        = np.array([450, 0, 450, 0])
q_2_viz         = np.array([0, 0, 0, 0])
x_2             = np.empty((2, N))           # frame position
cost_2          = np.empty(N)
grad_norm_2     = np.empty(N)             # gradient norm


x_des = np.empty(2)
time_linspace = np.linspace(0, 10*np.pi, 400)

viz.display(q_2[:,0])
time.sleep(2)


for t in time_linspace:

    x_des[0] = 100*np.sin(t + 1.5)
    x_des[1] = -250 + 100*np.cos(t + 1.5)
    time.sleep(0.05)

    q_1_next = True
    q_2_next = True
    for i in range(N):

        # CHAIN 1
        if(q_1_next is not None):
            H1 = robot.framePlacement(q_1[:,i], ROD1_FRAME_INDEX)
            x_1[:,i] = np.array([H1.translation[0], H1.translation[2]])  # take the X-Z position of the end-effector

            J1_full = robot.computeFrameJacobian(q_1[:,i], ROD1_FRAME_INDEX)
            J1 = J1_full[[0,2],:]

            q_1_next = inverse_geometry_step(q_1[:,i], x_1[:,i], x_des, J1, i, N, robot, ROD1_FRAME_INDEX)
            q_1[:,i+1] = q_1_next

            q_1_viz = q_1[:,i]
        # CHAIN 1 END

        # CHAIN 2
        if(q_2_next is not None):
            H2 = robot.framePlacement(q_2[:,i], ROD2_FRAME_INDEX)
            x_2[:,i] = np.array([H2.translation[0], H2.translation[2]])  # take the X-Z position of the end-effector
            
            J2_full = robot.computeFrameJacobian(q_2[:,i], ROD2_FRAME_INDEX)
            J2 = J2_full[[0,2],:]

            q_2_next = inverse_geometry_step(q_2[:,i], x_2[:,i], x_des, J2, i, N, robot, ROD2_FRAME_INDEX)
            q_2[:,i+1] = q_2_next

            q_2_viz = q_2[:,i]
        # CHAIN 2 END


        q = np.concatenate((q_1_viz[0:2], q_2_viz[2:4]))
        
        # time.sleep(0.4)
        viz.display(q)
        
        if(q_1_next is None and q_2_next is None):
            break

    q_1[:,0] = q_1_viz
    q_2[:,0] = q_2_viz