import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.visualize import GepettoVisualizer
import numpy as np

from os.path import dirname, join, abspath
import sys
import time
import timeit

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


# start and end points
P0 = np.array([-200, 0, -300])
P3 = np.array([200, 0, -240])

# via points
z_offset = 120
P1 = np.array([P0[0], 0, P0[2]+z_offset])
P2 = np.array([P3[0], 0, P3[2]+z_offset])

T = 1.5
# time scaling function parameters
a3 = 10 / pow(T, 3)
a4 = -(15 / pow(T, 4))
a5 = 6 / pow(T, 5)

x_des = np.empty(3)

q_steps = 50
# q_linspace = np.linspace(0, T, q_steps)
for i in range(1):
    if(i%2 == 0):
        q_linspace = np.linspace(0, T, q_steps)
    else:
        q_linspace = np.linspace(T, 0, q_steps)

    for q in q_linspace:
        t = a3*pow(q, 3) + a4*pow(q, 4) + a5*pow(q, 5)
        x_des = pow(1-t, 3)*P0 + 3*pow(1-t, 2)*t*P1 + 3*(1-t)*pow(t, 2)*P2 + pow(t, 3)*P3
        print(x_des)

        q_1 = solver_1.solve_GN(q_1, x_des)
        q_2 = solver_2.solve_GN(q_2, x_des)
        q = np.concatenate([q_1[0:2], q_2[2:4]])

        viz.display(q)
        time.sleep(T/q_steps)
