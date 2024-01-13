import pinocchio as pin
from pinocchio.visualize import GepettoVisualizer

from inverse_geometry import InverseGeometry

from os.path import join, abspath, dirname
import numpy as np
import time


 
# Load the URDF model.
urdf_file_name = "deltarobot.urdf"

package_path = join(dirname(str(abspath(__file__))),"deltarobot_description")
urdf_file_path = join(join(package_path, "urdf"), urdf_file_name)
mesh_dir = join(package_path, "meshes")

# Initialize the model
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_file_path, mesh_dir, geometry_types=[pin.GeometryType.COLLISION,pin.GeometryType.VISUAL])
robot = pin.RobotWrapper(model, collision_model, visual_model)

# Initialize the viewer
viz = GepettoVisualizer(model, collision_model, visual_model)
viz.initViewer(loadModel=True)
viz.loadViewerModel("pinocchio")



ig = InverseGeometry(robot)

frame_1_id = 10
frame_2_id = 22
frame_3_id = 34

ee_radius = 25
h = 50          # interasse braccia

R = 150         # raggio circonferenza
Z = -250        # piano circonferenza

t_instance = np.linspace(0, 2*np.pi, 50)
q = np.zeros(robot.nq)


while True:

    for t in t_instance:
        pos_des = np.array([R*np.cos(t), R*np.sin(t), Z])

        # chain 1 - red
        ee_1_offset = np.array([np.sin(0), np.cos(0), 0])*ee_radius + np.array([np.cos(0), np.sin(0), 0])*h/2
        pos_next_1 = pos_des + ee_1_offset

        q1 = ig.compute_inverse_geometry(q, pos_next_1, frame_1_id)
        q1[3:5] = q1[1:3]

        # chain 2 - green
        ee_2_offset = np.array([-np.sin((2/3)*np.pi), np.cos((2/3)*np.pi), 0])*ee_radius  + np.array([-np.cos((2/3)*np.pi), -np.sin((2/3)*np.pi), 0])*h/2
        pos_next_2 = pos_des + ee_2_offset
        
        q2 = ig.compute_inverse_geometry(q, pos_next_2, frame_2_id)
        q2[8:10] = q2[6:8]

        # chain 3 - blue
        ee_3_offset = np.array([-np.sin((4/3)*np.pi), np.cos((4/3)*np.pi), 0])*ee_radius + np.array([np.cos((4/3)*np.pi), np.sin((4/3)*np.pi), 0])*h/2
        pos_next_3 = pos_des + ee_3_offset
        
        q3 = ig.compute_inverse_geometry(q, pos_next_3, frame_3_id)
        q3[13:15] = q3[11:13]


        q = np.concatenate([q1[0:5], q2[5:10], q3[10:15]])
        viz.display(q)

        time.sleep(0.1)
