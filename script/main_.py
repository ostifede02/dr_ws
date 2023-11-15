import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.visualize import GepettoVisualizer
import numpy as np

from os.path import dirname, join, abspath
import sys
import time

from kinematics.IK_solver import IK_solver
from trajectory_planning.configuration_functions import trajectory_configuration_block
from trajectory_planning.trajectory_functions import bezier_curve, time_optimal_bang_bang_profile
from collision.collision_detection import is_collision
from robot_configuration.robot_configuration import robot_conf


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


# instance of inverse kinematic solver
ik_solver_1 = IK_solver(robot, frame_id=8)     # solver_1 solves the ik for the red left chain aka chain 1
ik_solver_2 = IK_solver(robot, frame_id=14)    # solver_2 solves the ik for the green right chain aka chain 2

# defining intial neutral joint position
q1 = pin.neutral(model)
q2 = pin.neutral(model)

s = 0
pos_current = np.array([-100, 0, 0])
state = "neutral"

while True:
    if state == "neutral":
        pos_des = np.array([0, 0, -100])
        state = "configure_trajectory"
        routine = "quick"

    if state == "configure_trajectory":
        trj_conf = trajectory_configuration_block(routine, pos_current, pos_des)
        x_current = 0
        t_current = 0
        
        state = "run"

    if state == "run":
        # calculate next via point
            # pos_des: s -> pos_des
        s_next = s + trj_conf["delta_s"]
        pos_des = bezier_curve(s_next, trj_conf["via_points"])

        # calculate inverse geometry
            # q1_next, q2_next: robot, q1, q2, pos_des -> q1_next, q2_next
        q1_next = ik_solver_1.solve_GN(q1, pos_des)
        q2_next = ik_solver_2.solve_GN(q2, pos_des)

        # check for collisions
            # true, false: robot, q1, q2, collision_pairs -> true, false
        # applying joint constraints
        q1_next[2] = q1_next[1]         # keeps the ee parallel to ground
        q2_next[5] = q2_next[4]         # keeps rod_3 parallel to rod_2
        q = np.concatenate((q1_next[0:3], q2_next[3:6]))

        # collision pairs: 0 -> rod1-rails and 1 -> rod2-rails
        if is_collision(robot, q, (0, 1)):
            break

        # calculate equivalent number of steps
            # steps_1, steps_2: delta_q1, delta_q2, steps_per_rev, d_pulley -> steps_1, steps_2
        # calculating the number of steps to do (further to send to the motorcontroller)
        c_pulley = robot_conf["pulley"]["n_teeth"] * robot_conf["pulley"]["module"]
        min_displacement = c_pulley / robot_conf["stepper"]["n_steps"]       # minimum carriage displacement

        # stepper 1
        delta_q_1 = q1_next[0] - q1
        stepper_1_steps = (delta_q_1 + q1_reminder) // min_displacement     # number of steps to do
        q1_reminder = (delta_q_1 + q1_reminder) % min_displacement          # the decimal part of steps

        # stepper 2
        delta_q_2 = q2_next[3] - q2
        stepper_2_steps = (delta_q_2 + q2_reminder) // min_displacement     # number of steps to do
        q2_reminder = (delta_q_2 + q2_reminder) % min_displacement          # the decimal part of steps


        # calculate delta t
            # delta_t: x_current, delta_x, t_current -> delta_t
        delta_x = np.linalg.norm(pos_des-pos_current)
        x_current += delta_x
        t_next = time_optimal_bang_bang_profile(x_current, 
                                                trj_conf["x_acc_flag"],
                                                trj_conf["x_total"],
                                                trj_conf["t_acc_flag"],
                                                trj_conf["t_total"],
                                                trj_conf["vel"],
                                                trj_conf["acc"],
                                                )
        
        delta_t = t_next - t_current
        t_current = t_next