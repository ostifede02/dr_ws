from pinocchio import RobotWrapper
import pinocchio as pin
import numpy as np

from os.path import dirname, join, abspath

from kinematics.inverse_geometry import InverseGeometry
from trajectory.trajectory import Trajectory
import configuration as conf


class DeltaRobot:

    def __init__(self, viewer = False):
        self.initRobot()
        
        if viewer:
            self.initViewer()

        self.trj = Trajectory()
        self.pos_start = conf.configuration["trajectory"]["pos_home"]

        self.frame_id_1 = conf.configuration["inverse_geometry"]["frame_ids"]["chain_1"]
        self.frame_id_2 = conf.configuration["inverse_geometry"]["frame_ids"]["chain_2"]
        self.ig = InverseGeometry(self.robot)
        
        q0 = pin.neutral(self.robot.model)
        self.q1_current = self.ig.compute_inverse_geometry(q0, self.pos_start, self.frame_id_1)
        self.q2_current = self.ig.compute_inverse_geometry(q0, self.pos_start, self.frame_id_2)

        circumference_pulley = conf.configuration["physical"]["pulley"]["n_teeth"] * conf.configuration["physical"]["pulley"]["module"]
        self.min_belt_displacement = circumference_pulley / conf.configuration["physical"]["stepper"]["n_steps"]       # minimum carriage displacement

        return
   
    # ******   MODEL   ******
    def initRobot(self):
        model_dir = join(dirname(str(abspath(__file__))),"delta_robot_description")
        mesh_dir = join(model_dir,"meshes")

        # Load the URDF model
        urdf_filename = "delta_robot.urdf"
        urdf_dir = join(model_dir,"urdf")
        urdf_file_trajectory = join(urdf_dir, urdf_filename)

        # Initialize the model
        model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_file_trajectory, mesh_dir, geometry_types=[pin.GeometryType.COLLISION,pin.GeometryType.VISUAL])
        self.robot = RobotWrapper(model, collision_model, visual_model)
        self.robot.collision_model.addAllCollisionPairs()
        self.robot.collision_data = pin.GeometryData(self.robot.collision_model)
        return
    
    
    # ******   VIEWER   ******
    def initViewer(self):
        self.robot.initViewer()
        return
    
    def display(self, q):
        self.robot.viz.display(q)
        return
    

    # ******   TRAJECTORY   ******
    def set_trajectory_routine(self, trajectory_routine_type, pos_end, t_total_input=-1):
        self.trj.set_trajectory_routine(trajectory_routine_type, self.pos_start, pos_end, t_total_input)
        
        self.s = 0
        self.x_current = 0
        self.t_current = 0
        self.pos_current = self.pos_start

        return
    

    def get_pos_next(self):
        if self.s == 1:
            self.pos_start = self.pos_next
            return None
        
        # set s -> parameter [0, 1], that controls the bezier curve
        self.s += self.trj.delta_s
        if self.s > 1-self.trj.delta_s:  # to not have in the next cycle too close points
            self.s = 1
        
        self.pos_next = self.trj.get_pos_bezier_poly(self.s)
        self.x_current += np.linalg.norm(self.pos_next - self.pos_current)

        return self.pos_next
    

    def get_q_next_continuos(self, pos_des):
        self.q1_next = self.ig.compute_inverse_geometry(self.q1_current, pos_des, self.frame_id_1)
        self.q2_next = self.ig.compute_inverse_geometry(self.q2_current, pos_des, self.frame_id_2)
        q_next = np.array([self.q1_next[0], self.q1_next[1], self.q1_next[1], 
                      self.q2_next[3], self.q2_next[4], self.q2_next[4]])

        self.q1_current = self.q1_next
        self.q2_current = self.q2_next

        return q_next
    

    # def get_delta_q_continuous(self, pos_des):
    #     q_next = self.get_q_next_continuos(pos_des)


    # # NOTE! should be delta q
    # def get_q_discrete(self, q):
        
    #     # stepper 1
    #     delta_q_1 = q1_next[0] - q1
    #     stepper_1_steps = (delta_q_1 + q1_reminder) // self.min_belt_displacement     # number of steps to do
    #     q1_reminder = (delta_q_1 + q1_reminder) % self.min_belt_displacement          # the decimal part of steps

    #     # stepper 2
    #     delta_q_2 = q2_next[3] - q2
    #     stepper_2_steps = (delta_q_2 + q2_reminder) // self.min_belt_displacement     # number of steps to do
    #     q2_reminder = (delta_q_2 + q2_reminder) % self.min_belt_displacement          # the decimal part of steps



