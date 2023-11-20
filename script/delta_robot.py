'''
NOTE: the coordinate system used in the URDF file is as follows
      since the robot has only 2 DOF, the y-axis is not to be taken into account   

          ^ z
          |
          |   ^ y
          |  /
          | /
          |/___________> x
          O
'''


from pinocchio import RobotWrapper
import pinocchio as pin
import numpy as np

from os.path import dirname, join, abspath

from kinematics.inverse_geometry import InverseGeometry
from trajectory.trajectory import Trajectory
import configuration as conf


class DeltaRobot:

    def __init__(self, viewer = False):
        self.init_robot()
        
        if viewer:
            self.init_viewer()

        self.trj = Trajectory()
        self.ig = InverseGeometry(self.robot)
        self.init_configuration_variables()
        
        return


    # ******   MODEL   ******
    def init_robot(self):
        model_dir = join(dirname(str(abspath(__file__))),"../delta_robot_description")
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
    

    def init_configuration_variables(self):
        # trajectory
        self.pos_start = conf.configuration["trajectory"]["pos_home"]
        self.end_effector_x_offset = conf.configuration["trajectory"]["end_effector_x_offset"]
        
        # kinematics
        self.frame_id_1 = conf.configuration["inverse_geometry"]["frame_ids"]["chain_1"]
        self.frame_id_2 = conf.configuration["inverse_geometry"]["frame_ids"]["chain_2"]
        
        q0 = pin.neutral(self.robot.model)
        self.q1_current = self.ig.compute_inverse_geometry(q0, self.pos_start, self.frame_id_1)
        self.q2_current = self.ig.compute_inverse_geometry(q0, self.pos_start, self.frame_id_2)
        self.q1_reminder = 0
        self.q2_reminder = 0

        # collison
        self.collision_pair = conf.configuration["inverse_geometry"]["collision_pair"]

        # physical properties
        circumference_pulley = conf.configuration["physical"]["pulley"]["n_teeth"] * conf.configuration["physical"]["pulley"]["module"]
        self.min_belt_displacement = circumference_pulley / conf.configuration["physical"]["stepper"]["n_steps"]       # minimum carriage displacement

        return
    
    # ******   VIEWER   ******
    def init_viewer(self):
        self.robot.initViewer()
        return
    
    def display(self, q):
        self.robot.viz.display(q)
        return
    

    # ******   TRAJECTORY   ******
    def set_trajectory_routine(self, trajectory_routine_type, pos_end, t_total_input=-1):
        error_check = self.trj.set_trajectory_routine(trajectory_routine_type, self.pos_start, pos_end, t_total_input)
        if error_check is None:
            return None
        
        self.s = 0
        self.x_next = 0
        self.t_current = 0
        self.pos_current = self.pos_start

        return
    

    def get_pos_next(self, simulation=False):
        if self.s == 1:
            self.pos_start = self.pos_next
            return None
        
        # set s -> parameter [0, 1], that controls the bezier curve
        if simulation:
            delta_s = 0.01                  # more fluid simultion display
        else:       
            delta_s = self.trj.delta_s

        self.s += delta_s      # more efficient

        if self.s > 1-delta_s:  # to not have in the next cycle too close points
            self.s = 1
        
        self.pos_next = self.trj.get_pos_bezier_poly(self.s)
        self.x_next += np.linalg.norm(self.pos_next - self.pos_current)
        self.pos_current = self.pos_next

        # self.pos_next[0] -= self.end_effector_x_offset                   # since the frame_id is the joint
        return self.pos_next
    

    def get_q_next_continuos(self, pos_des):
        q1_current = self.q1_current        # PROBLEM!: after function also self.q1_current was changed
        q2_current = self.q2_current

        self.q1_next = self.ig.compute_inverse_geometry(q1_current, pos_des, self.frame_id_1)
        self.q2_next = self.ig.compute_inverse_geometry(q2_current, pos_des, self.frame_id_2)

        q_next = np.array([self.q1_next[0], self.q1_next[1], self.q1_next[1], 
                           self.q2_next[3], self.q2_next[4], self.q2_next[4]])

        self.delta_q1 = self.q1_next[0] - q1_current[0]
        self.delta_q2 = self.q2_next[3] - q2_current[3]

        self.q1_current = self.q1_next
        self.q2_current = self.q2_next

        return q_next


    def get_number_of_steps(self):
        
        # stepper 1 
        stepper_1_steps = (self.delta_q1 + self.q1_reminder) // self.min_belt_displacement     # number of steps to do
        self.q1_reminder = round((self.delta_q1 + self.q1_reminder) % self.min_belt_displacement, 4)     # the decimal part of steps

        # stepper 2
        stepper_2_steps = (self.delta_q2 + self.q2_reminder) // self.min_belt_displacement     # number of steps to do
        self.q2_reminder = round((self.delta_q2 + self.q2_reminder) % self.min_belt_displacement, 4)     # the decimal part of steps

        return stepper_1_steps, stepper_2_steps


    def check_collisions(self, q):
        ## checking for collisions
        # Compute for each pair of collision
        pin.updateGeometryPlacements(self.robot.model, self.robot.data, self.robot.collision_model, self.robot.collision_data, q)
        pin.computeCollision(self.robot.collision_model, self.robot.collision_data, self.collision_pair[0]) # chain 2
        pin.computeCollision(self.robot.collision_model, self.robot.collision_data, self.collision_pair[1]) # chain 1

        if(self.robot.collision_data.collisionResults[0].isCollision()):
            print(f"Collision detected: {self.robot.collision_model.collisionPairs[0]}")
            return True
        elif(self.robot.collision_data.collisionResults[1].isCollision()):
            print(f"Collision detected: {self.robot.collision_model.collisionPairs[1]}")
            return True
        
        return False
    

    def get_delta_t(self):
        t_next = self.trj.get_t_next(self.x_next)
        delta_t = t_next - self.t_current
        self.t_current = t_next
        return delta_t