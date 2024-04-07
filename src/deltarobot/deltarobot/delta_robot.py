import numpy as np

from os.path import join

import pinocchio as pin

from deltarobot.inverse_geometry import InverseGeometry
from deltarobot.trajectory_generator import TrajectoryGenerator

from deltarobot import configuration as conf


class DeltaRobot():

    def __init__(self):
        ## define robot parameters
        self.pos_current            = conf.pos_home
        self.pos_current_volatile   = conf.pos_home

        end_effector_radius = conf.end_effector_radius
        self.ee_1_offset = np.array([np.sin(0), np.cos(0), 0])*end_effector_radius
        self.ee_2_offset = np.array([-np.sin((2/3)*np.pi), np.cos((2/3)*np.pi), 0])*end_effector_radius
        self.ee_3_offset = np.array([-np.sin((4/3)*np.pi), np.cos((4/3)*np.pi), 0])*end_effector_radius

        ## define joint limits
        self.q_min  = conf.q_min
        self.q_max  = conf.q_max

        ## init joints
        self.q1 = np.zeros(3)
        self.q2 = np.zeros(3)
        self.q3 = np.zeros(3)
        
        ## import urdf model
        model_1, data_1 = self.init_robot_model(conf.urdf_filename_1)   # chain 1
        model_2, data_2 = self.init_robot_model(conf.urdf_filename_2)   # chain 2
        model_3, data_3 = self.init_robot_model(conf.urdf_filename_3)   # chain 3

        ## init InverseGeometry
        self.ig_1 = InverseGeometry(model_1, data_1, conf.frame_id_1)    # chain 1
        self.ig_2 = InverseGeometry(model_2, data_2, conf.frame_id_2)    # chain 2
        self.ig_3 = InverseGeometry(model_3, data_3, conf.frame_id_3)    # chain 3
        
        ## init TrajectoryGenerator
        self.tg = TrajectoryGenerator(  
            max_vel                 = conf.max_vel,                 # [ m/s ]
            max_acc                 = conf.max_acc,                 # [ m/s2 ]
            via_points_distance     = conf.via_points_distance,     # [ mm ]
            via_points_threshold    = conf.via_points_threshold     # [ mm ]
        )

        return
    

    ###################################################################################
    #                                                                                 #
    #                              JOINT TRAJECTORIES                                 #
    #                                                                                 #
    ###################################################################################

    def generate_joint_trajectory__ptp_task_space(self, pos_end, time_total):
        ## get task space trajectory
        via_points_trajectory = self.tg.ptp_task_space(self.pos_current, pos_end, time_total)

        ## compute inverse geometry for each via point
        joint_trajectory = np.empty((len(via_points_trajectory), 4))
        for index, via_point in enumerate(via_points_trajectory):
            # chain 1
            pos_des_1 = via_point[0:3] + self.ee_1_offset
            self.q1 = self.ig_1.compute_inverse_geometry(self.q1, pos_des_1)
            # chain 2
            pos_des_2 = via_point[0:3] + self.ee_2_offset
            self.q2 = self.ig_2.compute_inverse_geometry(self.q2, pos_des_2)
            # chain 3
            pos_des_3 = via_point[0:3] + self.ee_3_offset
            self.q3 = self.ig_3.compute_inverse_geometry(self.q3, pos_des_3)
            
            ## check for joint limits
            if self.is_joint_collision(self.q1[0], self.q2[0], self.q3[0]):
                return None
            
            ## pack joint positions
            joint_trajectory[index, 0] = round(self.q1[0], 3)       # q1    [ mm ]       
            joint_trajectory[index, 1] = round(self.q2[0], 3)       # q2    [ mm ]
            joint_trajectory[index, 2] = round(self.q3[0], 3)       # q3    [ mm ]
            joint_trajectory[index, 3] = round(via_point[3], 6)     # time  [ sec ]

        ## update pos current volatile --> definitive pos current will be updated after ack
        self.pos_current_volatile = pos_end

        return joint_trajectory


    def generate_joint_trajectory__ptp_joint_space(self, pos_end, time_total):
        return

    ###################################################################################
    #                                                                                 #
    #                                    UTILS                                        #
    #                                                                                 #
    ###################################################################################

    def is_joint_collision(self, q1, q2, q3):
        
        if q1 < self.q_min or q1 > self.q_max:
            return True
        elif q2 < self.q_min or q2 > self.q_max:
            return True
        elif q3 < self.q_min or q3 > self.q_max:
            return True
        
        return False


    def init_robot_model(self, urdf_filename):
        ## import urdf file path
        package_path = conf.package_path
        urdf_file_name = urdf_filename
        urdf_file_path = join(join(package_path, "urdf"), urdf_file_name)

        ## Load the urdf model
        model = pin.buildModelFromUrdf(urdf_file_path)
        data = model.createData()
        
        return model, data
    

    def update_pos_current(self):
        self.pos_current = self.pos_current_volatile
        return

















###################################################################################################
#                                                                                                 #
#                                      How to use the class                                       #
#                                                                                                 #
###################################################################################################

import time


def main():
    robot = DeltaRobot()
    print("***********  HOME  ***********")
    print(f"pos_current: {robot.pos_current}\n\n")


    #
    #   travel nr.1
    #
    print("***********  travel nr.1  ***********")

    pos_end = np.array([20, -60, -320])
    time_total = -1

    t_start = time.time()
    joint_trajectory = robot.generate_joint_trajectory__ptp_task_space(pos_end, time_total)
    t_end = time.time()

    if joint_trajectory is None:
        print("[ERROR!] joints limit exceeded!")
    else:
        robot.update_pos_current()
        print(joint_trajectory)
        print(f"\n\nComputed time: {round((t_end - t_start)*1e3, 3)} [ ms ]")

    print(f"pos_end: {pos_end}")
    print(f"pos_current: {robot.pos_current}\n\n")



    #
    #   travel nr.2
    #
    print("***********  travel nr.2  ***********")

    pos_end = np.array([60, 100, -180])
    time_total = 4.62

    t_start = time.time()
    joint_trajectory = robot.generate_joint_trajectory__ptp_task_space(pos_end, time_total)
    t_end = time.time()

    if joint_trajectory is None:
        print("[ERROR!] joints limit exceeded!")
    else:
        robot.update_pos_current()
        print(joint_trajectory)
        print(f"\nComputed time: {round((t_end - t_start)*1e3, 3)} [ ms ]")

    print(f"pos_end: {pos_end}")
    print(f"pos_current: {robot.pos_current}\n\n")

    return


if __name__ == "__main__":
    main()