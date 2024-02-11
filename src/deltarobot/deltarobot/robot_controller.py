import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import TrajectoryTask
from deltarobot_interfaces.msg import JointTrajectory
from deltarobot_interfaces.msg import JointTrajectoryArray

from micro_custom_messages.msg import JointTrajectoryReduced
from micro_custom_messages.msg import JointTrajectoryReducedArray

from deltarobot.inverse_geometry import InverseGeometry
from deltarobot.trajectory_generator import TrajectoryGenerator
from deltarobot import configuration as conf

import pinocchio as pin
import numpy as np

from os.path import join
import time



class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller_node')

        self.joint_trajectory_reduced_pub = self.create_publisher(
            JointTrajectoryReducedArray,
            'joint_trajectory_reduced',
            10)
        
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectoryArray,
            'joint_trajectory',
            10)
        
        self.trajectory_task_sub = self.create_subscription(
            TrajectoryTask,
            'trajectory_task',
            self.robot_controller_callback,
            10)
        self.trajectory_task_sub

        ## import urdf file path
        package_path = conf.configuration["paths"]["package_path"]
        # chain 1
        urdf_file_name_chain_1 = "deltarobot_c1.urdf"
        urdf_file_path_chain_1 = join(join(package_path, "urdf"), urdf_file_name_chain_1)
        # chain 2
        urdf_file_name_chain_2 = "deltarobot_c2.urdf"
        urdf_file_path_chain_2 = join(join(package_path, "urdf"), urdf_file_name_chain_2)
        # chain 3
        urdf_file_name_chain_3 = "deltarobot_c3.urdf"
        urdf_file_path_chain_3 = join(join(package_path, "urdf"), urdf_file_name_chain_3)

        ## Load the urdf models
        # chain 1
        model_chain_1 = pin.buildModelFromUrdf(urdf_file_path_chain_1)
        data_chain_1 = model_chain_1.createData()
        # chain 2
        model_chain_2 = pin.buildModelFromUrdf(urdf_file_path_chain_2)
        data_chain_2 = model_chain_2.createData()
        # chain 3
        model_chain_3 = pin.buildModelFromUrdf(urdf_file_path_chain_3)
        data_chain_3 = model_chain_3.createData()

        # total number of joints 
        self.robot_nq = model_chain_1.nq + model_chain_2.nq + model_chain_3.nq

        ## initialize InverseGeometry object
        self.ig_chain_1 = InverseGeometry(model_chain_1, data_chain_1, 10)
        self.ig_chain_2 = InverseGeometry(model_chain_2, data_chain_2, 10)
        self.ig_chain_3 = InverseGeometry(model_chain_3, data_chain_3, 10)

        ## initialize trajectory generator
        self.trajectory_generator = TrajectoryGenerator()

        # define initial conditions
        self.pos_current = conf.configuration["trajectory"]["pos_home"]     ## after home calibration
        
        self.ee_radius = conf.configuration["physical"]["ee_radius"]

        self.q1 = np.zeros(3)
        self.q2 = np.zeros(3)
        self.q3 = np.zeros(3)

        return
    

    def robot_controller_callback(self, trajectory_task_msg):
        # unpack message
        pos_start = np.array([trajectory_task_msg.pos_start.x,
                            trajectory_task_msg.pos_start.y,
                            trajectory_task_msg.pos_start.z])
        
        pos_end = np.array([trajectory_task_msg.pos_end.x,
                            trajectory_task_msg.pos_end.y,
                            trajectory_task_msg.pos_end.z])
        
        delta_t_input = trajectory_task_msg.trajectory_delta_time
        task_type = trajectory_task_msg.task_type
        

        ## generate trajectory task space
        if task_type == conf.PLACE_TRAJECTORY:
            task_space_trajectory_vector = self.trajectory_generator.generate_trajectory__task_space__bezier(
                pos_start, pos_end, delta_t_input, task_type)
        
        elif task_type == conf.PICK_TRAJECTORY:
            task_space_trajectory_vector = self.trajectory_generator.generate_trajectory__task_space__bezier(
                pos_start, pos_end, delta_t_input, task_type)
        
        elif task_type == conf.P2P_DIRECT_TRAJECTORY:
            task_space_trajectory_vector = self.trajectory_generator.generate_trajectory__task_space__point_to_point__direct(
                pos_start, pos_end, delta_t_input)
        
        elif task_type == conf.P2P_CONTINUOUS_TRAJECTORY:
            task_space_trajectory_vector = self.trajectory_generator.generate_trajectory__task_space__point_to_point__continuous(
                pos_start, pos_end, delta_t_input)             

        # check for valid trajectory
        # if task_space_trajectory_vector == conf.ERROR__INVALID_TRAJECTORY:
        #     # publish nak
        #     pass


        ## inverse geometry
        joint_trajectory_vector = np.empty((len(task_space_trajectory_vector), self.robot_nq+1))
            
        # end effector offsets
        ee_1_offset = np.array([np.sin(0), np.cos(0), 0])*self.ee_radius
        ee_2_offset = np.array([-np.sin((2/3)*np.pi), np.cos((2/3)*np.pi), 0])*self.ee_radius
        ee_3_offset = np.array([-np.sin((4/3)*np.pi), np.cos((4/3)*np.pi), 0])*self.ee_radius

        for set_point_index, set_point in enumerate(task_space_trajectory_vector):
            pos_des = set_point[0:3]
            time = set_point[3]
            
            ## compute inverse geometry
            # chain 1
            pos_des_1 = pos_des + ee_1_offset
            self.q1 = self.ig_chain_1.compute_inverse_geometry(np.copy(self.q1), pos_des_1)

            # chain 2
            pos_des_2 = pos_des + ee_2_offset
            self.q2 = self.ig_chain_2.compute_inverse_geometry(np.copy(self.q2), pos_des_2)

            # chain 3
            pos_des_3 = pos_des + ee_3_offset
            self.q3 = self.ig_chain_3.compute_inverse_geometry(np.copy(self.q3), pos_des_3)

            # check for collisions
            # TO DO...

            joint_trajectory_vector[set_point_index, 0:self.robot_nq] = np.concatenate([
                self.q1, self.q2, self.q3
            ])
            joint_trajectory_vector[set_point_index, self.robot_nq] = time

        # generate joint velocity profiles
        if trajectory_task_msg.is_joint_velocity_profile:
            q_start = np.array([
                joint_trajectory_vector[0, 0],
                joint_trajectory_vector[0, 3],
                joint_trajectory_vector[0, 6]])
            q_end = np.array([
                joint_trajectory_vector[1, 0],
                joint_trajectory_vector[1, 3],
                joint_trajectory_vector[1, 6]])
            
            joint_trajectory_vector_joint_space = self.trajectory_generator.generate_trajectory_joint_space(q_start, q_end, time)

            # publish joint trajectory
            self.publish_joint_trajectory_reduced(joint_trajectory_vector_joint_space)
        
        else:
            # publish joint trajectory
            self.publish_joint_trajectory(joint_trajectory_vector)
            self.publish_joint_trajectory_reduced(joint_trajectory_vector)

        return

    def publish_joint_trajectory(self, joint_trajectory_vector):
        joint_trajectory_array_msg = JointTrajectoryArray()
        joint_trajectory_array_msg.array_size = len(joint_trajectory_vector)

        for joint_trajectory in joint_trajectory_vector:
            joint_trajectory_msg = JointTrajectory()

            joint_trajectory_msg.q1_1 = float(joint_trajectory[0])
            joint_trajectory_msg.q1_2 = float(joint_trajectory[1])
            joint_trajectory_msg.q1_3 = float(joint_trajectory[2])

            joint_trajectory_msg.q2_1 = float(joint_trajectory[3])
            joint_trajectory_msg.q2_2 = float(joint_trajectory[4])
            joint_trajectory_msg.q2_3 = float(joint_trajectory[5])

            joint_trajectory_msg.q3_1 = float(joint_trajectory[6])
            joint_trajectory_msg.q3_2 = float(joint_trajectory[7])
            joint_trajectory_msg.q3_3 = float(joint_trajectory[8])

            joint_trajectory_msg.time = float(joint_trajectory[9])

            joint_trajectory_array_msg.set_points.append(joint_trajectory_msg)

        self.joint_trajectory_pub.publish(joint_trajectory_array_msg)
        return
    

    def publish_joint_trajectory_reduced(self, joint_trajectory_vector):
        joint_trajectory_array_msg = JointTrajectoryReducedArray()
        joint_trajectory_array_msg.array_size = len(joint_trajectory_vector)
        nq = len(joint_trajectory_vector[0])-1

        for joint_trajectory in joint_trajectory_vector:
            joint_trajectory_msg = JointTrajectoryReduced()

            joint_trajectory_msg.q1 = float(joint_trajectory[0])
            joint_trajectory_msg.q2 = float(joint_trajectory[int(nq/3)])
            joint_trajectory_msg.q3 = float(joint_trajectory[int(nq*2/3)])
            joint_trajectory_msg.time = float(joint_trajectory[int(nq)])

            joint_trajectory_array_msg.set_points.append(joint_trajectory_msg)

        self.joint_trajectory_reduced_pub.publish(joint_trajectory_array_msg)
        return



def main(args=None):
    rclpy.init(args=args)

    rc_node = RobotController()

    rclpy.spin(rc_node)

    rc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
        main()














# def robot_controller_callback(self, trajectory_task_msg):
#         start = time.time()

#         # unpack message
#         pos_start = np.copy(self.pos_current)
#         pos_end = np.array([trajectory_task_msg.pos_end.x,
#                             trajectory_task_msg.pos_end.y,
#                             trajectory_task_msg.pos_end.z])
#         task_time = trajectory_task_msg.task_time
#         task_type = trajectory_task_msg.task_type.data
        
#         # distinguish between joint space and task space
#         if task_type == conf.JOINT_SPACE_DIRECT_TRAJECTORY_ROUTINE:
#             ## implement the joint space trajectory planning and control
#             self.robot_controller_joint_space_trajectory(pos_end, task_time)
#         else:
#             ## task space trajectory planning and control
#             self.robot_controller_task_space_trajectory(pos_start, pos_end, task_time, task_type)

#         # last msg to show graph
#         # self.publish_joint_position_telemetry(0,0,0,-1)

#         stop = time.time()
#         self.get_logger().info(f"computing time: {(stop-start)*1e3} [ms]")
#         return


#     def robot_controller_task_space_trajectory(self, pos_start, pos_end, task_time, task_type):
#         if task_type == conf.TASK_SPACE_SIMPLE_P2P_TRAJECTORY_ROUTINE:
#             set_points_vector = self.trajectory_generator.generate_trajectory_task_space_simple_point_to_point(
#                 pos_start, pos_end, task_time)
#         else:
#             # returns an array of x-y-z setpoints and time
#             set_points_vector = self.trajectory_generator.generate_trajectory_task_space(
#                 pos_start, pos_end, task_time, task_type)
        
#         t_current = 0

#         joint_trajectory_vector = np.empty((len(set_points_vector), 4))
#         for i, set_point in enumerate(set_points_vector):
#             pos_des = set_point[0:3]
            
#             # compute inverse geometry
#             # chain 1
#             ee_1_offset = np.array([np.sin(0), np.cos(0), 0])*self.ee_radius
#             pos_next_1 = pos_des + ee_1_offset
#             q1_next = self.ig_chain_1.compute_inverse_geometry(np.copy(self.q1_current), pos_next_1)

#             # chain 2
#             ee_2_offset = np.array([-np.sin((2/3)*np.pi), np.cos((2/3)*np.pi), 0])*self.ee_radius
#             pos_next_2 = pos_des + ee_2_offset
#             q2_next = self.ig_chain_2.compute_inverse_geometry(np.copy(self.q2_current), pos_next_2)

#             # chain 3
#             ee_3_offset = np.array([-np.sin((4/3)*np.pi), np.cos((4/3)*np.pi), 0])*self.ee_radius
#             pos_next_3 = pos_des + ee_3_offset
#             q3_next = self.ig_chain_3.compute_inverse_geometry(np.copy(self.q3_current), pos_next_3)

#             # check for collisions
#             # TO DO...

#             # get delta_t
#             t_next = set_point[3]
#             delta_t = t_next - t_current

#             # get delta_q
#             delta_q1 = q1_next[0] - self.q1_current[0]
#             delta_q2 = q2_next[0] - self.q2_current[0]
#             delta_q3 = q3_next[0] - self.q3_current[0]

#             # update current position
#             self.pos_current = pos_des
#             self.q1_current = q1_next
#             self.q2_current = q2_next
#             self.q3_current = q3_next
#             t_current = t_next

#             # create array of delta X
#             joint_trajectory_vector[i, :] = np.array([delta_q1, delta_q2, delta_q3, delta_t])
            
#             ## publish to geppetto viewer
#             self.publish_joint_position_viewer(q1_next, q2_next, q3_next, delta_t)         

#         ## publish to micro
#         # self.publish_joint_position_micro(joint_trajectory_vector)

#         # create graphs
#         # self.publish_joint_position_telemetry(q1_next[0], q2_next[0], q3_next[0], t_current)

#         return
    

#     def robot_controller_joint_space_trajectory(self, pos_end, task_time):
#         # get current joint configuration in pos_start
#         q1_start = self.q1_current
#         q2_start = self.q2_current
#         q3_start = self.q3_current
        
#         # compute joint configuration in pos_end
#         # chain 1
#         ee_1_offset = np.array([np.sin(0), np.cos(0), 0])*self.ee_radius
#         pos_next_1 = pos_end + ee_1_offset
#         q1_end = self.ig_chain_1.compute_inverse_geometry(np.copy(self.q1_current), pos_next_1)

#         # chain 2
#         ee_2_offset = np.array([-np.sin((2/3)*np.pi), np.cos((2/3)*np.pi), 0])*self.ee_radius
#         pos_next_2 = pos_end + ee_2_offset
#         q2_end = self.ig_chain_2.compute_inverse_geometry(np.copy(self.q2_current), pos_next_2)

#         # chain 3
#         ee_3_offset = np.array([-np.sin((4/3)*np.pi), np.cos((4/3)*np.pi), 0])*self.ee_radius
#         pos_next_3 = pos_end + ee_3_offset
#         q3_end = self.ig_chain_3.compute_inverse_geometry(np.copy(self.q3_current), pos_next_3)

#         # get joint trajectory as delta_X
#         joint_trajectory_vector = self.trajectory_generator.generate_trajectory_joint_space(
#                                         np.array([q1_start, q2_start, q3_start]),
#                                         np.array([q1_end, q2_end, q3_end]),
#                                         task_time
#                                     )
        
#         # publish trajectory as delta_X
#         self.publish_joint_position_micro(joint_trajectory_vector)

#         # update current position
#         self.pos_current = pos_end
#         self.q1_current = q1_end
#         self.q2_current = q2_end
#         self.q3_current = q3_end

#         return


#     def publish_joint_position_telemetry(self, q1, q2, q3, t_current):
#         micro_msg = JointPositionTelemetry()
#         micro_msg.q = [float(q1), float(q2), float(q3)]
#         micro_msg.t = float(t_current)
#         self.joint_position_telemetry_pub.publish(micro_msg)
#         return


#     def publish_joint_position_viewer(self, q1, q2, q3, delta_t):
#         viz_msg = JointPositionViz()
#         viz_msg.q = [   q1[0], q1[1], q1[2],
#                         q2[0], q2[1], q2[2],
#                         q3[0], q3[1], q3[2]]
#         viz_msg.delta_t = float(delta_t)
#         self.joint_position_viz_pub.publish(viz_msg)
#         return
    

#     def publish_joint_position_micro(self, msg_array):
#         msg_micro_array = SetPointArray()
#         for set_point_index, msg in enumerate(msg_array):
#             msg_micro = SetPoint()
#             msg_micro.delta_q1 = round(msg[0], 2)          # [ millimeters ]
#             msg_micro.delta_q2 = round(msg[1], 2)          # [ millimeters ]
#             msg_micro.delta_q3 = round(msg[2], 2)          # [ millimeters ]
#             msg_micro.delta_t = round(msg[3]*1e6, 1)       # [ microseconds ]
            
#             msg_micro_array.set_points[set_point_index] = msg_micro

#         # end of array
#         msg_micro = SetPoint()
#         msg_micro.delta_t = float(-1)      # [ microseconds ]
#         msg_micro_array.set_points[set_point_index+1] = msg_micro

#         self.joint_position_micro_pub.publish(msg_micro_array)

#         return
