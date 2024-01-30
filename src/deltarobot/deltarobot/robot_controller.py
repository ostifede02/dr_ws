import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import TrajectoryTask
from deltarobot_interfaces.msg import JointPositionViz
from deltarobot_interfaces.msg import JointPositionTelemetry

from micro_custom_messages.msg import SetPoint
from micro_custom_messages.msg import SetPointArray

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

        self.joint_position_viz_pub = self.create_publisher(
            JointPositionViz,
            'joint_position_viz',
            100)
        
        self.joint_position_telemetry_pub = self.create_publisher(
            JointPositionTelemetry,
            'joint_position_telemetry',
            100)
        
        self.joint_position_micro_pub = self.create_publisher(
            SetPointArray,
            'joint_position_micro',
            100)
        
        self.trajectory_task_sub = self.create_subscription(
            TrajectoryTask,
            'trajectory_task',
            self.robot_controller_callback,
            10)
        self.trajectory_task_sub

        # import urdf file path
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

        # Load the urdf models
        model_chain_1 = pin.buildModelFromUrdf(urdf_file_path_chain_1)
        data_chain_1 = model_chain_1.createData()
        model_chain_2 = pin.buildModelFromUrdf(urdf_file_path_chain_2)
        data_chain_2 = model_chain_2.createData()
        model_chain_3 = pin.buildModelFromUrdf(urdf_file_path_chain_3)
        data_chain_3 = model_chain_3.createData()

        # initialize InverseGeometry object
        self.ig_chain_1 = InverseGeometry(model_chain_1, data_chain_1, 10)
        self.ig_chain_2 = InverseGeometry(model_chain_2, data_chain_2, 10)
        self.ig_chain_3 = InverseGeometry(model_chain_3, data_chain_3, 10)

        # initialize trajectory
        self.trajectory_generator = TrajectoryGenerator()

        # define initial conditions
        self.pos_current = conf.configuration["trajectory"]["pos_home"]     ## after home calibration
        
        self.ee_radius = conf.configuration["physical"]["ee_radius"]

        # init chain 1
        q0_1 = pin.neutral(model_chain_1)
        ee_1_offset = np.array([np.sin(0), np.cos(0), 0])*self.ee_radius
        pos_next_1 = self.pos_current + ee_1_offset
        self.q1_current = self.ig_chain_1.compute_inverse_geometry(q0_1, pos_next_1)
        
        # init chain 2
        q0_2 = pin.neutral(model_chain_2)
        ee_2_offset = np.array([-np.sin((2/3)*np.pi), np.cos((2/3)*np.pi), 0])*self.ee_radius
        pos_next_2 = self.pos_current + ee_2_offset
        self.q2_current = self.ig_chain_2.compute_inverse_geometry(q0_2, pos_next_2)

        # init chain 3
        q0_3 = pin.neutral(model_chain_3)
        ee_3_offset = np.array([-np.sin((4/3)*np.pi), np.cos((4/3)*np.pi), 0])*self.ee_radius
        pos_next_3 = self.pos_current + ee_3_offset
        self.q3_current = self.ig_chain_3.compute_inverse_geometry(q0_3, pos_next_3)

        return


    def robot_controller_callback(self, trajectory_task_msg):
        start = time.time()

        # unpack message
        pos_start = np.copy(self.pos_current)
        pos_end = np.array([trajectory_task_msg.pos_end.x,
                            trajectory_task_msg.pos_end.y,
                            trajectory_task_msg.pos_end.z])
        task_time = trajectory_task_msg.task_time
        task_type = trajectory_task_msg.task_type.data
        
        # only the POINT to POINT joint space trajectory is implemented
        if task_type == conf.JOINT_SPACE_DIRECT_TRAJECTORY_ROUTINE:
            ## implement the joint space trajectory planning and control
            pass
        else:
            ## task space trajectory planning and control
            self.robot_controller_task_space_trajectory(pos_start, pos_end, task_time, task_type)

        # last msg to show graph
        # self.publish_joint_position_telemetry(0,0,0,-1)

        stop = time.time()
        self.get_logger().info(f"computing time: {(stop-start)*1e3} [ms]")
        return


    def robot_controller_task_space_trajectory(self, pos_start, pos_end, task_time, task_type):
        # returns an array of x-y-z setpoints and time
        set_points_vector = self.trajectory_generator.generate_trajectory_task_space(pos_start, pos_end, task_time, task_type)
        t_current = 0

        joint_trajectory_vector = np.empty((len(set_points_vector), 4))
        for i, set_point in enumerate(set_points_vector):
            pos_des = set_point[0:3]
            
            # compute inverse geometry
            # chain 1
            ee_1_offset = np.array([np.sin(0), np.cos(0), 0])*self.ee_radius
            pos_next_1 = pos_des + ee_1_offset
            q1_next = self.ig_chain_1.compute_inverse_geometry(np.copy(self.q1_current), pos_next_1)

            # chain 2
            ee_2_offset = np.array([-np.sin((2/3)*np.pi), np.cos((2/3)*np.pi), 0])*self.ee_radius
            pos_next_2 = pos_des + ee_2_offset
            q2_next = self.ig_chain_2.compute_inverse_geometry(np.copy(self.q2_current), pos_next_2)

            # chain 3
            ee_3_offset = np.array([-np.sin((4/3)*np.pi), np.cos((4/3)*np.pi), 0])*self.ee_radius
            pos_next_3 = pos_des + ee_3_offset
            q3_next = self.ig_chain_3.compute_inverse_geometry(np.copy(self.q3_current), pos_next_3)

            # check for collisions
            # TO DO...

            # get delta_t
            t_next = set_point[3]
            delta_t = t_next - t_current

            # get delta_q
            delta_q1 = q1_next[0] - self.q1_current[0]
            delta_q2 = q2_next[0] - self.q2_current[0]
            delta_q3 = q3_next[0] - self.q3_current[0]

            # update current position
            self.pos_current = pos_des
            self.q1_current = q1_next
            self.q2_current = q2_next
            self.q3_current = q3_next
            t_current = t_next

            joint_trajectory_vector[i, :] = np.array([delta_q1, delta_q2, delta_q3, delta_t])

        ## publish to micro
        self.publish_joint_position_micro(joint_trajectory_vector)

        # create graphs
        # self.publish_joint_position_telemetry(q1_next[0], q2_next[0], q3_next[0], t_current)

        ## publish to viz
        # self.publish_joint_position_viewer(q1_next, q2_next, q3_next, delta_t)         

        return
        

    def publish_joint_position_telemetry(self, q1, q2, q3, t_current):
        micro_msg = JointPositionTelemetry()
        micro_msg.q = [float(q1), float(q2), float(q3)]
        micro_msg.t = float(t_current)
        self.joint_position_telemetry_pub.publish(micro_msg)
        return
    

    def publish_joint_position_viewer(self, q1, q2, q3, delta_t):
        viz_msg = JointPositionViz()
        viz_msg.q = [   q1[0], q1[1], q1[2],
                        q2[0], q2[1], q2[2],
                        q3[0], q3[1], q3[2]]
        viz_msg.delta_t = float(delta_t)
        self.joint_position_viz_pub.publish(viz_msg)
        return
    
    def publish_joint_position_micro(self, msg_array):
        msg_micro_array = SetPointArray()
        for set_point_index, msg in enumerate(msg_array):
            msg_micro = SetPoint()
            msg_micro.delta_q1 = round(msg[0], 2)          # [ millimeters ]
            msg_micro.delta_q2 = round(msg[1], 2)          # [ millimeters ]
            msg_micro.delta_q3 = round(msg[2], 2)          # [ millimeters ]
            msg_micro.delta_t = round(msg[3]*1e6, 1)      # [ microseconds ]
            
            msg_micro_array.set_points[set_point_index] = msg_micro

        # end of array
        msg_micro = SetPoint()
        msg_micro.delta_t = float(-1)      # [ microseconds ]
        msg_micro_array.set_points[set_point_index+1] = msg_micro

        self.joint_position_micro_pub.publish(msg_micro_array)
        return


def main(args=None):
    rclpy.init(args=args)

    rc_node = RobotController()

    rclpy.spin(rc_node)

    rc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()