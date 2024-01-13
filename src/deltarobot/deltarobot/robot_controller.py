import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import TrajectoryTask
from deltarobot_interfaces.msg import JointPositionViz

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
        
        self.trajectory_task_sub = self.create_subscription(
            TrajectoryTask,
            'trajectory_task',
            self.robot_controller_callback,
            10)
        self.trajectory_task_sub

        # import urdf file path
        package_path = "/home/ostifede02/Documents/2dr_ws/src/deltarobot_description"
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

        q0_1 = pin.neutral(model_chain_1)
        self.q1_current = self.ig_chain_1.compute_inverse_geometry(q0_1, self.pos_current)
        
        q0_2 = pin.neutral(model_chain_2)
        self.q2_current = self.ig_chain_2.compute_inverse_geometry(q0_2, self.pos_current)

        q0_3 = pin.neutral(model_chain_3)
        self.q3_current = self.ig_chain_3.compute_inverse_geometry(q0_3, self.pos_current)

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
        
        t_current = 0
        # returns an array of x-z setpoints and time
        set_points_vector = self.trajectory_generator.generate_trajectory(pos_start, pos_end, task_time, task_type)

        for set_point in set_points_vector:
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


            ## publish to viz 
            viz_msg = JointPositionViz()
            # chain 1
            viz_msg.q = [self.q1_current[0], self.q1_current[1], self.q1_current[2],
                            self.q2_current[0], self.q2_current[1], self.q2_current[2],
                            self.q3_current[0], self.q3_current[1], self.q3_current[2]]
            viz_msg.delta_t = float(delta_t)

            self.joint_position_viz_pub.publish(viz_msg)

        stop = time.time()
        self.get_logger().info(f"computing time: {(stop-start)*1e3} [ms]")
        return




def main(args=None):
    rclpy.init(args=args)

    rc_node = RobotController()

    rclpy.spin(rc_node)

    rc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()