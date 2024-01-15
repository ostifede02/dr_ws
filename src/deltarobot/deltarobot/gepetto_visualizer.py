import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import JointPositionViz
from deltarobot import configuration as conf

import pinocchio as pin
from pinocchio import RobotWrapper
import numpy as np

from os.path import join
import time


class GepettoVisualizer(Node):

    def __init__(self):
        super().__init__('gepetto_visualizer_node')
       
        self.sub = self.create_subscription(
            JointPositionViz,
            'joint_position_viz',
            self.display_callback,
            100)
        self.sub

        ## init robot model
        # import URDF files
        package_path = conf.configuration["paths"]["package_path"]
        urdf_file_name = "deltarobot.urdf"
        urdf_file_path = join(join(package_path, "urdf"), urdf_file_name)
        mesh_dir = join(package_path,"meshes")

        # Initialize the model
        model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_file_path, mesh_dir, geometry_types=[pin.GeometryType.COLLISION,pin.GeometryType.VISUAL])
        self.robot = RobotWrapper(model, collision_model, visual_model)

        # Initialize the VIEWER
        self.robot.initViewer(loadModel=True)
        self.robot.viz.displayVisuals(True)



    def display_callback(self, joint_position_msg):
            # unpack the message
            q = np.empty(self.robot.nq)

            # chain 1
            q[0] = joint_position_msg.q[0]
            q[1] = joint_position_msg.q[1]
            q[2] = joint_position_msg.q[2]
            q[3] = joint_position_msg.q[1]
            q[4] = joint_position_msg.q[2]
            # chain 2
            q[5] = joint_position_msg.q[3]
            q[6] = joint_position_msg.q[4]
            q[7] = joint_position_msg.q[5]
            q[8] = joint_position_msg.q[4]
            q[9] = joint_position_msg.q[5]
            # chain 3
            q[10] = joint_position_msg.q[6]
            q[11] = joint_position_msg.q[7]
            q[12] = joint_position_msg.q[8]
            q[13] = joint_position_msg.q[7]
            q[14] = joint_position_msg.q[8]
            
            delta_t = joint_position_msg.delta_t

            self.robot.viz.display(q)
            time.sleep(delta_t)
            return




def main(args=None):
    rclpy.init(args=args)

    viz_node = GepettoVisualizer()

    rclpy.spin(viz_node)

    viz_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()