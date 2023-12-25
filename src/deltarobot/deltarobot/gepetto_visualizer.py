import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import JointPositionViz

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
        package_path = "/home/ostifede02/Documents/2dr_ws/src/deltarobot_description"
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
            q = np.empty(6)
            q[0] = joint_position_msg.q0
            q[1] = joint_position_msg.q1
            q[2] = joint_position_msg.q2
            q[3] = joint_position_msg.q3
            q[4] = joint_position_msg.q4
            q[5] = joint_position_msg.q5
            
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