import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import TrajectoryTask
from deltarobot_interfaces.msg import SetPointsVector


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.pub = self.create_publisher(
            SetPointsVector,
            'joint_position',
            100)
        
        self.sub = self.create_subscription(
            TrajectoryTask,
            'trajectory_vector',
            self.robot_controller_callback,
            10)
        self.sub


    def robot_controller_callback(self, set_points_vector_msg):
            # do something
            return




def main(args=None):
    rclpy.init(args=args)

    rc_node = RobotController()

    rclpy.spin(rc_node)

    rc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()