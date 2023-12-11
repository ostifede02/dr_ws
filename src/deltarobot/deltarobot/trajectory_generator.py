import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import TrajectoryTask
from deltarobot_interfaces.msg import SetPointsVector
from deltarobot_interfaces.msg import SetPoint


class TrajectoryGenerator(Node):

    def __init__(self):
        super().__init__('trajectory_generator')

        self.pub = self.create_publisher(
            SetPointsVector,
            'trajectory_vector',
            10)
        
        self.sub = self.create_subscription(
            TrajectoryTask,
            'trajectory_task',
            self.trajectory_generator_callback,
            10)
        self.sub


    def trajectory_generator_callback(self, trajectory_task_msg):
            # do something
            return




def main(args=None):
    rclpy.init(args=args)

    tg_node = TrajectoryGenerator()

    rclpy.spin(tg_node)

    tg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()