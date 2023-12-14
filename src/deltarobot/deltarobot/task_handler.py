import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import TrajectoryTask
from deltarobot_interfaces.msg import SetPointsVector


class TaskHandler(Node):

    def __init__(self):
        super().__init__('task_handler')

        self.pub = self.create_publisher(
            SetPointsVector,
            'trajectory_task',
            10)
        
        self.sub = self.create_subscription(
            TrajectoryTask,
            'user_input',
            self.task_handler_callback,
            10)
        self.sub


    def task_handler_callback(self, task_input_msg):
            # do something
            return




def main(args=None):
    rclpy.init(args=args)

    th_node = TaskHandler()

    rclpy.spin(th_node)

    th_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()