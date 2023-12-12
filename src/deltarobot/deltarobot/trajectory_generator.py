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
        ## unpack the message
        pos_start = trajectory_task_msg.pos_start
        pos_end = trajectory_task_msg.pos_end
        t_total_input = trajectory_task_msg.task_time        
        path_routine_type = trajectory_task_msg.task_type

        ## do the rest

        ## publish the vector of {x, z, dt}
       
        return




def main(args=None):
    rclpy.init(args=args)

    tg_node = TrajectoryGenerator()

    rclpy.spin(tg_node)

    tg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()