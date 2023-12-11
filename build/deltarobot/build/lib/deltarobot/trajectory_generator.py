import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import TrajectoryTask
from deltarobot_interfaces.msg import SetPointsVector
from deltarobot_interfaces.msg import SetPoint


class TrajectoryGenerator(Node):

    def __init__(self):
        super().__init__('trajectory_generator')

        self.publisher = self.create_publisher(
            SetPointsVector,
            'trajectory_vector',
            10)
        
        self.subscription = self.create_subscription(
            TrajectoryTask,
            'trajectory_task',
            self.listener_callback,
            10)
        self.subscription


    def listener_callback(self, trj_task_msg):
            trj_vector_msg = SetPointsVector()
            
            for i in range(100):
                set_point_ = SetPoint()
                set_point_.x = float(trj_task_msg.pos_start.x + 2*i)
                set_point_.z = float(trj_task_msg.pos_start.y + i)
                set_point_.delta_t = float(trj_task_msg.pos_start.z + i + 12)
                
                trj_vector_msg.set_points.append(set_point_)

            self.publisher.publish(trj_vector_msg)
            return


def main(args=None):
    rclpy.init(args=args)

    tg_node = TrajectoryGenerator()

    rclpy.spin(tg_node)

    tg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()