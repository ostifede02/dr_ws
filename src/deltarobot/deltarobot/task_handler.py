import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import TrajectoryTask
from geometry_msgs.msg import Point32

from deltarobot import configuration as conf


class TaskHandler(Node):

    def __init__(self):
        super().__init__('task_handler_node')

        self.pub = self.create_publisher(
            TrajectoryTask,
            'trajectory_task',
            10)
        
        self.sub = self.create_subscription(
            TrajectoryTask,
            'input_gui',
            self.task_handler_callback,
            10)
        self.sub

        self.pos_current = conf.configuration["trajectory"]["pos_home"]     ## after home calibration


    def task_handler_callback(self, task_input_msg):
        task_output_msg = TrajectoryTask()

        pos_start_ = Point32()
        pos_start_.x = float(self.pos_current[0])
        pos_start_.y = float(self.pos_current[1])
        pos_start_.z = float(self.pos_current[2])
        task_output_msg.pos_start = pos_start_
        
        task_output_msg.pos_end = task_input_msg.pos_end
        task_output_msg.trajectory_delta_time = task_input_msg.trajectory_delta_time
        task_output_msg.task_type = task_input_msg.task_type

        self.pub.publish(task_output_msg)

        self.pos_current[0] = task_input_msg.pos_end.x
        self.pos_current[1] = task_input_msg.pos_end.y
        self.pos_current[2] = task_input_msg.pos_end.z
        return




def main(args=None):
    rclpy.init(args=args)

    th_node = TaskHandler()

    rclpy.spin(th_node)

    th_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()