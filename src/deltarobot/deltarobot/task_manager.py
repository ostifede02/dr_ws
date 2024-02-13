import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import TrajectoryTask
from deltarobot_interfaces.msg import TaskAck
from geometry_msgs.msg import Point32
from std_msgs.msg import String

from deltarobot_interfaces.srv import RobotState

from deltarobot import configuration as conf

import numpy as np


class TaskManager(Node):

    def __init__(self):
        super().__init__('task_manager_node')

        self.trajectory_task_pub = self.create_publisher(
            TrajectoryTask,
            'trajectory_task',
            10)
        
        self.trajectory_task_input_sub = self.create_subscription(
            TrajectoryTask,
            'trajectory_task_input',
            self.trajectory_task_input_callback,
            10)
        self.trajectory_task_input_sub

        self.robot_state_service = self.create_service(
            RobotState,
            'robot_state',
            self.robot_state_callback)

        self.task_ack_sub = self.create_subscription(
            TaskAck,
            'task_ack',
            self.task_ack_callback,
            10)
        self.trajectory_task_input_sub


        self.pos_current = conf.configuration["trajectory"]["pos_home"]     ## after home calibration
        self.pos_current_volatile = np.empty(3)
        self.robot_state = "idle"
        return



    def trajectory_task_input_callback(self, task_input_msg):
        task_output_msg = TrajectoryTask()

        pos_start_ = Point32()
        pos_start_.x = float(self.pos_current[0])
        pos_start_.y = float(self.pos_current[1])
        pos_start_.z = float(self.pos_current[2])
        
        task_output_msg.pos_start = pos_start_
        task_output_msg.pos_end = task_input_msg.pos_end
        task_output_msg.time = task_input_msg.time
        task_output_msg.task_type = task_input_msg.task_type
        task_output_msg.is_trajectory_absolute_coordinates = task_input_msg.is_trajectory_absolute_coordinates

        self.trajectory_task_pub.publish(task_output_msg)
        self.state = "run"

        self.pos_current_volatile[0] = task_input_msg.pos_end.x
        self.pos_current_volatile[1] = task_input_msg.pos_end.y
        self.pos_current_volatile[2] = task_input_msg.pos_end.z
        return
    
    
    def robot_state_callback(self, request, response):
        self.get_logger().info(f"request recived: {request.robot_state_request.data}")

        if request.robot_state_request.data == "request":
            response.robot_state_response.data = self.robot_state
        
        elif request.robot_state_request.data == "stop":
            self.robot_state = "stop"
            response.robot_state_response.data = self.robot_state
        
        return response


    def task_ack_callback(self):
        
        return

def main(args=None):
    rclpy.init(args=args)

    tm_node = TaskManager()

    rclpy.spin(tm_node)

    tm_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()