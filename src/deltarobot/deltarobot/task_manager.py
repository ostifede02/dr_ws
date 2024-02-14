import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import TrajectoryTask
from deltarobot_interfaces.msg import TaskAck

from std_msgs.msg import String

from deltarobot import configuration as conf

import numpy as np
import time as time_

class TaskManager(Node):

    def __init__(self):
        super().__init__('task_manager_node')

        ## trajectory task
        self.trajectory_task_input_sub = self.create_subscription(
            TrajectoryTask,
            'trajectory_task_input',
            self.trajectory_task_input_callback,
            1)
        self.trajectory_task_input_sub

        self.trajectory_task_pub = self.create_publisher(
            TrajectoryTask,
            'trajectory_task',
            1)
        
        ## trajectory ack
        self.task_ack_sub = self.create_subscription(
            TaskAck,
            'task_ack',
            self.task_ack_callback,
            1)
        self.task_ack_sub

        ## robot state
        self.robot_state_pub = self.create_publisher(
            String,
            'robot_state',
            1)
        
        self.robot_state_sub = self.create_subscription(
            String,
            'robot_state',
            self.robot_state_callback,
            1)
        self.robot_state_sub


        self.pos_current = conf.configuration["trajectory"]["pos_home"]     ## after home calibration
        self.pos_current_volatile = np.empty(3)
        
        # initialize robot state
        self.robot_state = "idle"
        

        # publish robot state
        self.publish_robot_state(self.robot_state)

        return



    def trajectory_task_input_callback(self, task_input_msg):
        # # check robot state
        # if self.robot_state == "run":
        #     # raise exception
        #     self.get_logger().warning("robot state is stop!")
        #     return

        # elif self.robot_state == "stop":
        #     # raise exception
        #     self.get_logger().warning("robot state is stop!")
        #     return
        
        task_output_msg = TrajectoryTask()

        # define pos_start
        task_output_msg.pos_start.x = float(self.pos_current[0])
        task_output_msg.pos_start.y = float(self.pos_current[1])
        task_output_msg.pos_start.z = float(self.pos_current[2])

        ## define pos_end
        if task_input_msg.is_trajectory_absolute_coordinates == True:
            task_output_msg.pos_end = task_input_msg.pos_end
        else:
            task_output_msg.pos_end.x = task_input_msg.pos_end.x + self.pos_current[0]
            task_output_msg.pos_end.y = task_input_msg.pos_end.y + self.pos_current[1]
            task_output_msg.pos_end.z = task_input_msg.pos_end.z + self.pos_current[2]

        # define other parameters
        task_output_msg.time = task_input_msg.time
        task_output_msg.task_type = task_input_msg.task_type
        task_output_msg.is_trajectory_absolute_coordinates = task_input_msg.is_trajectory_absolute_coordinates

        # publish task message
        self.trajectory_task_pub.publish(task_output_msg)

        # store the future pos_current in a volatile variable
        #   - in case of nak, the pos_current will remain the same
        #   - in case of ack, the pos_current will be updated with pos_current_volatile
        self.pos_current_volatile[0] = task_output_msg.pos_end.x
        self.pos_current_volatile[1] = task_output_msg.pos_end.y
        self.pos_current_volatile[2] = task_output_msg.pos_end.z
        
        # update robot state
        self.robot_state = "run"
        # publish robot state
        self.publish_robot_state(self.robot_state)
        
        return


    def task_ack_callback(self, task_ack_msg):
        # the task has been succesfull
        if task_ack_msg.task_ack == True:
            # update current positions
            self.pos_current = self.pos_current_volatile
            
            # update robot state
            self.robot_state = "idle"
            # publish robot state
            self.publish_robot_state(self.robot_state)

        elif task_ack_msg.task_ack == False:
            # raise error
            pass

        return
    

    def robot_state_callback(self, robot_state_msg):
        # respond with the robot state
        if robot_state_msg.data == "request":
            self.publish_robot_state(self.robot_state)
        else:
            self.robot_state = robot_state_msg.data
        return
    

    def publish_robot_state(self, state):
        # publish robot state
        robot_state_output_msg = String()
        robot_state_output_msg.data = state
        self.robot_state_pub.publish(robot_state_output_msg)
        return


def main(args=None):
    rclpy.init(args=args)

    tm_node = TaskManager()

    rclpy.spin(tm_node)

    tm_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()