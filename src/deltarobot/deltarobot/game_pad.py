import rclpy
from rclpy.node import Node

from deltarobot import configuration as conf

from deltarobot_interfaces.msg import TrajectoryTask
from std_msgs.msg import String

import random
import numpy as np
import time

class GamePad(Node):

    def __init__(self):
        super().__init__('game_pad_node')

        self.trajectory_task_input_pub = self.create_publisher(
            TrajectoryTask,
            'trajectory_task_input',
            1)
        
        timer_period = 2  # seconds
        self.timer = self.create_timer(
            timer_period, 
            self.timer_callback)
        
        return
    

    def timer_callback(self):
        self.get_logger().info("timer called")

        # if robot is in idle, it can move
        trajectory_task_msg = TrajectoryTask()
        trajectory_task_msg.pos_end.x = float(5)
        trajectory_task_msg.pos_end.y = float(5)
        trajectory_task_msg.pos_end.z = float(5)
        trajectory_task_msg.time = float(0.1)
        trajectory_task_msg.task_type = int(conf.P2P_DIRECT_TRAJECTORY)
        trajectory_task_msg.is_trajectory_absolute_coordinates = int(False)

        self.get_logger().info(f"task_msg: {trajectory_task_msg}")
        self.trajectory_task_input_pub.publish(trajectory_task_msg)
        
        return


    def get_game_pad_state(self):
        sign_seed = random.random()
        if sign_seed < 0.5:
            sign = -1
        else:
            sign = 1
        
        x = 20*random.random()*sign
        y = 20*random.random()*sign
        z = 20*random.random()*sign
        # seed = random.random()
        # if seed < 0.5:
        #     return None
        # elif seed >= 0.5:

        return np.array([x, y, z])


def main(args=None):
    rclpy.init(args=args)

    gp_node = GamePad()

    rclpy.spin(gp_node)

    gp_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

