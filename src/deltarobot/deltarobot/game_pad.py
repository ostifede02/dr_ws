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
        
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(
            timer_period, 
            self.timer_callback)
        
        self.robot_state_sub = self.create_subscription(
            String,
            'robot_state',
            self.robot_state_callback,
            1)
        self.robot_state_sub
        
        self.robot_state_local = "idle"
        return
    

    def timer_callback(self):
        self.get_logger().info("timer called")

        x, y, z = self.get_game_pad_state()

        # gamepad state has not changed
        if x is None:
            return

        # if robot is in idle, it can move
        if self.robot_state_local == "idle":
            trajectory_task_msg = TrajectoryTask()
            trajectory_task_msg.pos_end.x = float(x)
            trajectory_task_msg.pos_end.y = float(y)
            trajectory_task_msg.pos_end.z = float(z)
            trajectory_task_msg.time = float(0.05)
            trajectory_task_msg.task_type = int(conf.P2P_DIRECT_TRAJECTORY)
            trajectory_task_msg.is_trajectory_absolute_coordinates = False

            self.get_logger().info(f"task_msg: {trajectory_task_msg}")
            self.trajectory_task_input_pub.publish(trajectory_task_msg)
            self.robot_state_local = "run"
        
        return


    def get_game_pad_state(self):
        sign_seed = random.random()
        if sign_seed < 0.5:
            sign = -1
        else:
            sign = 1
        
        x = 5*random.random()*sign
        y = 5*random.random()*sign
        z = -5*random.random()*sign
        seed = random.random()
        
        return np.array([x, y, z])
        # if seed < 0.5:
        #     return np.array([None, None, None])
        # elif seed >= 0.5:
        #     return np.array([x, y, z])


    def robot_state_callback(self, robot_state_msg):
        self.robot_state_local = robot_state_msg.data
        return



def main(args=None):
    rclpy.init(args=args)

    gp_node = GamePad()

    rclpy.spin(gp_node)

    gp_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

