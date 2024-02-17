import rclpy
from rclpy.node import Node

from deltarobot import configuration as conf

from deltarobot_interfaces.msg import TrajectoryTask
from std_msgs.msg import String

import random
import numpy as np
import inputs
import time

class GamePadController(Node):

    def __init__(self):
        """
        Initializes the GamePadController node.
        """
        super().__init__('gamepad_controller_node')

        # Create publisher for trajectory task input
        self.trajectory_task_input_pub = self.create_publisher(
            TrajectoryTask,
            'trajectory_task_input',
            1)
        
        # Set up timer to periodically send trajectory task
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(
            timer_period, 
            self.timer_callback)
        
        # Subscribe to robot state topic
        self.robot_state_sub = self.create_subscription(
            String,
            'robot_state',
            self.robot_state_callback,
            10)
        
        # self.gamepad = inputs.devices.gamepads[0]

        # Initialize lock to avoid publishing double tasks
        self.pub_task_lock = False
        return
    

    def timer_callback(self):
        """
        Timer callback function to send trajectory task based on gamepad input.
        """

        x, y, z = self.get_game_pad_state()

        # Gamepad state has not changed
        if x is None:
            return

        # If robot is in idle, it can move
        if self.pub_task_lock == False:
            # Create trajectory task message
            trajectory_task_msg = TrajectoryTask()
            trajectory_task_msg.pos_end.x = float(x)
            trajectory_task_msg.pos_end.y = float(y)
            trajectory_task_msg.pos_end.z = float(z)
            trajectory_task_msg.task_time = float(0.05)
            trajectory_task_msg.task_type.data = str(conf.P2P_DIRECT_TRAJECTORY)
            trajectory_task_msg.is_trajectory_absolute_coordinates = False

            # Publish trajectory task message
            self.trajectory_task_input_pub.publish(trajectory_task_msg)
            # set a lock to publish once
            self.pub_task_lock = True
        return


    def get_game_pad_state(self):
        """
        Simulates getting gamepad state.

        Returns:
            np.array: A numpy array containing the x, y, z coordinates of the simulated gamepad state.
        """
        # Generate random gamepad input
        sign_seed = random.random()
        if sign_seed < 0.5:
            sign = -1
        else:
            sign = 1
        
        x = 15 * random.random() * sign
        y = 15 * random.random() * sign
        z = -15 * random.random() * sign
        
        return np.array([x, y, z])


    def robot_state_callback(self, robot_state_msg):
        """
        Callback function for receiving robot state.

        Args:
            robot_state_msg (String): The received robot state message.
        """
        
        # robot is ready to recive new task
        if robot_state_msg.data == conf.ROBOT_STATE_IDLE:
            self.pub_task_lock = False
        else:
            self.pub_task_lock = True

        return


def main(args=None):
    """
    Main function.
    """
    rclpy.init(args=args)

    gp_node = GamePadController()

    time.sleep(1)
    rclpy.spin(gp_node)

    gp_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
