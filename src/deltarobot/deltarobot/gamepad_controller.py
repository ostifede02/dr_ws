#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from deltarobot import configuration as conf

from deltarobot_interfaces.msg import TrajectoryTask
from std_msgs.msg import String

import inputs
import numpy as np


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
        timer_period = 0.1  # seconds
        self.publish_task_timer = self.create_timer(
            timer_period, 
            self.publish_task_timer_callback)
        
        # Set up timer to periodically get gamepad state
        timer_period = 0.0005  # seconds
        self.read_gamepad_timer = self.create_timer(
            timer_period, 
            self.read_gamepad_timer_callback)
        
        # Subscribe to robot state topic
        self.robot_state_sub = self.create_subscription(
            String,
            'robot_state',
            self.robot_state_callback,
            1)

        # Initialize lock to avoid publishing double tasks
        self.pub_task_lock = False

        # Get the gamepad device
        self.gamepad = inputs.devices.gamepads[0]
        self.x = 0                  # [ mm ]
        self.y = 0                  # [ mm ]
        self.z = 0                  # [ mm ]
        self.velocity = 50          # [ mm/s ]
        self.delta_time = 0.05      # [ s ]
        self.MAX_VELOCITY = 250     # [ mm/s ]
        self.MIN_VELOCITY = 10      # [ mm/s ]

        return
    

    def publish_task_timer_callback(self):
        """
        Timer callback function to send trajectory task based on gamepad input.
        """

        # If robot is in idle, it can move
        if self.pub_task_lock == False:
            if self.x == 0 and self.y == 0 and self.z == 0:
                return
            
            # Create trajectory task message
            trajectory_task_msg = TrajectoryTask()
            trajectory_task_msg.pos_end.x = float(self.x)
            trajectory_task_msg.pos_end.y = float(self.y)
            trajectory_task_msg.pos_end.z = float(self.z)
            trajectory_task_msg.task_time = float(self.delta_time)
            trajectory_task_msg.task_type.data = str(conf.P2P_DIRECT_TRAJECTORY)
            trajectory_task_msg.is_trajectory_absolute_coordinates = False

            # Publish trajectory task message
            self.trajectory_task_input_pub.publish(trajectory_task_msg)
            # set a lock to publish once
            self.pub_task_lock = True
        return
    

    def read_gamepad_timer_callback(self):      
        events = self.gamepad._do_iter(timeout=0.0002)

        # do not update gamepad states if no data available        
        if events is None:
            return

        for event in events:
            ## get value X coordinate
            # joy
            if event.code == 'ABS_X':
                # normalized value [-1, 1]
                x_input = (event.state-127.5)/127.5
                if abs(self.x) < 0.2:
                    self.x = 0
                # self.x = self.x*self.velocity*self.delta_time
            # hat
            elif event.code == 'ABS_HAT0X':
                self.x = event.state*self.velocity*self.delta_time


            ## get value Y coordinate
            # joy
            elif event.code == 'ABS_Y':
                # normalized value [-1, 1]
                y_input = -(event.state-127.5)/127.5
                if abs(self.y) < 0.2:
                    self.y = 0
                # self.y = self.y*self.velocity*self.delta_time
            # hat
            elif event.code == 'ABS_HAT0Y':
                self.y = -event.state*self.velocity*self.delta_time


            ## get value Y coordinate
            elif  event.code == 'ABS_RZ':
                # normalized value [-1, 1]
                z_input = -(event.state-127.5)/127.5
                if abs(self.z) < 0.2:
                    self.z = 0
                # self.z = self.z*self.velocity*self.delta_time

            ## set velocity
            # increase velocity
            elif  event.code == 'BTN_TR':
                self.velocity += 25
                self.velocity = min(self.velocity, self.MAX_VELOCITY)
                
            # decrease velocity
            elif  event.code == 'BTN_TL':
                self.velocity -= 25
                self.velocity = max(self.velocity, self.MIN_VELOCITY)

            ## control gripper
                # A -> "BTN_SOUTH"
                # B -> "BTN_WEST"
                # X -> "BTN_NORTH"
                # Y -> "BTN_EAST"

        # normalize
        coordinates_norm = np.linalg.norm([x_input, y_input, z_input])
        self.x = (x_input/coordinates_norm) * self.velocity*self.delta_time
        self.y = (y_input/coordinates_norm) * self.velocity*self.delta_time
        self.z = (z_input/coordinates_norm) * self.velocity*self.delta_time

        return
    

    def robot_state_callback(self, robot_state_msg):
        """
        Callback function for receiving robot state.

        Args:
            robot_state_msg (String): The received robot state message.
        """
        
        # robot is ready to receive new task
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

    rclpy.spin(gp_node)

    gp_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
