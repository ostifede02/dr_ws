#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import TrajectoryTask
from deltarobot_interfaces.msg import JointTrajectory
from deltarobot_interfaces.msg import JointTrajectoryArray

from micro_custom_messages.msg import JointTrajectoryReduced
from micro_custom_messages.msg import JointTrajectoryReducedArray
from micro_custom_messages.msg import TaskAck


from deltarobot.inverse_geometry import InverseGeometry
from deltarobot.trajectory_generator import TrajectoryGenerator
from deltarobot import configuration as conf

import pinocchio as pin
import numpy as np

from os.path import join
import time


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller_node')
        
        ## define robot parameters

        ## define publishers and subscribers
        
        ## import urdf models
        
        ## init InverseGeometry
        
        ## init TrajectoryGenerator

        return
    
    ###################################################################################
    #                                                                                 #
    #                              CALLBACK FUNCTIONS                                 #
    #                                                                                 #
    ###################################################################################

    def input_cmds__move__task_space__ptp__callback(self, msg):
        return
    
    def input_cmds__gripper__em__callback(self, msg):
        return
    
    def input_cmds__homing__callback(self, msg):
        return


    def robot_state_callback(self, msg):
        return
    

    def robot_feedback_callback(self, msg):
        return
    

    ###################################################################################
    #                                                                                 #
    #                               PUBLISH FUNCTIONS                                 #
    #                                                                                 #
    ###################################################################################

    def robot_cmds__move__joint_trajectory__publish(self, np_array):
        return

    def robot_cmds__move__joint_trajectory_reduced__publish(self, np_array):
        return
    
    def robot_cmds__gripper__em__publish(self, np_array):
        return
    
    def robot_cmds__homing__publish(self, np_array):
        return


    ###################################################################################
    #                                                                                 #
    #                                UTILS FUNCTIONS                                  #
    #                                                                                 #
    ###################################################################################

    def import_robot_model(self):
        return









###################################################################################
#                                                                                 #
#                                      MAIN                                       #
#                                                                                 #
###################################################################################

def main(args=None):
    rclpy.init(args=args)

    rc_node = RobotController()

    rclpy.spin(rc_node)

    rc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
        main()