#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from deltarobot.delta_robot import DeltaRobot

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller_node')
        
        ## define publishers and subscribers
        
        ## init DeltaRobot
        self.robot = DeltaRobot()

        return
    
    ###################################################################################
    #                                                                                 #
    #                              CALLBACK FUNCTIONS                                 #
    #                                                                                 #
    ###################################################################################

    def input_cmds__move__task_space__ptp__callback(self, msg):
        ## unpack message

        ## generate joint trajectory

        return
    
    def input_cmds__gripper__em__callback(self, msg):
        return
    
    def input_cmds__homing__callback(self, msg):
        return


    def robot_state__callback(self, msg):
        return
    

    def robot_feedback__ack__callback(self, msg):
        return
    

    ###################################################################################
    #                                                                                 #
    #                             PUBLISHING FUNCTIONS                                #
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