import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import SetPointsVector
from deltarobot_interfaces.msg import SetPoint

import pinocchio as pin

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller_node')

        self.pub = self.create_publisher(
            SetPoint,
            'set_point',
            10)
        
        self.sub = self.create_subscription(
            SetPointsVector,
            'set_points_vector',
            self.robot_controller_callback,
            10)
        self.sub


    def robot_controller_callback(self, set_points_vector_msg):
            for set_point_msg in set_points_vector_msg.set_points:
                # compute inverse geometry
                # check for collisions
                self.get_logger().info(f"x: {set_point_msg.x} z: {set_point_msg.z} t: {set_point_msg.t}")

                # get delta_t

                # update current position

                # publish to viz            
                pass

            # publish feedback to task handler
            return




def main(args=None):
    rclpy.init(args=args)

    rc_node = RobotController()

    rclpy.spin(rc_node)

    rc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()