import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import JointPositionTelemetry

import numpy as np
import matplotlib.pyplot as plt



class Telemetry(Node):

    def __init__(self):
        super().__init__('telemetry_node')

        self.sub = self.create_subscription(
            JointPositionTelemetry,
            'joint_position_telemetry',
            self.telemetry_callback,
            100)
        self.sub

        return


    def telemetry_callback(self, joint_position_msg):
        ## unpack the msg
        t = joint_position_msg.t

        if t == 0:
            self.plot_index = 0
            self.q_carriage_1_array = np.empty((2,120))
            self.q_carriage_2_array = np.empty((2,120))
            self.q_carriage_3_array = np.empty((2,120))
        
        elif t == -1.0:
            self.plot_datas()
            return

        q1_current = joint_position_msg.q[0]
        q2_current = joint_position_msg.q[1]
        q3_current = joint_position_msg.q[2]

        self.q_carriage_1_array[0, self.plot_index] = t
        self.q_carriage_1_array[1, self.plot_index] = q1_current

        self.q_carriage_2_array[0, self.plot_index] = t
        self.q_carriage_2_array[1, self.plot_index] = q2_current

        self.q_carriage_3_array[0, self.plot_index] = t
        self.q_carriage_3_array[1, self.plot_index] = q3_current        

        self.plot_index += 1
        return

    def plot_datas(self):
        # initialize plot
        fig = plt.figure()
        plot = fig.add_subplot(111)

        # plot position
        plot.plot(self.q_carriage_1_array[0, 0:self.plot_index], self.q_carriage_1_array[1, 0:self.plot_index], "r", label="joint 1")
        plot.plot(self.q_carriage_2_array[0, 0:self.plot_index], self.q_carriage_2_array[1, 0:self.plot_index], "g", label="joint 2")
        plot.plot(self.q_carriage_3_array[0, 0:self.plot_index], self.q_carriage_3_array[1, 0:self.plot_index], "b", label="joint 3")
        
        # compute and plot velocity
        ## TO DO...

        plt.legend()
        plt.title("carriage joints position")
        plt.xlabel("time [ s ]")
        plt.ylabel("joint position [ mm ]")
        plt.show()
        return


def main(args=None):
    rclpy.init(args=args)

    tl_node = Telemetry()

    rclpy.spin(tl_node)

    tl_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()