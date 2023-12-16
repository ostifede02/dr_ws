import rclpy
from rclpy.node import Node

import tkinter as tk
from tkinter import messagebox

from geometry_msgs.msg import Point32


class RobotControllerGUI(Node):

    def __init__(self):
        super().__init__('input_gui')

        self.pub = self.create_publisher(
            Point32,
            'input_gui',
            10)
        
        self.init_gui()
        
        # Execute the main event loop
        self.root.mainloop()
        return
        

    def init_gui(self):
        # Create the main window
        self.root = tk.Tk()
        self.root.title("GUI")

        # Set background color to a light gray
        background_color = '#f4f4f4'
        self.root.configure(bg=background_color)

        # Adding a title label within the window
        title_label = tk.Label(self.root, text="Robot Controller", font=("Helvetica", 16), bg=background_color, fg='#333333')
        title_label.grid(row=0, column=0, columnspan=3, pady=10)

        # Create input labels and entry widgets for x and z
        tk.Label(self.root, text="x:", bg=background_color, fg='#333333').grid(row=1, column=0, padx=10, pady=10)
        self.x_entry = tk.Entry(self.root)
        self.x_entry.grid(row=1, column=1, padx=10, pady=10)

        tk.Label(self.root, text="z:", bg=background_color, fg='#333333').grid(row=2, column=0, padx=10, pady=10)
        self.z_entry = tk.Entry(self.root)
        self.z_entry.grid(row=2, column=1, padx=10, pady=10)

        # Create Start and Stop buttons
        start_button = tk.Button(self.root,
                                text="Start", 
                                command=self.start_button, 
                                bg='#4CAF50', 
                                activebackground="#388E3C", 
                                fg='white', 
                                activeforeground="white")
        start_button.grid(row=3, column=1, padx=30, pady=10, sticky=tk.E)


        stop_button = tk.Button(self.root, 
                                text="Stop", 
                                command=self.stop_button,
                                bg='#F44336', 
                                activebackground="#D32F2F", 
                                fg='white',
                                activeforeground="white")

        stop_button.grid(row=3, column=1, padx=30, pady=10, sticky=tk.W)

        return
    

    def start_button(self):
        msg = Point32()

        self.x_entry.delete(0, tk.END)
        self.z_entry.delete(0, tk.END)

        try:
            msg.x = float(self.x_entry.get())
            msg.z = float(self.z_entry.get())
            self.pub.publish(msg)

        except ValueError:
            self.get_logger().error("Insert a numerical value.")
        return

    def stop_button(self):
        msg = Point32()
        msg.y = float(-1)
        self.x_entry.delete(0, tk.END)
        self.z_entry.delete(0, tk.END)

        self.pub.publish(msg)
        return




def main(args=None):
    rclpy.init(args=args)

    gui_node = RobotControllerGUI()

    rclpy.spin(gui_node)

    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

