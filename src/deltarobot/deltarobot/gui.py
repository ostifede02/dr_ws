import rclpy
from rclpy.node import Node

import tkinter as tk
from tkinter import ttk

from deltarobot import configuration as conf

from deltarobot_interfaces.msg import TrajectoryTask
from std_msgs.msg import String

import time as time_


class GUI(Node):

    def __init__(self):
        super().__init__('gui_node')

        self.trajectory_task_input_pub = self.create_publisher(
            TrajectoryTask,
            'trajectory_task_input',
            1)
        
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

        # initialize gui
        self.init_gui()

        # initialize robot state
        self.robot_state_local = "idle"
        
        return
    

    def relative_to_assets(self, path):
        ASSETS_PATH = conf.configuration["paths"]["gui_assets_path"]
        return str(ASSETS_PATH + path)
        

    def init_gui(self):
        ## parameters
        bg_color = "#2B3499"
        fg_color = "#FFFFFF"
        entry_font = ("Helvetica", 16, "bold")

        ## self.window
        self.window = tk.Tk()
        self.window.title("GUI robot controller")
        self.window.geometry("640x980")
        self.window.resizable(False, False)

        self.canvas = tk.Canvas(
            self.window,
            height = 980,
            width = 640,
            bd = 0,
            highlightthickness = 0,
        )

        self.canvas.place(x = 0, y = 0)
        self.image_bg = tk.PhotoImage(
            file=self.relative_to_assets("bg.png"))
        self.canvas.create_image(
            320.0,
            490.0,
            image=self.image_bg
        )

        ## STOP button
        self.button_stop_img = tk.PhotoImage(
            file=self.relative_to_assets("button_stop.png"))
        self.button_stop = tk.Button(
            image=self.button_stop_img,
            borderwidth=0,
            highlightthickness=0,
            bg=bg_color,
            activebackground=bg_color,
            command=lambda: self.stop_button_pressed()
        )
        self.button_stop.place(
            x=64.0,
            y=778.0,
            width=512.0,
            height=110.0
        )

        ## START button
        self.button_start_img = tk.PhotoImage(
            file=self.relative_to_assets("button_start.png"))
        button_start = tk.Button(
            image=self.button_start_img,
            borderwidth=0,
            highlightthickness=0,
            bg=bg_color,
            activebackground=bg_color,
            command=lambda: self.start_button_pressed()
        )
        button_start.place(
            x=64.0,
            y=690.0,
            width=512.0,
            height=72.0
        )

        ## input X coordinate
        self.entry_x_img = tk.PhotoImage(
            file=self.relative_to_assets("entry_x.png"))
        self.canvas.create_image(
            320.0,
            266,
            image=self.entry_x_img
        )
        self.entry_x = tk.Entry(
            bd=0,
            bg=bg_color,
            fg=fg_color,
            font=entry_font,
            highlightthickness=0,
        )
        self.entry_x.place(
            x=210,
            y=245,
            width=256,
            height=46
        )

        ## input Y coordinate
        self.entry_y_img = tk.PhotoImage(
            file=self.relative_to_assets("entry_y.png"))
        self.canvas.create_image(
            320.0,
            343.5,
            image=self.entry_y_img
        )
        self.entry_y = tk.Entry(
            bd=0,
            bg=bg_color,
            fg=fg_color,
            font=entry_font,
            highlightthickness=0,
        )
        self.entry_y.place(
            x=210,
            y=323,
            width=256,
            height=46
        )

        ## input Z coordinate
        self.entry_z_img = tk.PhotoImage(
            file=self.relative_to_assets("entry_z.png"))
        self.canvas.create_image(
            320.0,
            420.5,
            image=self.entry_z_img
        )
        self.entry_z = tk.Entry(
            bd=0,
            bg=bg_color,
            fg=fg_color,
            font=entry_font,
            highlightthickness=0
        )
        self.entry_z.place(
            x=210,
            y=400,
            width=256,
            height=46
        )

        ## input time
        self.entry_time_img = tk.PhotoImage(
            file=self.relative_to_assets("entry_time.png"))
        self.canvas.create_image(
            320.0,
            522.0,
            image=self.entry_time_img
        )
        self.entry_time = tk.Entry(
            bd=0,
            bg=bg_color,
            fg=fg_color,
            font=entry_font,
            highlightthickness=0
        )
        self.entry_time.place(
            x=210,
            y=501,
            width=256,
            height=46
        )

        ## input type
        self.entry_path_type_img = tk.PhotoImage(
            file=self.relative_to_assets("entry_type.png"))
        self.canvas.create_image(
            320.0,
            600,
            image=self.entry_path_type_img
        )

        combostyle = ttk.Style()

        combostyle.theme_create('combostyle', parent='alt',
                                settings = {'TCombobox':
                                            {'configure':
                                            {"anchor": "se",
                                            "relief": "flat",
                                            'selectbackground': bg_color,
                                            'fieldbackground': bg_color,
                                            "foreground": fg_color,
                                            "arrowcolor": "false",
                                            }}}
                                )
        # ATTENTION: this applies the new s
        combostyle.theme_use('combostyle')

        self.options_list = {
            "P2P_DIRECT_TRAJECTORY": conf.P2P_DIRECT_TRAJECTORY,
            "P2P_JOINT_TRAJECTORY": conf.P2P_JOINT_TRAJECTORY,
            "P2P_CONTINUOUS_TRAJECTORY": conf.P2P_CONTINUOUS_TRAJECTORY,
            "PICK_TRAJECTORY": conf.PICK_TRAJECTORY,
            "PLACE_TRAJECTORY": conf.PLACE_TRAJECTORY}
        
        options = ["", 
                   "P2P_DIRECT_TRAJECTORY",
                   "P2P_JOINT_TRAJECTORY",
                   "P2P_CONTINUOUS_TRAJECTORY",
                   "PICK_TRAJECTORY",
                   "PLACE_TRAJECTORY"]
        self.combo_task_type = ttk.Combobox(self.window, 
            values=options,
            background=bg_color,
            foreground=fg_color,
            state="readonly"
        )

        self.combo_task_type.configure(
            background=bg_color,
            foreground=fg_color,
            font=entry_font
        )
        self.combo_task_type.place(
            x= 190,
            y= 577,
            width= 350,
            height= 45
        )
        return
    

    def start_button_pressed(self):
        self.get_logger().info(f"robot_state_local: {self.robot_state_local}")

        # if robot is in idle, it can move
        if self.robot_state_local == "idle":    
            trajectory_task_msg = TrajectoryTask()
            # get trajectory task from gui
            try:
                trajectory_task_msg.pos_end.x = float(self.entry_x.get())
                trajectory_task_msg.pos_end.y = float(self.entry_y.get())
                trajectory_task_msg.pos_end.z = float(self.entry_z.get())
                trajectory_task_msg.time = float(self.entry_time.get())
                trajectory_task_msg.task_type = int(self.options_list[self.combo_task_type.get()])
                trajectory_task_msg.is_trajectory_absolute_coordinates = False
            except:
                self.get_logger().error("insert valid input")

            # publish task
            self.trajectory_task_input_pub.publish(trajectory_task_msg)
        
        elif self.robot_state_local == "run":
            # manage exception
            self.get_logger().warning("robot state is run!")

        elif self.robot_state_local == "stop":
            # ignore the input
            self.get_logger().warning("robot state is stop!")

        return


    def stop_button_pressed(self):
        self.publish_robot_state("stop")
        self.raise_exception("stop")
        return
    

    def robot_state_callback(self, robot_state_msg):
        self.robot_state_local = robot_state_msg.data
        return
    

    def publish_robot_state(self, state):
        # publish robot state
        robot_state_output_msg = String()
        robot_state_output_msg.data = state
        self.robot_state_pub.publish(robot_state_output_msg)
        self.robot_state_local = state
        return


    def raise_exception(self, exception):
        self.popup = tk.Toplevel()
        self.popup.title("Exception")

        label = tk.Label(self.popup, 
                         text=f"The robot state is: {exception}.\nPress the button to continue.")
        label.pack(pady=10)

        button = tk.Button(self.popup, text="continue", command=self.resolve_exception)
        button.pack(pady=5)
        return
    
    def resolve_exception(self):
        self.publish_robot_state("idle")
        self.popup.destroy()
        return


def main(args=None):
    rclpy.init(args=args)

    gui_node = GUI()

    # Execute the main event loop
    gui_node.window.mainloop()
    # rclpy.spin(gui_node)

    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

