import rclpy
from rclpy.node import Node

import tkinter as tk
from tkinter import ttk

from deltarobot import configuration as conf

from deltarobot_interfaces.msg import TrajectoryTask



class GUI(Node):

    def __init__(self):
        super().__init__('gui_node')

        self.pub = self.create_publisher(
            TrajectoryTask,
            'input_gui',
            10)
        
        self.init_gui()
        
        return
    

    def relative_to_assets(self, path):
        ASSETS_PATH = "/home/ostifede02/Documents/2dr_ws/assets/gui/"
        return str(ASSETS_PATH + path)
        

    def init_gui(self):
        ## parameters
        bg_color = "#2B3499"
        fg_color = "#FFFFFF"
        entry_font = ("Helvetica", 18, "bold")

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
            y=776.0,
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
            y=688.0,
            width=512.0,
            height=72.0
        )

        ## input X coordinate
        self.entry_x_img = tk.PhotoImage(
            file=self.relative_to_assets("entry_x.png"))
        self.canvas.create_image(
            320.0,
            310.5,
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
            y=293,
            width=256,
            height=46
        )

        ## input Z coordinate
        self.entry_z_img = tk.PhotoImage(
            file=self.relative_to_assets("entry_z.png"))
        self.canvas.create_image(
            320.0,
            391.5,
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
            y=374,
            width=256,
            height=46
        )

        ## input time
        self.entry_time_img = tk.PhotoImage(
            file=self.relative_to_assets("entry_time.png"))
        self.canvas.create_image(
            320.0,
            472.5,
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
            y=455,
            width=256,
            height=46
        )

        ## input type
        self.entry_path_type_img = tk.PhotoImage(
            file=self.relative_to_assets("entry_type.png"))
        self.canvas.create_image(
            320.0,
            554.0,
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

        options = ["", 
                   conf.DIRECT_TRAJECTORY_ROUTINE,
                   conf.PICK_TRAJECTORY_ROUTINE,
                   conf.PLACE_TRAJECTORY_ROUTINE,
                   ]
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
            x= 210,
            y= 531,
            width= 256,
            height= 45
        )
        return
    

    def start_button_pressed(self):
        msg = TrajectoryTask()
        try:
            msg.pos_end.x = float(self.entry_x.get())
            msg.pos_end.z = float(self.entry_z.get())
            msg.task_time = float(self.entry_time.get())
            msg.task_type.data = self.combo_task_type.get()
            pass

        except:
            self.get_logger().error("insert valid input")
        
        self.pub.publish(msg)
        return

    def stop_button_pressed(self):
        # msg = TrajectoryTask()

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

