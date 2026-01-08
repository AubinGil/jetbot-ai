#!/usr/bin/env python3
"""
Robot Face Control Panel - GUI Application
Easy-to-use desktop interface for controlling robot face expressions and gaze
"""

import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys


class RobotFaceControlGUI(Node):
    def __init__(self, master):
        super().__init__('robot_face_control_gui')

        self.master = master
        self.master.title("Robot Face Control Panel")
        self.master.geometry("500x600")
        self.master.resizable(False, False)

        # Publishers
        self.expression_pub = self.create_publisher(String, 'robot_face/expression', 10)
        self.gaze_pub = self.create_publisher(String, 'robot_face/gaze', 10)

        # Create GUI
        self.create_widgets()

        # Start ROS spin in background thread
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

        self.get_logger().info('Robot Face Control GUI started')

    def spin_ros(self):
        """Spin ROS in background thread"""
        rclpy.spin(self)

    def create_widgets(self):
        """Create all GUI widgets"""

        # Title
        title = tk.Label(
            self.master,
            text="🤖 Robot Face Control Panel",
            font=("Arial", 20, "bold"),
            pady=20
        )
        title.pack()

        # Expression Frame
        expr_frame = tk.LabelFrame(
            self.master,
            text="Expressions",
            font=("Arial", 14, "bold"),
            padx=20,
            pady=20
        )
        expr_frame.pack(padx=20, pady=10, fill="both", expand=True)

        # Expression buttons - 2 rows
        expressions = [
            ("😊 Happy", "happy", "#90EE90"),
            ("😮 Surprised", "surprised", "#87CEEB"),
            ("😴 Tired", "tired", "#DDA0DD"),
            ("😠 Angry", "angry", "#FF6B6B"),
            ("🔍 Scanning", "scanning", "#98FB98"),
            ("😐 Default", "default", "#D3D3D3")
        ]

        row1 = tk.Frame(expr_frame)
        row1.pack()
        row2 = tk.Frame(expr_frame)
        row2.pack(pady=(10, 0))

        for i, (label, expr, color) in enumerate(expressions):
            parent = row1 if i < 3 else row2
            btn = tk.Button(
                parent,
                text=label,
                font=("Arial", 12),
                bg=color,
                width=15,
                height=2,
                command=lambda e=expr: self.set_expression(e)
            )
            btn.pack(side=tk.LEFT, padx=5)

        # Gaze Direction Frame
        gaze_frame = tk.LabelFrame(
            self.master,
            text="Gaze Direction",
            font=("Arial", 14, "bold"),
            padx=20,
            pady=20
        )
        gaze_frame.pack(padx=20, pady=10, fill="both", expand=True)

        # Create 3x3 grid for gaze directions
        directions = [
            ["nw", "n", "ne"],
            ["w", "center", "e"],
            ["sw", "s", "se"]
        ]

        direction_labels = [
            ["↖", "↑", "↗"],
            ["←", "●", "→"],
            ["↙", "↓", "↘"]
        ]

        for i, row in enumerate(directions):
            row_frame = tk.Frame(gaze_frame)
            row_frame.pack()

            for j, direction in enumerate(row):
                label = direction_labels[i][j]
                btn = tk.Button(
                    row_frame,
                    text=label,
                    font=("Arial", 20),
                    width=4,
                    height=2,
                    command=lambda d=direction: self.set_gaze(d)
                )
                btn.pack(side=tk.LEFT, padx=3, pady=3)

        # Status Frame
        status_frame = tk.Frame(self.master)
        status_frame.pack(padx=20, pady=10, fill="x")

        tk.Label(
            status_frame,
            text="Status:",
            font=("Arial", 11, "bold")
        ).pack(side=tk.LEFT)

        self.status_label = tk.Label(
            status_frame,
            text="Ready",
            font=("Arial", 11),
            fg="green"
        )
        self.status_label.pack(side=tk.LEFT, padx=10)

        # Info label
        info = tk.Label(
            self.master,
            text="Click buttons to control robot face\nMake sure robot_face_node is running!",
            font=("Arial", 9),
            fg="gray"
        )
        info.pack(pady=5)

    def set_expression(self, expression):
        """Publish expression command"""
        msg = String()
        msg.data = expression
        self.expression_pub.publish(msg)
        self.status_label.config(text=f"Expression: {expression}", fg="blue")
        self.get_logger().info(f'Set expression to: {expression}')

    def set_gaze(self, direction):
        """Publish gaze direction command"""
        msg = String()
        msg.data = direction
        self.gaze_pub.publish(msg)
        self.status_label.config(text=f"Looking: {direction}", fg="blue")
        self.get_logger().info(f'Set gaze to: {direction}')


def main():
    # Initialize ROS
    rclpy.init()

    # Create Tkinter root
    root = tk.Tk()

    # Create GUI node
    try:
        gui = RobotFaceControlGUI(root)

        # Run Tkinter main loop
        root.mainloop()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        try:
            gui.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
