#!/usr/bin/env python3
"""
小窗口：底部红色「正常退出」按钮。
点击后发布 /request_normal_exit，触发 gcode_writer 关闭串口并退出，再通知 launch 结束。
"""

import os
import signal
import threading
import time
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class NormalExitButtonNode(Node):
    def __init__(self):
        super().__init__("normal_exit_button_node")
        self.pub = self.create_publisher(Empty, "/request_normal_exit", 10)
        self.get_logger().info("Normal exit button node started, topic: /request_normal_exit")

    def request_normal_exit(self):
        msg = Empty()
        self.pub.publish(msg)
        self.get_logger().info("Published normal exit request")


def spin_thread(executor):
    try:
        executor.spin()
    except Exception:
        pass


def main():
    rclpy.init()
    node = NormalExitButtonNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    t = threading.Thread(target=spin_thread, args=(executor,), daemon=True)
    t.start()

    root = tk.Tk()
    root.title("Mirobot 控制")
    root.resizable(False, False)

    # 红色「正常退出」按钮
    btn_font = tkfont.Font(family="Helvetica", size=14, weight="bold")
    btn = tk.Button(
        root,
        text="正常退出",
        fg="white",
        bg="#c0392b",
        activeforeground="white",
        activebackground="#a93226",
        font=btn_font,
        relief="flat",
        padx=24,
        pady=12,
        cursor="hand2",
    )

    def on_click():
        btn.config(state="disabled", text="正在退出…")
        root.update()
        node.request_normal_exit()
        time.sleep(0.8)
        root.quit()
        root.destroy()
        try:
            os.kill(os.getppid(), signal.SIGINT)
        except Exception:
            pass

    btn.config(command=on_click)
    btn.pack(padx=16, pady=16)

    root.protocol("WM_DELETE_WINDOW", on_click)
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
