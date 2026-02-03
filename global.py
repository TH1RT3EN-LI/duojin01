#!/usr/bin/env python3
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TTY_PORT = "/dev/ttyUSB0"
A_FIXED = 0.00
F_FIXED = 200.00

def send_arm_homing(pub, logger):
    # 机械臂回零 send_arm_homing(pub, node.get_logger())
    cmd = "$H"
    pub.publish(String(data=cmd))
    logger.info("已发布 homing: %s" % cmd)
    time.sleep(10.0)

def send_arm_move(pub, logger, mode, x, y, z, wait=3.0):
    # 坐标模式 send_arm_move(pub, node.get_logger(), "M20", , , , 3.0)
    # 角度模式 send_arm_move(pub, node.get_logger(), "M21", , , , 3.0)
    cmd = "%s G90 G00 X%.2f Y%.2f Z%.2f A%.2f F%.2f" % (mode, x, y, z, A_FIXED, F_FIXED)
    pub.publish(String(data=cmd))
    logger.info("已发布: %s" % cmd)
    time.sleep(wait)

def send_inhale(pub, logger):
    cmd = "M3S1000"
    pub.publish(String(data=cmd))
    logger.info("已发布: 开启气泵")
    time.sleep(3.0)

def send_exhale(pub, logger):
    cmd = "M3S0"
    pub.publish(String(data=cmd))
    logger.info("已发布: 关闭气泵")

def main(args=None):
    rclpy.init(args=args)
    node = Node('my_arm_controller')
    pub = node.create_publisher(String, 'arm_gcode_command', 10)
    # 等待订阅者（桥接节点）
    time.sleep(1.0)

    # 机械臂回零
    send_arm_homing(pub, node.get_logger())
    send_arm_move(pub, node.get_logger(), "M20", 110.00, -160.00, 169.00, 3.0)
    send_inhale(pub, node.get_logger())
    send_arm_move(pub, node.get_logger(), "M20", 110.00, 160.00, 269.00, 3.0)
    send_exhale(pub, node.get_logger())

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()