#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
ROS node for controling a Robotiq 2F gripper using the Modbus RTU protocol.

The script takes as an argument the IP address of the gripper. It initializes a baseRobotiq2FGripper object and adds a comModbusTcp client to it. It then loops forever, reading the gripper status and updating its command. The gripper status is published on the 'Robotiq2FGripperRobotInput' topic using the 'Robotiq2FGripper_robot_input' msg type. The node subscribes to the 'Robotiq2FGripperRobotOutput' topic for new commands using the 'Robotiq2FGripper_robot_output' msg type. Examples are provided to control the gripper (Robotiq2FGripperSimpleController.py) and interpreting its status (Robotiq2FGripperStatusListener.py).
"""

import rclpy
from rclpy.node import Node
import sys
import os

# 修改导入路径
from robotiq_interfaces.msg import GripperInput, GripperOutput
from . import baseRobotiq2FGripper
from . import comModbusRtu

import rclpy.logging

class RobotiqGripperNode(Node):  # 继承自 Node

    def __init__(self, device):
        super().__init__('robotiq_2f_gripper_node')  # ROS 2 节点初始化
        
        self.gripper = baseRobotiq2FGripper.robotiqbaseRobotiq2FGripper()
        self.gripper.client = comModbusRtu.communication()
        
        # 连接到设备
        if not self.gripper.client.connectToDevice(device):
            self.get_logger().error(f"Failed to connect to device: {device}")
            raise Exception("Connection failed")
        
        # 创建状态发布器 - ROS 2 方式
        self.publisher = self.create_publisher(
            GripperInput, 
            'gripper_input', 
            10
        )
        
        # 创建命令订阅器 - ROS 2 方式
        self.subscription = self.create_subscription(
            GripperOutput,
            'gripper_output',
            self.command_callback,
            10
        )
        
        # 创建定时器替代 ROS 1 的 rate.sleep() 循环
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz
        #self.get_logger().info("Robotiq 2F Gripper Node Started")

    def command_callback(self, msg):
        """处理收到的命令"""
        self.gripper.refreshCommand(msg)

    def timer_callback(self):
        """定时器回调函数，替代主循环"""
        #try:
        self.gripper.sendCommand()
        status = self.gripper.getStatus()
        self.publisher.publish(status)
        #print(status)
            #self.get_logger().debug(f"Published status: {status}", throttle_duration_sec=1.0)
        #except Exception as e:
            #self.get_logger().error(f"Error in timer callback: {str(e)}")

    def destroy_node(self):
        """节点销毁时清理资源"""
        self.gripper.client.disconnectFromDevice()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run robotiq_2f_gripper_ros2 gripper_node <device>")
        return 1
    
    device = sys.argv[1]
    node = RobotiqGripperNode(device)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
