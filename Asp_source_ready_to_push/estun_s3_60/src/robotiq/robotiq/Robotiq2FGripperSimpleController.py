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
Command-line interface for sending simple cmds to a ROS node controlling a 2F gripper.

This serves as an example for publishing messages on the 'Robotiq2FGripperRobotOutput' topic using the 'Robotiq2FGripper_robot_output' msg type for sending cmds to a 2F gripper.
"""

from __future__ import print_function
import math
#import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
#import rospy
#from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
#from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8  # 添加这行导入
# 修改导入路径
from robotiq_interfaces.msg import GripperInput
from robotiq_interfaces.msg import GripperOutput
from rclpy.qos import QoSProfile
import threading
from time import sleep

pre_gPO = 0
pre_gCU = 0
pre_pre_gCU = 0
pr = 0
a = GripperOutput()
Gripper_Goal = 3

class SimpleController(Node):

    def __init__(self):
        super().__init__('gripper_simple_controller')
        
        # 创建命令发布器
        self.publisher = self.create_publisher(
            GripperOutput, 
            'gripper_output', 
            10
        )
        
        # 创建状态订阅器
        self.subscription = self.create_subscription(
            GripperInput,
            'gripper_input',
            self.msg_callback,
            10
        )

        # 创建开度指令订阅器 - 接收外部开度指令
        self.goal_subscription = self.create_subscription(
            UInt8,  # 8位整型消息
            'gripper_goal',  # 开度指令话题
            self.gripper_goal_callback,
            10
        )
        
        self.current_cmd = GripperOutput()
        self.get_logger().info("Simple Controller Started. Type cmds in terminal.")
        
        # 在单独线程中处理用户输入，避免阻塞 ROS
        self.input_thread = threading.Thread(target=self.user_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def gripper_goal_callback(self, msg):
        """处理期望开度更新"""
        global Gripper_Goal
        Gripper_Goal = msg.data  # 修正：使用 msg.data 而不是整个消息对象
        #self.get_logger().info(f"Received gripper goal: {Gripper_Goal}")

    def msg_callback(self, msg):
        """处理状态更新"""
        # 定时器回调中执行工作
        """Ask the controller for a refer position to send to the gripper."""
        global pre_gPO, pre_gCU, pre_pre_gCU, pr, Gripper_Goal
    
        self.current_cmd.r_act = 1
        self.current_cmd.r_gto = 1
        self.current_cmd.r_sp  = 90   #180
        Kp = 0.2
        self.current_cmd.r_pr = Gripper_Goal
    
        if msg.g_cu > 4:
            print("Adjust!!")
            #self.current_cmd.r_pr = msg.g_po + (msg.g_cu - pre_gCU) +(pre_gCU - pre_pre_gCU)
            
            #Kp = 0.4 - 0.05*(msg.g_cu - pre_gCU) 
            #Kp  = max(0, Kp)
            Kp = 0.5#0.2
    
        r_fr = round(0.6 * (0 - msg.g_po + pre_gPO) + Kp * (self.current_cmd.r_pr - msg.g_po))
        if r_fr < 0:
            r_fr = abs(r_fr)
        r_fr = max(0, r_fr)
        r_fr = min(255, r_fr)
        self.current_cmd.r_fr = r_fr

        r_sp = round(0.3 * (0 - msg.g_po + pre_gPO) + 0.4 * (self.current_cmd.r_pr - msg.g_po))
        if r_sp < 0:
            r_sp = abs(r_sp)
        r_sp = max(0, r_sp)
        r_sp = min(255, r_sp)
        self.current_cmd.r_sp = r_sp
    
        pre_gPO = msg.g_po
        #print(self.current_cmd.r_sp)
        #print("msg.g_cu\n")
        #print(msg.g_cu)
        #print("r_fr")
        #print(self.current_cmd.r_fr)
    
        pre_pre_gCU = pre_gCU
        pre_gCU = msg.g_cu


    def user_input_loop(self):
        """在单独线程中处理用户输入"""
        while rclpy.ok():
            try:
                #self.print_current_cmd()
                #char = input(self.get_cmd_prompt())
                
                #print("\n")
                #print(self.current_cmd)
                #print("\n")
                #print("r_fr")
                #print(self.current_cmd.r_fr)
                self.publisher.publish(self.current_cmd)
                # 处理命令
                #new_cmd = self.process_cmd(self.current_cmd, char)
                #if new_cmd:
                    #self.current_cmd = new_cmd  # 更新当前命令
                    #self.publisher.publish(new_cmd)
                    
            except Exception as e:
                self.get_logger().error(f"Input error: {e}")

    def process_cmd(self, cmd, char):
        """处理用户输入命令"""
        
        if char == 'a':  # Activate
            cmd = GripperOutput()
            cmd.r_act = 1
            cmd.r_gto = 1
            cmd.r_sp = 255
            cmd.r_fr = 150
            
        if char == 'r':  # Reset
            cmd = GripperOutput()
            cmd.r_act = 0
            
        if char == 'c':  # Close
            #cmd.r_act = 1
            #cmd.r_gto = 1
            cmd.r_pr = 255
            
        if char == 'o':  # Open
            #cmd.r_act = 1
            #cmd.r_gto = 1
            cmd.r_pr = 0
            
        try:
            cmd.r_pr = int(char)   # 设置属性
            if cmd.r_pr > 255:
                cmd.r_pr = 255
            if cmd.r_pr < 0:
                cmd.r_pr = 0
            
        except ValueError:
            pass
            
        if char == 'f':
            cmd.r_sp += 5
            if cmd.r_sp > 255:
                cmd.r_sp = 255

        if char == 'l':
            cmd.r_sp -= 5
            if cmd.r_sp < 0:
                cmd.r_sp = 0

        if char == 'i':
            cmd.r_fr += 5
            if cmd.r_fr > 255:
                cmd.r_fr = 255

        if char == 'd':
            cmd.r_fr -= 5
            if cmd.r_fr < 0:
                cmd.r_fr = 0
        
        return cmd

    def print_current_cmd(self):
        """打印当前命令状态"""
        self.get_logger().info(f"""
Current Command:
r_act: {self.current_cmd.r_act}, r_gto: {self.current_cmd.r_gto}, r_pr: {self.current_cmd.r_pr}, r_sp: {self.current_cmd.r_sp}, r_fr: {self.current_cmd.r_fr}
        """)

    def get_cmd_prompt(self):
        """返回命令提示符"""
        return """
Available cmds:
a: Activate, r: Reset
c: Close, o: Open
f: Faster, l: Slower
i: Increase force, d: Decrease force
0-255: Set position
Enter cmd: """

    # def control_loop(self):
    #     """在单独线程中定期发布控制命令"""
    #     while rclpy.ok() and self.running:
    #         try:
    #             # 定期发布当前控制命令
    #             self.publisher.publish(self.current_cmd)
    #             sleep(0.1)  # 10Hz 控制频率
    #
    #         except Exception as e:
    #             self.get_logger().error(f"Control loop error: {e}")
    #
    # def destroy_node(self):
    #     """重写销毁方法，确保线程安全退出"""
    #     self.running = False
    #     if self.control_thread.is_alive():
    #         self.control_thread.join(timeout=1.0)
    #     super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
