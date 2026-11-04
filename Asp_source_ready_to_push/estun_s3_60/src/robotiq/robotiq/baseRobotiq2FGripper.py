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
Module baseRobotiq2FGripper: defines a base class for handling command and status of the Robotiq 2F gripper.

After being instanciated, a 'client' member must be added to the object. This client depends on the communication protocol used by the Gripper. As an example, the ROS node 'Robotiq2FGripperTcpNode.py' instanciate a robotiqbaseRobotiq2FGripper and adds a client defined in the module comModbusTcp.
"""

#from   robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
#from   robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

from robotiq_interfaces.msg import GripperInput, GripperOutput

class robotiqbaseRobotiq2FGripper:
    """Base class (communication protocol agnostic) for sending commands and receiving the status of the Robotic 2F gripper"""

    def __init__(self):

        #Initiate output message as an empty list
        self.message = []

        #Note: after the instantiation, a ".client" member must be added to the object

    def verifyCommand(self, command):
        """Function to verify that the value of each variable satisfy its limits."""

        #Verify that each variable is in its correct range
        command.r_act = max(0, command.r_act)
        command.r_act = min(1, command.r_act)

        command.r_gto = max(0, command.r_gto)
        command.r_gto = min(1, command.r_gto)

        command.r_atr = max(0, command.r_atr)
        command.r_atr = min(1, command.r_atr)

        command.r_pr  = max(0,   command.r_pr)
        command.r_pr  = min(255, command.r_pr)

        command.r_sp  = max(0,   command.r_sp)
        command.r_sp  = min(255, command.r_sp)

        command.r_fr  = max(0,   command.r_fr)
        command.r_fr  = min(255, command.r_fr)

        #Return the modified command
        return command
        
    def refreshCommand(self, command):
        """Function to update the command which will be sent during the next sendCommand() call."""

        #Limit the value of each variable
        command = self.verifyCommand(command)

        #Initiate command as an empty list
        self.message = []

        #Build the command with each output variable
        #To-Do: add verification that all variables are in their authorized range
        self.message.append(command.r_act + (command.r_gto << 3) + (command.r_atr << 4))
        self.message.append(0)
        self.message.append(0)
        self.message.append(command.r_pr)
        self.message.append(command.r_sp)
        self.message.append(command.r_fr)


    def sendCommand(self):
        """Send the command to the Gripper."""
        self.client.sendCommand(self.message)

    def getStatus(self):
        """Request the status from the gripper and return it in the Robotiq2FGripper_robot_input msg type."""

        #Acquire status from the Gripper
        status = self.client.getStatus(6);

        #Message to output
        message = GripperInput()

        #Assign the values to their respective variables
        message.g_act = (status[0] >> 0) & 0x01;
        message.g_gto = (status[0] >> 3) & 0x01;
        message.g_sta = (status[0] >> 4) & 0x03;
        message.g_obj = (status[0] >> 6) & 0x03;
        message.g_flt =  status[2]
        message.g_pr  =  status[3]
        message.g_po  =  status[4]
        message.g_cu  =  status[5]
        
        return message

