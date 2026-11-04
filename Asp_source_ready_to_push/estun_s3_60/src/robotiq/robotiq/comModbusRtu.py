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
#
# Modifed from the orginal comModbusTcp by Kelsey Hawkins @ Georgia Tech



"""@package docstring
Module comModbusRtu: defines a class which communicates with Robotiq Grippers using the Modbus RTU protocol. 

The module depends on pymodbus (http://code.google.com/p/pymodbus/) for the Modbus RTU client.
"""
#from __future__ import print_function
from pymodbus.exceptions import ModbusException

from pymodbus.client import ModbusSerialClient

from pymodbus.payload import BinaryPayloadDecoder, BinaryPayloadBuilder
from pymodbus.constants import Endian
#from pymodbus.client import ModbusSerialClient
from math import ceil
import serial
import time

import logging
logging.getLogger("pymodbus").setLevel(logging.WARNING)

device = "/dev/ttyUSB0"

class communication:	

   def __init__(self):
      self.client = None
      
   def connectToDevice(self, device):
      """Connection to the client - the method takes the IP address (as a string, e.g. '192.168.1.11') as an argument."""
                  
      self.client = ModbusSerialClient(method='rtu',port=device,stopbits=1, bytesize=8, baudrate=115200, timeout=0.5, parity='N')
      
      # 启用详细日志（关键调试步骤）
      logging.basicConfig(
          format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
          level=logging.INFO  # 默认级别为INFO，需要调试时改为DEBUG
      )
      self.logger = logging.getLogger(__name__)
      
      
   
      if not self.client.connect():
          print("Unable to connect to {}".format(device))
          
          return False
      else:
      
          builder = BinaryPayloadBuilder(
            byteorder=Endian.BIG,
            wordorder=Endian.BIG
          )
          builder.add_16bit_uint(0x0000)# rACT先置0
          payload = builder.to_registers()
          write_res = self.client.write_registers(
            address=1000,     # 控制寄存器地址
            values=payload,
            slave=9
          )
          print("reset")
          time.sleep(0.5)
          builder.add_16bit_uint(0x0100)# rACT置1
          payload = builder.to_registers()
          write_res = self.client.write_registers(
            address=1000,     # 控制寄存器地址
            values=payload,
            slave=9
          )
          print("set")
          
          response = self.client.read_holding_registers(address=0, count=1, slave=9)
          status_register = response.registers[0]
          gSTA = (status_register & 0x0600) >> 9  # 提取 gSTA 位
          
          move_builder = BinaryPayloadBuilder(byteorder=Endian.BIG, wordorder=Endian.BIG)
          move_builder.add_16bit_uint(0x0900)  # 立即执行命令0x0900
          move_builder.add_16bit_uint(0)     # 位置
          move_builder.add_16bit_uint(65304)     # 速度
          self.client.write_registers(address=1000, values=move_builder.to_registers(), slave=9)
          time.sleep(1)
         
      
          return True
      
      

   def disconnectFromDevice(self):
      """Close connection"""
      self.client.close()

   def sendCommand(self, data):   
      """Send a command to the Gripper - the method takes a list of uint8 as an argument. The meaning of each variable depends on the Gripper model (see support.robotiq.com for more details)"""
      

      #make s ure data has an even number of elements   
      if(len(data) % 2 == 1):
         data.append(0)

      #Initiate message as an empty list
      message = []

      #Fill message by combining two bytes in one register

      for i in range(0, len(data)//2):
         message.append((data[2*i] << 8) + data[2*i+1])
      #print(data)
      
      
      
      #builder.add_16bit_uint(0x0100)  # 激活命令
      #payload = builder.to_registers()
      
      #write_res = self.client.write_registers(
        #address = 1000,     # 控制寄存器地址
        #values = 0x0000,
        #lave = 9
      #)
      #time.sleep(0.02)
      
      # 验证激活状态
      #response = self.client.read_holding_registers(address=2000, count=1, slave=9)
      #if response.isError():
          #self.logger.error(f"读取状态错误: {response}")
          #return False
      #status = response.registers[0]
      #if status & 0x0100:  # 检查激活位
          #self.logger.info("夹爪激活成功")

      if not data:
        return False
      else:
        #if self.client.connect():
          move_builder = BinaryPayloadBuilder(byteorder=Endian.BIG, wordorder=Endian.BIG)
          move_builder.add_16bit_uint(0x0900)  # 立即执行命令
          move_builder.add_16bit_uint(data[3])     # 位置data[3]
          move_builder.add_16bit_uint((data[4] << 8) | data[5])     # 速度(data[4] << 8) | data[5]
          message = move_builder.to_registers()
          self.client.write_registers(address=1000, values = move_builder.to_registers(), slave=9)
          #print(move_builder.to_registers())

          return True
          

   def getStatus(self, numBytes):
      """Sends a request to read, wait for the response and returns the Gripper status. The method gets the number of bytes to read as an argument"""
      numRegs = int(ceil(numBytes/2.0))

      #To do!: Implement try/except 
      #Get status from the device
      
      # 启用详细日志（关键调试步骤）
      logging.basicConfig()
      log = logging.getLogger()
      log.setLevel(logging.DEBUG)

      
      try:
        response = self.client.read_holding_registers(2000, numRegs, slave=9)
        #response = self.client.read_holding_registers(0x1F4, numRegs, unit=0x0009)
        
        
        # 检查是否错误响应
        if hasattr(response, 'isError') and response.isError():
            raise ModbusException(f"Modbus error: {response}")
            
        # 确保有registers属性
        if not hasattr(response, 'registers'):
            raise ModbusException("Invalid response format")
            
      except ModbusException as e:
          print(f"Error reading status: {str(e)}")
          # 返回全0状态表示错误
          return [0] * numBytes

      #Instantiate output as an empty list
      output = []

      #Fill the output with the bytes in the appropriate order
      for i in range(0, numRegs):
         output.append((response.registers[i] & 0xFF00) >> 8)
         output.append( response.registers[i] & 0x00FF)
         
      #Output the result
      return output
