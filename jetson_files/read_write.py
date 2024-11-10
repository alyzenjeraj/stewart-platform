#!/usr/bin/env python
# -*- coding: utf-8 -*-

#*******************************************************************************
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#*******************************************************************************


#*******************************************************************************
#***********************     Read and Write Example      ***********************
#  Required Environment to run this example :
#    - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
#    - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
#  How to use the example :
#    - Select the DYNAMIXEL in use at the MY_DXL in the example code. 
#    - Build and Run from proper architecture subdirectory.
#    - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
#    - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
#  Author: Ryu Woon Jung (Leon)
#  Maintainer : Zerom, Will Son
# *******************************************************************************

import os
import inverse_kinematics
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import * # Uses Dynamixel SDK library

#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V

calibration_1 = 10

# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 0      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 4000000
elif MY_DXL == 'PRO_SERIES':
    ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 596
    ADDR_PRESENT_POSITION       = 611
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
    ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 564
    ADDR_PRESENT_POSITION       = 580
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'XL320':
    ADDR_TORQUE_ENABLE          = 24
    ADDR_GOAL_POSITION          = 30
    ADDR_PRESENT_POSITION       = 37
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the CW Angle Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 1023      # Refer to the CCW Angle Limit of product eManual
    BAUDRATE                    = 1000000   # Default Baudrate of XL-320 is 1Mbps

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID                      = 3

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()


# Disable Dynamixel Torque
for i in range(3):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i+1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

#wait 2 seconds for platform to come to rest
time.sleep(2)
#[1112, 1125, 1158]
motor_offsets = [0,0,0]
resting_angle = 133
for i in range(3):
    current_pos, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, i+1, ADDR_PRESENT_POSITION)
    print(current_pos)
    motor_offsets[i] = current_pos + int(inverse_kinematics.MotorOutputs.STEPS_PER_DEGREE*resting_angle)

#inverse_kinematics.MotorOutputs.update_offsets(*motor_offsets)
#print(motor_offsets)
#exit()
# Enable Dynamixel Torque
for i in range(1, 4):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print(f"Dynamixel {i} has been successfully connected")

ik = inverse_kinematics.InverseKinematics()

goal_index = 0
target_direction = 0
from inputs import get_gamepad

import threading

x = 0
y = 0
h = 0

def update_gamepad():
    global x
    global y
    global h
    while(1):
        events = get_gamepad()
        for event in events:
            if(event.ev_type == "Absolute"):
                if(event.code == "ABS_X"):
                    x = (int(event.state) - 128) / 10
                if(event.code == "ABS_Y"):
                    y = (int(event.state) - 128) / -10
                if(event.code == "ABS_GAS"):
                    h = int(event.state) / 10

t1 = threading.Thread(target=update_gamepad)

t1.start()

starting_height = -45.5
# slowly rise from resting position (-45.5, 0, 0) to (0, 0, 0) over 2 seconds
starttime = time.time()
while (time.time() - starttime) < 2:
    time_elapsed = time.time() - starttime
    height = (2 - time_elapsed)/2*starting_height
    goal_orientation = [height, 0, 0]
    arm_angles = ik.compute(*goal_orientation)
    motor_outputs = inverse_kinematics.MotorOutputs.compute_motor_outputs(*arm_angles)
    for i in range(1, 4):
        packetHandler.write4ByteTxOnly(portHandler, i, ADDR_GOAL_POSITION, motor_outputs[i-1])
    time.sleep(0.001)
    print(motor_outputs)
goal_orientation = [0, 0, 0]
a = time.time()
while 1:
    #print("Press any key to continue! (or press ESC to quit!)")
    #if getch() == chr(0x1b):
    #    break
    goal_orientation = [h,x,y]
    target_angle = 15
    print(goal_orientation)
    #goal_orientation = goal_orientations[goal_index]
    arm_angles = ik.compute(*goal_orientation)
    motor_outputs = inverse_kinematics.MotorOutputs.compute_motor_outputs(*arm_angles)
    print(motor_outputs)

    for i in range(1, 4):
        packetHandler.write4ByteTxOnly(portHandler, i, ADDR_GOAL_POSITION, motor_outputs[i-1])
    #    if dxl_comm_result != COMM_SUCCESS:
    #        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #    elif dxl_error != 0:
    #        print("%s" % packetHandler.getRxPacketError(dxl_error))
    time.sleep(0.001)
    #target_direction += 3
    #target_direction = target_direction % 360
    #goal_index += 1
    #goal_index = goal_index % len(goal_orientations)
    print(time.time() - a)
    a = time.time()


# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
