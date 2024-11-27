
import os
import inverse_kinematics
import time
import math

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
portHandler = PortHandler(DEVICENAME)
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
#from inputs import get_gamepad

import cv2 as cv
import numpy as np
from time import sleep
from threading import Thread, Lock
import pid2
import pid

x = 0
y = 0
h = 0


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
    # print(motor_outputs)
goal_orientation = [0, 0, 0]
a = time.time()


x_coord, y_coord = 0, 0



import cv2 as cv
import numpy as np
from time import sleep
from threading import Thread, Lock
from picamera2 import Picamera2
# '***' stands for things to modify for your own webcam, display, and ball if needed

coordinates = (None, None)
lock = Lock()


# 100PX PADDING
PADDING=150
DIM=(940, 780)
K=np.array([[309.25607908113443, 0.0, 521.0620311621693], [0.0, 310.0674874745459, 357.45612856132215], [0.0, 0.0, 1.0]])
D=np.array([[-0.02342920854215038], [-0.01566216211458551], [0.009476349248383406], [-0.0032151564179149824]])

frame_width = 640
frame_height = 480

imgMASK = None
imageHSV = None
lower = np.array([0, 0, 0])
upper = np.array([255, 255, 255])
def on_trackbar(val):
    global lower
    global upper
    hue_min = cv.getTrackbarPos("Hue Min", "TrackedBars")
    hue_max = cv.getTrackbarPos("Hue Max", "TrackedBars")
    sat_min = cv.getTrackbarPos("Sat Min", "TrackedBars")
    sat_max = cv.getTrackbarPos("Sat Max", "TrackedBars")
    val_min = cv.getTrackbarPos("Val Min", "TrackedBars")
    val_max = cv.getTrackbarPos("Val Max", "TrackedBars")

    lower = np.array([hue_min, sat_min, val_min])
    upper = np.array([hue_max, sat_max, val_max])


cv.namedWindow("TrackedBars")
cv.resizeWindow("TrackedBars", 640, 240)

cv.createTrackbar("Hue Min", "TrackedBars", 0, 179, on_trackbar)
cv.createTrackbar("Hue Max", "TrackedBars", 179, 179, on_trackbar)
cv.createTrackbar("Sat Min", "TrackedBars", 189, 255, on_trackbar)
cv.createTrackbar("Sat Max", "TrackedBars", 255, 255, on_trackbar)
cv.createTrackbar("Val Min", "TrackedBars", 138, 255, on_trackbar)
cv.createTrackbar("Val Max", "TrackedBars", 192, 255, on_trackbar)

# Start capturing video from the webcam. If multiple webcams connected, you may use 1,2, etc.
picam2 = Picamera2()
picam2.video_configuration.controls.FrameRate = 120.0
height = 480
width = 640
picam2.configure(picam2.create_video_configuration(raw = picam2.sensor_modes[1], main={"format": 'RGB888', "size": (width, height)}))
# picam2.set_controls({"FrameRate": 206.0})
picam2.start()
# *1 CAP_PROP_FPS sets the frame rate of the webcam to 30 fps here
map1, map2 = cv.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv.CV_16SC2)

starttime = time.time()
t = 0
tc = 0

ANGLE_PER_PIXEL = (49/(height/2-40))*2

platform_center = ((width//2 + 27)//2, (height//2 - 20)//2)

x_angle = 0
y_angle = 0

def get_ball_position(ball_angle, platform_angle = 0):
    ball_angle = ball_angle/180*math.pi
    platform_angle = (90-platform_angle)/180*math.pi

    third_angle = math.pi - ball_angle - platform_angle
    ball_pos = (85+h)/math.sin(third_angle)*math.sin(ball_angle)
    return ball_pos*1.2


pid_x = pid.PID(kp=0.03, ki=0.0000004, kd=0.001, setpoint=0, output_limits=(-20,20))
pid_y = pid.PID(kp=0.03, ki=0.0000004, kd=0.001, setpoint=0, output_limits=(-20,20))

pid_pos_x = pid.PID(kp=3, ki=0.00000000004, kd=0.1, setpoint=0, output_limits=(-500,500))
pid_pos_y = pid.PID(kp=3, ki=0.00000000004, kd=0.1, setpoint=0, output_limits=(-500,500))

x_calibrated = 0
y_calibrated = 0

pos_smooth = [0,0]
vel_smooth = [0,0]
pos_weight = 0.6
vel_weight = 0.6

PATH = True

counter = 0
ball_height = 0

prev_output = [0,0]

output_slew_rate = 7500 # degrees per second
while 1:
    if (time.time() - t > 1):
        print(f"{tc} fps")
        t = time.time()
        tc = 0
    tc += 1

    counter += 0.02
    if PATH:
        pid_pos_x.setpoint = 45*math.sin(counter)
        pid_pos_x.setpoint = 45*math.cos(counter)


    # Read a frame from the webcam
    frame = picam2.capture_array()
    frame = cv.copyMakeBorder(frame, PADDING, PADDING, PADDING, PADDING, cv.BORDER_CONSTANT, None)
    frame = cv.remap(frame, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)

    frame = cv.resize(frame, (320, 240))
    frame = cv.GaussianBlur(frame, (3, 3), 0)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 
    ball_color_lower = np.array([20, 100, 100]) # [lower Hue, lower Saturation, lower Value]
    ball_color_upper = np.array([30, 255, 255]) # [upper Hue, upper Saturation, upper Value]
    mask = cv.inRange(hsv, lower, upper)
    cv.imshow("mask", mask)
    contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv.contourArea)
        ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
        if radius > 3:

            x_coord = (int(x) - platform_center[0])
            y_coord = -(platform_center[1] - int(y))

            x_angle = x_coord * ANGLE_PER_PIXEL
            y_angle = y_coord * ANGLE_PER_PIXEL

            cv.circle(frame, (int(x_coord + platform_center[0]), int(y_coord + platform_center[1])), 2, (0, 0, 255), -1)

            x_calibrated = get_ball_position(x_angle, goal_orientation[1])
            y_calibrated = get_ball_position(y_angle, goal_orientation[2])

            ball_height = -math.sin(goal_orientation[1]/180*math.pi)*x_calibrated - math.sin(goal_orientation[2]/180*math.pi)*y_calibrated
            prev_pos = [pos_smooth[0], pos_smooth[1]]

            pos_smooth[0] = (1-pos_weight)*pos_smooth[0] + pos_weight*(x_calibrated)
            pos_smooth[1] = (1-pos_weight)*pos_smooth[1] + pos_weight*(y_calibrated)

            vel_smooth[0] = (1-vel_weight)*vel_smooth[0] + vel_weight*(pos_smooth[0]-prev_pos[0])/0.033
            vel_smooth[1] = (1-vel_weight)*vel_smooth[1] + vel_weight*(pos_smooth[1]-prev_pos[1])/0.033

    x_speed_command = pid_pos_x.compute(pos_smooth[0])
    y_speed_command = pid_pos_y.compute(pos_smooth[1])

    print(x_speed_command, y_speed_command)

    pid_x.setpoint = x_speed_command
    pid_y.setpoint = y_speed_command

    x_output = pid_x.compute(vel_smooth[0])
    y_output = pid_y.compute(vel_smooth[1])

    x_output = max(prev_output[0] - output_slew_rate*0.033, x_output)
    x_output = min(prev_output[0] + output_slew_rate*0.033, x_output)

    y_output = max(prev_output[1] - output_slew_rate*0.033, y_output)
    y_output = min(prev_output[1] + output_slew_rate*0.033, y_output)

    prev_output = [x_output, y_output]

    #x_output = 15
    #y_output = 15

    h_compensated = h - ball_height
    h_compensated = max(-25, h_compensated)
    h_compensated = min(25, h_compensated)
    goal_orientation = [h_compensated,x_output, y_output]

    arm_angles = ik.compute(*goal_orientation)
    motor_outputs = inverse_kinematics.MotorOutputs.compute_motor_outputs(*arm_angles)

    for i in range(1, 4):
        packetHandler.write4ByteTxOnly(portHandler, i, ADDR_GOAL_POSITION, motor_outputs[i-1])

    cv.circle(frame, platform_center, 2, (0, 0, 255), -1)
    cv.circle(frame, (platform_center[0], height-60), 2, (0, 0, 255), -1)
    cv.circle(frame, (int(pid_x.setpoint + platform_center[0]), int(pid_y.setpoint + platform_center[1])), 10, (255, 255, 255), 2)
    cv.imshow('frame', frame)
    cv.pollKey()
    a = time.time()


# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
