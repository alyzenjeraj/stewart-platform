import cv2 as cv
import numpy as np
from time import sleep
import time
from threading import Thread, Lock
from picamera2 import Picamera2
# '***' stands for things to modify for your own webcam, display, and ball if needed

coordinates = (None, None)
lock = Lock()


# 100PX PADDING
PADDING = 100
DIM=(840, 680)
K=np.array([[317.1032601121787, 0.0, 427.84295702569545], [0.0, 316.9109999392102, 322.1899226520426], [0.0, 0.0, 1.0]])
D=np.array([[-0.03271833759372577], [0.01650599446263598], [-0.03136567080274809], [0.013848250351526226]])


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
cv.createTrackbar("Hue Max", "TrackedBars", 46, 179, on_trackbar)
cv.createTrackbar("Sat Min", "TrackedBars", 9, 255, on_trackbar)
cv.createTrackbar("Sat Max", "TrackedBars", 255, 255, on_trackbar)
cv.createTrackbar("Val Min", "TrackedBars", 207, 255, on_trackbar)
cv.createTrackbar("Val Max", "TrackedBars", 255, 255, on_trackbar)

# Define a function to detect a yellow ball
def detect_yellow_ball():
    # Start capturing video from the webcam. If multiple webcams connected, you may use 1,2, etc.
    picam2 = Picamera2()
    # Set up video configuration
    config = picam2.create_video_configuration(main={"size": (640, 480)})
    picam2.video_configuration.controls.FrameRate = 90.0
    picam2.configure(config)

    # Start the camera
    picam2.start()
    # *1 CAP_PROP_FPS sets the frame rate of the webcam to 30 fps here
    map1, map2 = cv.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv.CV_16SC2)
    t = time.time()
    while True:
        print(time.time() - t)
        t = time.time()
        # Read a frame from the webcam
        frame = picam2.capture_array()
        frame = cv.copyMakeBorder(frame, PADDING, PADDING, PADDING, PADDING, cv.BORDER_CONSTANT, None)
        frame = cv.remap(frame, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)
        continue
        # *2 Set the image resolution to 480x480. Note increasing resolution increases processing power used, and may slow down video feed.
        frame = cv.resize(frame, (frame_width, frame_height))
        frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
        # Convert the frame from BGR to HSV color space to easily identify a colour
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 

        # *3 Define the range of yellow color in HSV [Hue, Saturation, Value]
        # SET THESE VALUES VIA THE METHOD EXPLAINED IN THE TUTORIAL
        ball_color_lower = np.array([20, 100, 100]) # [lower Hue, lower Saturation, lower Value]
        ball_color_upper = np.array([30, 255, 255]) # [upper Hue, upper Saturation, upper Value]

        # Threshold the HSV image to get the colors defined above
        # Pixels in the range are set to white (255) and those that aren't are set to black (0), creating a binary mask 
        #mask = cv.inRange(hsv, ball_color_lower, ball_color_upper)
        
        mask = cv.inRange(hsv, lower, upper)
        cv.imshow("mask", mask)
        # Find contours in the mask
        # RETR_TREE retrieves all hierarchical contours and organizes them
        # CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments, leaving only their end points
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Find the index of the largest contour
        if contours:
            # Determines the larget contour size using the cv.contour Area function
            largest_contour = max(contours, key=cv.contourArea)
            # Computes the minimum enclosing circle aroudn the largest contour
            ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
            # * 4 Only consider large enough objects. If it only detects a small portion of your ball, you can test higher radius values to capture more of the ball
            if radius > 10:
                # Draw a yellow circle around the ball
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # Draw a red dot in the center of the ball
                cv.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)  # (image to draw dot on, x,y pixel coordinates, radius in pixels, RGB values in this case red, -1 indicates to fill the circle)
                # Display the position of the ball

                x_from_center = int(x) - frame_width//2
                y_from_center = frame_height//2 - int(y)

                print(f"Yellow ball detected at position: ({int(x)}, {int(y)})")

                print(x_from_center, y_from_center)

                

                with lock:
                    coordinates = (x_from_center, y_from_center)

        # Display the resulting frame
        cv.imshow('frame', frame)

        # Break the loop when 'q' is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture when everything is done
    cap.release()
    # Close all windows
    cv.destroyAllWindows()

# Call the function to detect the yellow ball
detect_yellow_ball()
