
import cv2
from picamera2 import Picamera2
import numpy as np
import time
PADDING=150
DIM=(940, 780)
K=np.array([[309.25607908113443, 0.0, 521.0620311621693], [0.0, 310.0674874745459, 357.45612856132215], [0.0, 0.0, 1.0]])
D=np.array([[-0.02342920854215038], [-0.01566216211458551], [0.009476349248383406], [-0.0032151564179149824]])


# Create a Picamera2 object
picam2 = Picamera2()
height = 480
width = 640
middle = (int(width / 2), int(height / 2))
picam2.configure(picam2.create_video_configuration(raw = picam2.sensor_modes[1], main={"format": 'RGB888', "size": (width, height)}))
picam2.start()
counter =0 
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
while True:
    frame = picam2.capture_array()
    frame = cv2.copyMakeBorder(frame, PADDING, PADDING, PADDING, PADDING, cv2.BORDER_CONSTANT, None)
    frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.resize(frame, (640, 480))
    # detect circles in the image
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 100)
    # ensure at least some circles were found
    if circles is not None:
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            # show the output image
    cv2.imshow("output", frame)
    cv2.waitKey(1)