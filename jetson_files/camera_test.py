import cv2
from picamera2 import Picamera2
import numpy as np

# NO PADDING
#PADDING = 0
#DIM=(640, 480)
#K=np.array([[318.3332505641655, 0.0, 329.2007903419929], [0.0, 318.27151735619617, 220.78113709586484], [0.0, 0.0, 1.0]])
#D=np.array([[-0.02353019923313928], [0.0016070770735420305], [-0.032918542172894166], [0.025711086834002594]])

# 100PX PADDING
PADDING = 100
DIM=(840, 680)
K=np.array([[317.1032601121787, 0.0, 427.84295702569545], [0.0, 316.9109999392102, 322.1899226520426], [0.0, 0.0, 1.0]])
D=np.array([[-0.03271833759372577], [0.01650599446263598], [-0.03136567080274809], [0.013848250351526226]])

# 200PX PADDING
#PADDING = 200
#DIM=(1040, 880)
#K=np.array([[334.8173896456007, 0.0, 529.9504310867952], [0.0, 334.9688743895399, 420.6220716293999], [0.0, 0.0, 1.0]])
#D=np.array([[-0.05181635282361849], [0.03119760854031567], [0.18447540383761696], [-0.46812371141838827]])
counter = 0
def live_camera_view():
    # Create a Picamera2 object
    picam2 = Picamera2()

    # Set up video configuration
    config = picam2.create_video_configuration(main={"size": (640, 480)})
    picam2.configure(config)

    # Start the camera
    picam2.start()

    print("Live camera feed started. Press 'q' to exit.")
    counter =0 
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    try:
        while True:
            # Capture an image (in numpy array format)
            frame = picam2.capture_array()
            frame = cv2.copyMakeBorder(frame, PADDING, PADDING, PADDING, PADDING, cv2.BORDER_CONSTANT, None)
            frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            #frame = cv2.resize(frame, (1280, 720))
            #frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)


            # Convert the image to BGR format (the format used by OpenCV)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            frame_bgr = cv2.resize(frame_bgr, (640, 480))

            # Display the image on the screen
            cv2.imshow('Live Camera Feed', frame_bgr)

            key = cv2.waitKey(1) & 0xFF
            # Exit the loop when 'q' is pressed
            if key == ord('q'):
                break
            elif key == ord('s'):
                cv2.imwrite("frame%d.jpg" % counter, frame_bgr)
                counter += 1

    except KeyboardInterrupt:
        # When the user wants to exit with Ctrl+C
        print("\nProgram is terminating...")

    finally:
        # Stop the camera and close OpenCV windows
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    live_camera_view()