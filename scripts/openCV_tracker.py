import cv2
import grpc
import sys
import os
import subprocess
import traceback

# Dynamically add the `proto` directory to the sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'proto')))

from frame import frame_pb2
from frame import frame_pb2_grpc


def generate_frames():
    # Capture video from the webcam
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        returnz

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Display the frame in a window before sending it via gRPC
            cv2.imshow('Webcam Frame', frame)

            # Wait for 1 ms between frames (adjust as needed)
            # Press 'q' to quit the video feed early
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Encode the frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame)

            # Yield the frame as a gRPC Frame message
            yield frame_pb2.Frame(data=buffer.tobytes())
    finally:
        cap.release()
        cv2.destroyAllWindows()

def run():
    try:
        # Connect to the gRPC server
        channel = grpc.insecure_channel('localhost:5051')
        stub = frame_pb2_grpc.FrameServiceStub(channel)

        # Stream frames to the server and print acknowledgments
        responses = stub.StreamFrames(generate_frames())
        for response in responses:
            print("Server acknowledgment:", response.message)
    except grpc.RpcError as e:
        print(f"gRPC error: {e}")

if __name__ == '__main__':
    run()
