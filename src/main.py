# Configure access to outer directories
import sys, os
dirs = ['calibration', 'camera', 'pose', 'imu']
for dirName in dirs:
    sys.path.append(os.path.abspath(os.path.join('..', dirName)))

# Import packages
import cv2 
import numpy as np
import argparse
import time
import math
from operator import sub

# Import modules
from calibration import parameters as param
from camera import webcam as wc
from camera import apriltag_detection as ap
from pose import localisation as loc
from pose import plot
from imu import imu_tracking as tc
from visualisation import camera_pose_visualizer as cpv

# Global variables
global position, offset

# Helper functions
def subtract_tuple(tup1, tup2):
    return tuple(map(sub, tup1, tup2))

def multiply_tuple(tup, scalar):
    return tuple([scalar*x for x in tup])

def rpy_to_pyramid(rpy):
    pyramid = rpy
    pyramid[1] -= math.pi/2.0
    pyramid[2] = -pyramid[2] + math.pi
    return pyramid

# Main
if __name__ == "__main__":

    global offset
    offset = (0, 0, 0)
    
    # For parsing command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--image", required=False, 
        help = "Path to AprilTag image")
    parser.add_argument("-u", "--user", required=True, help = "User folder to search for calibration data")
    parser.add_argument("-c", "--calibrate", help = "Use calibration images instead of stored matrices", action="store_true")
    args = vars(parser.parse_args())
    
    # User variable is always going to be something
    user = args["user"]
    user_calibration_path = f"calibration/{user}/*.jpg"
    user_calibration_file = f"calibration/{user}/params"
    
    if args["calibrate"] is True:
        # Calibrate camera
        # TODO make the 2.1 a variable based on who is using the script
        side_len = float(input("How large is the checkerboard used in the calibration photos?"))
        _, cameraMatrix, distCoeffs, _, _ = param.get_calibration_values(side_len, user_calibration_path)
        param.write_parameter_files(user_calibration_file, cameraMatrix, distCoeffs)

    else:
        (cameraMatrix, distCoeffs) = param.read_values_from_file(user_calibration_file)
        # print(f"{cameraMatrix=}, {distCoeffs=}")

    # Set up camera pose visulariser
    visualizer = cpv.CameraPoseVisualizer([-100, 3000], [-100, 3000], [-100, 3000])

    # If no image flag provided, open webcam and do live localisation
    if isinstance(args["image"],type(None)):
        
        # Start tracking camera
        tracking_cam = tc.TrackingCam()
        tracking_cam.start_stream()
        time.sleep(2)   # This sleep is required for receiving data
        
        # Open webcam for image capture
        capture = wc.open_webcam()

        # Initialise plot handle
        hl = plot.init_position_figure()

        # Run continuously 
        while True:

            # Get frame
            frame = wc.get_current_webcam_frame(capture)

            # Detect apriltags
            results = ap.detect_apriltag(frame, silent=True)
            boxes, centers = ap.get_box_coords(results)
            ids = ap.get_detected_ids(results)

            # Localise if at least one aprilTag detected
            if (len(centers) >= 1):

                print("Using AprilTags")
                position, orientation = loc.results_to_global_pose(boxes, centers, ids, cameraMatrix, distCoeffs)

                # Update global variable for position according to imu
                imu_position = multiply_tuple(tracking_cam.receive_data(["POSITION"], turn_off=False)[0][0], 1000)

                # Find the offset between the last known Apriltag-deduced position and the imu_position
                offset = subtract_tuple(imu_position, position)

            # Otherwise, deadreckon using IMU
            else:
                print("Using IMU")
                imu_position = multiply_tuple(tracking_cam.receive_data(["POSITION"], turn_off=False)[0][0], 1000)
                position = subtract_tuple(imu_position, offset)
                orientation = tracking_cam.pose_to_rpy()

            # Plot points
            plot.update_line(hl, np.asarray(position))

            # Rotation matrix
            pyramid_orientation = rpy_to_pyramid(np.array(orientation))
            R = loc.euler_zyx_to_rotm(pyramid_orientation)

            # Translation vector
            t = (np.array([position]).T).reshape((3,1))
            temp_matrix = np.hstack((R,t))

            # Extrinsic matrix 
            extrinsic_matrix = np.vstack((temp_matrix, np.array([0,0,0,1])))
            visualizer.extrinsic2pyramid(extrinsic_matrix, 'c', 400)
            visualizer.show()

            # Draw boxes
            img = ap.draw_apriltag_boxes(results, frame)
            cv2.imshow("Image", img)

            # Check if 'q' was pressed
            key = cv2.waitKey(1)
            if key == ord('q'):
                quit()

    # Else if an image is provided, run localisation on it     
    else:

        # Get frame
        frame = cv2.imread(args["image"])

        # Detect apriltags
        results = ap.detect_apriltag(frame)
        boxes, centers = ap.get_box_coords(results)

        # Localise
        ids = ap.get_detected_ids(results)
        position, orientation = loc.results_to_global_pose(boxes, centers, ids, cameraMatrix, distCoeffs)

        # Draw boxes
        img = ap.draw_apriltag_boxes(results, frame)
        cv2.imshow("Image", img)

        # Check if 'q' was pressed
        while True:
            key = cv2.waitKey(1)
            if key == ord('q'):
                quit()

    capture.release()
    cv2.destroyAllWindows()
