# Configure access to outer directories
import sys, os
dirs = ['calibration', 'camera', 'pose']
for dirName in dirs:
    sys.path.append(os.path.abspath(os.path.join('..', dirName)))

# Import packages
import cv2 
import numpy as np
import argparse

# Import modules
from calibration import parameters as param
from camera import webcam as wc
from camera import apriltag_detection as ap
from pose import localisation as loc


if __name__ == "__main__":

    # Calibrate camera
    _, cameraMatrix, distCoeffs, _, _ = param.get_calibration_values(2.4)

    # For parsing command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--image", required=False, 
        help = "Path to AprilTag image")
    args = vars(parser.parse_args())

    # If no image flag provided, open webcam and do live localisation
    if isinstance(args["image"],type(None)):
        
        # Open webcam for image capture
        capture = wc.open_webcam()

        # Run continuously 
        while True:

            # Get frame
            frame = wc.get_current_webcam_frame(capture)

            # Detect apriltags
            results = ap.detect_apriltag(frame)
            boxes, centers = ap.get_box_coords(results)

            # Localise
            ids = ap.get_detected_ids(results)
            position, orientation = loc.results_to_global_pose(boxes, centers, ids, cameraMatrix, distCoeffs)
            print('position (xyz) (mm):')
            print(position)
            print('orientation (RPY) (rad):')
            print(orientation)

            # Draw boxes
            img = ap.draw_apriltag_boxes(results, frame)
            cv2.imshow("Image", img)

            # Check if 'q' was pressed
            key = cv2.waitKey(1)
            if key == ord('q'):
                break

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
        print('position (xyz) (mm):')
        print(position)
        print('orientation (RPY) (rad):')
        print(orientation)

        # Draw boxes
        img = ap.draw_apriltag_boxes(results, frame)
        cv2.imshow("Image", img)

        # Check if 'q' was pressed
        while True:
            key = cv2.waitKey(1)
            if key == ord('q'):
                break

    capture.release()
    cv2.destroyAllWindows()