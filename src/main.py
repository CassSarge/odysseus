# Configure access to outer directories
import sys, os
dirs = ['calibration', 'camera', 'pose']
for dirName in dirs:
    sys.path.append(os.path.abspath(os.path.join('..', dirName)))

# Import packages
import cv2 
import numpy as np

# Import modules
from calibration import parameters as param
from camera import webcam as wc
from camera import apriltag_detection as ap
from pose import localisation as loc


if __name__ == "__main__":

    # Calibrate camera
    _, cameraMatrix, distCoeffs, _, _ = param.get_calibration_values(2.4)

    # Open webcam for image capture
    capture = wc.open_webcam()

    # Run continuously 
    while True:

        # Get frame
        frame = wc.get_current_webcam_frame(capture)

        # Detect apriltags
        results = ap.detect_apriltag(frame, silent=True)

        # Draw boxes
        img = ap.draw_apriltag_boxes(results, frame)
        cv2.imshow("Image", img)

        # TODO: Get bounding box + center coordinates in image frame
        (boxes, centers) = ap.get_box_coords(results)
        tag_ids = ap.get_detected_ids(results)
        
        print(f"{boxes=}, {centers=}, {tag_ids=}")
        

        # TODO: Convert from image frame to world frame
        #START test
        objectPoints = np.array([
                                (0.0, 0.0, 0.0),
                                (0.0, 1.0, 0.0),
                                (1.0, 0.0, 0.0),
                                (1.0, 1.0, 0.0),
                                ])      # test points

        imagePoints = np.array([
                                (0.0, 0.0),
                                (0.0, 100.0),
                                (100.0, 0.0),
                                (100.0, 100.0),
                                ])      # test points

        position, orientation = loc.global_pose(objectPoints, imagePoints, cameraMatrix, distCoeffs)
        print("position")
        print(position)
        print("orientation")
        print(orientation)
        break
        #END test

        # Check if 'q' was pressed
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    capture.release()
    cv2.destroyAllWindows()