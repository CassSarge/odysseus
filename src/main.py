# Configure access to outer directories
import sys, os
dirs = ['calibration', 'camera', 'pose']
for dirName in dirs:
    sys.path.append(os.path.abspath(os.path.join('..', dirName)))

# Import packages
import cv2 

# Import modules
from calibration import parameters as param
from camera import webcam as wc
from camera import apriltag_detection as ap
from pose import localisation as loc


if __name__ == "__main__":

    # Calibrate camera
    _, cameraMatrix, distCoeffs, _, _ = param.get_calibration_values()

    # Open webcam for image capture
    capture = wc.open_webcam()

    # Run continuously 
    while True:

        # Get frame
        frame = wc.get_current_webcam_frame(capture)

        # Detect apriltags
        results = ap.detect_apriltag(frame)

        # Draw boxes
        img = ap.draw_apriltag_boxes(results, frame)
        cv2.imshow("Image", img)

        # TODO: Get bounding box + center coordinates in image frame

        # TODO: Convert from image frame to world frame

        # Check if 'q' was pressed
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    capture.release()
    cv2.destroyAllWindows()