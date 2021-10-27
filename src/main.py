# Configure access to outer directories
import sys, os
dirs = ['calibration', 'camera', 'pose']
for dirName in dirs:
    sys.path.append(os.path.abspath(os.path.join('..', dirName)))

# Import packages
import cv2 
import numpy as np
import argparse
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt

# Import modules
from calibration import parameters as param
from camera import webcam as wc
from camera import apriltag_detection as ap
from pose import localisation as loc

def update_line(hl, new_data):
	xdata, ydata, zdata = hl._verts3d
	hl.set_xdata(np.append(xdata, new_data[0]))
	hl.set_ydata(np.append(ydata, new_data[1]))
	hl.set_3d_properties(np.append(zdata, new_data[2]))
	plt.draw()

if __name__ == "__main__":

    # Calibrate camera
    _, cameraMatrix, distCoeffs, _, _ = param.get_calibration_values(2.1)

    # For parsing command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--image", required=False, 
        help = "Path to AprilTag image")
    args = vars(parser.parse_args())

    # If no image flag provided, open webcam and do live localisation
    if isinstance(args["image"],type(None)):
        
        # Open webcam for image capture
        capture = wc.open_webcam()

        # Initialise plot handler
        fig = plt.figure()
        ax = fig.add_subplot(111,projection="3d")
        ax.set_title("Camera position in global frame")
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        ax.set_zlabel("z (mm)")
        ax.set_xlim3d(-1000, 1000)
        ax.set_ylim3d(-1000, 1000)
        ax.set_zlim3d(-1000, 1000)
        xs = np.array([])
        ys = np.array([])
        hl, = plt.plot(xs,ys)

        # Run continuously 
        while True:

            # Get frame
            frame = wc.get_current_webcam_frame(capture)

            # Detect apriltags
            results = ap.detect_apriltag(frame)
            boxes, centers = ap.get_box_coords(results)
            ids = ap.get_detected_ids(results)

            # Localise if at least one aprilTag detected
            if (len(centers) >= 1):
                position, orientation = loc.results_to_global_pose(boxes, centers, ids, cameraMatrix, distCoeffs)
                print('position (xyz) (mm):')
                print(position)
                print('orientation (RPY) (rad):')
                print(orientation)

                # Plot points
                update_line(hl, np.asarray(position))
                plt.show(block=False)
                plt.pause(0.1)


            # Draw boxes
            img = ap.draw_apriltag_boxes(results, frame)
            cv2.imshow("Image", img)

            # Check if 'q' was pressed
            key = cv2.waitKey(1)
            if key == ord('q'):
                capture.release()
                break

        plt.show()

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

    cv2.destroyAllWindows()
    while True:
        pass