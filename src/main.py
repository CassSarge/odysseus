# Configure access to outer directories
import sys, os
sys.path.append(os.path.abspath(os.path.join('..', 'camera')))
sys.path.append(os.path.abspath(os.path.join('..', 'pose')))

# Import packages
import cv2 

# Import modules
from camera import webcam as wc
from camera import apriltag_detection as ap
from pose import localisation as loc

if __name__ == "__main__":

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

        # Check if 'q' was pressed
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    capture.release()
    cv2.destroyAllWindows()