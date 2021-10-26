from src.camera import webcam as wc, apriltag_detection as ap
import cv2
import argparse

if __name__ == "__main__":

    # For parsing command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", required=True, 
        help = "Path to AprilTag image")
    args = vars(ap.parse_args())


    # Get frame
    frame = v2.imread(args["image"])

    # Detect apriltags
    results = ap.detect_apriltag(frame)

    # Draw boxes
    img = ap.draw_apriltag_boxes(results, frame)

    cv2.imshow("Image", img)

    # Check if 'q' was pressed
    key = cv2.waitKey(1)
    if key == ord('q'):
        img = cv2.imread(args["image"])
