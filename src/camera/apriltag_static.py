import webcam as wc, apriltag_detection as apd
import cv2
import argparse

# Testing file for testing other files functions with a still image, feel free to add anything
# Usage running from the project root folder would be
# python src/camera/apriltag_static.py --image src/camera/images/example_img1.png
if __name__ == "__main__":

    # For parsing command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", required=True, 
        help = "Path to AprilTag image")
    args = vars(ap.parse_args())

    # Get frame
    frame = cv2.imread(args["image"])

    # Detect apriltags
    results = apd.detect_apriltag(frame)

    # Draw boxes
    img = apd.draw_apriltag_boxes(results, frame)

    cv2.imshow("Image", img)

    # Wait for a key to be pressed
    key = cv2.waitKey(0)
