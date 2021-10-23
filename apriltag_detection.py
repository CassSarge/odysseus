# apriltag reading
import cv2
import apriltag
import argparse

def detect_apriltag(img):
    # Image and convert it to grayscale
    grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print("Looking for AprilTag/s from the 36h11 family...")
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(grey_img)
    print("{} AprilTags successfully detected".format(len(results)))
    return results

def draw_apriltag_boxes(results, img):
    # Loop through all the results
    for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(img, ptA, ptB, (0, 255, 0), 2)
        cv2.line(img, ptB, ptC, (0, 255, 0), 2)
        cv2.line(img, ptC, ptD, (0, 255, 0), 2)
        cv2.line(img, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(img, tagFamily, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print("Tag family: {}".format(tagFamily))
    # show the output image after AprilTag detection
    cv2.imshow("Image", img)

def test_apriltag_detection(args):
    
    # Image and convert it to grayscale
    print("Loading image...")
    img = cv2.imread(args["image"])

    # Detect apriltags
    results = detect_apriltag(img)

    # Draw boxes
    draw_apriltag_boxes(results, img)

    # Wait for user to press a key
    cv2.waitKey(0)

if __name__ == "__main__":
    # Example input: python apriltag_detection.py --image images/example_img1.png

    # For parsing command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", required=True, 
        help = "Path to AprilTag image")
    args = vars(ap.parse_args())
    test_apriltag_detection(args)