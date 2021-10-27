# apriltag reading
import cv2
import apriltag
import argparse

def detect_apriltag(img, silent = False):
    # Image and convert it to grayscale
    grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if silent is False:
        print("Looking for AprilTag/s from the 36h11 family...")
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(grey_img)
    if silent is False:
        print("{} AprilTags successfully detected".format(len(results)))
    return results

def get_box_coords(results):
    """Takes an apriltag detection result and returns the bounding box (corners and centre) for all visible apriltags

    Returns a tuple of boxes (numpy array of lists containing 4 points), and centres, (numpy array of points)
    """

    boxes = []
    centers = []
    for res in results:
        boxes.append(res.corners)
        centers.append(res.center)
        
    # if (boxes == [] or centers == []):
    #     print("No detection results found")

    return (boxes, centers)
    
def get_detected_ids(results):
    """Takes a detection result and returns the ids of the tags visible within the frame

    Returns:
        [list(int)]: list of the ids of the tags in detection order
    """
    ids = []
    for res in results:
        ids.append(res.tag_id)
        
    return ids

def draw_apriltag_boxes(results, img):
    # Loop through all the results
    for r in results:

        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = map(lambda x: (int(x[0]), int(x[1])), r.corners)

        # draw the bounding box of the AprilTag detection
        cv2.line(img, ptA, ptB, (0, 255, 0), 2)
        cv2.line(img, ptB, ptC, (0, 255, 0), 2)
        cv2.line(img, ptC, ptD, (0, 255, 0), 2)
        cv2.line(img, ptD, ptA, (0, 255, 0), 2)

        # colour code corners
        cv2.circle(img, ptA, 5, (255, 0, 0), -1)        # blue
        cv2.circle(img, ptB, 5, (255, 0, 255), -1)      # magenta
        cv2.circle(img, ptC, 5, (0, 255, 255), -1)      # yellow
        cv2.circle(img, ptD, 5, (255, 255, 255), -1)    # white

        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)

        # draw the tag family on the image
        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(img, tagFamily, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print("Tag family: {}".format(tagFamily))

    # return the output image after AprilTag detection
    return img

def test_apriltag_detection(args):
    
    # Image and convert it to grayscale
    print("Loading image...")
    img = cv2.imread(args["image"])

    # Detect apriltags
    results = detect_apriltag(img)

    # Draw boxes
    img = draw_apriltag_boxes(results, img)

    cv2.imshow("Image", img)

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