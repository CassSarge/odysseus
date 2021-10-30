import webcam as wc, apriltag_detection as apd
import cv2

if __name__ == "__main__":
    capture = wc.open_webcam()

    while True:
        # Get frame
        frame = wc.get_current_webcam_frame(capture)

        # Detect apriltags
        results = apd.detect_apriltag(frame)

        # Draw boxes
        img = apd.draw_apriltag_boxes(results, frame)

        cv2.imshow("Image", img)

        # Check if 'q' was pressed
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    capture.release()
    cv2.destroyAllWindows()