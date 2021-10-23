# File for opening webcam
import cv2
# import apriltag

def open_webcam():
    print("Opening webcam, please wait...")
    capture = cv2.VideoCapture(0)
    # Ensure that the webcam was opened correctly
    if not capture.isOpened():
        raise IOError("Cannot open webcam")

    print("Webcam opened, press 'q' to quit")
    return capture

def get_current_webcam_frame():
    ret, frame = capture.read()
    # If frame was read correctly, then ret will be True
    if not ret:
        print("Cannot receive frame")
        raise IOError("Webcam frame not received")
    # Perform some operation on the frame here
    # Resize the frame
    frame = cv2.resize(frame, None, fx = 0.9, fy = 0.9, interpolation = cv2.INTER_AREA)
    return frame

if __name__ == "__main__":
    
    capture =  open_webcam()

    while True:
        # Get frame
        frame = get_current_webcam_frame()

        # Display frame
        cv2.imshow('Webcam', frame)

        # Check if 'q' was pressed
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    capture.release()
    cv2.destroyAllWindows()
