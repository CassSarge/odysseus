# File for opening webcam
import cv2


def open_webcam():
    """open the first webcam connected to the computer to obtain the video stream for the project

    Raises:
        IOError: if no webcam can be opened

    Returns:
        capture stream: the stream of video from which images can be obtained
    """
    print("Opening webcam, please wait...")
    capture = cv2.VideoCapture(0)
    # Ensure that the webcam was opened correctly
    if not capture.isOpened():
        raise IOError("Cannot open webcam")

    print("Webcam opened, press 'q' to quit")
    return capture


def get_current_webcam_frame(capture):
    """get the current frame from the capture stream specified

    Args:
        capture (stream): the stream from which to grab a frame

    Raises:
        IOError: if a frame cannot be read from the specified stream

    Returns:
        image: the frame that was successfully read from the stream
    """
    ret, frame = capture.read()
    # If frame was read correctly, then ret will be True
    if not ret:
        print("Cannot receive frame")
        raise IOError("Webcam frame not received")
    # Perform some operation on the frame here
    return frame


if __name__ == "__main__":

    capture = open_webcam()

    while True:
        # Get frame
        frame = get_current_webcam_frame(capture)

        # Display frame
        cv2.imshow('Webcam', frame)

        # Check if 'q' was pressed
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        if key == ord('s'):
            result = cv2.imwrite(r'screenshot.jpg', frame)
            if result == True:
                print("File saved sucessfully")
            else:
                print("Error saving file")
            pass

    capture.release()
    cv2.destroyAllWindows()
