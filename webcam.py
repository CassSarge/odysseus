# File for opening webcam
import cv2

capture = cv2.VideoCapture(0)

# Ensure that the webcam was opened correctly
if not capture.isOpened():
    raise IOError("Cannot open webcam")

while True:
    ret, frame = capture.read()
    frame = cv2.resize(frame, None, fx = 0.9, fy = 0.9, interpolation = cv2.INTER_AREA)
    cv2.imshow('Webcam', frame)

    # Check if esc was pressed
    c = cv2.waitKey(1)
    if c == 27:
        break