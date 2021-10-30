import numpy as np
import cv2
import glob


def get_calibration_values(square_size_cm, image_dir_path="calibration/images/*.jpg", internal_cols=8, internal_rows=6):

    # Termination criteria for refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points in global reference frame
    objp = np.zeros((internal_rows*internal_cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:internal_cols, 0:internal_rows].T.reshape(-1, 2)
    objp *= square_size_cm

    # Arrays to store object points and image points from all the calibration images
    objpoints = [] # 3d points in real world space
    imgpoints = [] # 2d points in image plane

    # Get filenames of calibration images
    # TODO: replace the placeholder image with the actual images
    images = glob.glob(image_dir_path)

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, (internal_cols, internal_rows), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners overlayed on the calibration image
            img = cv2.drawChessboardCorners(img, (internal_cols, internal_rows), corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()

    # Get calibration values from matched object and image points
    retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print("Camera calibration complete.")
    print(f'Camera matrix:\n{cameraMatrix}')
    print(f'Distortion coefficients:\n{distCoeffs}')
    print(f'Rotation vector:\n{rvecs}')
    print(f'Translation vector:\n{tvecs}')

    return retval, cameraMatrix, distCoeffs, rvecs, tvecs
    
def write_parameter_files(filename, cam, dist):
    print(cam.dtype)
    np.save(filename+"_cam.npy", cam)
    np.save(filename+"_dist.npy", dist)
    
def read_values_from_file(filename):
    camera_Matrix = np.load(filename+"_cam.npy")
    distCoeffs = np.load(filename+"_dist.npy")
    return (camera_Matrix, distCoeffs)

if __name__ == "__main__":

    # Must be running at src level, otherwise image directory path should be specified
    retval, cameraMatrix, distCoeffs, rvecs, tvecs = get_calibration_values(2.4)
