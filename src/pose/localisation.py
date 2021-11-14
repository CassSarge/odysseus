# Import packages
import cv2
import math
import numpy as np

# Import modules
from camera import apriltag_detection as ap


def results_to_global_pose(boxes, centers, ids, cameraMatrix, distCoeffs):
    """Converts the bounding boxes and centers for the given tag id detections to positions in the global frame using a pnp solver
    Calculates this using the distortion matrix and coefficients given from the calibration module

    Args:
        boxes (list(float)): bounding boxes for all detections
        centers (list(float)): center of tag for all detections
        ids (list(float)): ids for tag detections
        cameraMatrix (3x3 matrix): camera matrix for distortion
        distCoeffs (1x4 array): distortion coefficients from the calibration module

    Returns:
        tuple(tuple(3), tuple(3)) : the corresponding position and orientation from the detections
    """

    # Construct a numpy array of image points
    imagePoints = []
    objectPoints = []

    # find the centers and corners for every tag that has been detected
    for tag in range(len(centers)):
        imagePoints.append(centers[tag])
        for corner in boxes[tag]:
            imagePoints.append(corner)

        # lookup the atlas file for the detected tag
        # note that this will break if a tag without a landmark file is detected
        atlas_name = f"pose/atlas/{ids[tag]}.lmk"
        (side_len, pose, orientation) = parse_landmark_file(atlas_name)

        # add the object points - which are the tag corners and center, to the array to be given to the PnP solver
        objectPoints.extend(tag_pose_to_object_points(
            pose, orientation, side_len))

    imagePoints = np.array(imagePoints)
    objectPoints = np.array(objectPoints)

    position, orientation = points_to_global_pose(
        objectPoints, imagePoints, cameraMatrix, distCoeffs)

    return position, orientation


def points_to_global_pose(objectPoints, imagePoints, cameraMatrix, distCoeffs):
    """convert the object points array to an array of points in the global frame

    Args:
        objectPoints (list): points of the detections in the object frame
        imagePoints (list): points of the detections in the image frame
        cameraMatrix (3x3 matrix): camera matrix from calibration
        distCoeffs (4x1 array): disortion coefficicents from calibration modul

    Returns:
        tuple(tuple(3), tuple(3)) : the corresponding position and orientation from the detections
    """
    _, rVec, tVec = cv2.solvePnP(
        objectPoints, imagePoints, cameraMatrix, distCoeffs, flags=cv2.SOLVEPNP_SQPNP)
    rotm_t = cv2.Rodrigues(rVec)[0]
    rotm = np.array(rotm_t).T
    position = np.matmul(-rotm, np.array(tVec))
    orientation = rotm_to_euler_zyx(rotm)

    return position, orientation


def tag_pose_to_object_points(pose, orientation, side_length):
    """convert a tag center position and orientation to an array of points defining the global position of the tag corners and center in the global frame

    Args:
        pose (tuple): position in the tag frame of the centre of the tag
        orientation (tuple): orientation in yaw pitch roll euler angles of the tage
        side_length (float): length of the sides of the tags (mm)

    Returns:
        list: points in the global frame corresponding to the tag position
    """

    center = np.array(pose)
    rotm_tag_to_global = euler_zyx_to_rotm(np.array(orientation))

    corner_top_left_tag_frame = np.array([0, side_length/2, side_length/2]).T
    corner_top_right_tag_frame = np.array([0, -side_length/2, side_length/2]).T
    corner_bottom_right_tag_frame = np.array(
        [0, -side_length/2, -side_length/2]).T
    corner_bottom_left_tag_frame = np.array(
        [0, side_length/2, -side_length/2]).T

    corner_top_left_global_frame = (
        np.matmul(rotm_tag_to_global, corner_top_left_tag_frame)+center).tolist()
    corner_top_right_global_frame = (
        np.matmul(rotm_tag_to_global, corner_top_right_tag_frame)+center).tolist()
    corner_bottom_right_global_frame = (
        np.matmul(rotm_tag_to_global, corner_bottom_right_tag_frame)+center).tolist()
    corner_bottom_left_global_frame = (
        np.matmul(rotm_tag_to_global, corner_bottom_left_tag_frame)+center).tolist()
    ls = [center.tolist(), corner_top_left_global_frame, corner_top_right_global_frame,
          corner_bottom_right_global_frame, corner_bottom_left_global_frame]

    return ls


'''
Return 3D rotation matrix given ZYX Euler angles
- Inputs:
    - euler_zyx: numpy column vector [roll; pitch; yaw] (rad)
- Outputs:
    - rotm: 3x3 rotation matrix
'''


def euler_zyx_to_rotm(euler_zyx):
    roll = euler_zyx.item(0)
    pitch = euler_zyx.item(1)
    yaw = euler_zyx.item(2)

    Rz_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]])

    Ry_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]])

    Rx_roll = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]])

    rotm = np.matmul(Rz_yaw, np.matmul(Ry_pitch, Rx_roll))

    return rotm


'''
Return ZYX Euler angles given 3D rotation matrix
- Inputs:
    - rotm: 3x3 rotation matrix
- Outputs:
    - euler_zyx: numpy column vector [yaw; pitch; roll] (rad)
'''


def rotm_to_euler_zyx(rotm):
    if (rotm.item(2, 0) < 1):
        if (rotm.item(2, 0) > -1):

            # Determine zyx Euler angles
            pitch = np.arcsin(-rotm.item(2, 0))
            yaw = np.arctan2(rotm.item(1, 0), rotm.item(0, 0))
            roll = np.arctan2(rotm.item(2, 1), rotm.item(2, 2))

        else:

            # Non-unique solution hard-coded
            pitch = math.pi / 2
            yaw = -np.arctan2(-rotm.item(1, 2), rotm.item(1, 1))
            roll = 0
    else:

        # Non-unique solution hard-coded
        pitch = -math.pi / 2
        yaw = np.arctan(-np.item(1, 2), np.item(1, 1))
        roll = 0

    euler_zyx = np.array([roll, pitch, yaw]).transpose()

    return euler_zyx


def parse_landmark_file(filename):
    """get the position of a defined tag by parsing the landmark file from the atlas folder

    Args:
        filename (string): the filename for the landmark file to parse

    Returns:
        tuple: side length of the tag, pose of the tag, orientation of the tag
    """

    with open(filename) as f:
        side_len = int(f.readline().strip())
        (x, y, z) = map(float, f.readline().strip().split(","))
        angles = list(map(float, f.readline().strip().split(",")))
        (roll, pitch, yaw) = map(math.radians, angles)

    pose = (x, y, z)
    orientation = (roll, pitch, yaw)

    return (side_len, pose, orientation)


if __name__ == "__main__":
    euler_zyx = np.array([0.1, 0.2, 0.3]).transpose()
    print(euler_zyx_to_rotm(euler_zyx))
    print(rotm_to_euler_zyx(euler_zyx_to_rotm(euler_zyx)))
