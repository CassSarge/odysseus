# Import packages
import cv2
import math
import numpy as np

# Import modules
from camera import apriltag_detection as ap

# Public Interface------------------------------------------------------------------------------------------------
def results_to_global_pose(boxes, centers, ids, cameraMatrix, distCoeffs):

    # Construct a numpy array of image points
    imagePoints = []
    objectPoints = []
    for tag in range(len(centers)):
        imagePoints.append(centers[tag])
        for corner in boxes[tag]:
            imagePoints.append(corner)
            
        atlas_name = f"pose/atlas/{ids[tag]}.lmk"
        (side_len, pose, orientation) = parse_landmark_file(atlas_name)
        
        objectPoints.extend(tag_pose_to_object_points(pose, orientation, side_len))
        
    imagePoints = np.array(imagePoints)
    objectPoints = np.array(objectPoints)
    #print(f"{objectPoints=}")
                  
    position, orientation = points_to_global_pose(objectPoints, imagePoints, cameraMatrix, distCoeffs)

    return position, orientation

def points_to_global_pose(objectPoints, imagePoints, cameraMatrix, distCoeffs):
    _, rVec, tVec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs)
    rotm_t = cv2.Rodrigues(rVec)[0]
    rotm = np.matrix(rotm_t).T
    position = -rotm * np.matrix(tVec)
    orientation = rotm_to_euler_zyx(rotm)

    return position, orientation

def tag_pose_to_object_points(pose, orientation, side_length):

    center = np.array(pose)
    rotm_tag_to_global = euler_zyx_to_rotm(np.array(orientation))

    corner_top_left_tag_frame = np.array([0, side_length/2, side_length/2]).T
    corner_top_right_tag_frame = np.array([0, -side_length/2, side_length/2]).T
    corner_bottom_right_tag_frame = np.array([0, -side_length/2, -side_length/2]).T
    corner_bottom_left_tag_frame = np.array([0, side_length/2, -side_length/2]).T

    corner_top_left_global_frame = (np.matmul(rotm_tag_to_global, corner_top_left_tag_frame)+center).tolist()
    corner_top_right_global_frame = (np.matmul(rotm_tag_to_global, corner_top_right_tag_frame)+center).tolist()
    corner_bottom_right_global_frame = (np.matmul(rotm_tag_to_global, corner_bottom_right_tag_frame)+center).tolist()
    corner_bottom_left_global_frame = (np.matmul(rotm_tag_to_global, corner_bottom_left_tag_frame)+center).tolist()
    ls = [center.tolist(), corner_top_left_global_frame, corner_top_right_global_frame, corner_bottom_right_global_frame, corner_bottom_left_global_frame]
    #print(f"{ls=}")
    #print(np.matmul(rotm_tag_to_global, corner_top_left_tag_frame))
    #print(np.matmul(rotm_tag_to_global, corner_top_right_tag_frame))
    #print(np.matmul(rotm_tag_to_global, corner_bottom_right_tag_frame))
    #print(np.matmul(rotm_tag_to_global, corner_bottom_left_tag_frame))
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
    
    with open(filename) as f:
        side_len = int(f.readline().strip())
        (x,y,z) = map(int, f.readline().strip().split(","))
        angles = list(map(int, f.readline().strip().split(",")))
        (roll, pitch, yaw) = map(math.radians, angles)
    
    pose = (x, y, z)
    orientation = (roll, pitch, yaw)
    
    return (side_len, pose, orientation) 

# Main------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    euler_zyx = np.array([0.1, 0.2, 0.3]).transpose()
    print(euler_zyx_to_rotm(euler_zyx))
    print(rotm_to_euler_zyx(euler_zyx_to_rotm(euler_zyx)))

