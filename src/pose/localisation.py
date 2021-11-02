# Import packages
import cv2
import math
import numpy as np
import os

cam_position = np.array([[0.0,0.0,0.0]]).T
cam_orientation = np.array([[0.0,0.0,0.0]]).T

# Public Interface------------------------------------------------------------------------------------------------
def results_to_global_pose(boxes, centers, ids, cameraMatrix, distCoeffs):

    global cam_position, cam_orientation

    # Construct a numpy array of image points
    imagePoints = []
    objectPoints = []
    for tag in range(len(centers)):
        imagePoints.append(centers[tag])
        for corner in boxes[tag]:
            imagePoints.append(corner)
            
        atlas_name = f"pose/atlas/{ids[tag]}.lmk"

        #rotm_cam_to_im = euler_zyx_to_rotm(np.array([0,-math.pi/2,math.pi/2]))
        #rotm_cam_to_im = euler_zyx_to_rotm(np.array([0,0,0]))

        # If landmark file already exists, parse it for pose data
        if os.path.isfile(atlas_name):
            (side_len, tag_position, tag_orientation) = parse_landmark_file(atlas_name)

        # Else, create a new landmark file and parse it for pose data
        else:
            # Assume tag side length of 144m for all tags
            # TODO: have this info stored per ID in file
            print("Assuming tag side length of 144mm")
            side_len = 144

            # Assume camera of (0,0,0), (0,0,0) if only 1 tag detected
            # TODO: allow for deadreckoning from start point to first tag detection
            # TODO: deadreckon from last known position to allow for pose estimate of new tag when old tag not in frame
            cam_pos_global_frame = cam_position
            cam_ori_global_frame = cam_orientation
            # im_pos_global_frame = cam_position
            # im_ori_global_frame = cam_orientation
            print("Using cam pose:")
            print(f"{cam_position=}")
            print(f"{cam_orientation=}")

            # Format image points for tag
            temp_imagePoints = []
            temp_imagePoints.append(centers[tag])
            for corner in boxes[tag]:
                temp_imagePoints.append(corner)
            temp_imagePoints = np.array(temp_imagePoints)
            # Determine the object points in the tag frame
            temp_objectPoints = []
            temp_objectPoints.extend(tag_pose_to_object_points((0.0,0.0,0.0), (0.0,0.0,0.0), side_len))
            temp_objectPoints = np.array(temp_objectPoints)
            # Solve for camera pose in the tag frame
            im_pos_tag_frame, im_ori_tag_frame = points_to_global_pose(temp_objectPoints, temp_imagePoints, cameraMatrix, distCoeffs)
            cam_pos_tag_frame = im_pos_tag_frame

            print("Camera Pose in Tag Frame:")
            print(f"{im_pos_tag_frame=}")

            #rotm_cam_to_im = euler_zyx_to_rotm(np.array([math.pi/2,0,-math.pi/2]))
            rotm_cam_to_im = euler_zyx_to_rotm(np.array([0,-math.pi/2,math.pi/2]))
            #rotm_cam_to_im = euler_zyx_to_rotm(np.array([0,0,0]))
            rotm_im_to_tag = euler_zyx_to_rotm(im_ori_tag_frame)
            rotm_cam_to_tag = np.matmul(rotm_cam_to_im, rotm_im_to_tag)
            #rotm_cam_to_tag = (np.matmul(rotm_cam_to_im, rotm_im_to_tag))
            #cam_ori_tag_frame = rotm_to_euler_zyx(rotm_cam_to_tag)
            # The tag pose in the camera frame
            #tag_ori_cam_frame = rotm_to_euler_zyx((euler_zyx_to_rotm(cam_ori_tag_frame)).T)
            #rotm_tag_to_cam = euler_zyx_to_rotm(np.array(tag_ori_cam_frame))
            #rotm_tag_to_cam = euler_zyx_to_rotm(np.array(cam_ori_tag_frame))
            tag_pos_cam_frame = np.matmul(rotm_cam_to_tag.T, -cam_pos_tag_frame)
            #tag_pos_im_frame = np.matmul(rotm_im_to_tag.T, -im_pos_tag_frame)
            # Determine tag pose in global frame
            #rotm_cam_to_global = (euler_zyx_to_rotm(np.array(cam_ori_global_frame)))
            rotm_cam_to_global = (euler_zyx_to_rotm(np.array(cam_ori_global_frame)))
            #rotm_im_to_global = (euler_zyx_to_rotm(np.array(im_ori_global_frame)))
            #rotm_tag_to_global = np.matmul(rotm_cam_to_tag.T, rotm_cam_to_global)
            #rotm_tag_to_global = np.matmul(rotm_im_to_tag.T, rotm_im_to_global)
            # tag_position = cam_pos_global_frame + np.matmul(rotm_tag_to_global,tag_pos_cam_frame)
            # tag_orientation = rotm_to_euler_zyx(np.array(rotm_tag_to_global))
            tag_position = cam_pos_global_frame + np.matmul(rotm_cam_to_global,tag_pos_cam_frame)
            #tag_position = im_pos_global_frame + np.matmul(rotm_im_to_global,tag_pos_im_frame)
            tag_orientation = rotm_to_euler_zyx(np.array(rotm_cam_to_global))
            #tag_orientation = rotm_to_euler_zyx(np.array(rotm_im_to_global))

            # print(f"{temp_imagePoints=}")
            # print(f"{temp_objectPoints=}")
            # print(f"{cam_pos_tag_frame=}")
            # print(f"{cam_ori_tag_frame=}")
            # print(f"{tag_pos_cam_frame=}")

            # Create new landmark file
            with open(atlas_name,'w+') as f:
                l1 = f"{side_len}"
                l2 = f"{tag_position.item(0)},{tag_position.item(1)},{tag_position.item(2)}"
                l3 = f"{math.degrees(tag_orientation[0])},{math.degrees(tag_orientation[1])},{math.degrees(tag_orientation[2])}"
                f.write('{}\n{}\n{}\n'.format(l1,l2,l3))
                print(f"Created file {atlas_name}")

            (side_len, tag_position, tag_orientation) = parse_landmark_file(atlas_name)

        objectPoints.extend(tag_pose_to_object_points(tag_position, tag_orientation, side_len))
        
    imagePoints = np.array(imagePoints)
    objectPoints = np.array(objectPoints)

    cam_position, cam_orientation = points_to_global_pose(objectPoints, imagePoints, cameraMatrix, distCoeffs)

    #rotm_cam_to_im2 = euler_zyx_to_rotm(np.array([0,0,0]))
    #cam_orientation = rotm_to_euler_zyx(np.matmul(rotm_cam_to_im2, euler_zyx_to_rotm(np.array(cam_orientation))))

    return cam_position, cam_orientation

def points_to_global_pose(objectPoints, imagePoints, cameraMatrix, distCoeffs):
    _, rVec, tVec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs)
    rotm_t = cv2.Rodrigues(rVec)[0]
    rotm = np.matrix(rotm_t).T
    position = -rotm * np.matrix(tVec)
    orientation = rotm_to_euler_zyx(rotm)

    return position, orientation

def tag_pose_to_object_points(position, orientation, side_length):

    center = np.array(position)
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
        (x,y,z) = map(float, f.readline().strip().split(","))
        angles = list(map(float, f.readline().strip().split(",")))
        (roll, pitch, yaw) = map(math.radians, angles)
    
    pose = (x, y, z)
    orientation = (roll, pitch, yaw)
    
    return (side_len, pose, orientation) 

# Main------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    euler_zyx = np.array([0.1, 0.2, 0.3]).transpose()
    print(euler_zyx_to_rotm(euler_zyx))
    print(rotm_to_euler_zyx(euler_zyx_to_rotm(euler_zyx)))

