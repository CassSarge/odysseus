import math
import numpy as np

'''
Pose struct format
- pose
    - linear: 3x1 numpy array [x; y; z]
    - angular: 3x1 numpy array [roll; pitch; yaw]
'''

# Public Interface------------------------------------------------------------------------------------------------
'''
- Inputs:
    - tag_pose_global_frame: pose of the tag in the global frame
    - tag_pose_camera_frame: pose of the tag in the camera frame
'''
def global_pose(tag_pose_global_frame, tag_pose_camera_frame):

    # IDK IF ANYTHING IN THIS FUNCTION IS LEGIT EEK!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    # Find the camera pose in the tag frame
    rotm_camera_to_tag = euler_zyx_to_rotm(tag_pose_camera_frame.euler_zyx)
    rotm_tag_to_camera = rotm_camera_to_tag.transpose()

    camera_pose_tag_frame.euler_zyx = rotm_to_euler_zyx(rotm_tag_to_camera)
    camera_pose_tag_frame.pos = np.matmul(rotm_camera_to_tag, -tag_pose_camera_frame.pos)   # relative position is negative

    # Find the camera pose in the global frame
    rotm_global_to_tag = euler_zyx_to_rotm(tag_pose_global_frame.euler_zyx)
    rotm_tag_to_global = rotm_global_to_tag.transpose()
    rotm_global_to_camera = np.matmul(rotm_global_to_tag, rotm_tag_to_camera)
    rotm_camera_to_global = rotm_global_to_camera.transpose()

    camera_pose_global_frame.euler_zyx = rotm_to_euler_zyx(rotm_global_to_camera)
    camera_pose_global_frame.pos = tag_pose_global_frame.pos + np.matmul(rotm_tag_to_global, camera_pose_tag_frame.pos)

    return camera_pose_global_frame

'''
Return 3D rotation matrix given ZYX Euler angles
- Inputs:
    - euler_zyx: numpy column vector [yaw; pitch; roll] (rad)
- Outputs:
    - rotm: 3x3 rotation matrix
'''
def euler_zyx_to_rotm(euler_zyx):

    yaw     = euler_zyx.item(0)
    pitch   = euler_zyx.item(1)
    roll    = euler_zyx.item(2)

    Rz_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [          0,            0, 1]])

    Ry_pitch = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [             0, 1,             0],
        [-np.sin(pitch), 0, np.cos(pitch)]])

    Rx_roll = np.array([
        [1,            0,             0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]])

    rotm = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))

    return rotm

'''
Return ZYX Euler angles given 3D rotation matrix
- Inputs:
    - rotm: 3x3 rotation matrix
- Outputs:
    - euler_zyx: numpy column vector [yaw; pitch; roll] (rad)
'''
def rotm_to_euler_zyx(rotm):

    if(rotm.item(2,0) < 1):
        if(rotm.item(2,0) > -1):

            # Determine zyx Euler angles
            pitch = np.arcsin(-rotm.item(2,0))
            yaw = np.arctan2(rotm.item(1,0), rotm.item(0,0))
            roll = np.arctan2(rotm.item(2,1), rotm.item(2,2))

        else:

            # Non-unique solution hard-coded
            pitch = math.pi/2
            yaw = -np.arctan2(-rotm.item(1,2), rotm.item(1,1))
            roll = 0
    else:

        # Non-unique solution hard-coded
        pitch = -math.pi/2
        yaw = np.arctan(-np.item(1,2), np.item(1,1))
        roll = 0 

    euler_zyx = np.array([yaw, pitch, roll]).transpose()

    return euler_zyx


# Main------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":

    euler_zyx = np.array([0.1,0.2,0.3]).transpose()
    print(euler_zyx_to_rotm(euler_zyx))
    print(rotm_to_euler_zyx(euler_zyx_to_rotm(euler_zyx)))

    