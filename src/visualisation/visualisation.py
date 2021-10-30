# Full credit for the visualisation code goes to Jeong Hyeonjin, https://pythonrepo.com/repo/demul-extrinsic2pyramid
# The code that Hyeonjin wrote is in camera_post_visualizer.py, and it is a way to turn an extrinsic matrix into a 
# visualised model, allowing pose to be seen without learning large new toolboxes
import numpy as np
from camera_pose_visualizer import CameraPoseVisualizer

if __name__ == '__main__':
    # argument : the minimum/maximum value of x, y, z
    visualizer = CameraPoseVisualizer([-100, 3000], [-100, 3000], [-100, 3000])

    # argument : extrinsic matrix, color, scaled focal length(z-axis length of frame body of camera
    visualizer.extrinsic2pyramid(np.eye(4), 'c', 400)


    visualizer.show()