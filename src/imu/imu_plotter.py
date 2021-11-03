import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import math
from imu_tracking import TrackingCam


def plot_coordinates(tracking_camera, plotting='POSITION', run_time=60, speed=0.2):
    """
    Allows the plotting of coordinates 

    :param plotting: stream image width - for image streams
    :type plotting: string
    :param run_time: how long the plot will last for (in seconds)
    :type run_time: int
    :param speed: how long the tracking camera sleeps for before each reading
    :type speed: int
    """
    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(111, projection='3d')

    time.sleep(2)
    x = []
    y = []
    z = []
    counter = math.ceil(run_time/speed)

    while(counter>0):

        position = tracking_camera.receive_data([plotting], turn_off=False)[0][0]
        time.sleep(speed)
        col = 'black'
        x.append(position[0])
        y.append(position[1])
        z.append(position[2])
        print(f'x: {position[0]}, y: {position[1]}, z: {position[2]}')
        
        if(len(z) > 2 and z[-1]<z[-2]):
            col = 'red' 
        ax.plot(x,y,z, c=col)
        plt.pause(0.05)
        counter -= 1
    plt.show()
        

def plot_YPR(tracking_camera, run_time=60, speed =0.2):
    """
    Allows the plotting of coordinates and orientation 

    :param run_time: How long the plot will last for (in seconds)
    :type run_time: int
    :param speed: How long the tracking camera sleeps for before each reading
    :type speed: int
    """
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    import numpy as np
    from itertools import product, combinations
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.invert_yaxis()

    counter = math.ceil(run_time/speed)
    x = []
    y= []
    z = []
    while(counter>0):
        position = tracking_camera.receive_data(["POSITION"], turn_off=False)[0][0]
        x.append(position[0])
        y.append(position[1])
        z.append(position[2])
        u, v, w = tracking_camera.pose_to_ypr()
        ax.plot(x,y,z, color='black')
        S =ax.quiver(x[-1], y[-1], z[-1], math.radians(u), math.radians(v), math.radians(w), length=0.05, color='red')
        plt.pause(0.05)
        counter -=1
        time.sleep(speed)
        S.remove()
        
    plt.show()


if __name__ == "__main__":

    # Test tracking camera plotting position
    tracking_cam = TrackingCam()
    tracking_cam.start_stream()
    time.sleep(2)   # This sleep is required for receiving data
    plot_coordinates(tracking_cam)
