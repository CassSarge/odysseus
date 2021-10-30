# Import packages
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import numpy as np

'''
adds a new data point to the 3D position line plot
- inputs:
    - hl: handle for 3D plot
    - new_data: 3 long numpy array (x, y, z) (mm)
'''
def update_line(hl, new_data):
    xdata, ydata, zdata = hl._verts3d
    hl.set_xdata(np.append(xdata, new_data[0]))
    hl.set_ydata(np.append(ydata, new_data[1]))
    hl.set_3d_properties(np.append(zdata, new_data[2]))
    plt.draw()
    plt.pause(0.01)

'''
returns the handle to a 3D position line plot
- outputs:
    - hl: handle for 3D plot
'''
def init_position_figure():
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")
    ax.set_title("Camera position in global frame")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_zlabel("z (mm)")
    ax.set_xlim3d(-100, 3000)
    ax.set_ylim3d(-100, 3000)
    ax.set_zlim3d(-100, 3000)
    xs = np.array([])
    ys = np.array([])
    hl, = plt.plot(xs,ys)
    plt.ion()

    return hl
'''
quits the whole program is 'q' is pressed
- inputs:
    - event: event.key is the char for the keypress
'''
def on_press(event):
    if event.key == 'q':
        quit()

if __name__ == "__main__":

    hl = init_position_figure()
    update_line(hl, np.asarray([100,200,300]))