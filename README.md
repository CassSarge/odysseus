# odysseus

Landmark Localisation and Pose Estimation using Fiducial Markers for Navigation Onboard the ISS.

## Setup

### Apriltags

In order to localise, Odysseus requires that AprilTags be placed at locations around the testing environment such that there is always one in view of the camera at all times. In order to do this, please print out some of the 36h11 family tags and note their ids. The size of each tag should also be recorded so that landmark file creation is easier later on.

### Positioning in Global Frame

After defining a right handed set of axes to use as the testing environment, please place the printed AprilTags at measured locations, and record these locations and positions in a landmark file in the `src/pose/atlas` directory. The name of the file should be `<tag_id>.lmk` and the  format should be as follows:

```
<side_length of tag (mm)>
<x>,<y>,<z>
<roll>,<pitch>,<yaw>
```

where `roll`, `pitch` and `yaw` are in degrees, referring to  the same global reference frame as defined earlier.

### IMU

In order to utilise the Intel Realsense Tracking Camera T265, please follow the below instructions to install the Intel realsense libraries for MacOSX. Instructions have not been provided for other operating systems (sorry!). 

To use the library on Mac OSX, it must be built from the source code. For CMake to work, first OpenSSL must be installed using ```brew install openssl```. The following lines must then be added to your ~/.bash_profile:

```
export LDFLAGS="-L/usr/local/opt/openssl@3/lib"
export CPPFLAGS="-I/usr/local/opt/openssl@3/include"
export PKG_CONFIG_PATH="/usr/local/opt/openssl@3/lib/pkgconfig"
```

Once this has been added, the library can be built successfully from the source code.

```
git clone https://github.com/IntelRealSense/librealsense
mkdir build
cd build
cmake -DOPENSSL_ROOT_DIR=/usr/local/opt/openssl . -DBUILD_PYTHON_BINDINGS=bool:true
make -j
sudo make install
```

The generated .so files in the /wrappers/python directory have been added to this project in the imu module.

## Running the code

#### Virtual Environment 

In order to run the any of the code, the dependencies for the project need to be installed. This should be done by creating a virtual environment so that already installed python packages do not interfere with the dependencies of the project

Ensure that you are in the `odysseus/` directory.

Run the command: `python3 -m venv env`

Then, run `source env/bin/activate` to activate the virtual environment.

In order to install all the dependencies required, run `pip3 install -r requirements.txt`

#### Basic AprilTag Demo

If you have not set up an environment, and would just like to test some basic AprilTag detection examples, please print out a tag or bring one up on a phone and perform the following: 

1. `cd` into `src/camera/` 
2. run `python3 apriltag_live.py`
3. Hold the AprilTag up in front of the camera to test that it is detected correctly

#### Calibration

You may also calibrate your camera without having an environment set  up. In order  to do this, print out the checkerboard pattern, located [at the ROS wiki page for camera calibration](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=view&target=check-108.pdf).

Then, create a folder in `src/calibration/` that will store the calibration images. Please take a number of photos, preferably $\geqslant 8$ showing the checkerboard from a number of different angles and also covering all corners of the screen.

Once this is complete, run `python3 main.py -u <new_folder_created> -c` to create the calibration data.  After running this once, the program will be able to be run with `python3 main.py -u <new_folder_created>` and will reuse the calibration profile generated for ease of use.

#### Localisation Code

After setting up a testing environment, and calibrating the camera with a set of images, simply run:

`python3 main.py -u <your_user_profile>` and move the camera around in view of other AprilTags.




