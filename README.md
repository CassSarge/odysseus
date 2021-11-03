# odysseus
Landmark Localisation and Pose Estimation using Fiducial Markers for Navigation Onboard the ISS.

## Using the Intel RealSense library on Mac OSX
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
