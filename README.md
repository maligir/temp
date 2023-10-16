# placard_discovery
rehashing of prism2 repo

to collect RGBD video feed from Azure camera run:

rosrun azure_kinect_interface collect_video

Instructions to Compile:

ensure azure kinect sdk is installed.

run:

git clone git@github.com:dbalaban/placard_discovery.git

git submodule update --init

edit third_party/octomap/octomap/CMakeLists.txt:85, change custom target uninstall to octomap_uninstall

opencv4 is included as a third_party dependency, remove old opencv install if necessary with:

sudo apt remove libopencv-dev

then install opencv4 from the third party source, be sure to set compile options for pkgconfig and extra modules. See instructions at link below for assistance.
https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/

build g2o ORB_SLAM dependency:

cd third_party/ORB_SLAM3/Thirdparty/g2o && mkdir build && cd build && cmake .. && make && cd ../../../../..

copy tesseract version header file: cp third_party/tesseract/include/version.h.in third_party/tesseract/include/version.h

from project directory run:

mkdir build && cd build && cmake ..

if running yolo on GPU include cmake compile option:

cmake .. -DCMAKE_C_COMPILER=/usr/bin/gcc-6

assuming no cmake errors, run make.
