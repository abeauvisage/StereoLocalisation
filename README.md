# StereoLocalisation
Testing various VO/SLAM algorithms

# Dependencies
- OpenCV 3 minimum
- yaml-cpp
- CDFF project
- Pangolin
- libviso2
- uasl_motion_estimation
- uasl_vins_fusion

# Installation

In root direcory:

$ mkdir build
$ cd build
$ cmake -DCDFF\_ROOT\_DIR=/PATH/TO/CDFF_ROOT -DLIBVISO\_DIR=/PATH/TO/LIBVISOCONFIG -DTRACKING\_DIR=/PATH/TO/uasl\_VINS\_fusion ..
LIBVISO\_DIR and TRACKING\_DIR are optional
$ make

# Usage
