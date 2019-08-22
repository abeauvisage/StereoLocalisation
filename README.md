# StereoLocalisation
Testing various VO/SLAM algorithms

# Dependencies
- OpenCV 3 minimum (https://opencv.org/releases/)
- yaml-cpp
- CDFF project (https://gitlab.com/h2020src/og3/cdff)
- Pangolin (installed with ROS)
- libviso2 (https://github.com/srv/viso2)
- uasl_motion_estimation (https://github.com/abeauvisage/uasl_motion_estimation)
- uasl_vins_fusion (https://github.com/abeauvisage/uasl_VINS_fusion)

# Installation

Before installing, run make install  in CDFF projetc and copy rsrc/share dir to CDFF_ROOT_DIR/install

In root directory:

1. create build dir:
- mkdir build

2. move to build dir:
- cd build

3. solve dependencies:
- cmake -DCDFF_ROOT_DIR=/PATH/TO/CDFF_ROOT -DLIBVISO_DIR=/PATH/TO/LIBVISOCONFIG -DTRACKING_DIR=/PATH/TO/uasl_VINS_fusion ..

> LIBVISO_DIR and TRACKING_DIR are optional

> specify -DOPENCV_CONFIG_ UASL_MOTION_ESTIMATION_CONFIG_FILE or YAML_CPP_CONFIG_PATH if necessary

4. compile:
- make

# Usage

- stereo_tracking_test PATH_TO_DATASET_CONFIG_FILE.yml (uasl bundle adjustment)
- stereo_libviso_test PATH_TO_DATASET_CONFIG_FILE.yml (libviso2)
- stereo_vo_test PATH_TO_DATASET_CONFIG_FILE.yml (Edres Magellium)
- stereo_dfn_slam_test PATH_TO_DATASET_CONFIG_FILE.yml PATH_TO_ORBSLAM_CONFIG_FILE.yaml (OrbSlam)

press 'r' to run the code continuously or resume when paused

press 'p' to pause

press 'q' to quit
