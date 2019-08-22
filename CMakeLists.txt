cmake_minimum_required(VERSION 3.5)
project(stereo_localisation)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "build type")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wno-comment")

#directory where CDFF is installed
set(CDFF_ROOT_DIR "/home/abeauvisage/CityLondon/cdff" CACHE STRING "CDFF root dir")
set(CDFF_INSTALL "${CDFF_ROOT_DIR}/install" CACHE STRING "CDFF install dir")
list(APPEND CMAKE_PREFIX_PATH "${CDFF_ROOT_DIR}/External/install/share/edres-wrapper" "${CDFF_INSTALL}/share/cmake")

# tracking code
set(TRACKING_DIR "/home/abeauvisage/Documents/uasl_VINS_fusion" CACHE STRING "Main directory for tracking code")
set(TRACKING_INCLUDE_DIRS ${TRACKING_DIR}/include)
find_library(MSVO_COMMON msvo_common PATHS ${TRACKING_DIR}/build/)
find_library(MSVO_BA bundle_adjustment PATHS ${TRACKING_DIR}/build/)
set(TRACKING_LIBRARIES ${MSVO_BA} ${MSVO_COMMON})

#to use project dependencies and not system libraries
set(BUNDLED_DEPENDENCIES_PREFIX External/install CACHE PATH "Location of the libraries in the source tree")
set(_DEPENDENCIES_PREFIX "${BUNDLED_DEPENDENCIES_PREFIX}")
set(CMAKE_PREFIX_PATH "${CDFF_ROOT_DIR}/${BUNDLED_DEPENDENCIES_PREFIX}")
set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "${CDFF_ROOT_DIR}/${BUNDLED_DEPENDENCIES_PREFIX}/include/")
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "${CDFF_ROOT_DIR}/${BUNDLED_DEPENDENCIES_PREFIX}/lib/")
set(CMAKE_MODULE_PATH "${CDFF_ROOT_DIR}/${BUNDLED_DEPENDENCIES_PREFIX}/share/cmake-3.11.4/Modules")

#finding necessary libraries to run VisualSlam and StereoMotionEstimation
find_package(MotionEstimation REQUIRED PATHS /home/abeauvisage/Documents/uasl_VINS_fusion/uasl_motion_estimation/share/MotionEstimation)
find_package(OpenCV 3 REQUIRED ${OPENCV_CONFIG_FILE})
find_package(yaml-cpp REQUIRED ${YAML_CPP_CONFIG_FILE})
find_package(CDFF REQUIRED PATHS ${CDFF_INSTALL}/share/cmake) #find only a tiny portion of CDFF libraries because install script is incomplete
find_package(Pangolin REQUIRED ${PANGOLIN_CONFIG_FILE})
find_package(LIBORBSLAM REQUIRED PATHS ${CDFF_ROOT_DIR}/build/Common/Core/liborbslam/)
find_package(DBoW2 REQUIRED PATHS ${CDFF_ROOT_DIR}/build/Common/Core/liborbslam/modules)
find_package(g2o REQUIRED PATHS ${CDFF_ROOT_DIR}/build/Common/Core/liborbslam/modules)

set(LIBVISO_DIR "/home/abeauvisage/catkin_ws/devel/share/libviso2/cmake")
find_package(libviso2 PATHS ${LIBVISO_DIR})

set(COMMON_LIBRARIES ${YAML_CPP_LIBRARIES} ${MotionEstimation_LIBRARIES} ${OpenCV_LIBRARIES})

# including missing libraries from CDFF and necessary to run
message("looking for lib in ${CDFF_ROOT_DIR}/build/DFNs/StereoMotionEstimation}")
find_library(DFN_SLAM cdff_dfn_stereo_slam PATHS ${CDFF_ROOT_DIR}/build/DFNs/StereoSlam)
find_library(DFPC_SLAM cdff_dfpc_visual_slam_stereo PATHS ${CDFF_ROOT_DIR}/build/DFPCs/VisualSlamStereo)
#find_library(DFPC_CONF libcdff_dfpc_configurator.a PATHS ${CDFF_ROOT_DIR}/build/DFPCs)
#find_library(DFN_BUILDER libdfns_builder.a PATHS ${CDFF_ROOT_DIR}/build/DFNs)
find_library(STEREOVO cdff_dfn_stereo_motion_estimation PATHS ${CDFF_ROOT_DIR}/build/DFNs/StereoMotionEstimation)

list(APPEND DFN_SLAM ${DBow2_LIBRARIES} ${g2o_LIBRARIES} ${LIBORBSLAM_LIBRARIES})
list(APPEND DFPC_SLAM ${DBow2_LIBRARIES} ${g2o_LIBRARIES} ${LIBORBSLAM_LIBRARIES})

find_package(Ceres 1)
find_package(Edres-Wrapper)
#test SteroVO
if(Ceres_FOUND AND Edres-Wrapper_FOUND)
message(STATUS "found Ceres and Edres. Adding stereo_vo_test executable")
add_executable(stereo_vo_test src/main_vo.cpp include/asnconversion.hpp src/asnconversion.cpp)
target_link_libraries(stereo_vo_test ${STEREOVO} ${CDFF_LIBRARIES} ${Edres-Wrapper_LIBRARIES} ${Ceres_LIBRARIES} ${COMMON_LIBRARIES})
endif(Ceres_FOUND AND Edres-Wrapper_FOUND)

#test DFN OrbSLAM
add_executable(stereo_dfn_slam_test src/main_orbslam.cpp include/asnconversion.hpp src/asnconversion.cpp)
target_link_libraries(stereo_dfn_slam_test ${DFN_SLAM} ${CDFF_LIBRARIES} ${COMMON_LIBRARIES})

# test DFPC VisualSLAM
add_executable(stereo_dfpc_slam_test src/main_visualslam.cpp include/asnconversion.hpp src/asnconversion.cpp)
target_link_libraries(stereo_dfpc_slam_test ${DFPC_SLAM} ${Edres-Wrapper_LIBRARIES} ${CDFF_LIBRARIES} ${COMMON_LIBRARIES})

# test feature tracking
if(TRACKING_LIBRARIES AND Ceres_FOUND)
message(STATUS "found Ceres. Adding stereo_tracking_test executable")
add_executable(stereo_tracking_test src/main_tracking.cpp)
target_link_libraries(stereo_tracking_test pthread ${TRACKING_LIBRARIES} ${CERES_LIBRARIES} ${COMMON_LIBRARIES})
endif(TRACKING_LIBRARIES AND Ceres_FOUND)


#test libviso
if(libviso2_FOUND)
message(STATUS "found libviso. Adding stereo_libviso_test executable")
add_executable(stereo_libviso_test src/main_viso.cpp)
target_link_libraries(stereo_libviso_test ${MSVO_COMMON} ${COMMON_LIBRARIES} ${libviso2_LIBRARIES})
endif(libviso2_FOUND)


include_directories(
    include
    ${CDFF_INCLUDE_DIRS}
    ${MotionEstimation_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${LIBORBSLAM_INCLUDE_DIRS}
		${TRACKING_INCLUDE_DIRS}
    ${libviso2_INCLUDE_DIRS}
)
