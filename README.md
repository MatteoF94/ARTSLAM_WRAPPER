# ARTSLAM_WRAPPER
***artslam_laser_3d_wrapper*** is a ROS wrapper for [ART-SLAM](https://github.com/MatteoF94/ARTSLAM). Right now, it accepts only point clouds as input, and it performs LiDAR SLAM, as intended. It also outputs, periodically, the built 3D map and the estimated positions of the robot (respectively on the /artslam_laser_3d_wrapper/single_cloud and /artslam_laser_3d_wrapper/markers topics).

## Requirements
***artslam_laser_3d_wrapper*** requires the following libraries:

- Eigen3
- Boost > 1.65.1
- PCL > 1.10
- OpenCV > 4.0
- g2o
- suitesparse

Moreover, it requires the following packages, along with the corresponding dependencies:

- [artslam_laser_3d](https://github.com/MatteoF94/ARTSLAM)

## Build
***artslam_laser_3d_wrapper*** is built using catkin, although ROS is not mandatory. After having build *artslam_laser_3d*:
```bash
cd catkin_ws/src
git clone https://github.com/MatteoF94/ARTSLAM_WRAPPER.git
cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release
```
Alternatively to catkin_make, you can use the "catkin build artslam_laser_3d_wrapper" approach, which will automatically build all packages. 

## Instructions
The file src/artslam_laser_3d_nodelet.cpp describes in detail how to build your own SLAM system, step by step. If you want to perform SLAM offline (no bags), do the following service call:
```bash
rosservice call /artslam_laser_3d_nodelet/OfflineSLAM
```
