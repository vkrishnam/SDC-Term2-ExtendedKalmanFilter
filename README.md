# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program


[//]: # (Image References)

[image1]: ./images/Radar_Lidar_SensorFusion_EKF.PNG "SensorFusionArch"


In this project utilizes a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.
Lidar measurments exploit the Kalman Filtering techninque whereas Radar measurments use the Extended Kalman Filter technique due to the fact that Radar measurements are in polar coordinates which are non-linear in relation to object state formulated in the cartesian coordinates.

The overall architecture of the project and data flow follows the illustration below:
 ![alt text][image1]

---

## Other Important Dependencies

* cmake >= 2.8
* make >= 4.1
* gcc/g++ >= 5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

## Accuracies Achieved

With the file /data/obj_pose-laser-radar-synthetic-input.txt follwing RMSE Errors are achieved.
   Accuracy - RMSE:
   0.0972414
   0.0859948
   0.450821
   0.442915

