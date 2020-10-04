# **Extended Kalman Filter Project**

---

## Summary
In this project, Extend Kalman Filter was implemented to estimate the state of a moving object of interest with noisy lidar and radar measurements.

[//]: # (Image References)

[image1]: ./EKF_architecture.png "function blocks"


## Intro.

This project is Udacity Self-driving car ND project to implement [EKF](https://en.wikipedia.org/wiki/Extended_Kalman_filter).
My project includes the following files:
* src/main.cpp uses for uWebSocketIO in communicating with the simulator.
* src/FusionEKF.cpp, src/FusionEKF.h contains (laser/radar)sensor-fusion process-measurement mgmt.
* src/kalman_filter.cpp, src/kalman_filter.h contains KF predict & update routines.
* src/tools.cpp, src/tools.h contains Jacobian/RMSE calculation routine.


This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

To run the code,
- run shell script to install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for Linux, Mac, or Ubuntu systems.
- check dependencies
    * cmake >= 3.5
      * All OSes: [click here for installation instructions](https://cmake.org/install/)
    * make >= 4.1 (Linux, Mac), 3.81 (Windows)
      * Linux: make is installed by default on most Linux distros
      * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
      * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
    * gcc/g++ >= 5.4
      * Linux: gcc / g++ is installed by default on most Linux distros
      * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
      * Windows: recommend using [MinGW](http://www.mingw.org/)
- install Udacity simulator from [here](https://github.com/udacity/self-driving-car-sim/releases).
  * INPUT: values provided by the simulator ["sensor_measurement"] to the c++ program
  * OUTPUT: values provided by the c++ program ["estimate_x"], ["estimate_y"], ["rmse_x"], ["rmse_y"], ["rmse_vx"], ["rmse_vy"] to the simulator

the main program can be built and run by doing the following from the project top directory.
```sh
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF
```
---

## Implementation

The Kalman filter(KF) is one of the most widely used methods for tracking and estimation due to its simplicity,
optimality, tractability and robustness. However, the application of the KF to nonlinear systems can be difficult.
Extended Kalman Filter (EKF) linearises all nonlinear models so that the traditional linear Kalman filter can be applied. 
But EKF still is only reliable for systems which are almost linear on the time scale of the update intervals.

In this implementation, there's two sensors(Lidar & Radar) to fuse, which spit out measurements in different units. Here, Radar sensor has polar coordinates and requires non-linear transform to get cartesian coordinate. For state x vector calculation, function h(x) was applied, but for covariance calculation Jacobian(Hj) matrix was used. R,Q matrix values are assumed to be given.

![alt text][image1] 

* Initializing the State Vector
Although radar gives velocity data in the form of the range rate rho, a radar measurement does not contain enough information to determine the state variable velocities vx and vy. Here, the radar measurements rho and phi was used to initialize the state variable locations px and py.

* Calculating y = z - H * x'
For lidar measurements, the error equation is y = z - H * x'. For radar measurements, the functions that map the x vector [px, py, vx, vy] to polar coordinates are non-linear. Instead of using H to calculate y = z - H * x', the equations that map from cartesian to polar coordinates: y = z - h(x') was used for radar measurements

* Normalizing Angles
In C++, atan2() returns values between -pi and pi. When calculating phi in y = z - h(x) for radar measurements, the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi. The Kalman filter is expecting small angle values between the range -pi and pi. 

* Divide by Zero Errors
When calculating the non-linear function h(x) or Jacobian Hj, devide-by-zero errors may occur. For example, both the x and y values might be zero or px*px + py*py might be close to zero. For this case, a very small protection values were given.


## Outro.

