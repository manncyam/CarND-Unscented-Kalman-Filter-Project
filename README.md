# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 
Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project reburic. 

# How to run the program
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF
6. Run the Term 2 Simulator which can be download [here](https://github.com/udacity/self-driving-car-sim/releases)

# Result
The values below I chose for both datasets:
std_a_ = 0.5
std_yawdd = 0.5
P_ = 0.025, 0, 0, 0, 0,
     0, 0.025, 0, 0, 0,
     0, 0, 1, 0, 0,
     0, 0, 0, 1, 0,
     0, 0, 0, 0, 1

1. Here is the RMSE result for dataset 1

![Dataset1 snapshot](/output_images/dataset1_sim.png)

2. Here is the RMSE result for dataset 2

![Dataset2 snapshot](/output_images/dataset2_sim.png)

3. Here is the Lidar NIS result

![Dataset2 Lidar NIS](/output_images/lidar_NIS.png)

4. Here is the Radar NIS result

![Dataset2 Radar NIS](/output_images/radar_NIS.png)


## Other Important Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)