# Extended Kalman Filter Project
---
#### This project implements Extended Kalman Filter in C++ for Radar and Lidar data. The Kalman filter estimates positions and velocities in the x and y directions.

## Dependencies
###### Works only with linux

* cmake >= 3.5
 * [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
 instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros

  ## Basic Build Instructions
  1. Clone this repo.
  2. Make a build directory: `mkdir build && cd build`
  3. Compile: `cmake .. && make`
  4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
     some sample inputs in 'data/'.
      - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Files In The Project

The project contains the following folders:
* Docs: Contains files which explain the input and output of the program and the structure of the input data
* Data: Two files containing samples of Lidar and Radar data.
* src: Contains are the c++ code that implements the kalman filer
