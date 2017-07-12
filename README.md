# Unscented Kalman Filter

This code implement a 2 dimensional particle filter in C++ to estimate the position of a moving vehicle using a map of landmark locations, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

This project involves the Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/v1.0)

The Udacity Repository for this project can be found [here](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).

The only file modifed for this project is `particle_filter.cpp` located in the `src` directory.

---

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

## Build Instructions

1. Clone this repo.

2. This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.
Install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) by running the script

  For Linux
  ```
  bash install-ubuntu.sh
  ```

  For Mac
  ```
  sh install-mac.sh
  ```

3. ./particle_filter

4. Start the simulator and navigate next to "Project 3: Kidnapped Vehicle" and press select.

If the above fails, install intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) seperatly, then build and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter
