# CV-introductory-task

## Brief

You will complete a light bar tracking task in order to get familiar with the following things:
1. OpenCV(3.2.0) APIs
2. Basic image processing techniques, like color extraction, color space transformation, contour detection, so on and so forth.
3. ROS APIs (ros_cpp)
4. catkin-make
5. (if not familiar yet) Ubuntu and Linux operating system basics 

* Follow the instructions in [Setup](#setup) section to setup the environment.
* Refer to the [Requirement](#requirement) section for the requirement of this task.
* Refer to the [Convention](#convention) section for the coding conventions.
* For the explanation on the example code, refer to [Example Code](#example-code)
* Video of the light bar can be found on [__OneDriver__](https://hkustconnect-my.sharepoint.com/:f:/g/personal/kikemura_connect_ust_hk/EvIow_ZuODtOuebddo8s0PABBDA1HFw6qJ9cmj9lDPp5eA?e=V8RdwS), the password is `123`

> Note: You MUST use Ubuntu in order to use ROS melodic.

## Setup

### 1. OpenCV 3.2.0 (C++)

* Install minimal prerequisites:  
`sudo apt update && sudo apt install -y cmake g++ wget unzip`  
* Download and unpack sources:  
`wget -O opencv.zip https://github.com/opencv/opencv/archive/master.zip`  
`unzip opencv.zip`  
* Create build directoy:  
`mkdir -p build && cd build`
* Configure:  
`cmake  ../opencv-master`
* Build:  
`cmake --build`

### 2. ROS Melodic

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
sudo rosdep init
sudo rosdep update
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## Requirement

1. Users are able to get the list of light bars within a single frame.  

2. Users need to be able to get the position of the center (in pixel coordinates, upper-left-most point as the origin) of each light in the list.  

3. Users need to be able to get the following attributes of each light bar:  
    * Absolute value of the tilt (angle), with respect to the normal line perpendicular to the ground, assuming light bar has zero tilt when the longer side is perpendicular to the ground.
    * The size of the light bar (width, height, and area, you can use `cv::Size` for convinience).
    * The rotated rectangle bounding the light bar (`cv::RotatedRect`).
    * The upright bounding box of the light bar (note that this is difference from c, this bounding box must be a non-rotated rectangle).
    * The aspect ratio of the light bar.  

4. To reduce the amount of noise (i.e. those that are not actually light bars), you need to do tests on each potential light bar, the tests may include:
    * aspect ratio test (approximately 0.3 <= aspect ratio <= 0.7, adjust according to the performance)
    * area test (test out by your self)
    * length test
    * tilt test (we don't want any light bar that has tilt >= 45 degrees, assuming light bar has zero tilt when the longer side is perpendicular to the ground).
    * and any test you think appropriate.  

5. Having a ros node performing the light bar tracking task, and produce the whole code base as a catkin package (i.e ros package).  

## Convention

1. Use the `.clang_format` provided to format your code.
2.  UpperCamelCase for __FILE NAMES__, __CLASS NAMES__, __ENUM NAMES__, __STRUCT NAMES__, and other user defined types.
3.  lowerCamelCase for __FUCNTION NAMES__, __VARIABLE NAMES__, __ARGUMENT NAMES__.
4. UPPER_SNAKE_CASE for __CONSTANTS__, __ENUMERATION MEMBERS__.
5. Whenever you want to define __CONSTANTS__, use `constexpr` or `const`, __DO NOT__ use `#define`. Use `#define` only for selection of compilation.
6. Avoid using magic numbers, all paramters/coefficients should be hold by __CONSTANTS__. Equations should be documented.
7. Comment only when you think needed. Extensive commenting will ruin your code.
8. Avoid typos as much as you could, this is about a good programmer habit, not about code performance.
9. Name your module/package starting with lower case letter and use lower_snake_case.
10. Name your nodes using lower_snake_case.


## Example Code

I've wrote some random codes just for illustration on how to write `CMakeList.txt` for ros package and some random uses of OpenCV APIs and ros_cpp APIs.

To run the code, you need to:
1. Compile by `catkin_make`
2. Source the bash script by `source ./devel/setup.bash`, assuming you are at the root directory of this catkin workspace.
3. Run the example node by `rosrun example_pkg example_node`

> While running the code, you can press `q` key to stop the execution, this is implemented with `cv::waitKey()`.

> Try to undetstand the code (understand the way to use the APIs), if you have any doubt, google it. If still in doubt, ask me directly, I am willing to help.


## HAPPY CODING!

