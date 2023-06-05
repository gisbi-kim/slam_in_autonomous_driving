## This is a Forked version to
- Refactor the code using C++17/20.
- Translate the comments into English.
- Test on my own Ubuntu 20.04 machine.

### Progress 
- Refactored code: [Ch2](https://github.com/gisbi-kim/slam_in_autonomous_driving_en/blob/master/src/ch2/motion.cc)
- English-translated comments: [Ch2](https://github.com/gisbi-kim/slam_in_autonomous_driving_en/blob/master/src/ch2/motion.cc), [Ch3](https://github.com/gisbi-kim/slam_in_autonomous_driving_en/blob/master/src/ch3/eskf.hpp)
- Tutorial videos: [Ch2 (youtube)](https://youtu.be/v4rsN_5y5y0) 

## SLAM in Autonomous Driving book (SAD book)

This book systematically introduces inertial navigation, integrated navigation, laser mapping, laser positioning, laser inertial odometer and other knowledge to readers. This warehouse is the source code warehouse corresponding to the book and can be used publicly.

## Notice
- The text of this book is currently under review. If you wish to be a reviewer for this book, please contact: gao.xiang.thu at gmail.com
- After becoming a reviewer, you can view the daily updated PDF manuscript of this book. At the same time, you need to give me your feedback within two months. You can send me comments via github issue or email.
- **The first draft of this book will be closed in the first round at the end of March 2023. I will hand over the manuscript to the publisher for processing. Reviewers are requested to update your review comments in time. Of course, subsequent issues will also be fixed with the publication printing, but your name may not be updated in the acknowledgment list in time.**
- If you are interested in writing a few words of recommendation for this book, please also contact me. Recommendations will appear in the preface of the book or on the back cover of the book.

## Contents of this book

- Chapter 1, Overview
- Chapter 2, review of basic knowledge of mathematics, geometry, kinematics, KF filter theory, matrix Lie group
- Chapter 3, Error State Kalman Filter, Inertial Navigation, Satellite Navigation, Integrated Navigation
- Chapter 4, Pre-integration, Graph Optimization, Pre-integration-Based Combined Navigation
- Chapter 5, Point Cloud Basic Processing, Various Nearest Neighbor Structures, Point Cloud Linear Fitting
- Chapter 6, 2D laser mapping, scan matching, likelihood field, submap, 2D loop closure detection, pose graph
- Chapter 7, 3D laser mapping, ICP, variant ICP, NDT, NDT LO, Loam-like LO, LIO loose coupling
- Chapter 8, Tightly Coupled LIO, IESKF, Preintegrated Tightly Coupled LIO
- Chapter 9, offline map building, front-end, back-end, batch loop detection, map optimization, slice export
- Chapter 10, fusion positioning, laser positioning, initialization search, slice map loading, EKF fusion

## Features of this book


- This book is probably the easiest book for mathematical derivation and code implementation among similar materials you can find.
- In this book, you will reproduce many classic algorithms and data structures in laser SLAM.
        - You need to derive and implement an error state Kalman filter (ESKF) by yourself, feed it the data of IMU and GNSS, and see how it calculates its own state.
        - You'll also implement the same functionality with pre-integration systems, and compare how they work.
        - Next, you will implement common algorithms in 2D laser SLAM: scan matching, likelihood field, submap, occupy grid, and then use loop closure detection to build a larger map. These need to be done by yourself.
        - In laser SLAM, you will also implement the Kd tree yourself, process the approximate nearest neighbor, and then use this Kd tree to implement ICP, point-plane ICP, and discuss what can be improved.
        - You'll then implement the classic NDT algorithm, test its registration performance, and use it to build a laser odometry. It is much faster than most existing LOs.
        - You'll also implement a laser odometry with point-plane ICP, which is also very fast. The way it works is similar to Loam, but simpler.
        - You'll want to put the IMU system into the laser odometry as well. We will implement loosely coupled and tightly coupled LIO systems. Likewise, you need to derive a pass for the iterative Kalman filter and pre-integrate graph optimization.
        - You need to change the above system to run offline, so that the loopback detection can run fully. Finally, make it into an offline mapping system.
        - Finally, you can segment the above map and use it for real-time positioning.
- Most of the implementations in this book are much simpler than similar algorithm libraries. You can quickly understand how they work without having to deal with complicated interfaces.
- This book will use concurrent programming very conveniently. You will find that the implementations in this book are often more efficient than existing algorithms. Of course, part of this is caused by historical reasons.
- Each chapter of this book will be accompanied by a dynamic demonstration, like this:

![](./doc/lio_demo.gif)
![](./doc/2dmapping_demo.gif)
![](./doc/lo_demo.gif)

I hope you enjoy the minimalist style of this book and discover the joy of algorithms.

## data set
Dataset download link:
- Baidu cloud link: https://pan.baidu.com/s/1ELOcF1UTKdfiKBAaXnE8sQ?pwd=feky Extraction code: feky
- OneDrive link: https://1drv.ms/u/s!AgNFVSzSYXMahcEZejoUwCaHRcactQ?e=YsOYy2
- Contains the following datasets. The total amount is large (270GB), please download according to the capacity of your own hard disk.
    - UrbanLoco (ULHK, 3D laser, road scenes)
    - NCLT (3D laser, RTK, campus scene)
    - WXB (3D laser, park scene)
    - 2dmapping (2D laser, shopping mall scene)
    - AVIA (DJI Solid State Laser)
    - UTBM (3D laser, road scene)
- Other built-in data
    - Chapters 3 and 4 use IMU in text format, RTK data
    - Chapter 7 uses a part of EPFL data as a point cloud source for registration
- You should download the above data to the ./dataset/sad/ directory, so many default parameters can work normally. Alternatively, you can manually specify these file paths. If your hard disk capacity is insufficient, you can soft-link the directories of other hard disks here.

## 编译

- The recommended compilation environment for this book is Ubuntu 20.04. Older Ubuntu versions need to adapt the gcc compiler, mainly the C++17 standard. For updated Ubuntu, you need to install the corresponding ROS version yourself.
- Before compiling the code in this book, please install the following libraries (if not installed on your machine)
    - ROS Noetic: http://wiki.ros.org/noetic/Installation/Ubuntu
    - 使用以下指令安装其余的库
    ```bash
    sudo apt install -y ros-noetic-pcl-ros ros-noetic-velodyne-msgs libopencv-dev libgoogle-glog-dev libeigen3-dev libsuitesparse-dev libpcl-dev libyaml-cpp-dev libbtbb-dev libgmock-dev
    ```
    - Pangolin: https://github.com/stevenlovegrove/Pangolin
    - `thirdparty/g2o` Install the library under this repo through cmake, make
- After that, use the usual cmake, make method to compile all the contents of this book. For example
    ```bash
    mkdir build
    cd build
    cmake ..
    make -j8
    ```
- The executable files of each chapter after compilation are located in the `bin` directory

## TODO items 
- Some illustrations require authorization
- Remove unnecessary information in the UI (right panel)
- Organize the data set (add a few seq)
- Chapter 9 There seems to be a problem with the 0th key frame of the front end
- LioPreiteg does not converge on some datasets

## NOTES
- [Confirmed] ULHK's IMU seems to be different from others, it has gone to gravity
- [Confirmed] NCLT's IMU was converted to a Lidar system when it was subcontracted, so there is no external parameter of rotation between the Lidar and the IMU (the Lidar was originally rotated 90 degrees), and now the Lidar is X, left, Y, and then Z, The original car is X front Y right Z bottom. The NCLT data used in this book are all based on the point cloud system, and the lever arm of the IMU is ignored.
- [Confirmed] NCLT's rtk fix is ​​not very stable, and the average error is at the meter level
