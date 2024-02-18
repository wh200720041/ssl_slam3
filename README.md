# SSL_SLAM3 
## Lightweight 3-D Localization and Mapping for Solid-State LiDAR (Intel Realsense L515 as an example)

### This work provides a basic fusion framework that fuses LiDAR and IMU information to improve the stability performance of SSL_SLAM

If you would like to enable save map and test localization separately, you can check this repo: [SSL_SLAM2](https://github.com/wh200720041/ssl_slam2)

This code is an improved implementation of paper "Lightweight 3-D Localization and Mapping for Solid-State LiDAR", accepted in IEEE Robotics and Automation Letters, 2021

A summary video demo can be found at [Video](https://youtu.be/Uy_2MKwUDN8) 

**Modifier:** [Wang Han](http://wanghan.pro), Nanyang Technological University, Singapore

## 1. Solid-State Lidar Sensor Example
### 1.1 Scene reconstruction
<p align='center'>
<a href="https://youtu.be/Ox7yDx6JslQ">
<img width="65%" src="/img/3D_reconstruction.gif"/>
</a>
</p>

### 1.2 SFM building example
<p align='center'>
<a href="https://youtu.be/D2xt_5xm_Ew">
<img width="65%" src="/img/3DMapping.gif"/>
</a>
</p>

### 1.3 Localization and Mapping with L515
<p align='center'>
<a href="https://youtu.be/G5aruo2bSxc">
<img width="65%" src="/img/3D_SLAM.gif"/>
</a>
</p>

## 2. Prerequisites
### 2.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04.

ROS Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 2.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 2.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Tested with 1.8.1

### 2.4. **Trajectory visualization**
For visualization purpose, this package uses hector trajectory sever, you may install the package by 
```
sudo apt-get install ros-noetic-hector-trajectory-server
```
Alternatively, you may remove the hector trajectory server node if trajectory visualization is not needed

## 3. Build 
### 3.1 Clone repository:
```
    cd ~/catkin_ws/src
    git clone https://github.com/wh200720041/ssl_slam3.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

### 3.2 Download test rosbag
You may download our [recorded data](https://drive.google.com/file/d/1ed5KSiXcmBxnIMcHpiy41E5sM5NtAZra/view?usp=sharing) (7.8GB) if you dont have realsense L515, and by defult the file should be under home/user/Downloads
unzip the file 
```
cd ~/Downloads
unzip ~/Downloads/library.zip
```

### 3.3 Launch ROS
```
    roslaunch ssl_slam3 ssl_slam3.launch
```

## 4. Sensor Setup
If you have new Realsense L515 sensor, you may follow the below setup instructions

### 4.1 IMU calibration (optional)
You may read official document [L515 Calibration Manual] (https://github.com/l515_calibration_manual.pdf) first

use the following command to calibrate imu, note that the build-in imu is a low-grade imu, to get better accurate, you may use your own imu
```
cd ~/catkin_ws/src/ssl_slam3/l515_imu_calibration
python rs-imu-calibration.py
```

### 4.2 L515
<p align='center'>
<img width="35%" src="/img/realsense_L515.jpg"/>
</p>

### 4.3 Librealsense
Follow [Librealsense Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

### 4.4 Realsense_ros
Copy [realsense_ros](https://github.com/IntelRealSense/realsense-ros) package to your catkin folder
```
    cd ~/catkin_ws/src
    git clone https://github.com/IntelRealSense/realsense-ros.git
    cd ..
    catkin_make
```

### 4.5 Launch ROS
Make Lidar still for 1 sec to estimate the initial bias, otherwise will cause localization failure!
```
    roslaunch ssl_slam3 ssl_slam3_L515.launch
```

## 5. Citation
If you use this work for your research, you may want to cite the paper below, your citation will be appreciated 
```
@article{wang2021lightweight,
  author={H. {Wang} and C. {Wang} and L. {Xie}},
  journal={IEEE Robotics and Automation Letters}, 
  title={Lightweight 3-D Localization and Mapping for Solid-State LiDAR}, 
  year={2021},
  volume={6},
  number={2},
  pages={1801-1807},
  doi={10.1109/LRA.2021.3060392}}
```
