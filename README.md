# ur-camera-calibration

Hand-eye calibration with a robot-mounted camera for UR robots.

## Hardware

* UR10 CB3 robot
* Raspberry Pi 4B
* Intel RealSense D415 RGBD camera
* PC with Windows/Linux and Python

## Scenario

UR10 CB3 robot with Ethernet connection and a RealSense D415 RGBD camera mounted on its end-effector. The camera itself is connected to a Raspberry Pi 4B. An external PC with Windows or Linux is used for calibration commands.

This software extracts the extrinsic parameters of a camera mounted on the end-effector of a robotic arm. A calibration grid has to be previously located in a known location of the scenario where the transformation matrix from the global origin to its position is known.

The calibration script moves the robot end-effector to several positions where this board is visible and it takes pictures at each of those positions, detecting the center of the board in every picture. Using this information it fits a transformation between the end-effector position and the camera origin using singular value decomposition (SVD) to minimize the least-square error between the point sets, thus obtaining the extrinsic parameters of the current camera configuration. 

<img src="/docs/camera_calibration.png" width="600">

## Software description

Two main parts.

### Camera TCP server

The Raspberry Pi is running a TCP server acting as a bridge between the RGBD camera and the calibration script.

### Calibration script

The calibration script is written in Python and sends commands over Ethernet to the camera TCP server and the UR10 robot.

## Parameter setup

### Camera TCP server

Set camera calibration parameters in the JSON file.

### Calibration script

1. The IP address of the camera TCP server must be set in **camera.py** script.
2. Set default home and photo positions in **robot.py** script.
3. Set IP address of the UR robot in **calibrate.py** script.
4. Set selected checkerboard size and world position in **calibrate.py** script.
5. Set end-effector pose when taking pictures in **calibrate.py** script.
6. Define workspace limits and step for robot movement in **calibrate.py** script. This will define the positions where the pictures of the calibration board are taken.

## Usage

Build and run camera TCP server on Raspberry Pi.

Run **calibrate.py** script on calibrating PC.

## Acknowledgments

https://github.com/IntelRealSense/librealsense

https://github.com/andyzeng/visual-pushing-grasping

https://github.com/hengguan/Hand-Eye-Calibration-Matlab

https://github.com/nghiaho12/rigid_transform_3D
