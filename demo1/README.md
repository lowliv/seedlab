# Demonstration 1

## Overview

This assignment aims to demonstrate and implement the main part of each of the project's subsystems. The computer vision module detects the angle and distance of a marker in relation to the camera, while the control module moves according to a specified angle and distance.

## Purpose

The main purpose of this project checkpoint is the development of the initial groundwork for the project itself as described above. Each subsystem was worked on separately from the other, and the connection between modules will be created later.

## Modules

The following is the general description for each module used in the system.

### Python

The camera was calibrated by using the provided python code to take pictures of differently angled grid and then the built-in calibration functions from openCV. The Raspberry Pi subsystem uses ArUco marker detection and then calculates the angle and distance by using both the FOV angle and the calibration camera matrix for maximum accuracy. Also, the code uses queue and threading to analyze camera output and display finding on the LCD screen at the same time.

### Arduino

The tuning values were calculated through 2 wheel odometry, and the controller parameters were found mostly through Simulink. The control subsystem's position-based driving calculates the target velocity and then starts the velocity-based drive function. Moreover, multiple variables are used for updating velocity based on several averages.
