# Mini Project

## Overview

This group assignment detects the location of the ArUco markers and moves the wheels of the robot to positions (0/180 degrees) according to the quadrant in which the marker was detected while displaying the position on the LCD screen.

## Purpose

The purpose of this project checkpoint is accomplishing the goal described above and practicing working in groups on integrated subsystems where collaboration is necessary. Additionally, this project facilitates the usage of GitHub and the importance of documentation with it.

## Modules

The following is the general description for each module used in the system.

### Python

Uses a function to open the camera and classes for finding the ArUco marker, tracking its quadrant location, and updating the LCD display accordingly. Also, the queue and threading python modules are used to speed up processing and allow multiple functions to be run at the same time.

### Arduino

Uses a separate encoder ISR interrupt for each wheel to allow independent movement. Also, two functions for drive are written: one for velocity, the other for position. Both functions factor in the error and integral defined based on the calculations of the current and intended positions.
