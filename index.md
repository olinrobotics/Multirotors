---
title: Olin Multirotor Projects
layout: template
filename: index
--- 

# What We Do
The [Olin Robotics Lab](olinrobotics.github.io) works on many projects involving multirotor aircraft equiped with pixhawk autopilots. The Multirotors repository contains all of our code for communicating with the Pixhawk, mainly using [Mavros](http://wiki.ros.org/mavros). We have a set of basic functions for sending missions to the pixhawk, changing its flight mode, and giving it pwm inputs.  This repository is also includes code for our specific software based projects.

## Our projects

### Landing on a fiducial
In an effort to establish a system in which drones are reliable research tools that require no previous piloting experience, we are working on an autonomous landing system in which drones will be able to return to a landing platform on a moving vehicle and land on as small of a platform as possible.  We are using AR codes on the platform and a downwards facing camera on the drone to accurately locate the drone relative to the target and land.

### Photogrametry
We are working on accurately dimensioning an object using a camera feed from a drone.  We are using the position of the drone and angle of the camera to find the distance between pixles in the image.

## Useful Information

- [Running Our Code](readme)