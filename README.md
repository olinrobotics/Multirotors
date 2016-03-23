# Multirotors
Research in Andrew Bennett's Lab. Using Multirotors to conduct environmental exploration, sensing, and sampling missions.

## Projects
This code repository contains the mission files and the general architecture for several mission types which can be executed with drones, namely:

* Breath Condensate Collection from a Cetacean
* Wireless Tracker Tagging for a large Cetacean
* Photogrammetry for Animal Studies
* Point-of-Interest Data Collection
* Environmental Exploration
* Waypoint Navigation
* Sense and Avoid of Unfriendly Multirotors

Each of these missions is designed in a mission file. This mission file is then executed by a generic autonomy structure based upon State Configured Layer Control (SCLC) which controls the actions of the drone.

## Running this Code
*Still under construction*

#### RC override:
All computer control can be overridden by switching ch. 6 on the transmitter to a value greater than 1500.

#### keyboard controls:

m = stabilize (sort of manual)
l = loiter
o = auto
r = arm
t = disarm
p = open planner

#### joystick controls:

specific to joystick and not finalized yet