# Code Organization

The folders have been temporarily organized by task, and the file `drone.py` contains a helper class that collects important data from `mavros` via callback functions.

# To Do
- [x] Merge all important files from old repo
- [ ] Update `/Fiducial Tracking` to use `/ar_pose` instead of old fiducial
- [ ] Review and revise `/Mission Planning` files (test that everything works)
- [x] Reorganize folders based on arbiter / state / sense

### Control of Drone
##### Keyboard Control
* `<space>`: RTL
* `<enter>`: Land
* `r`: arm
* `t`: disarm
* `o`: auto
* `l`: Loiter
* `m`: Stabilize (think manual)
* `p`: start planner
TODO: implement arrows and wasd to control drone, with q and z as alt adjustments

##### Joystick Control
We have control systems implemented for a joystick, an xbox controller, and a flight sim controller
TODO: document how all of those controls work (for now ask someone)