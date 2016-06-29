# Code Organization

The folders have been temporarily organized by task, and the file `drone.py` contains a helper class that collects important data from `mavros` via callback functions.

# To Do
- [x] Merge all important files from old repo
- [ ] Update `/Fiducial Tracking` to use `/ar_pose` instead of old fiducial
- [ ] Review and revise `/Mission Planning` files (test that everything works)
- [x] Reorganize folders based on arbiter / state / sense

### Control of Drone
##### Manual Override
Flipping ch6 on the RC transmitter to a pwm value above 1500 will give all control of RC channels back to the RC transmitter regardless of what the code is doing.

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

### Getting Code to Run
##### Dependencies
* ROS Indigo [install instructions](http://wiki.ros.org/indigo/Installation)
* OpenCV (`sudo apt-get install python-opencv`)
* Tkinter (`sudo apt-get install python-tk`)
* Numpy ([install instructions](http://www.scipy.org/install.html))
* ros_pid (`sudo apt-get install ros-indigo-pid`)
* usb_cam (`sudo apt-get install ros-indigo-usb-cam`)
* mavros (`sudo apt-get install ros-indigo-mavros ros-indigo-mavros-extras`)
* joy_node (`sudo apt-get install ros-indigo-joy`)
* TODO: there are probably things from fiducial tracking like ar_pose

##### Silly Nonsense
* after installing the dependencies, clone the Multirotors repository into `~/catkin_ws/src`
* from `~/catkin_ws` run `catkin_make`
* open your `.bashrc` file (in home directory) and add the line: `source ~/catkin_ws/devel/setup.sh` to the bottom
* run the command `source ~/.bashrc`
* now you should be able to run the code using `roslaunch multirotors barebones.launch`

##### Launch Files
###### `barebones.launch`
Input arguments:
- `joy_port` (default: `js0`) - location of joystick (`/dev/input/<joy_port>`)
- `drone_port` (default: `ttyUSB0`) - location of drone (`/dev/<drone_port`)

Other requirements:
- Requres a 900MHz radio plugged into a USB port to communicate with drone
- Will error without a joystick plugged into a USB port
  - Change variable in `joysticktest.py` to set which joystick you are using


###### `fiducial.launch`
start code for landing on a fiducial
TODO: document better
