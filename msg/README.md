# Custom messages to make things easier

### mission.msg:

Holds all of the variables we want to play with when making waypoints,
	Used to pass waypoints from the map gui to mission_planner.py
Contains a 3D polygon of waypoints, booleans for start and end conditions,
	and a string for where the waypoints should go (mission, guided, bounds)

### stick_cmd.msg:

Holds the four stick commands for controlling a multirotor
	(roll, pitch, yaw, throttle)
Used in modes where anything connected to the computer is directly sending
	RC commands to control the drone

### toggle_cmd.msg

Holds all toggle-able commands such as arm, disarm, flight mode, rtl
Always available as overrides for whatever is currently happening