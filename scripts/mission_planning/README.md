# Using linux mission planning

## Opening the planner (requires internet)
- With the main code running (and your cursor in the launch file's terminal), press 'p' to open the planner.
- A tkinter window will appear with center latitude longitude coordinates for the map.
  - These coordinates are in the format latitude, longitude in1 decimal degrees
  - I suggest selecting the center by going to google maps and clicking on a location, the latitude, longitude will come up in the correct format for the planner gui, so you can just coppy paste the whole line into the prompt.
  - press enter or continue to map

## Using the map
- set the altitude of waypoints in the set alt box
- click anywhere to add a waypoint
- right click to delete last waypoint
- use arrows to go back to past waypoints and change altitude
  - you must hit enter to save the new altitude
note: the green dot is always the first waypoint

- clear clears all waypoints
- start with takeoff will add a takeoff command to the beginning of the mission
- end with RTL will put an rtl command at the end of the mission
- hold last waypoint will keep the drone at the final waypoint from the mission until you give it a new command

- Save as boundary saves the current set of waypoints as the outer bound for the mission
  - this must be a convex polygon
- Save to guided saves the current set of waypoints to a list which can be cycled through in guided mode
- Save to mission writes the current mission to the connected drone.

## When you're done
- Just close the map, everything is saved to the rest of the program