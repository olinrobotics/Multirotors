'''
    OLD CODE

    Parses an XML file into a mission for the drone to follow
    Needs to be reviewed to see if it still works
'''

import xml.etree.ElementTree as ET
import rospkg
from mavros_msgs.msg import Waypoint

def get_mission(mission_num=0):
    rospack = rospkg.RosPack() 
    package_path = rospack.get_path('snotbot')
    tree = ET.parse(package_path + '/scripts/missions.xml')

    for mission in tree.getroot():
        if mission.attrib['id'] == mission_num:
            break

    waypoint_list = []
    for item in mission:
        waypoint = Waypoint()

        waypoint.frame = int(item.find('frame').text)
        waypoint.command = int(item.find('command').text)
        waypoint.is_current = bool(item.find('is_current').text)
        waypoint.autocontinue = bool(item.find('autocontinue').text)

        waypoint.param1 = float(item.find('param1').text)
        waypoint.param2 = float(item.find('param2').text)
        waypoint.param3 = float(item.find('param3').text)
        waypoint.param4 = float(item.find('param4').text)
        waypoint.x_lat = float(item.find('latitude').text)
        waypoint.y_long = float(item.find('longitude').text)
        waypoint.z_alt = float(item.find('altitude').text)
        
        waypoint_list.append(waypoint)

    return waypoint_list

def make_waypoint(frame, command, is_current, lat, lon, alt, hold):
    waypoint = Waypoint()

    waypoint.frame = frame
    waypoint.command = command
    waypoint.is_current = is_current
    waypoint.autocontinue = True
    waypoint.param1 = hold #hold time
    waypoint.param2 = 0
    waypoint.param3 = 0
    waypoint.param4 = 0
    waypoint.x_lat = lat
    waypoint.y_long = lon
    waypoint.z_alt = alt

    return waypoint

# Returns set of waypoints that allows drone to takeoff from its current position
def takeoff_waypoints(alt=5.0):
    waypoint_list = get_mission('takeoff')
    waypoint_list[0].z_alt = alt
    waypoint_list[1].z_alt = alt

    return waypoint_list
