#mission functions
def continue_mission(self):
    ''' make drone hold position forever at end of mission instead of land '''
    final_lat = self.waypoints[-1].x_lat
    final_lon = self.waypoints[-1].y_long
    hold_pos = self.make_global_waypoint(final_lat, final_lon)
    hold_pos.command = 17
    self.waypoints.append(hold_pos)

def end_with_rtl(self):
    rtl = Waypoint()
    rtl.command = 20
    self.waypoints.append(rtl)

def start_with_takeoff(self):
    start = self.make_global_waypoint(0, 0)
    takeoff = Waypoint()
    takeoff.command = 22
    takeoff.z_alt = DEFAULT_ALT
    self.waypoints.insert(0, takeoff)
    self.waypoints.insert(0, start)

def make_global_waypoint(self, lat, lon, alt=DEFAULT_ALT, hold=5):
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('drone_control')
    waypoint = Waypoint()

    waypoint.frame = 3
    waypoint.command = 16
    waypoint.is_current = 0
    waypoint.autocontinue = False
    waypoint.param1 = hold #hold time
    waypoint.param2 = 2
    waypoint.param3 = 0
    waypoint.param4 = 0
    waypoint.x_lat = lat
    waypoint.y_long = lon
    waypoint.z_alt = alt

    return waypoint

def save_mission_to_file(self):
    root = ET.Element('missions')
    mission = ET.SubElement(root, 'mission', id='0')
    for (i, point) in enumerate(self.waypoints):
        waypoint = ET.SubElement(mission, 'waypoint', num=str(i))
        frame = ET.SubElement(waypoint, 'frame').text = str(point.frame)
        is_current = ET.SubElement(waypoint, 'is_current').text = str(point.is_current)
        autocontinue = ET.SubElement(waypoint, 'autocontinue').text = str(point.autocontinue)
        command = ET.SubElement(waypoint, 'command').text = str(point.command)
        param1 = ET.SubElement(waypoint, 'param1').text = str(point.param1)
        param2 = ET.SubElement(waypoint, 'param2').text = str(point.param2)
        param3 = ET.SubElement(waypoint, 'param3').text = str(point.param3)
        param4 = ET.SubElement(waypoint, 'param4').text = str(point.param4)
        lat = ET.SubElement(waypoint, 'latitude').text = str(point.x_lat)
        lon = ET.SubElement(waypoint, 'longitude').text = str(point.y_long)
        alt = ET.SubElement(waypoint, 'altitude').text = str(point.z_alt)
    tree = ET.ElementTree(root)
    package_path = rospkg.RosPack().get_path('drone_control')
    tree.write(package_path + '/scripts/last_mission.xml')
    print 'missoin saved: copy last_mission.xml to new file to keep'

def restart_mission(self):
    success = True
    for i in xrange(self.num_drones):
        self.srv_set_current[i](0)
    print "restarted mission"