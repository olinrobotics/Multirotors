#map gui functions
def map_callback(self, data):
    self.google_waypoints = [[point.x, point.y, point.z] for point in data.waypoint.points]
    self.do_takeoff_start = data.takeoff
    self.do_rtl_end = data.rtl
    self.do_hold_forever_end = data.hold_forever
    if data.dest == 'mission':
        self.set_mission_to_map()
    if data.dest == 'guided':
        self.set_guided_to_map()
    if data.dest == 'bounds':
        self.set_bounds_to_map()

def launch_map(self):
    rospack = rospkg.RosPack()
    os.system('gnome-terminal -x %s/scripts/start_map.sh' %rospack.get_path('drone_control'))

def set_guided_to_map(self):
    self.guided_waypoints = self.google_waypoints
    self.guided_index = 0
    print "guided points reset"

def set_mission_to_map(self):
    self.waypoints = [self.make_global_waypoint(lat, lon, alt) for [lat, lon, alt] in self.google_waypoints]
    if self.do_takeoff_start:
        self.start_with_takeoff()
    if self.do_rtl_end:
        self.end_with_rtl()
    if self.do_hold_forever_end:
        self.continue_mission()
    self.push_waypoints()

def set_bounds_to_map(self):
    #set the boundaries of a bounded flight (z is unused)
    print 'setting bounds'
    self.BOUNDEDFLIGHT = True
    self.bounds = [[x, y] for [x, y, z] in self.google_waypoints]

#guided mode functions
def start_guided(self):
    self.mode[0] = 'guided'
    self.srv_mode[0](0, '4')
    self.guided_index = 0
    print "guided mode started"

def set_guided_waypoint(self, lat, lon, alt=DEFAULT_ALT):
    waypoint = self.make_global_waypoint(lat, lon, alt)
    waypoint.is_current = 2
    res = self.srv_wp_push[0]([waypoint])
    if res.success:
        print "set waypoint"
    else:
        print "waypoint failed"