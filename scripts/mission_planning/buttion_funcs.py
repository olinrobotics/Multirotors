#button action functions
def guided_function(self):
    if self.mode[0] == 'guided':
        if self.guided_index >= len(self.guided_waypoints):
            self.RTL()
        else:
            waypoint = self.guided_waypoints[self.guided_index]
            self.set_guided_waypoint(waypoint[0], waypoint[1], waypoint[2])
            self.guided_index += 1
    else:
        self.start_guided()

def RTL(self):
    self.mode[0] = 'RTL'
    self.srv_mode[0](0, '6')
    print 'returning to launch'

def push_waypoints(self):
    success = True
    for i in xrange(self.num_drones):
        res = self.srv_wp_push[i](self.waypoints)
        success &= res.success
    if success:
        print "Pushed waypoints"
    else:
        print "Failed to push waypoints"
    return success

def clear_waypoints(self):
    success = True
    for i in xrange(self.num_drones):
        res = self.srv_wp_clear[i]()
        success &= res.success

    if success:
        self.waypoints = []
        print "Cleared waypoints"
    else:
        print "Failed to clear waypoints"

def add_waypoints(self, coordinates):
    '''adds waypoints to the end of the current mission'''
    new_waypoints = [self.make_global_waypoint(lat, lon) for [lat, lon] in coordinates]
    self.waypoints.extend(new_waypoints)
    #self.continue_mission() #don't land at end of mission
    self.push_waypoints()

def reset_waypoints(self, waypoints):
    self.waypoints = waypoints
    #self.continue_mission()
    self.push_waypoints()
    self.restart_mission()

def begin_mission(self):
    self.mode[0] = 'auto'
    self.srv_mode[0](0, '3')
    rc_msg = OverrideRCIn()
    (rc_msg.channels[0], rc_msg.channels[1], rc_msg.channels[2], rc_msg.channels[3], rc_msg.channels[4]) = (1500, 1500, 1250, 1500, 1400) 
    self.pub_rc[0].publish(rc_msg)
    print "Beginning mission"

def end_mission(self):
    self.mode[0] = 'manual'
    self.srv_mode[0](0, '5')
    print "Ending mission"

def arm(self):
    for i in xrange(self.num_drones):
        self.srv_arm[i](True)
    print "Arming drones"

def disarm(self):
    for i in xrange(self.num_drones):
        self.srv_arm[i](False)
    print "Disarming drones"