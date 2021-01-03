import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

# ==========================================================================
import matplotlib.pyplot as plt
from supporting_functions import extract_polygons, collides, random_goal, user_input_goal, drone_heading
from planning_utils import prune_path
import csv
# ==========================================================================


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 4.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING

        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        print('Define home position :')
        # read lat0, lon0 from colliders into floating point values

        with open('colliders.csv', newline='') as f:
            reader = csv.reader(f)
            first_row = next(reader)  # gets the first line
        lat0 = float(first_row[0].split()[1])
        lon0 = float(first_row[1].split()[1])
        
        print('Lat0 =',lat0,' Lon0 =',lon0)
        
        # set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        plt.imshow(grid, origin='lower', cmap='Greys')

        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
    
        # convert start position to current position rather than map center
        current_north = self.local_position[0]
        current_east = self.local_position[1]
        grid_start = (int(-north_offset + np.rint(current_north)),int(-east_offset + np.rint(current_east)))


        # Set goal as some arbitrary position on the grid
        print(''' 

            *** Define Goal ***

            Select option and press enter :

            "0" : Original home coordinate ( Map Center)
            "1" : Original goal coordinate ( 10 m North, 10 m East )
            "2" : Fixed goal coordinate - Lat 37.795663 Lon -122.40105
            "3" : User input coordinate
            "4" : Random goal coordinate

              ''')

        user_option = input()

        if user_option == '0':
            goal_x = 0
            goal_y = 0
            
        elif user_option == '1':
            goal_x = 10
            goal_y = 10

        elif user_option == '2':
            goal_lat = 37.791693
            goal_lon =  -122.394335
            [goal_x, goal_y, goal_z] = global_to_local([goal_lon, goal_lat,TARGET_ALTITUDE],self.global_home)
            goal_x = int(goal_x)
            goal_y = int(goal_y)
        elif user_option == '3':
            goal_lon, goal_lat = user_input_goal(data, 0, 0,self.global_home)
            [goal_x, goal_y, goal_z] = global_to_local([goal_lon, goal_lat,0],self.global_home)
            goal_x = int(goal_x)
            goal_y = int(goal_y)
        elif user_option == '4':
            goal_x, goal_y = random_goal(data,TARGET_ALTITUDE)
        else :
            print('INVALID OPTION')    


        grid_goal = (-north_offset + goal_x, -east_offset + goal_y)

        # Run A* to find a path from start to goal
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)

        #plt.plot(grid_start[1], grid_start[0], 'rx')
        #plt.plot(grid_goal[1], grid_goal[0], 'gx')

        #plt.xlabel('EAST')
        #plt.ylabel('NORTH')

        print('*** CALLING A_STAR ***')
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print('*** PRUNING PATH ***')
        # prune path to minimize number of waypoints
        pruned_path = prune_path(path)

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
        # Adjust drone heading on each leg
        wayponts = drone_heading(waypoints)
        
        #plt.show()
        
        # Set self.waypoints
        self.waypoints = waypoints
        # send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
