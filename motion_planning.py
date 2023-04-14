import argparse
import time
# fast and small binary serialization
# alternative to json https://msgpack.org/
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from udacidrone.frame_utils import local_to_global


class States(Enum):
    # auto() generates enum value automatically, in this case value will be "MANUAL" for the first value
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

def get_grid_center_lat_lon():
    with open('colliders.csv') as f:
        coords = f.readline().split()
        lat0 = float(coords[1].replace(',', ''))
        lon0 = float(coords[3])
        return lat0, lon0

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
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
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

    def get_goal_not_in_obstacle(self, grid, grid_goal):
        if grid[grid_goal] != 1:
            return grid_goal
        north, east = grid_goal
        new_north = north + 1
        while new_north < grid.shape[0]:
            if grid[new_north, east] != 1:
                return (new_north, east)
            new_north += 1
        # if this strategy didn't work move east
        new_east = east + 1
        while new_north < grid.shape[1]:
            if grid[north, new_east] != 1:
                return (north, new_east)
            new_east += 1

        # if it doesn't work move south
        new_north = north - 1
        while new_north >= 0:
            if grid[new_north, east] != 1:
                return (new_north, east)
            new_north -= 1
        # if this strategy didn't work move east
        new_east = east - 1
        while new_north > 0:
            if grid[north, new_east] != 1:
                return (north, new_east)
            new_east -= 1

        # todo in the future do something smarter, like selecting the closest point
        raise NotImplementedError("Correct searching for closest place without "
                                  "obstacle needs to be implemented")
        # point in terms of distance


    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        lat0, lon0 = get_grid_center_lat_lon()

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(float(lon0), float(lat0), 0)

        # TODO: retrieve current global position
        global_position = self.global_position
        print(global_position)

        # TODO: convert to current local position using global_to_local()
        # print("local position: " + str(self.local_position))
        current_local_position = global_to_local(global_position, self.global_home)
        print("current local posistion: " + str(current_local_position))
        # print("local position updated: " + str(self.local_position))
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)

        # TODO: convert start position to current position rather than map center
        # the problem is that drone is right in obstacle which is impossible
        grid_start = (int(current_local_position[0]) - north_offset, int(current_local_position[1]) - east_offset)

        print("start position:" + str(grid_start))
        print("grid start is in obstacle:" + str(grid[grid_start]))
        # # there is a bug in data - start postion is right in the obstacle - so we can't plan any path
        # grid_start = self.get_goal_not_in_obstacle(grid, grid_start)
        print("grid start after update: " + str(grid_start))
        # print(current_local_position)


        
        # Set goal as some arbitrary position on the grid
        # grid_goal = (-north_offset + 10, -east_offset + 10)
        # TODO: adapt to set goal as latitude / longitude position and convert
        # find goal coordinates in 10 meters
        # in order to calculate coords in 10 meters from here
        # -122.39722237,  37.79266315
        # goal_local_position_orig = [current_local_position[0] + 10,
        #                                     current_local_position[1] + 10,
        #                                     current_local_position[2]]
        #
        # grid_goal_global = local_to_global(goal_local_position_orig, self.global_home)
        # print("grid_goal_global: " + str(grid_goal_global))

        # global goal in 10 meters
        # goal_global = [-122.39722237,  37.79266315, TARGET_ALTITUDE]
        # test some other global goals
        # Set goal as some arbitrary position on the grid
        # goal_global = [-122.398805, 37.793372, 0]  # up market
        goal_global = [-122.398805, 37.793372, 0]  # next corner
        # goal_global = [-122.398321, 37.791719, 0]  # down market
        # goal_global = [-122.397762, 37.793118, 0]  # around the building

        print(goal_global)
        goal_local_position = global_to_local(goal_global, self.global_home)
        print("grid_goal_local " + str(goal_local_position))
        grid_goal = (int(goal_local_position[0]) -north_offset, int(goal_local_position[1]) - east_offset)


        # we can't reach the goal if it's near the obstacle
        # so let's move to north and to the east until we find a point without
        # and obstacle
        print(grid[grid_goal])
        print(grid_goal)
        grid_goal = self.get_goal_not_in_obstacle(grid, grid_goal)
        print(grid[grid_goal])
        print(grid_goal)

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        print("len of orig path: " + str(len(path)))
        path = prune_path(path)
        print("len of pruned path: " + str(len(path)))

        # TODO (if you're feeling ambitious): Try a different approach altogether!
        # todo: move to graph

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
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
