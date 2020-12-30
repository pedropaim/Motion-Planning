# Motion-Planning
Udacity - Autonomous Flight Nanodegree Program - Motion Planning Project

## Rubric Points

### Explaining Starter Code

#### 1. Explain the funcionality of what is provided in 'motion_planning.py'and 'planning_utils.py'

##### a. The 'planning_utils.py' file

The 'planning_utils.py' file contains functions to support the 'motion_planning.py' file. Relevant functions in the 'planning_utils.py' file are :
- the 'create_grid' function, which takes in variables 'data' containing the location and dimensions of all the obstacles, the selected drone altitude and safety distance to be adopted. The function outputs a grid containing a representation of the 2D configuration space, where the spaces occupied by obstacles are assigned with the value '1' (considering the adopted safety distance and drone altitude), and free spaces are assigned with '0'.
- the 'a_star' function, which plans the trajectory from the 'start' node to the 'goal' node, using the A* algorithm. The 'a_star' function in turn uses a 'valid_actions' function, which determines what actions are feasible from a given node, considering the presence of obstacles and the limits of the  grid. 

##### b. The 'motion_planning.py' file

The 'motion_planning.py' file contains the declaration of a class MotionPlanning, which inherits from a 'Drone' class. The main program creates an instance of the MotionPlanning class, called 'drone', and launches class method 'start', which initiates the connection with the drone. The MotionPlanning class also contains methods describing what actions the drone shall take whenever it receives message and  and contains methods for what the drone should do whenever 'Local Position', 'Velocity' or 'State' messages. These methods cause the drone to initially ARM, then TAKEOFF to a selected height, then plan a path using the 'plan_path' function, send the waypoints to the simulator, fly towards each waypoint in sequence, then land and disarm the drone.

As provided, the 'plan_path' function sets the start position as the center of the map and the goal position as a point located 10 m north and 10 m east of the center. This causes the drone to move to takeoff, move towards the center of the map, then follow a set of waypoints along a zig-zag trajectory towards the goal position. 

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

The global home position is initially extracted from the first line of the 'collision.csv' file, which contains information 'lat0 37.792480 lon0 -122.397450'. This is accomplished using function csv.reader, as follows :

```
with open('colliders.csv', newline='') as f:
            reader = csv.reader(f)
            first_row = next(reader)  # gets the first line
        lat0 = float(first_row[0].split()[1])
        lon0 = float(first_row[1].split()[1])
```        

The values in lat0 and lon0 are then assigned as the global home position as :

```self.set_home_position(lon0, lat0, 0)```

#### 2. Set your current local position

The drone's current position is obtained using:

```
        current_north = self.local_position[0]
        current_east = self.local_position[1]
```

#### 3. Set grid start position from local position

The grid start position is obtained from variables 'current_north' and 'current_east', by converting these values to integers using the np.rint function and by adjusting north and east grid offset :

```
        grid_start = (int(-north_offset + np.rint(current_north)),int(-east_offset + np.rint(current_east)))
```

#### 4. Set grid goal position from geodetic coords

The goal coordinates are entered as latitude and longitude through variables goal_lat, goal_lon. These variables are then converted to local coordinates using function 'global_to_local'. The result is then converted to integers:

```
            [goal_x, goal_y, goal_z] = global_to_local([goal_lon, goal_lat,TARGET_ALTITUDE],self.global_home)
            goal_x = int(goal_x)
            goal_y = int(goal_y)
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

Actions to directions 'NORTHWEST', 'NORTHEAST', 'SOUTHWEST'and 'SOUTHEAST' are included in the list of possible actions, using a cost of 1.414 (square-root of two) :

```
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTHWEST = (-1, -1, 1.414)
    NORTHEAST = (-1, 1, 1.414)
    SOUTHWEST = (1, -1, 1.414)
    SOUTHEAST = (1, 1, 1.414)

```

The 'valid_actions' function is modified to check if actions in a diagonal direction will result off the grid or cause a collision, and remove these actions from the list of valid actions as appropriate :

```

    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTHWEST)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTHEAST)
    if x + 1 > n or y - 1  < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTHWEST)
    if x + 1 > n or y + 1  < m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTHEAST)

``` 


#### 6. Cull waypoints


### Execute the flight

#### 1. Does it work?

# Extra Challenges: Real World Planning
