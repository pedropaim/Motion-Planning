# Motion-Planning
Udacity - Autonomous Flight Nanodegree Program - Motion Planning Project

## Rubric Points

### Explaining Starter Code

#### 1. Explain the funcionality of what is provided in 'motion_planning.py'and 'planning_utils.py'

##### a. The 'motion_planning.py' file

The 'motion_planning.py' file contains the declaration of a class MotionPlanning, which inherits from a 'Drone' class. The main program creates an instance of the MotionPlanning class, called 'drone', and launches class method 'start', which initiates the connection with the drone. The MotionPlanning class also contains methods describing what the drone should do whenever 'Local Position', 'Velocity' or 'State' messages are received. These methods cause the drone to initially ARM, then TAKEOFF to a selected height, then plan a path using the 'plan_path' function, send the waypoints to the simulator, fly towards each waypoint in sequence, then land and disarm the drone.

As provided, the 'plan_path' function sets the start position as the center of the map and the goal position as a point located 10 m north and 10 m east of the center. This causes the drone to takeoff, move towards the center of the map, then follow a set of waypoints along a zig-zag trajectory towards the goal position. The zig-zag trajectory comes from the fact that the A* planning algorithm provided only adopts motions in the North, East, South and West directions as valid actions, and that it uses a 1m x 1m grid for planning, without any path pruning.

##### b. The 'planning_utils.py' file

The 'planning_utils.py' file contains functions to support the 'motion_planning.py' file. Relevant functions in the 'planning_utils.py' file are :
- the 'create_grid' function, which takes in variables 'data' containing the location and dimensions of all the obstacles, the selected drone altitude and safety distance to be adopted. The function outputs a grid containing a representation of the 2D configuration space, where the spaces occupied by obstacles are assigned with the value '1' (considering the adopted safety distance and drone altitude), and free spaces are assigned with '0'.
- the 'a_star' function, which plans the trajectory from the 'start' node to the 'goal' node, using the A* algorithm. The 'a_star' function in turn uses a 'valid_actions' function, which determines what actions are feasible from a given node, considering the presence of obstacles and the limits of the  grid. 


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

The program allows the goal position to be entered in four different ways, which are selectable by the user :

![alt text][figure]

[figure]: https://github.com/pedropaim/Motion-Planning/blob/main/Figure_01.png "Goal Selection Menu"

* Option '0' will set the goal as the home coordinates, at the center of the map.
* Option '1' will set the goal as in the original 'motion_planning.py' file, as 10 m North and 10 m East of the center of the map.
* Option '2' will set the goal as a pre-defined position at Lat 37.795663 Lon -122.40105.
* Option '3' will set the goal as latitude and longitude values input by the user, while also checking the validity of the user input (within the map's limits and not colliding with any obstacle). This is accomplished through function 'user_input_goal', which is contained in file 'supporting_functions.py'. The function initially prompts the user to input latitude and longitude values, while also presenting the map's latitude and longitude limits.  
* Option '4' will set the goal as a random coordinate inside the map. This is accomplished through function 'random_goal', which is contained in file 'supporting_functions.py'. The function generates two random numbers for local x and y coordinates within the map limits, then checks for collision with any obstacles. If there is a collision, then a new pair of random x and y coordinates is generated.

Whenever the goal coordinates are entered as latitude and longitude through variables goal_lat, goal_lon (options 2, 3 or 4), these variables are then converted to local coordinates using function 'global_to_local'. The result is then converted to integers:

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
    SOUTH = (-1, 0, 1)
    NORTH = (1, 0, 1)
    SOUTHWEST = (-1, -1, 1.414)
    SOUTHEAST = (-1, 1, 1.414)
    NORTHWEST = (1, -1, 1.414)
    NORTHEAST = (1, 1, 1.414)

```

The 'valid_actions' function is modified to check if actions in a diagonal direction will result off the grid or cause a collision, and remove these actions from the list of valid actions as appropriate :

```

    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTHWEST)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTHEAST)
    if x + 1 > n or y - 1  < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.NORTHWEST)
    if x + 1 > n or y + 1  < m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.NORTHEAST)

``` 

#### 6. Cull waypoints

After the computation of the trajectory using the A* algorithm, the path is pruned to remove unnecessary waypoints by conducting a collinearity check. Function 'prune_path' cycles through all the waypoints of the path and checks for collinearity. If a given set of waypoints is collinear (within a specified tolerance epsilon), then the center waypoint is removed from the path and the function moves on to check collinearity using the subsequent waypoint. The 'prune_path' function is transcribed below: 

```
def prune_path(path):
    if path != None and (len(path) >= 3):
        if len(path) >= 3:
            p_a = path[0]
            p_b = path[1]
            p_c = path[2]
            pruned_path = []
            pruned_path.append(p_a)
            for i in range(len(path)-3) :
                if collinearity_check(p_a,p_b,p_c):
                    p_b = p_c
                    p_c = path[i+3]
                else :
                    pruned_path.append(p_b)
                    p_a = p_b
                    p_b = p_c
                    p_c = path[i+3]
            pruned_path.append(p_c)
    else :
        pruned_path = path
    return pruned_path
```


The 'prune_path' function calls a supporting function 'collinearity_check', which receives a set of three 2D points and checks if they are collinear based on the idea that if the points are collinear, then the determinant of a 3x3 matrix composed of the three points (and 1 on the last column) will be equal to zero. It also adopts a tolerance 'epsilon' for the computation of the determinant in the collinearity check. The function is transcribed below:

```

def collinearity_check(p1, p2, p3, epsilon = 3):
    collinear = False
    x_1 = p1[0]
    y_1 = p1[1]
    x_2 = p2[0]
    y_2 = p2[1]
    x_3 = p3[0]
    y_3 = p3[1]
    det = x_1*(y_2 - y_3) + x_2*(y_3 - y_1) + x_3*(y_1 - y_2)
    if abs(det) < epsilon:
        collinear = True
    return collinear

```
#### 7. Adjusting Drone Heading

After path planning and removing unnecessary waypoints, the program calls function 'drone_heading' (declared in file supporting_functions.py), which adjusts the heading of the drone on each leg of the path, so that the drone's heading will always be towards the next waypoint:

```
def drone_heading(waypoints):
    for i in range(1,len(waypoints)-1):
        north_2 = waypoints[i][0]
        north_1 = waypoints[i-1][0]
        east_2 = waypoints[i][1]
        east_1 = waypoints[i-1][1]
        heading = np.arctan2(east_2 - east_1, north_2 - north_1)
        waypoints[i][3] = heading
    return waypoints
```


### Executing the flight

The figure below presents the trajectory followed by the drone from the original starting point (map center) to the original goal position (10 m North and 10 m East of map center) after modifying the 'valid_actions' function to include diagonal motions and after implementing a collinearity check for path pruning. The path, which had been previously a zig-zagging path with only North and East motions with one waypoint at every grid point, became a direct path from start waypoint to the goal position with one single waypoint.

![alt text][figure_02]

[figure_02]: https://github.com/pedropaim/Motion-Planning/blob/main/Figure_02.png "Single waypoint path"

The figure below presents a more complex trajectory followed by the drone, when it is originally several blocks from the home position and is commanded to return to the home position.

![alt text][figure_03]

[figure_03]: https://github.com/pedropaim/Motion-Planning/blob/main/Figure_03.png "Several waypoint path"
