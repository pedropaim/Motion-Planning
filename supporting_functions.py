import numpy as np
from shapely.geometry import Polygon, Point
from udacidrone.frame_utils import global_to_local, local_to_global

def extract_polygons(data):
    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        corner_1 = (north - d_north, east - d_east)
        corner_2 = (north - d_north, east + d_east)
        corner_3 = (north + d_north, east + d_east)
        corner_4 = (north + d_north, east - d_east)
        corners = [corner_1, corner_2, corner_3, corner_4]
        height = alt + d_alt
        p = Polygon(corners)
        polygons.append((p, height))
    return polygons

def collides(polygons, point):
    for polygon in polygons :
        (p, height) = polygon
        if point[2] <= height :
            if p.contains(Point(point[0],point[1])) == True :
                return True
    else :
        return False

def random_goal(data, z_candidate):
    print('*** ASSIGN_GOAL ***')
    xmin = np.min(data[:, 0] - data[:, 3])
    xmax = np.max(data[:, 0] + data[:, 3])
    ymin = np.min(data[:, 1] - data[:, 4])
    ymax = np.max(data[:, 1] + data[:, 4])

    valid_goal = False

    print('*** CONVERT DATA TO POLYGONS ***')
    polygons = extract_polygons(data)

    while valid_goal == False:
         x_candidate = int(np.random.uniform(xmin, xmax, 1)[0])
         y_candidate = int(np.random.uniform(ymin, ymax, 1)[0])
         print('Random Point :',x_candidate, y_candidate)
         print('TEST FOR COLLISION WITH OBSTACLES')
         collision = False
         for polygon in polygons :
             (p, height) = polygon
             if z_candidate <= height:
                 if p.contains(Point(x_candidate,y_candidate)) == True :
                     collision = True
         if collision == False :
             valid_goal = True
    return (x_candidate,y_candidate)
    
def user_input_goal(data, drone_altitude, safety_distance, global_home):
        
    # Find limits :
    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))
    
    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))
    
    lon_min, lat_min, up_min = local_to_global((north_min,east_min,0),global_home)
    lon_max, lat_max, up_max = local_to_global((north_max,east_max,0),global_home)
    
    valid_input = False
    
    while valid_input == False :
    
    
        print('''
    
    Grid limits :
    
    Longitude : [ {0} .. {1} ]
    Latitude  : [ {2} .. {3} ]
    
    Select desired goal coordinates :
    
        '''.format(lon_min,lon_max,lat_min,lat_max))
    
        print('Goal longitude :   (or press enter for -122.399130)')
        user_input = input() or "-122.399130"
        goal_lon = float(user_input)
        print('Goal latitude : (or press enter for 37.793586)')
        user_input = input() or "37.793586"
        goal_lat = float(user_input)
    
        print('Selected Coordinates :')
        print('Latitude : ',goal_lat)
        print('Longitude : ',goal_lon)
    
        # Check if user input is valid :

        # a. check if within limits
        
        if goal_lon < lon_min or goal_lon > lon_max or goal_lat < lat_min or goal_lat > lat_max :
            print('Input outside grid limits.')
        # b. check if collision with obstacles
        else :
            polygons = extract_polygons(data)
            goal_x, goal_y, goal_z = global_to_local([goal_lon, goal_lat,0],global_home)
            collision = False
            
            for polygon in polygons :
                (p, height) = polygon
                if p.contains(Point(goal_x,goal_y)) == True :
                    collision = True
            if collision == False :
                valid_input = True
                
    return goal_lon, goal_lat
    
def drone_heading(waypoints):
    for i in range(1,len(waypoints)):
        north_2 = waypoints[i][0]
        north_1 = waypoints[i-1][0]
        east_2 = waypoints[i][1]
        east_1 = waypoints[i-1][1]
        heading = np.arctan2(east_2 - east_1, north_2 - north_1)
        waypoints[i][3] = heading
    return waypoints
        

           
