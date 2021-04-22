"""lab5 controller."""
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

# The Tiago robot has a couple more sensors than the e-Puck
# Some of them are mentioned below. We will use its LiDAR for Lab 5

# range = robot.getDevice('range-finder')
# range.enable(timestep)
# camera = robot.getDevice('camera')
# camera.enable(timestep)
# camera.recognitionEnable(timestep)
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# The display is used to display the map. We are using 360x360 pixels to
# map the 12x12m2 apartment
display = robot.getDevice("display")

# Odometry
pose_x     = 2.58
pose_y     = 8.9
pose_theta = 0

vL = 0
vR = 0

<<<<<<< HEAD
##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
mode = 'manual' # Part 1.1: manual mode
# mode = 'planner'
# mode = 'autonomous'
=======
Dist_Error = 0
Bearing_Error = 0
gain = .25
xR = 0
thetaR = 0
final = 0

##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
mode = 'manual' # Part 1.1: manual mode
#mode = 'planner'
#mode = 'autonomous'
>>>>>>> 073794a7113aa67c4f552db5d3e03444f0c64922

lidar_sensor_readings = []
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # remove blocked sensor rays

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

###################
#
# Planner
#
###################
<<<<<<< HEAD
if mode == 'planner':
# Part 2.3: Provide start and end in world coordinate frame and convert it to map's frame
    start_w = None # (Pose_X, Pose_Z) in meters
    end_w = None # (Pose_X, Pose_Z) in meters

    # Convert the start_w and end_W from webot's coordinate frame to map's
    start = None # (x, y) in 360x360 map
    ens = None # (x, y) in 360x360 map

# Part 2.3: Implement A* or Dijkstra's
=======

if mode == 'planner':
# Part 2.3: Provide start and end in world coordinate frame and convert it to map's frame
    """
    start_w = (pose_x,pose_y) # (Pose_X, Pose_Z) in meters
    end_w = (10,7) # (Pose_X, Pose_Z) in meters

    # Convert the start_w and end_W from webot's coordinate frame to map's
    start = (int(start_w[0]*30)-1,(360-int(start_w[1]*30)-1)) # (x, y) in 360x360 map
    end = (int(end_w[0]*30)-1,(360-int(end_w[1]*30)-1)) # (x, y) in 360x360 map
    print(start)
    print(end) 
    """
    
    start_w = (5.0,2.0) # (Pose_X, Pose_Z) in meters
    #start_w = (pose_x, pose_y)
    end_w = (8.5,10.5) # (Pose_X, Pose_Z) in meters

    # Convert the start_w and end_W from webot's coordinate frame to map's
    start = (int(start_w[0]*30),360-int(start_w[1]*30)) # (x, y) in 360x360 map
    end = (int(end_w[0]*30),360-int(end_w[1]*30)) # (x, y) in 360x360 map
    
# Part 2.3: Implement A* or Dijkstra's
    def heuristic(point1, point2):
        x1,y1 = point1
        x2, y2= point2
        return abs(x1 - x2) + abs(y1 - y2)

>>>>>>> 073794a7113aa67c4f552db5d3e03444f0c64922
    def path_planner(map, start, end):
        '''
        :param map: A 2D numpy array of size 360x360 representing the world's cspace with 0 as free space and 1 as obstacle
        :param start: A tuple of indices representing the start cell in the map
        :param end: A tuple of indices representing the end cell in the map
        :return: A list of tuples as a path from the given start to the given end in the given maze
        '''
<<<<<<< HEAD
        pass


# Part 2.1: Load map (map.npy) from disk and visualize it



# Part 2.2: Compute an approximation of the “configuration space”



# Part 2.3 continuation: Call path_planner


# Part 2.4: Turn paths into goal points and save on disk as path.npy and visualize it

=======
        
        # A* SEARCH ALGORITHM: We used psuedocode from wikipedia as a guide for our code:
        # https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
        
        map = np.fliplr(map)
        map = np.rot90(map)
        openSet = [start]
        
        map_w = len(map)
        map_l = len(map[0])
        
        cameFrom = [[(0,0) for i in range(map_l)]for j in range(map_l)]

        g_score = [[float('inf') for i in range(map_l)]for j in range(map_w)]
        f_score = [[float('inf') for i in range(map_l)]for j in range(map_w)]
        
        g_score[start[0]][start[1]] = 0
        f_score[start[0]][start[1]] = heuristic(start, end)
        
        while len(openSet) > 0:
            current = openSet[0]
            for i in openSet:
                if f_score[i[0]][i[1]] < f_score[current[0]][current[1]]:
                    current = i
            openSet.remove(current)
            if current == end:
                #reconstruct the path
                total_path = [current]
                while current != start:
                    current = cameFrom[current[0]][current[1]]
                    total_path.insert(0,current)
                return total_path
                             
            neighbors = [(current[0]-1, current[1]-1), (current[0]-1, current[1]),
                (current[0]-1, current[1]+1), (current[0], current[1]-1),
                (current[0], current[1]+1), (current[0]+1, current[1]-1),
                (current[0]+1, current[1]), (current[0]+1, current[1]+1)
                ]
            
            for n in neighbors:
                i = current[0] - n[0]
                j = current[1] - n[1]
                tentative_score = g_score[current[0]][current[1]] + np.linalg.norm([i,j])
                if 0 <= n[0] < len(map[0]) and 0 <= n[1] < len(map):
                    if map[n[0]][n[1]] < 0.1 and tentative_score < g_score[n[0]][n[1]]:
                        cameFrom[n[0]][n[1]] = current
                        g_score[n[0]][n[1]] = tentative_score
                        f_score[n[0]][n[1]] = tentative_score + np.linalg.norm([end[0]-n[0],end[1]-n[1]])
                        if (n[0],n[1]) not in openSet:
                            openSet.append((n[0],n[1]))         


    # Part 2.1: Load map (map.npy) from disk and visualize it
    map = np.load("map.npy")
    
    #map = np.flipud(map)
 
    map[map>.5] = 1
    map[map<=.5] = 0
    fig,ax = plt.subplots()
    ax.set_title('Map')
    plt.imshow(map)
    #plt.imshow(map)



    # Part 2.2: Compute an approximation of the "configuration space"
    # Use convolution package to give some space from obstacles
    filter = np.ones((13,13))
    obs_map = convolve2d(map,filter)
    obs_map = obs_map[:-12,:-12]
    
    x = np.linspace(0, 359, 360)
    y = np.linspace(0, 359, 360)
    X, Y = np.meshgrid(x, y)

    obs_map[obs_map>=1] = 1
    fig,ax = plt.subplots()
    
    cp = plt.contourf(X, Y, obs_map)
    
    ax.set_title('Configuration Space Map')
    plt.imshow(obs_map)
    #plt.show()


# Part 2.3 continuation: Call path_planner
    path = path_planner(obs_map,start,end)

# Part 2.4: Turn paths into goal points and save on disk as path.npy and visualize it
    x = [i[0] for i in path]
    y = [i[1] for i in path]
    plt.plot(x,y)
    plt.show()
    
    path = np.array([((i[0])/30.0,(360.0-i[1])/30.0) for i in path])
    np.save("path.npy",path)
>>>>>>> 073794a7113aa67c4f552db5d3e03444f0c64922


# Part 1.2: Map Initialization

# Initialize your map data structure here as a 2D floating point array
if mode == 'manual':
<<<<<<< HEAD
    map = None # Replace None by a numpy 2D floating point array
=======
    map = np.zeros([360,360]) # Replace None by a numpy 2D floating point array
>>>>>>> 073794a7113aa67c4f552db5d3e03444f0c64922


if mode == 'autonomous':
# Part 3.1: Load path from disk and visualize it (Make sure its properly indented)
<<<<<<< HEAD
    pass
=======
    path = np.load("path.npy")
    
    
    for i in range(len(path)):
        x = path[i][0]
        y = path[i][1]
        pt_x, pt_y = (int(x*30)), (360-int(y*30))
        print((pt_x, pt_y))
        display.setColor( int(0xFF0000))
        display.drawPixel(pt_y,pt_x)
        
    #pass
>>>>>>> 073794a7113aa67c4f552db5d3e03444f0c64922

state = 0 # use this to iterate through your path

while robot.step(timestep) != -1 and mode != 'planner':

###################
#
# Sensing
#
###################
    # Ground truth pose
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]

    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            rho = LIDAR_SENSOR_MAX_RANGE

        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy =  -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y

        #print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))

        if rho < 0.5*LIDAR_SENSOR_MAX_RANGE:
# Part 1.3: visualize map gray values.
<<<<<<< HEAD

            # You will eventually REPLACE the following 2 lines with a more robust version of map
            # and gray drawing that has more levels than just 0 and 1.
            display.setColor(0xFFFFFF)
            display.drawPixel(360-int(wy*30),int(wx*30))

=======
            if mode == 'manual':
            # You will eventually REPLACE the following 2 lines with a more robust version of map
            # and gray drawing that has more levels than just 0 and 1.
            
                obs_x = int(wx*30)
                obs_y = int(wy*30)
                
                if(obs_x >= 0 and obs_x < 360 and obs_y >= 0 and obs_y < 360):
                    map[obs_x][obs_y] += 0.005
                    g = int(min(map[obs_x][obs_y], 1.0) * 255)
                    g = int((g*256**2+g*256+g))
                    display.setColor(g)
                    display.drawPixel(360-int(wy*30), int(wx*30))
           
>>>>>>> 073794a7113aa67c4f552db5d3e03444f0c64922
    display.setColor(int(0xFF0000))
    display.drawPixel(360-int(pose_y*30),int(pose_x*30))



###################
#
# Controller
#
###################
    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
# Part 1.4: Save map to disc
<<<<<<< HEAD

=======
            np.save("map.npy",map)
>>>>>>> 073794a7113aa67c4f552db5d3e03444f0c64922
            print("Map file saved")
        elif key == ord('L'):
            # You will not use this portion but here's an example for loading saved a numpy array
            map = np.load("map.npy")
<<<<<<< HEAD
=======
            plt.imshow(map, cmap='gray')
            plt.show()
>>>>>>> 073794a7113aa67c4f552db5d3e03444f0c64922
            print("Map loaded")
        else: # slow down
            vL *= 0.75
            vR *= 0.75
    else: # not manual mode
<<<<<<< HEAD
        pass
# Part 3.2: Feedback controller
        #STEP 1: Calculate the error


        #STEP 2: Controller


        #STEP 3: Compute wheelspeeds

=======
        #pass
# Part 3.2: Feedback controller
        #STEP 1: Calculate the error
        distance_error = math.hypot(path[state][0]-pose_x, path[state][1]-pose_y)
        bearing_error = (math.atan2(-(path[state][1]-pose_y), path[state][0]-pose_x)-pose_theta + math.pi)%(2*math.pi)-math.pi
    


        #STEP 2: Controller
        if (distance_error < 0.2 and state != len(path)-1):
            state = state + 1
            #Hit final waypoint stop
            if state == len(path):
                robot_parts[MOTOR_RIGHT].setVelocity(0)
                robot_parts[MOTOR_LEFT].setVelocity(0)
                break
            continue
        elif -0.5 < bearing_error < 0.5:
            gain1 = 5
        else:
            gain1 = 0
        
        rotation = 5*math.sqrt(abs(bearing_error))
        if bearing_error < 0:
            rotation = -rotation

        #STEP 3: Compute wheelspeeds
        
        vR = gain1*math.sqrt(distance_error) + (AXLE_LENGTH*rotation)/2
        vL = gain1*math.sqrt(distance_error) - (AXLE_LENGTH*rotation)/2
        
        #STEP 4: Normalize wheelspeed
        if vL > MAX_SPEED:
            vR = (vR/vL)*MAX_SPEED
            vL = MAX_SPEED

        if vR > MAX_SPEED:
            vR = MAX_SPEED
            vL = (vL/vR)*MAX_SPEED

        if vL < -MAX_SPEED:
            vR = -(vR/vL)*MAX_SPEED
            vL = -MAX_SPEED

        if vL < -MAX_SPEED:
            vR = -MAX_SPEED
            vL = -(vL/vR)*MAX_SPEED
    
>>>>>>> 073794a7113aa67c4f552db5d3e03444f0c64922

    # Normalize wheelspeed
    # Keep the max speed a bit less to minimize the jerk in motion


    # Odometry code. Don't change speeds after this
    # We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    # print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta)) #/3.1415*180))

    # Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
<<<<<<< HEAD
    robot_parts[MOTOR_RIGHT].setVelocity(vR)
=======
    robot_parts[MOTOR_RIGHT].setVelocity(vR)
>>>>>>> 073794a7113aa67c4f552db5d3e03444f0c64922
