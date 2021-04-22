from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d


"""
class Color(Enum):
    RED = 0
    GREEN = 1
    BLUE = 2
    CYAN = 3
    NONE = 4

color_names = ["red", "green", "blue", "cyan"]
"""

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
    
    
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

#initialize camera
camera = robot.getDevice('camera')
camera.enable(timestep)
width = camera.getWidth()
height = camera.getHeight()


camera.recognitionEnable(timestep)
camera.enableRecognitionSegmentation()
print("Recognition enabled")
print(camera.getRecognitionSamplingPeriod())
print(camera.getRecognitionNumberOfObjects())


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

"""
def cam():
    image = camera.getImage()
    side=""
    obj=""
    red = 0
    green = 0
    blue = 0
    #width of 0-20px 1/2 height
    for i in range(0, 20):
        for j in range(0, int(height / 2)):
            red = camera.imageGetRed(image, width, i, j)
            blue = camera.imageGetBlue(image, width, i, j)
            green = camera.imageGetGreen(image, width, i, j)
    if(red!=0 and green!=0 and blue!=0):
        #print(red / (red + green + blue),green / (red + green + blue),blue / (red + green + blue))
        if (red/(red+green+blue)>0.45):
            current_blob = Color.RED.value
        elif ((green > 3 * red) and (green > 3 * blue)):
            current_blob = Color.GREEN.value
        elif (blue / (red + green + blue) > 0.45):
            if(red / (red + green + blue)<0.1 and green>red):
                current_blob = Color.CYAN.value
            else:
                current_blob = Color.BLUE.value

        else:
            current_blob = Color.NONE
        # print(current_blob)

        if (current_blob == Color.NONE):
            pass
        else:
            #print("Looks like I found a", color_names[current_blob],"0-20");
            side="L"
            obj=color_names[current_blob]
        # width of 20-59px 1/2 height
    for i in range(20,59):
        for j in range(0, int(height / 2)):
            red = camera.imageGetRed(image, width, i, j)
            blue = camera.imageGetBlue(image, width, i, j)
            green = camera.imageGetGreen(image, width, i, j)
    if(red!=0 and green!=0 and blue!=0):
        #print(red / (red + green + blue),green / (red + green + blue),blue / (red + green + blue))
        if(red / (red + green + blue)<0.4 and green / (red + green + blue)<0.4 and blue / (red + green + blue)<0.4):
            current_blob = Color.NONE
        if (red/(red+green+blue)>0.45):
            current_blob = Color.RED.value
        elif ((green > 3 * red) and (green > 3 * blue)):
            current_blob = Color.GREEN.value
        elif (blue / (red + green + blue) > 0.45):
            if (red / (red + green + blue) < 0.1 and green > red):
                current_blob = Color.CYAN.value
            else:
                current_blob = Color.BLUE.value
        else:
            current_blob = Color.NONE
        # print(current_blob)

        if (current_blob == Color.NONE):
            pass
        #else:
            #print("Looks like I found a", color_names[current_blob],"20-59");
    for i in range(59,80):
        for j in range(0, int(height / 2)):
            red = camera.imageGetRed(image, width, i, j)
            blue = camera.imageGetBlue(image, width, i, j)
            green = camera.imageGetGreen(image, width, i, j)
    if(red!=0 and green!=0 and blue!=0):
        #print(red / (red + green + blue),green / (red + green + blue),blue / (red + green + blue))
        if (red/(red+green+blue)>0.45):
            current_blob = Color.RED.value
        elif ((green > 3 * red) and (green > 3 * blue)):
            current_blob = Color.GREEN.value
        elif (blue / (red + green + blue) > 0.45):
            if (red / (red + green + blue) < 0.1 and green > red):
                current_blob = Color.CYAN.value
            else:
                current_blob = Color.BLUE.value
        else:
            current_blob = Color.NONE
        # print(current_blob)

        if (current_blob == Color.NONE):
            pass
        #else:
            #print("Looks like I found a", color_names[current_blob],"60-80");
            # width of 100-128px 1/2 height
    for i in range(100, 128):
                for j in range(0, int(height / 2)):
                    red = camera.imageGetRed(image, width, i, j)
                    blue = camera.imageGetBlue(image, width, i, j)
                    green = camera.imageGetGreen(image, width, i, j)
    if (red != 0 and green != 0 and blue != 0):
                #print(red / (red + green + blue),green / (red + green + blue),blue / (red + green + blue))
                if (red / (red + green + blue) > 0.45):
                    current_blob = Color.RED.value
                elif ((green > 3 * red) and (green > 3 * blue)):
                    current_blob = Color.GREEN.value
                elif (blue / (red + green + blue) > 0.45):
                    if (red / (red + green + blue) < 0.1 and green > red):
                        current_blob = Color.CYAN.value
                    else:
                        current_blob = Color.BLUE.value
                else:
                    current_blob = Color.NONE
                # print(current_blob)

                if (current_blob == Color.NONE):
                    pass
                else:
                    #print("Looks like I found a", color_names[current_blob], "100-128");
                    side="R"
                    obj=color_names[current_blob]
    return side,obj 

"""


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
    
    
    
if mode == 'manual':
    map = np.zeros([360,360]) # Replace None by a numpy 2D floating point array

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
    #camera_image = camera.getImage()
    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            rho = LIDAR_SENSOR_MAX_RANGE

        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy =  -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y

        #print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))

        #if rho < 0.5*LIDAR_SENSOR_MAX_RANGE:
# Part 1.3: visualize map gray values.
            #if mode == 'manual':
            # You will eventually REPLACE the following 2 lines with a more robust version of map
            # and gray drawing that has more levels than just 0 and 1.
                
                #obs_x = int(wx*30)
                #obs_y = int(wy*30)
                
                #if(obs_x >= 0 and obs_x < 360 and obs_y >= 0 and obs_y < 360):
                    #map[obs_x][obs_y] += 0.005
                    #g = int(min(map[obs_x][obs_y], 1.0) * 255)
                    #g = int((g*256**2+g*256+g))
                    #display.setColor(g)
                    #display.drawPixel(360-int(wy*30), int(wx*30))

    
    
    #camera_image = camera.getImageArray()
    #display.setColor(camera_image[10][10][0])
    #display.drawPixel(100,100)
    
    
    
    #display.setColor(int(0xFF0000))
    #display.drawPixel(360-int(pose_y*30),int(pose_x*30))



###################
#
# Controller
#
###################
    #print(camera.getRecognitionObjects())
    print(camera.getRecognitionNumberOfObjects())
    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            vL = -MAX_SPEED
            vR = MAX_SPEED
            #print(vL, vR)
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
            #np.save("map.npy",map)
            camera.saveRecognitionSegmentationImage("image.jpeg", 100)
            print("Recognition file saved")
        elif key == ord('L'):
            # You will not use this portion but here's an example for loading saved a numpy array
            map = np.load("map.npy")
            plt.imshow(map, cmap='gray')
            plt.show()
            print("Map loaded")
        elif key == ord('W'):
            robot_parts[3].setVelocity(1.0)
        elif key == ord('E'):
            robot_parts[3].setVelocity(-1.0)
        else: # slow down
            vL *= 0.75
            vR *= 0.75
            
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    # print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta)) #/3.1415*180))

    # Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)
