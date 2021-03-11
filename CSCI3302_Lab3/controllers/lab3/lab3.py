"""lab3 controller."""
from controller import Robot, Motor, Camera, RangeFinder, Lidar
import math

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.5725 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10 # Left wheel index
MOTOR_RIGHT = 11 # Right wheel index
N_PARTS = 12 # Total joints

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
# Note that positions for the joints are in radians.
target_pos = (0.0, 0.0, 0.09, 0.07, 0.26, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts = []

for i in range(N_PARTS):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))
        robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

##### Note that the following sensors will not be used for Lab 3.######

# We want to just show that these rich sensors are available with Tiago 
# some of which will be used later.Feel free to play with them. 
# The Tiago robot has a couple more sensors than the e-Puck.
# range = robot.getDevice('range-finder')
# range.enable(timestep)
# camera = robot.getDevice('camera')
#camera.enable(timestep)
#camera.recognitionEnable(timestep)
# lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
#lidar.enable(timestep)
#######################################################################

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0
goal = [(0.3,0.4),(1,1),(1.2,1.7)]
i = 0
vL = 0
vR = 0
        
while robot.step(timestep) != -1:

    ## webots offer builtin object detectors which will not be used in Lab 3 ###
    
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Webots simulates the equivalent of a Mask R-CNN deep learning-based object
    # recognition framework. We print out the names of the objects Tiago sees. You
    # can plan with this data structure eventually. 
    #objects = camera.getRecognitionObjects()
    #for i in objects:
    #    print(i.get_model()) 
    ##########################################################################


    #STEP 1: Calculate the error
    next_goal = goal[i]

    #pos_err =math.sqrt(((next_goal[0]-pose_x)**2)+((next_goal[1]-pose_y)**2))

    #STEP 2: Controller (with gains)
    
    #alpha = math.atan2(((next_goal[1],next_goal[0])) - pose_theta
    pos_error = math.sqrt(((next_goal[0] - pose_x)**2) + ((next_goal[1] - pose_y)**2))
    angle_error = math.atan2(next_goal[1],next_goal[0]) - pose_theta

    #p1 = x/p
    #x_r = pos_err
    #theta_r = alpha

    x_r = pos_error
    theta_r = angle_error
    

    #STEP 3: Compute wheelspeeds
    
    vL = (2*x_r - theta_r*AXLE_LENGTH)/ AXLE_LENGTH
    vR = (2*x_r + theta_r*AXLE_LENGTH)/ AXLE_LENGTH


    #STEP 4: Normalize wheelspeed
    if (vL > MAX_SPEED):
        vL = MAX_SPEED
    if (vR > MAX_SPEED):
        vR = MAX_SPEED
    if (vL < -MAX_SPEED):
        vL = -MAX_SPEED
    if (vR < -MAX_SPEED):
        vR = -MAX_SPEED
 
        
    print("position: ",pos_error)
    print("angle: ",angle_error)
    print("Left: ",vL)
    print("Right: ",vR)


    ################# Do not modify this block of the code ########################
    # Odometry code. Don't change speeds after this
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vL-vR)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0
    print("X: %f Y: %f Theta: %f" % (pose_x, pose_y, pose_theta)) # Comment out this if you want
    #############################################################################
    #print((goal_x, goal_y, pose_theta, p))
    # Enter here functions to send actuator commands
    if ((abs(next_goal[0]) - pose_x) < 0.2):
        if ((abs(next_goal[1]) - pose_y) < 0.2):
            i = i + 1
            print("goal update")
    # Enter here functions to send actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)
    

   
    