"""final_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import time

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 14

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint",
              "arm_1_joint",  "arm_2_joint",  "arm_3_joint",
              "arm_4_joint",  "arm_5_joint",  "arm_6_joint",
              "arm_7_joint",  "wheel_left_joint", "wheel_right_joint", 
              "gripper_left_finger_joint", "gripper_right_finger_joint")
      
        


robot_parts=[]
target_pos = (0.0, 0.0, 0.0, 
              0.07, 1.02, -3.16, 
              1.27, 1.32, 0.0, 
              1.41, 'inf', 'inf',
              0.0, 0.0)
     
#horizontal         
hor_side = (0.0, 0.0, 0.0,
              1.62, 0.0, 1.5,
              0.0, 0.0, 0.0,
              0.0, 'inf', 'inf',
              0.0, 0.0)
              
hor_frwd = (0.0, 0.0, 00,
              0.07, 0.0, 1.5,
              0.0, 0.0, 0.0,
              0.0, 'inf', 'inf',
              0.0, 0.0)

hor_bkwd = (0.0, 0.0, 0.0,
              2.68, 0.0, 1.5,
              0.0, 0.0, 0.0,
              0.0, 'inf', 'inf',
              0.0, 0.0)

#downwards          
down_side = (0.0, 0.0, 0.0,
              1.62, -0.5, 1.5,
              0.0, 0.0, -0.5,
              0.0, 'inf', 'inf',
              0.0, 0.0)
              
down_frwd = (0.0, 0.0, 0.0,
              0.07, -1.0, 1.5,
              0.0, 0.0, -1.0,
              0.0, 'inf', 'inf',
              0.0, 0.0)

down_bkwd = (0.0, 0.0, 0.0,
              2.68, -0.5, 1.5,
              0.0, 0.0, -0.5,
              0.0, 'inf', 'inf',
              0.0, 0.0)
              
#upwards
up_side = (0.0, 0.0, 0.0,
              1.62, 0.5, 1.5,
              0.0, 0.0, 0.5,
              0.0, 'inf', 'inf',
              0.0, 0.0)
              
up_frwd = (0.0, 0.0, 0.1,
              0.07, 0.5, 1.5,
              0.0, 0.0, 0.5,
              0.0, 'inf', 'inf',
              0.0, 0.0)

up_bkwd = (0.0, 0.0, 0.1,
              2.68, 0.5, 1.5,
              0.0, 0.0, 0.5,
              0.0, 'inf', 'inf',
              0.0, 0.0)
              
orientations = [hor_side, hor_frwd, hor_bkwd,
                down_side, down_frwd, down_bkwd,
                up_side, up_frwd, up_bkwd]
                

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)
    

def set_orientation(pos):
    for i in range(N_PARTS):
        robot_parts[i].setPosition(float(orientations[pos][i]))
        
def open_gripper():
    robot_parts[12].setPosition(float(0.045))
    robot_parts[13].setPosition(float(0.045))
    
def close_gripper():
    robot_parts[12].setPosition(float(0.00))
    robot_parts[13].setPosition(float(0.00))
    
#def look_down():
    
    


camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

display = robot.getDevice("display")
display_w = display.getWidth()
display_h = display.getHeight()
#print(display_w, display_h)

vL = 0
vR = 0

#mode = 'manual'
#mode = 'manual2'
mode = 'state_mode'

states = ('find_and_approach', 'reach_for_object',
          'grabbing_position', 'grab_object', 'pick_up', 'return')
          
current_state = states[0]

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller

find_object = False
move_forward = False

head_1_joint_cur = 0.0
head_1_joint_MAX = 1.24
head_1_joint_MAX = -1.24

head_2_joint_cur = 0.0
head_2_joint_MAX = 0.79
head_2_joint_MAX = -0.98

obj_found = False

while robot.step(timestep) != -1:

    if mode == 'state_mode':
        
        # collect object recognition info
        objs = camera.getRecognitionObjects()
        
        if current_state == 'find_and_approach':
        
            # if objects are detected, check for the desired item
            if len(objs) > 0:
                for i in range(len(objs)):
                    if objs[i].get_id() == 842:
                    
                        print("Rubber ducky located")
                        
                        # Get x display position of object
                        obj = objs[i]
                        obj_pose_x = obj.get_position_on_image()[0]
                        obj_pose_y = obj.get_position_on_image()[1]
                        
                        # Center object on screen
                        if obj_pose_x < 150:
                            
                            vL = -MAX_SPEED*0.08
                            vR = MAX_SPEED*0.08
                        elif obj_pose_x >= 154:
                            ###
                            
                            vL = MAX_SPEED*0.08
                            vR = -MAX_SPEED*0.08
                            
                        # If item is centered enough, move forward
                        else:
                            
                            obj_rel_pose = obj.get_position()
                            obj_rel_pose_z = obj_rel_pose[2]
                            if abs(obj_rel_pose_z) > 0.9:
                                vL = 0.3*MAX_SPEED
                                vR = 0.3*MAX_SPEED
                            else:
                                vL = 0.0
                                vR = 0.0
                                current_state = states[1]
                            
                            #adjust head pitch to center item vertically
                            if obj_pose_y < 158:
                                head_2_joint_cur += 0.01
                                robot_parts[0].setPosition(head_2_joint_cur)
                            elif obj_pose_y >= 162:
                                head_2_joint_cur -= 0.01
                                robot_parts[0].setPosition(head_2_joint_cur)
                        
            # Otherwise, rotate to locate objects in room
            else:
                vL = 0.2*MAX_SPEED
                vR = -0.2*MAX_SPEED
                
        elif current_state == 'reach_for_object':
            print("Reaching for object")
            
            # joint positions to place arm above object
            reach_pos = (0.0, 0.0, 0.0,
              1.62, 1.0, 0.0,
              1.0, -1.6, -0.04,
              0.0, 'inf', 'inf',
              0.045, 0.045)
            for i in range(N_PARTS):
                robot_parts[i].setPosition(float(reach_pos[i]))
                
                time.sleep(1.0)
                current_state = states[2]
        elif current_state == 'grabbing_position':
            print("Grabbing object")
            
            # joint positions to place grippers around object
            grab_pos = (0.0, 0.0, 0.0,
              1.62, 0.84, 0.0,
              1.04, -1.6, -0.2,
              0.0, 'inf', 'inf',
              0.045, 0.045)
            for i in range(N_PARTS):
                robot_parts[i].setPosition(float(grab_pos[i]))
                
                time.sleep(1.0)
                current_state = states[3]
            
        elif current_state == 'grab_object':
            print("Gripping object")
            
            robot_parts[12].setPosition(0.0)
            robot_parts[13].setPosition(0.0)
            
            time.sleep(1.0)
            current_state = states[4]
            
        elif current_state == 'pick_up':
            print("Raising object")
            
            for i in range(N_PARTS):
                robot_parts[i].setPosition(float(target_pos[i]))
                
                time.sleep(5)
                current_state = states[5]
        elif current_state == 'return':
            print("Finished")
            continue
        
        
    
    
    
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
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
        elif 48 < key < 58:
            pos = key - 49
            set_orientation(pos)
        elif key == ord('O'):
            open_gripper()
        elif key == ord('P'):
            close_gripper()
        else: # slow down
            vL *= 0.5
            vR *= 0.5
            
        
        objs = camera.getRecognitionObjects()
        
        #if len(objs) > 0 and obj_found == False:
        if len(objs) > 0:
            for i in range(len(objs)):
                #print(objs[i].get_id(), objs[i].get_model())
                if objs[i].get_id() == 31774:
                    #print('found')
                    obj_index = i
                    obj_found = True
                    find_object = True
        
        if key == ord('C') and obj_found == True:
            print(objs[obj_index].get_id(), objs[obj_index].get_model())
        
                    
                    
        
        if key == ord('F'):
            if len(objs) > 0:
                find_object = True
            else:
                print("No objects detected.")
                
            
            
        if key == ord('H'):
            find_object = False
        
        
        if find_object:
            obj = objs[obj_index]
            obj_pose_x = obj.get_position_on_image()[0]
            obj_pose_y = obj.get_position_on_image()[1]
            if obj_pose_x < 158:
                ###
                
                vL = -MAX_SPEED*0.1
                vR = MAX_SPEED*0.1
                """
                head_1_joint_cur += 0.01
                robot_parts[1].setPosition(head_1_joint_cur)
                """
            elif obj_pose_x >= 162:
                ###
                
                vL = MAX_SPEED*0.05
                vR = -MAX_SPEED*0.05
                """
                head_1_joint_cur -= 0.01
                robot_parts[1].setPosition(head_1_joint_cur)
                """
            if obj_pose_y < 158:
                ###
                """
                vL = -MAX_SPEED*0.1
                vR = MAX_SPEED*0.1
                """
                head_2_joint_cur += 0.01
                robot_parts[0].setPosition(head_2_joint_cur)
            elif obj_pose_y >= 162:
                ###
                """
                vL = MAX_SPEED*0.05
                vR = -MAX_SPEED*0.05
                """
                head_2_joint_cur -= 0.01
                robot_parts[0].setPosition(head_2_joint_cur)
            """
            else:
                vL = 0
                vR = 0
            """
            #print(obj_pose_x, obj_pose_y)
            
        if key == ord('T'):
            move_forward = True
        
        if move_forward:
            obj_rel_pose = objs[0].get_position()
            if abs(obj_rel_pose[2]) > 1.2:
                vL = MAX_SPEED*0.5
                vR = MAX_SPEED*0.5
            else:
                move_forward = False
    
    #arm_2_cur = 0.0
    arm_2_MAX = 1.00
    arm_2_MIN = -0.80
    #arm_4_cur = 0.0
    arm_4_MAX = 1.30
    arm_4_MIN = -0.30
    #arm_6_cur = 0.0
    arm_6_MAX = 1.30
    arm_6_MIN = -1.30
    
    
    if mode == 'manual2':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == ord('P'):
            set_orientation(0)
            arm_2_cur = 0.0
            arm_4_cur = 0.0
            arm_6_cur = 0.0
            robot_parts[5].setPosition(0.0)
            robot_parts[7].setPosition(-1.60)
            robot_parts[9].setPosition(-1.60)
        elif key == ord('Q') and arm_2_cur < arm_2_MAX:
            arm_2_cur += 0.01
            arm_6_cur += 0.01
            robot_parts[4].setPosition(arm_2_cur)
            robot_parts[8].setPosition(arm_6_cur)
            ###
            ###
        elif key == ord('W') and arm_2_cur > arm_2_MIN:
            arm_2_cur -= 0.01
            arm_6_cur -= 0.01
            robot_parts[4].setPosition(arm_2_cur)
            robot_parts[8].setPosition(arm_6_cur)
            ###
            ###
        elif key == ord('A') and arm_4_cur < arm_4_MAX:
            arm_4_cur += 0.01
            arm_6_cur -= 0.01
            robot_parts[6].setPosition(arm_4_cur)
            robot_parts[8].setPosition(arm_6_cur)
        elif key == ord('S') and arm_4_cur > arm_4_MIN:
            arm_4_cur -= 0.01
            arm_6_cur += 0.01
            robot_parts[6].setPosition(arm_4_cur)
            robot_parts[8].setPosition(arm_6_cur)
        
        elif key == ord('O'):
            open_gripper()
        elif key == ord('P'):
            close_gripper()
        
        if key == ord('Y'):
            print("arm 2:", arm_2_cur)
            print("arm 4:", arm_4_cur)
            print("arm 6:", arm_6_cur)
        
        if key == ord('T'):
            move_forward = True
        
        set_pickup_above = (1.0, 1.0, -0.04)
        set_pickup = (0.84, 1.04, -0.2)
        stop_dist = 0.82
        object_reached = False
        pickup_above = False
        pickuip = False
        grab_object = False
        
        objs = camera.getRecognitionObjects()
        if move_forward:
            
            obj = objs[0]
            
            obj_pose_x = obj.get_position_on_image()[0]
            obj_pose_y = obj.get_position_on_image()[1]
            obj_rel_pose = objs[0].get_position()
            
            if obj_pose_y < 158:
                head_2_joint_cur += 0.01
                robot_parts[0].setPosition(head_2_joint_cur)
            elif obj_pose_y >= 162:
                head_2_joint_cur -= 0.01
                robot_parts[0].setPosition(head_2_joint_cur)
            
            #print(objs[0].get_model())
            if abs(obj_rel_pose[2]) > stop_dist:
                vL = MAX_SPEED*0.5
                vR = MAX_SPEED*0.5
            else:
                move_forward = False
                vL = 0.0
                vR = 0.0
    
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)
# Enter here exit cleanup code.
