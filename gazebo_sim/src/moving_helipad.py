#!/usr/bin/env python
import rospy
import math
import time
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates


#Set parameter values
x_vel = 1.0 #Velocity in the horizontal direction
y_vel = 0.5 #Velocity in the vertical direction
path_number = 1.0 #Set which path 
px = 5.0 #Position in horizontal direction
py = 5.0 #position in vertical direction
origin_x = 0.0 #Origin of the circle
origin_y = 0.0 #Origin of the circle
radius = 3.0 #Radius of the circle
start_angle = 0 
stop_angle = 360.0
resolution = 0.005 #Smallest possible measurement
count = 0 
rate_param = 5 #Desired loop rate
# Trajectory dimensions for infinity path
path_length = 4
path_width = 1

#GetTime and instantiate
time_start = time.time() 

#ModelState
model_name = ""
model_state_pub = None
model_state_sub = None

# Position calclulation for Sine wave
def calculate_position():
    global count
    if count  >   300 :
        count  =   0
    count = count+ 0.05
    x = path_length * math.cos(count*0.02)
    y = path_width * math.sin (2*count*0.02)
    position  = (x,y)
    return position

#Init node
def init():
    global model_name,rate,rate_param,path_number,count, model_state_pub, x_vel, y_vel, model_state_sub   #define global variables
    rospy.init_node('moving_helipad') #Init node name
    rate_param = rospy.get_param("~Rate")
    rate = rospy.Rate(rate_param) #To set speed of motion 
    model_name = rospy.get_param("~model_name", "marker3") #Retrieve marker parameters
    x_vel = rospy.get_param("~x_vel", 1.0) #Retrieve X vel
    y_vel = rospy.get_param("~y_vel", 0.5) # Retrieve Y vel
    seconds_before_moving = rospy.get_param("~seconds_before_moving", 10) #Retrieve seconds before moving param
    path_number = rospy.get_param("~path_number")
    rospy.sleep(seconds_before_moving) #Set node to sleep before "seconds_before_moving"
    model_state_sub = rospy.Subscriber("gazebo/model_states", ModelStates, modelStatesCallback) #Standard subscriber
    model_state_pub = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=1) #Standard publisher
    while not rospy.is_shutdown():
        rospy.spin() # keep your node from exiting until the node has been shutdown.

def modelStatesCallback(msg):
    global model_name,time_start,time_reverse,path_number,count,rate, model_state_pub, x_vel, y_vel, model_state_sub #define global variables
    
    now = rospy.get_rostime() #Time
    index_of_interest = -1 #Set index for message list
    for i in range(len(msg.name)):  #Retrieve index from message list
        if msg.name[i] == model_name:
            index_of_interest = i
            break
    if index_of_interest >= 0: #Check if index exists
        model_state = ModelState() #Instantiate Model State
        model_state.model_name = model_name #Set model name

        if path_number == 1.0: #Linear unidirectonal X___Velocity defined as a parameter
            model_state.pose = msg.pose[index_of_interest] #Set model state pose from list of pose(representation of pose in free space, composed of position and orientation.) 
            twist = Twist() #expresses velocity in free space broken into its linear and angular parts.
            deltaT = (time.time() - time_start)
            if (deltaT) <= 15:
                twist.linear.x = x_vel #Set linear X velocity
            elif deltaT <= 30:    
                twist.linear.x = -x_vel #Set linear X velocity
            else:
                time_start = time.time() #Reset time counter

            if msg.twist[index_of_interest] != twist: #If velocity is not equal to current velocity
                model_state.twist = twist #Set twist vel
                model_state_pub.publish(model_state)  #Publish model state

        elif path_number == 2.0: #CIRCLE
            model_state.pose = msg.pose[index_of_interest]
            pose = Pose()
            for deg in np.arange(start_angle,stop_angle,resolution):
                px = radius*math.cos(deg) + origin_x # Point x(t) on the circumference of a circle 
                py = radius*math.sin(deg) + origin_y # Point y(t) on the circumference of a circle
                pose.position.x = px                 # Set pose.x   
                pose.position.y = py                 # Set pose.y
                if msg.pose[index_of_interest] != pose:
                    model_state.pose = pose
                    model_state_pub.publish(model_state)
                    rate.sleep()
        
        elif path_number == 3.0: #iNFINITY
            model_state.pose = msg.pose[index_of_interest]
            pose = Pose()
            twist = Twist()
            px = calculate_position()[0] #Call function for x(t)
            py = calculate_position()[1] #Call function for y(t)
            pose.position.x = px  # Set pose.x
            pose.position.y = py   # Set pose.y
            if msg.pose[index_of_interest] != pose:
                model_state.pose = pose  
                model_state_pub.publish(model_state) 
                rate.sleep()
        
        elif path_number == 4.0: #ZIGZAG
            model_state.pose = msg.pose[index_of_interest] #Set model state pose from list of pose(representation of pose in free space, composed of position and orientation.) 
            pose = Pose() #Instantiating
            twist = Twist() #expresses velocity in free space broken into its linear and angular parts.
            deltaT = (time.time() - time_start)
            if (deltaT) <= 5:
                twist.linear.x = x_vel #Set linear X velocity
                twist.linear.y = y_vel #Set linear Y velocity
            elif deltaT <= 10:    
                twist.linear.x = x_vel #Set linear X velocity
                twist.linear.y = -y_vel #Set linear Y velocity
            else:
                time_start = time.time()

            if msg.twist[index_of_interest] != twist: #If velocity is not equal to current velocity
                model_state.twist = twist #Set twist vel
                if model_state.pose.position.x > 10: #Reset position
                    model_state.pose.position.x = 0
                    model_state.pose.position.y = 0   
                model_state_pub.publish(model_state)  #Publish model state       

if __name__ == '__main__':
    try:
        init() #Call node
    except rospy.ROSInterruptException:
      pass