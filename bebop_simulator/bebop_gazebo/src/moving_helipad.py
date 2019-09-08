#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates

x_vel = 1.0
y_vel = 0.5
cnt = 0
r = 1 

t = 0
px = 3
py = 0

#trajectory dimensions
path_length = 3.5
path_width = 3.5
t_step = 0.05
path_resolution = 0.005             # 0.005 for slow motion 


model_name = ""
model_state_pub = None
model_state_sub = None
#rate = rospy.Rate(1) # 10h

def init():
    global model_name, model_state_pub, x_vel, y_vel, model_state_sub
    rospy.init_node('moving_helipad')
    model_name = rospy.get_param("~model_name", "marker3")
    x_vel = rospy.get_param("~x_vel", 1.0)
    y_vel = rospy.get_param("~y_vel", 0.5)
    seconds_before_moving = rospy.get_param("~seconds_before_moving", 10)
    rospy.sleep(seconds_before_moving)
    model_state_sub = rospy.Subscriber("gazebo/model_states", ModelStates, modelStatesCallback)
    model_state_pub = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=1)
    while not rospy.is_shutdown():
        rospy.spin()

def modelStatesCallback(msg):
    global model_name, model_state_pub, x_vel, y_vel, model_state_sub, r, t, rate
    index_of_interest = -1			#index of the last element in the model_states msg provided by gazebo, which corresponds to the pose information of the helipad.
    for i in range(len(msg.name)):
        if msg.name[i] == model_name:		#way to ensure that the index actually corresponds to the helipad's index
            index_of_interest = i
            break
    
    #rate.sleep()
    now = rospy.get_rostime()
    vel_prm = rospy.get_param("helipad_vel", 50000)
    if (now.nsecs % vel_prm) == 0:          #this line sets the speed of the helipad
        if t>1260:
            t = 0
        else:
            # t = t + t_step
            t = t + rospy.get_param("time_step", 0.05)
    
    # Infinity sign path   
    #px = path_length*(math.cos(path_resolution*t))
    #py = path_width*(math.sin(2*path_resolution*t))   
    # Circular path
    px = path_length*(math.cos(path_resolution*t))
    py = path_width*(math.sin(path_resolution*t))


    if index_of_interest >= 0:
        model_state = ModelState()
        model_state.model_name = model_name
        #model_state.pose = msg.pose[index_of_interest]
        #model_state.twist = msg.twist[index_of_interest]
        #twist = Twist()
        #twist.linear.x = 1.0
        #twist.angular.z = 1.0
        
        pose = Pose()
        pose.position.x = px
        pose.position.y =py
        
        #if msg.twist[index_of_interest] != twist:
        if msg.pose[index_of_interest] != pose:
            #model_state.twist = twist
            model_state.pose = pose
            model_state_pub.publish(model_state)
            #print(px)
            #print(py)
            #print(t)
            #print(now)

if __name__ == '__main__':
    init()
