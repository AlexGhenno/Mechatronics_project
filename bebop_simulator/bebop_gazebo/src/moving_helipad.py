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
t =1
model_name = ""
model_state_pub = None
model_state_sub = None

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
    global model_name, model_state_pub, x_vel, y_vel, model_state_sub, r, t
    index_of_interest = -1			#index of the last element in the model_states msg provided by gazebo, which corresponds to the pose information of the helipad.
    t=0;
    for i in range(len(msg.name)):
        if msg.name[i] == model_name:		#way to ensure that the index actually corresponds to the helipad's index
            index_of_interest = i
            break
    
    if t >=310:
    	t =0
    else:
	t=i+1
        vx=math.cos(0.2*t)
        vy=math.sin(0.2*t)

    if index_of_interest >= 0:
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = msg.pose[index_of_interest]
        twist = Twist()
        #pose.position.x = vx
        #pose.position.y =vy
	twist.linear.x=x_vel
	twist.linear.y=y_vel
        if msg.twist[index_of_interest] != twist:
            model_state.twist = twist
            model_state_pub.publish(model_state)

if __name__ == '__main__':
    init()
