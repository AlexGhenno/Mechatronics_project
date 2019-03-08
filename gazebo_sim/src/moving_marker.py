#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates

x_vel = 0.5
y_vel = 0.5
model_name = ""
model_state_pub = None
model_state_sub = None

def init():
    global model_name, model_state_pub, x_vel, y_vel, model_state_sub
    rospy.init_node('moving_marker')
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
    global model_name, model_state_pub, x_vel, y_vel, model_state_sub
    index_of_interest = -1
    for i in range(len(msg.name)):
        if msg.name[i] == model_name:
            index_of_interest = i
            break
    if index_of_interest >= 0:
        model_state = ModelState()
        model_state.model_name = model_name

	if cnt > 310:
        	cnt = 0
  	cnt = cnt + 1
   	x = math.cos(cnt*0.02)
   	y = math.sin(cnt*0.02)

	model_state.pose = msg.pose[index_of_interest]
        twist = Twist()
        twist.linear.x = -0.1
        twist.linear.y = 0.1
        if msg.twist[index_of_interest] != twist:
            model_state.twist = twist
            model_state_pub.publish(model_state)

	#model_state_pose=Pose()
        #model_state_pose = (x,y)
        #model_state.position.x=model_state_pose[0]
        #model_state.position.y=model_state_pose[1]
	#twist = Twist()	
	#twist.linear.x = x_vel
        #twist.linear.y = y_vel
	#model_state_pub.publish(model_state)


if __name__ == '__main__':
    init()
