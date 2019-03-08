#!/usr/bin/env python

import rospy
import math
import tf2_ros
import tf_conversions
import geometry_msgs
from nav_msgs.msg import Odometry

t=geometry_msgs.msg.TransformStamped()
broadc=tf2_ros.TransformBroadcaster()

def t_based_on_pose(state_uav):	
	t.header.stamp=rospy.Time.now()
	t.header.frame_id="odom"
	t.child_frame_id="world"
	t.transform.translation.x=state_uav.pose.pose.position.x
	t.transform.translation.y=state_uav.pose.pose.position.y
	t.transform.translation.z=state_uav.pose.pose.position.z
	q=tf_conversions.transformations.quaternion_from_euler(0,0,0)
	t.transform.rotation.x=q[0]
	t.transform.rotation.y=q[1]
	t.transform.rotation.z=q[2]
	t.transform.rotation.w=q[3]
	

Sub=rospy.Subscriber('ground_truth_to_tf/pose', Odometry, t_based_on_pose)

if __name__ == '__main__':
	rospy.init_node("uav_odom_simulator")
	rate=rospy.Rate(100)
	
while not rospy.is_shutdown():
	odom_to_base_footprint=t
	#broadcasting transform
	broadc.sendTransform(odom_to_base_footprint)
	rate.sleep()
