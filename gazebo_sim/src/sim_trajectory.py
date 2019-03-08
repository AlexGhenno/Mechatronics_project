#!/usr/bin/env python

import rospy
import math
import tf2_ros
import tf_conversions
import geometry_msgs

br = tf2_ros.TransformBroadcaster()
cnt = 0
position_turtlebot = [0.,0.]
def calculate_position():
    global cnt
    if cnt > 310:
        cnt = 0
    cnt = cnt + 1
    x = 4*math.cos(cnt*0.02)
    y = 3*math.sin(cnt*0.02)
    position =(x,y)
    return position

def calculate_tf():
    t = geometry_msgs.msg.TransformStamped()
    #getting actual position
    position_turtlebot = calculate_position()
    #calculating transform
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "ref_frame"
    t.transform.translation.x = position_turtlebot[0]
    t.transform.translation.y = position_turtlebot[1]
    t.transform.translation.z = 0
    q = tf_conversions.transformations.quaternion_from_euler(0,0,0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t

if __name__ == '__main__' :
    rospy.init_node("dynamic_tf_broadcaster")
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        odom_to_base_footprint = calculate_tf()
        #broadcasting transform
        br.sendTransform(odom_to_base_footprint)
        rate.sleep()
