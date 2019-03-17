#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from hector_uav_msgs.msg import LandingActionGoal

def init():
    rospy.init_node('land_cmd')
    land_pub = rospy.Publisher('action/landing/goal', LandingActionGoal, queue_size=1)
    
    empty = LandingActionGoal()
    empty.header.seq=0
    empty.header.stamp.secs=0
    empty.header.frame_id=''
    empty.goal_id.stamp.secs=0
    empty.goal_id.stamp.nsecs=0
    empty.goal_id.id=''
    r=rospy.Rate(10)

    while not rospy.is_shutdown():
        land_pub.publish(empty)
        r.sleep()

if __name__ == '__main__':
    init()