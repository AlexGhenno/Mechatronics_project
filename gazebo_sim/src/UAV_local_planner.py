#!/usr/bin/env python
# -*- coding: utf-8 -*-
import tf
import math
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from tf import TransformListener
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from hector_uav_msgs.msg import LandingActionGoal

robot_goal = None
cmd_vel_pub = None  
land_pub = None

z_tolerance = 0.0
xy_tolerance = 0.0
max_trans_vel = 0.5
min_trans_vel = 0.1
xy_landing_tolerance = 0.1

landed = False
yaw_diff2 = 0.0
x_diff = 0.0
y_diff = 0.0
goal_history = [Pose()]
goal_history[0].position.x = 0.0
goal_history[0].position.y = 0.0
vx_history = [0]
vy_history = [0]


tf_ = None
goal_prediction_pose = None
#rospy.set_param('controller/velocity/max_xy',1.0)       #Set the parameter to adjust the maximum xy velocity of the UAV

# get the uclidea distance between two points
def distance(x1, y1, x2, y2):
    return math.sqrt( (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))
# calculate the yaw angle 
def lookAt(robotx, roboty, helix, heliy):
    dx = helix - robotx
    dy = heliy - roboty
    return math.atan2(dy,dx)

def init():
    global cmd_vel_pub, land_pub
    global max_trans_vel, min_trans_vel
    global  tf_, xy_tolerance, z_tolerance, goal_flag
    rospy.init_node('aerial_local_planner')
    max_trans_vel = rospy.get_param('~max_trans_vel', 5.0)
    min_trans_vel = rospy.get_param('~min_trans_vel', 0.1)
    xy_tolerance = rospy.get_param('~xy_tolerance', 0.05)
    z_tolerance = rospy.get_param('~z_tolerance', 0.35)        # minimum distance in z direction to land  ¡if the z ofset goal coord. is modified, change this tolerance too!
    #theta_tolerance = rospy.get_param() trueTODO
    #pos_tolerance = rospy.get_param() TODO
    cmd_vel_pub = rospy.Publisher('UAV/cmd_vel', Twist, queue_size=1)
    land_pub = rospy.Publisher('action/landing/goal', LandingActionGoal, queue_size=1)
    rospy.Subscriber('ground_truth/state', Odometry, odom_callback)
    rospy.Subscriber('aerial_global_planner/plan', Path, path_callback) #global planner predicted goal subscribtion
    tf_ = TransformListener()
    rospy.Subscriber('tf', TFMessage, tf_callback)
    goal_flag = False

    while not rospy.is_shutdown():
        rospy.spin()



def odom_callback(odom):
    global cmd_vel_pub, robot_goal, land_pub, goal_history, vx_history, vy_history
    global max_trans_vel, min_trans_vel
    global landed, goal_prediction_pose, xy_tolerance, z_tolerance, yaw_diff2, d_left, zd_left
    twist = Twist()
    if not landed and robot_goal != None:
        d = 0.0
        curr_pos = odom.pose.pose.position          #gets the current pose of the UAV (from base_link to world frames)
        curr_or = odom.pose.pose.orientation
        if robot_goal != None:
            d = distance(curr_pos.x, curr_pos.y, robot_goal.position.x, robot_goal.position.y)  #computes the xy plane distance between the UAV current position and the goal predicted
            d2 = 1000   #???
            zd2 = 1000
            if goal_prediction_pose != None:
                d2 = distance(curr_pos.x, curr_pos.y, goal_prediction_pose.position.x, goal_prediction_pose.position.y) #computes the xy plane distance between the UAV current position and the goal position frame. This frame was created by the global planner
                zd2 = curr_pos.z - goal_prediction_pose.position.z
            #calculate z distance 
            zd = abs(curr_pos.z - robot_goal.position.z)
            # print d2, ' ' ,zd2, ' ' , goal_flag      #print the xy and z distance measured from the UAV to the helipad.
            print d2, ' ' ,zd2

            if (d2 < xy_landing_tolerance and zd2 < z_tolerance and zd2 > 0.0):    #condition for landing the UAV. 0.2 is the magnitude tolerance distance between the UAV and the platform
                empty = LandingActionGoal()
                empty.header.seq=0
                empty.header.stamp.secs=0
                empty.header.frame_id=''
                empty.goal_id.stamp.secs=0
                empty.goal_id.stamp.nsecs=0
                empty.goal_id.id=''
                land_pub.publish(empty) 
                landed = True
                # goal_flag = False
                return
            if not (d <= xy_tolerance and zd < z_tolerance and zd2):
                (gr, gp, gy) = tf.transformations.euler_from_quaternion([robot_goal.orientation.x, robot_goal.orientation.y, robot_goal.orientation.z, robot_goal.orientation.w])
                (rr, rp, ry) = tf.transformations.euler_from_quaternion([curr_or.x, curr_or.y, curr_or.z, curr_or.w])
                yaw_diff = gy - ry

                yaw_diff /= 1
                # yaw angle to make the UAV face towards the helipad
                straight_yaw = lookAt(curr_pos.x, curr_pos.y, robot_goal.position.x, robot_goal.position.y)
                yaw_diff2 = straight_yaw - ry

                # polar coordinates r,θ
                # θ = yaw_diff2
                # r = d
                #estimated robot velocities 
                x_diff = d * math.cos(yaw_diff2)
                y_diff = d * math.sin(yaw_diff2)
                z_diff = 2*(robot_goal.position.z - (curr_pos.z - 0.18)) # up is positive  // 0.25 ensures that the robot never gets closer than 30cm to the platform in z

                x_diff = max(min_trans_vel, min(x_diff,max_trans_vel))  #retuns the maximum velocity (saturation of the output between the min and max allowed velocities). These velocities are different from the ctlr velocities
                y_diff = max(min_trans_vel, min(y_diff,max_trans_vel))
                
                # save the history of the three latest velocities
                if len(vx_history) >= 3:
                    del vx_history[0]              #remove the first goal stored
                    del vy_history[0]
                    vx_history.append(x_diff)
                    vy_history.append(y_diff)
                else:
                    vx_history.append(x_diff)
                    vy_history.append(y_diff)
                    
                #set linear and angular velocities
                twist.linear.x = x_diff
                twist.linear.y = y_diff
                twist.linear.z = z_diff
                twist.angular.z = yaw_diff2*10

                # goal_flag = True


    elif len(goal_history) >= 2  and not landed:
        curr_pos = odom.pose.pose.position         
        curr_or = odom.pose.pose.orientation
        d_left = distance(curr_pos.x, curr_pos.y, goal_history[-1].position.x, goal_history[-1].position.y)
        zd_left = 2*(goal_history[-1].position.z - (curr_pos.z - 0.18))
        print 'Marker lost, landing in the last goal predicted (', goal_history[-1].position.x, goal_history[-1].position.y, goal_history[-1].position.z, ')' 
        print d_left
        print zd_left
        
        twist.linear.x = vx_history[-1]
        twist.linear.y = vy_history[-1]
        twist.linear.z = zd_left
        twist.angular.z = yaw_diff2*10
        if d_left < 0.4:
            empty = LandingActionGoal()
            empty.header.seq=0
            empty.header.stamp.secs=0
            empty.header.frame_id=''
            empty.goal_id.stamp.secs=0
            empty.goal_id.stamp.nsecs=0
            empty.goal_id.id=''
            land_pub.publish(empty) 
            landed = True
            return

    if yaw_diff2 >= 0:
        twist.angular.z = 0.2
    # else:
    #     twist.angular.z = -0.2
    #     if odom.pose.pose.position.z < 0.3:
    #         twist.linear.z = 0.5

    cmd_vel_pub.publish(twist)                  # cmd velocity publication to move the UAV! 

def path_callback(path):
    global robot_goal, goal_history
    if len(path.poses) > 0:
        robot_goal = path.poses[0].pose     #gets the goal position from the global planner
        #print(size(goal_history))
        if len(goal_history) >= 5:
            del goal_history[0]              #remove the first goal stored
            goal_history.append(robot_goal)
        else:
            goal_history.append(robot_goal)
        
    else:
        robot_goal = None       #an empty pose msg for the goal was received, do not move the UAV

def tf_callback(tf2):
    global goal_prediction_pose, tf_
    try:
        t = tf_.getLatestCommonTime('/odom', '/goal_prediction')
        position, quaternion = tf_.lookupTransform('/odom', '/goal_prediction', t) #gets the transform from odom to the goal predicted
        goal_prediction_pose = Pose()
        goal_prediction_pose.position.x = position[0]       
        goal_prediction_pose.position.y = position[1]
        goal_prediction_pose.position.z = position[2]
    except Exception as e:
        pass

if __name__ == '__main__':
    init()
