#!/usr/bin/env python
import tf
import math
import rospy
from nav_msgs.msg import Path
from tf import TransformListener
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from tf_velocity_estimator.msg import Velocity              #these two type of msgs are custom (created by the author)
from tf_velocity_estimator.msg import PosesAndVelocities

tf_ = None
path_pub = None
max_velocity = 1 # m/s
robot_pose = None
tf_broadcaster = None
future_horizon = 3          #time in seconds used as the max future horizon to predict the helipad's pose 

def init():
    global path_pub, tf_, max_velocity, tf_broadcaster
    rospy.init_node('aerial_global_planner')
    max_velocity = rospy.get_param('~max_velocity', 5)              #
    rospy.Subscriber('tf_velocity_estimator/poses_velocities', PosesAndVelocities, p_v_callback) #subscription to the topic that was generated in marker_vel_estimator.py
    tf_ = TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber('tf', TFMessage, tf_callback)
    path_pub = rospy.Publisher('aerial_global_planner/plan', Path, queue_size=1)  #the UAV goal is published with this topic
    while not rospy.is_shutdown():
        rospy.spin()

#callback function to get the position of the UAV
def tf_callback(tf2):
    global robot_pose, tf_
    try:
        t = tf_.getLatestCommonTime('/odom', '/base_link')
        position, quaternion = tf_.lookupTransform('/odom', '/base_link', t)  #transformation between the fixed frame /odom and the /base_link of the UAV
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = '/odom'
        ps.pose.position.x = position[0]
        ps.pose.position.y = position[1]
        ps.pose.position.z = position[2]
        ps.pose.orientation.x = quaternion[0]
        ps.pose.orientation.y = quaternion[1]
        ps.pose.orientation.z = quaternion[2]
        ps.pose.orientation.w = quaternion[3]
        robot_pose = ps
        # TODO subscribe to odom instead of tf! wtf!!
    except Exception as e:
        pass

#callback function to get the pose info (latest position and average velocity) of the helipad
def p_v_callback(pvmsg):
    global path_pub, robot_pose, max_velocity, tf_broadcaster
    if robot_pose != None:
        path_msg = Path()
        path_msg.header.frame_id = '/odom'
        path_msg.header.stamp = rospy.Time.now()

        latest_pose = pvmsg.latest_poses[-1]    #from the received msg, just the last pose of the helipad is taken
        lastx = latest_pose.pose.position.x
        lasty = latest_pose.pose.position.y
        lastz = latest_pose.pose.position.z
        vx = []
        vy = []
        for v in pvmsg.latest_velocities:       #X and Y components of the velocity are taken from msg and stored in the empty arrays vx and vy
            vx.append(v.vx)
            vy.append(v.vy)

        meanvx = sum(vx) / float(len(vx))       #The mean of 10 latest velocities is computed, for each component 
        meanvy = sum(vy) / float(len(vy))
        lastvx = 0
        lastvy = 0
        noise = False
        stabilized = False
        while not stabilized:
            stabilized = True                       #if none of the if conditions below are met, this remains true and the while loop ends
            if len(vx) >= 3:                        #by default the condition is true since a msg containing 10 velocities was received, after some iterations of the while loop the number of elements in vx and vy is reduced
                if abs(vx[0] - meanvx) >= 0.02:     #condition to check if the helipad is moving with constant velocity (elimination of noisy values) 
                    stabilized = False
                    noise = True
                    del vx[0]
                    meanvx = sum(vx) / float(len(vx))   #a new mean velocity is calculated without taking into account the disperse velocity
                if abs(vx[-1] - meanvx) >= 0.02:
                    stabilized = False
                    noise = True
                    del vx[-1]
                    meanvx = sum(vx) / float(len(vx))
            if len(vy) >= 3:
                if abs(vy[0] - meanvy) >= 0.02:
                    stabilized = False
                    noise = True
                    del vy[0]
                    meanvy = sum(vy) / float(len(vy))
                if abs(vy[-1] - meanvy) >= 0.02:
                    stabilized = False
                    noise = True
                    del vy[-1]
                    meanvy = sum(vy) / float(len(vy))
        #if noise:
        #    lastvx = sum(vx)/float(len(vx))
        #    lastvy = sum(vy)/float(len(vy))
        #else:
        lastvx = meanvx
        lastvy = meanvy

        t_ = (rospy.Time.now().to_sec() - latest_pose.header.stamp.to_sec())    #difference between the current time and the latest helipad's pose time

        goalx , goaly, goalz, yaw = rendezvous(t_, lastx, lasty, lastz, lastvx, lastvy, robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z, max_velocity)

        if goalx != None:
            latest_pose.pose.position.x = goalx
            latest_pose.pose.position.y = goaly
            latest_pose.pose.position.z = goalz
            path_msg.poses.append(latest_pose)
            path_msg.poses.append(robot_pose)
            path_pub.publish(path_msg)              #publication of the goal position!

            goal_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            tf_broadcaster.sendTransform(
                (goalx, goaly, goalz),
                goal_quat,
                rospy.Time.now(),
                'robot_goal',
                'odom')
        else: # The robot cannot reach the goal, apath msg without poses is published
            path_pub.publish(path_msg)


# UNTESTED FUNCTION
def rendezvous(t_, helix, heliy, heliz, helivx, helivy, robotx, roboty, robotz, maxrobotv):
    global tf_broadcaster
    goalx = None
    goaly = None
    goalz = None
    needed_velocities = []

    for t in float_range(t_,future_horizon, 0.1):      #loop to compute the helipad position in a range of future horizons, and the required velocities to reach it
        # Helipad position after time = t
        hx = helix + (helivx * t)
        hy = heliy + (helivy * t)
        if t == t_:                         #condition met in the first iteration of the for loop 
            tf_broadcaster.sendTransform(   #broadcast the goal position considering the position prediction of the helipad after the elapsed time it was last detected 
                (hx, hy, heliz),
                (0, 0, 0, 1),
                rospy.Time.now(),
                'goal_prediction',          #frame created to point the goal position 
                'odom')                     #referenced child frame
        # Robot needed velocity to reach helipad's position
        neededvx = (robotx - hx) / t
        neededvy = (roboty - hy) / t
        neededvz = (robotz - heliz) / t
        if abs(neededvx) < maxrobotv and abs(neededvy) < maxrobotv and abs(neededvz) < maxrobotv:
            needed_velocities.append([[neededvx, neededvy, neededvz], [hx, hy, heliz]])
    minv = max_velocity + 1
    minvxvy = max_velocity + 1
    for i in needed_velocities:
        sumv = abs(i[0][0]) + abs(i[0][1]) + abs(i[0][2])       #Sum of the needed velocities in one future horizon
        sumvxvy = abs(i[0][0]) + abs(i[0][1])
        if sumv < minv:                     #if the sum of velocities (magnitude of the velocity) is less than the max vel. set as goal position the predicted helipad position in one of the future horizons
            goalx = i[1][0]                 #with this iteration process the goal position with the lowest required velocity is determined 
            goaly = i[1][1]
            goalz = i[1][2]
            minv = sumv
            minvxvy = sumvxvy
        elif sumv == minv and sumvxvy < minvxvy: #once the goal position with the minimum required velocity is determined, the goal with the minimum required vel in xy direction is determined 
            goalx = i[1][0]
            goaly = i[1][1]
            goalz = i[1][2]
            minv = sumv
            minvxvy = sumvxvy

    yaw = 0.0
    if goalx!= None:
        yaw = lookAt(goalx, goaly, helix, heliy)
    return goalx, goaly, goalz, yaw

def lookAt(goalx, goaly, helix, heliy):
    dx = helix - goalx
    dy = heliy - goaly
    return math.atan2(dy,dx)

def float_range(x, y, jump):        #function to increment x by steps equal to jump, until it reaches the value y
    while x < y:
        yield x
        x += jump

if __name__ == '__main__':
    init()
