#!/usr/bin/env python
import tf
import rospy
from tf import TransformListener
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from tf_velocity_estimator.msg import PosesAndVelocities
from tf_velocity_estimator.msg import Velocity

tf_ = None
p_v_pub = None
targeted_tf = ''
sliding_window = []
sliding_window_v = []
sliding_window_sz = 0
latest_common_time = None

def init():
    global targeted_tf, tf_, sliding_window_sz
    global p_v_pub, latest_common_time
    rospy.init_node('tf_velocity_estimator')
    targeted_tf = rospy.get_param('~targeted_tf', 'helipad')	#read the targeted tf, be default is the helipad frame generated in the ar_helipad.py node when detecting the marker
    sliding_window_sz = rospy.get_param('~sliding_window_sz', 10)   #defines the size of the transformations number used to estimate the helipad's velocity
    tf_ = TransformListener()
    latest_common_time = rospy.Time.now()
    rospy.Subscriber('tf', TFMessage, tf_callback)
    p_v_pub = rospy.Publisher('tf_velocity_estimator/poses_velocities', PosesAndVelocities, queue_size=1)

    while not rospy.is_shutdown():
        rospy.spin()

def tf_callback(tf2):
    global targeted_tf, tf_
    global sliding_window_sz, sliding_window, sliding_window_v
    global p_v_pub, latest_common_time
    try:
        t = tf_.getLatestCommonTime('/odom', targeted_tf)	#Determines that most recent time for which Transformer can compute the transform between the two given frames. Returns a rospy.Time.
        if latest_common_time < t:
            latest_common_time = t
            position, quaternion = tf_.lookupTransform('/odom', targeted_tf, t)
            ps = PoseStamped()
            ps.header.stamp = latest_common_time
            ps.header.frame_id = '/odom'
            ps.pose.position.x = position[0]
            ps.pose.position.y = position[1]
            ps.pose.position.z = position[2]
            ps.pose.orientation.x = quaternion[0]
            ps.pose.orientation.y = quaternion[1]
            ps.pose.orientation.z = quaternion[2]
            ps.pose.orientation.w = quaternion[3]
            sliding_window.append(ps)               #append the transformation from /odom to the marker frame in the sliding windows empty array
            if len(sliding_window) >= sliding_window_sz:
                del sliding_window[0]               # when the array already contains 10 elements (10 transformations) the first element is deleted. This creates the effect of a shift register with 10 elements
                if len(sliding_window_v) >= sliding_window_sz:
                    del sliding_window_v[0]         

            v = Velocity()
            if len(sliding_window) > 1:			#calculate the velocities when a second transformation is received
                dx = sliding_window[-1].pose.position.x - sliding_window[-2].pose.position.x
                dy = sliding_window[-1].pose.position.y - sliding_window[-2].pose.position.y
                dz = sliding_window[-1].pose.position.z - sliding_window[-2].pose.position.z
                dt = sliding_window[-1].header.stamp.to_sec() - sliding_window[-2].header.stamp.to_sec()
                if dt > 0:
                    v.vx = dx / dt
                    v.vy = dy / dt
                    v.vz = dz / dt
                    sliding_window_v.append(v)  #it appends the calculated velocites (x,y,z), which is limited to 10 elements
            else:
                sliding_window_v.append(v)      #at the first run, an empty element v is appended to the empty sliding_window_v array
    except Exception as e:
        pass

    if len(sliding_window_v) > sliding_window_sz / 2 and len(sliding_window) > sliding_window_sz / 2:
        pvmsg = PosesAndVelocities()
        pvmsg.latest_poses = sliding_window
        pvmsg.latest_velocities = sliding_window_v
        p_v_pub.publish(pvmsg)          #both arrays containing the latest 10 poses and velocities of the helipad are published

if __name__ == '__main__':
    init() 
