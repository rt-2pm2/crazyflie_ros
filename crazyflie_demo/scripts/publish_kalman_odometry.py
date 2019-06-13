#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped, PoseStamped #PoseStamped added to support vrpn_client
from nav_msgs.msg import Odometry

def onNewMessage(pose):
    global est_odom_pub
    global odom_broadcaster

    # Compose the PointStamped message 
    est_odom.header.stamp = pose.header.stamp

    x = pose.pose.position.x
    y = pose.pose.position.y
    z = pose.pose.position.z
    quat = pose.pose.orientation


    ## Position/Orientation
    est_odom.pose.pose.position.x = x
    est_odom.pose.pose.position.y = y 
    est_odom.pose.pose.position.z = z 
    est_odom.pose.pose.orientation = quat

    ## Velocity
    # Compute the velocity from the measurements (Adding a fading filter)
    est_odom.twist.twist.linear.x = 0.0

    est_odom.twist.twist.linear.y = 0.0
    
    est_odom.twist.twist.linear.z = 0.0 

    est_odom_pub.publish(est_odom)

    
    explicit_quat = [quat.x, quat.y, quat.z, quat.w]
    euler = tf.transformations.euler_from_quaternion(explicit_quat)
    quat_tf = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2]) 
    odom_broadcaster.sendTransform(
            (x, y, z), quat_tf,  
            rospy.Time.now(), "world", "est_odom") 


if __name__ == '__main__':
    rospy.init_node('publish_est_odometry', anonymous=True)
    topic = rospy.get_param("~topic", "/cf1/pose")

    # Tf broadcaster
    odom_broadcaster = tf.TransformBroadcaster()

    # Messages
    est_odom = Odometry()
    est_odom.header.seq = 0
    est_odom.child_frame_id = "est_cf1"
    est_odom.header.frame_id = "world"
    est_odom.header.stamp = rospy.Time.now()

    # Publishers
    est_odom_pub = rospy.Publisher("est_odom", Odometry, queue_size = 3)
    
    # Subscribe
    rospy.Subscriber(topic, PoseStamped, onNewMessage)

    rospy.spin()
