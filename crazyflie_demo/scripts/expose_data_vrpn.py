#!/usr/bin/env python
#
# This node subscribe to the pose message coming from the VRPN client.
# Using the pose information the node extract other information:
# - Velocity
# - Quaternion time derivative
# - Angular velocity (body frame)
# - Euler Angles (from the pose quaternion) 
#
#
#

import rospy
import tf
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped #PoseStamped added to support vrpn_client
from nav_msgs.msg import Odometry
from crazyflie_driver.srv import UpdateParams
import numpy as np


def qd2w(q, qd):
    """
    This function maps the quaternion derivative into the angular velocity
    in body frame
    """
    W = np.array(
            [
                [-q[1], q[0], q[3], -q[2]],
                [-q[2], -q[3], q[0], q[1]],
                [-q[3], q[2], -q[1], q[0]]
            ])
    w = np.matmul(W,qd)
    return w


def qMsg2qTf(quat):
    """
    This function maps the quaternion Msg into a quaternion TF, which in ROS
    are considered to be different (And the python library makes the conversion 
    cumbersome)
    """
    explicit_quat = [quat.x, quat.y, quat.z, quat.w]
    euler = tf.transformations.euler_from_quaternion(explicit_quat)
    quat_tf = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2]) 
    return quat_tf


def onNewPose(pose_msg):
    global position_msg
    global pub_pos
    global opti_odom_pub
    global odom_broadcaster
    global firstTransform
    global valpha
    global qdalpha

    # Store variables for computing the velocities
    global pos_
    global vel_
    global q_
    global qd_
    global t_

    position = pose_msg.pose.position
    quat = pose_msg.pose.orientation

    if firstTransform:
        print("PublishExternalPosition: First Transform")
        # Initialize the position
        pos_ = position 
        q_ = quat
        t_ = rospy.get_time() 


## This part should be moved somewhere else outside
#        # initialize kalman filter
#        if (rospy.get_param('stabilizer/estimator') == 2): 
#            rospy.set_param("kalman/initialX", x)
#            rospy.set_param("kalman/initialY", y)
#            rospy.set_param("kalman/initialZ", z)
#            update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])
#
#            rospy.set_param("kalman/resetEstimation", 1)
#            update_params(["kalman/resetEstimation"]) 
#            firstTransform = False
#        else:
#            rospy.set_param('stabilizer/estimator', 2)

    else:
        rosT = rospy.Time.now()

        # Compose the PointStamped message 
        position_msg.header.frame_id = pose_msg.header.frame_id
        position_msg.header.stamp = pose_msg.header.stamp
        position_msg.header.seq += 1
        position_msg.point = position

            
        ## Compose the Transform message
        quat_tf = qMsg2qTf(quat)
        odom_broadcaster.sendTransform( (x, y, z), quat_tf, 
                rosT, "world", "opti_odom") 
                
        dt = (rospy.get_time() - t_)
        t_ = rospy.get_time() 
        
        ## Evaluate Velocity (Fading Filtered)
        vel = (1.0 - v_alpha) * vel_ + v_alpha * (x - pos_)/dt
        qd = (1.0 - qd_alpha) & qd_ + qd_alpha * (quat - q_)/dt
        w = qd2w(quat, qd)

        pos_ = position
        vel_ = vel
        q_ = quat
        qd_ = qdot

        # Compose the Odometry message
        opti_odom.header.stamp = rosT 
        opti_odom.pose_msg.pose.position.x = position
        opti_odom.pose_msg.pose.orientation =quat 
        opti_odom.twist.twist.linear = vel 
        opti_odom.twist.twist.angular = w
         
        ## Publish
        pub_pos.publish(position_msg)
        pub_pose.publish(pose_msg)
        opti_odom_pub.publish(opti_odom)


if __name__ == '__main__':
    rospy.init_node('data_expose_vrpn', anonymous=True)

    print("Start expose_vrpn node")
    topic = rospy.get_param("~topic", "/vrpn_client_node/cf1/pose")
    
    v_alpha = rospy.get_param("~valpha", 0.7)
    qd_alpha = rospy.get_param("~qdalpha", 0.7)

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    firstTransform = True

    pos_ = np.zeros((3), dtype=float)
    vel_ = np.zeros((3), dtype=float)
    q_ = np.zeros((4), dtype=float)
    qd_ = np.zeros((4), dtype=float)

    # Tf broadcaster
    odom_broadcaster = tf.TransformBroadcaster()

    # Messages
    position_msg = PointStamped()
    position_msg.header.seq = 0
    position_msg.header.stamp = rospy.Time.now()

    opti_odom = Odometry()
    opti_odom.child_frame_id = "cf1"
    opti_odom.header.frame_id = "world"

    # Publishers
    pub_pos = rospy.Publisher("external_position", PointStamped, queue_size=3)
    pub_pose = rospy.Publisher("external_pose", PoseStamped, queue_size=3)
    opti_odom_pub = rospy.Publisher("opti_odom", Odometry, queue_size = 3)
 
    # Subscribe
    rospy.Subscriber(topic, PoseStamped, onNewPose)
    
    rospy.spin()
