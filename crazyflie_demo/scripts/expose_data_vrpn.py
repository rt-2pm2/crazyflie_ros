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
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped
from geometry_msgs.msg import Vector3Stamped
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
                [-q.x, q.w, q.z, -q.y],
                [-q.y, -q.z, q.w, q.x],
                [-q.z, q.y, -q.x, q.w]
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
    return (quat_tf, euler)


def onNewPose(pose_msg):
    global position_msg
    global attrpy_msg
    global pub_pos
    global opti_odom_pub
    global odom_broadcaster
    global firstTransform
    global valpha
    global qdalpha
    global t_delay

    # Store variables for computing the velocities
    global pos_
    global vel_
    global q_
    global qd_
    global t_

    p = pose_msg.pose.position
    quat = pose_msg.pose.orientation
    
    # Extract the array version
    p_a = np.array([p.x, p.y, p.z])
    q_a = np.array([quat.x, quat.y, quat.z, quat.w])
    
    if firstTransform:
        print("PublishExternalPosition: First Transform")
        # Initialize the position
        pos_ = p_a 
        q_ = q_a
        t_ = rospy.get_time() 
        
        firstTransform = False

    else:
        rosT = rospy.Time.now()

        dt = (rospy.get_time() - t_)
        t_ = rospy.get_time()
        
        ## Evaluate Velocity (Fading Filtered)
        vel = (1.0 - v_alpha) * vel_ + v_alpha * (p_a - pos_)/dt
        qd = (1.0 - qd_alpha) * qd_ + qd_alpha * (q_a - q_)/dt
        w = qd2w(quat, qd)

        pos_ = p_a
        vel_ = vel
        q_ = q_a
        qd_ = qd


        # Push forward position and attitude knowing the delay
        p.x = p.x + vel[0] * t_delay
        p.y = p.y + vel[1] * t_delay
        p.z = p.z + vel[2] * t_delay
   
        # Use q_a to compute intermediate values
        #   (It will be overwritten the next cycle)
        q_a = q_a + qd * t_delay
        q_a = q_a / np.linalg.norm(q_a) 

        quat.x = q_a[0]
        quat.y = q_a[1]
        quat.z = q_a[2]
        quat.w = q_a[3]


        # Compose the PointStamped message 
        position_msg.header.frame_id = pose_msg.header.frame_id
        position_msg.header.stamp = pose_msg.header.stamp
        position_msg.header.seq += 1
        position_msg.point = p

            
        ## Compose the Transform message
        (quat_tf, euler) = qMsg2qTf(quat)
        odom_broadcaster.sendTransform( (p.x, p.y, p.z), 
            quat_tf, rosT, "world", "opti_odom") 
        
        attrpy_msg.header.seq += 1
        attrpy_msg.header.frame_id = pose_msg.header.frame_id
        attrpy_msg.header.stamp = rosT
        attrpy_msg.vector.x = euler[0] * 180.0 / np.pi
        attrpy_msg.vector.y = euler[1] * 180.0 / np.pi
        attrpy_msg.vector.z = euler[2] * 180.0 / np.pi
        
        
        # Compose the Odometry message
        opti_odom.header.stamp = rosT 
        opti_odom.pose.pose.position = p
        opti_odom.pose.pose.orientation =quat 
        opti_odom.twist.twist.linear = vel 
        opti_odom.twist.twist.angular = w
         
        ## Publish

        # External Position (For the EKF)
        pub_pos.publish(position_msg)

        # External Pose (For the EKF)
        pub_pose.publish(pose_msg)

        # Odomtry (For plotting)
        pub_pose_rpy.publish(attrpy_msg)
        opti_odom_pub.publish(opti_odom)
        


if __name__ == '__main__':
    rospy.init_node('expose_data_vrpn', anonymous=True)

    print("Start expose_vrpn node")
    topic = rospy.get_param("~topic", "/vrpn_client_node/cf1/pose")
   
    v_alpha = rospy.get_param("~valpha", 0.7)
    qd_alpha = rospy.get_param("~qdalpha", 0.7)

    t_delay = rospy.get_param("~time_delay", 0.006)

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

    attrpy_msg = Vector3Stamped()
    attrpy_msg.header.seq = 0

    # Publishers
    pub_pos = rospy.Publisher("external_position", PointStamped, queue_size=3)
    pub_pose = rospy.Publisher("external_pose", PoseStamped, queue_size=3)
    pub_pose_rpy = rospy.Publisher("external_pose_rpy", Vector3Stamped, queue_size=3)
    opti_odom_pub = rospy.Publisher("opti_odom", Odometry, queue_size = 3)
 
    # Subscribe
    rospy.Subscriber(topic, PoseStamped, onNewPose)
    
    rospy.spin()
