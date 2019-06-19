#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped #PoseStamped added to support vrpn_client
from nav_msgs.msg import Odometry
from crazyflie_driver.srv import UpdateParams
import numpy as np

def onNewTransform(pose):
    global msg
    global pub
    global opti_odom_pub
    global odom_broadcaster
    global firstTransform
    global alpha

    global pos_
    global vel_
    global t_

    x = pose.pose.position.x
    y = pose.pose.position.y
    z = pose.pose.position.z
    quat = pose.pose.orientation


    if firstTransform:
        print("PublishExternalPosition: First Transform")
        # Initialize the position
        opti_odom.pose.pose.position.x = x
        opti_odom.pose.pose.position.y = y
        opti_odom.pose.pose.position.z = z
        t_ = rospy.get_time() 
        # initialize kalman filter
        if (rospy.get_param('stabilizer/estimator') == 2): 
            rospy.set_param("kalman/initialX", x)
            rospy.set_param("kalman/initialY", y)
            rospy.set_param("kalman/initialZ", z)
            update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])

            rospy.set_param("kalman/resetEstimation", 1)
            update_params(["kalman/resetEstimation"]) 
            firstTransform = False
        else:
            rospy.set_param('stabilizer/estimator', 2)

    else:
        # Compose the PointStamped message 
        msg.header.frame_id = pose.header.frame_id
        msg.header.stamp = pose.header.stamp
        msg.header.seq += 1
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        ## Publish the position message
        pub.publish(msg)

        ## Publish the pose message
        pub_pose.publish(pose)
    
        ## Compose the Transform message
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        euler = tf.transformations.euler_from_quaternion(explicit_quat)
        quat_tf = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2]) 
        odom_broadcaster.sendTransform(
            (x, y, z), quat_tf,  
            rospy.Time.now(), "world", "opti_odom") 
                

        # Compose the Odometry message
        dt = (rospy.get_time() - t_)
        t_ = rospy.get_time() 

        ## Position/Orientation
        opti_odom.header.stamp = rospy.Time.now() 
        opti_odom.pose.pose.position.x = x
        opti_odom.pose.pose.position.y = y
        opti_odom.pose.pose.position.z = z
        opti_odom.pose.pose.orientation = pose.pose.orientation

        ## Velocity
        # Compute the velocity from the measurements (Adding a fading filter)
        opti_odom.twist.twist.linear.x = (1.0 - v_alpha) * opti_odom.twist.twist.linear.x + \
                                            v_alpha * (x - pos_[0])/dt

        opti_odom.twist.twist.linear.y = (1.0 - v_alpha) * opti_odom.twist.twist.linear.y + \
                                            v_alpha * (y - pos_[1])/dt
        
        opti_odom.twist.twist.linear.z = (1.0 - v_alpha) * opti_odom.twist.twist.linear.z + \
                                            v_alpha * (z - pos_[2])/dt

        # Save the current position for the next derivative step
        pos_[0] = x
        pos_[1] = y
        pos_[2] = z
        opti_odom_pub.publish(opti_odom)


if __name__ == '__main__':
    rospy.init_node('publish_external_position_vrpn', anonymous=True)

    print("Start ExternalPosition_Publisher")
    topic = rospy.get_param("~topic", "/crazyflie1/vrpn_client_node/crazyflie1/pose")
    
    v_alpha = rospy.get_param("~alpha", 0.5)

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    firstTransform = True

    pos_ = np.zeros((3), dtype=float)
    vel_ = np.zeros((3), dtype=float)

    # Tf broadcaster
    odom_broadcaster = tf.TransformBroadcaster()

    # Messages
    msg = PointStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()

    opti_odom = Odometry()
    opti_odom.child_frame_id = "cf1"
    opti_odom.header.frame_id = "world"

    # Publishers
    pub = rospy.Publisher("external_position", PointStamped, queue_size=2)
    pub_pose = rospy.Publisher("external_pose", PoseStamped, queue_size=2)
    opti_odom_pub = rospy.Publisher("opti_odom", Odometry, queue_size = 3)

        
    # Subscribe
    rospy.Subscriber(topic, PoseStamped, onNewTransform)
    
    print("Start streaming position information")
    rospy.spin()
