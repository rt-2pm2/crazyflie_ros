#!/usr/bin/env python

# Python script to setup the crazyflie algorithms
# 
# - Set the commander level
# - Set the estimation algorithm
# - Set the control algorithm
# 

import rospy
import crazyflie
import time

from crazyflie_driver.srv import UpdateParams
from geometry_msgs.msg import PointStamped

# Callback to reset the initial status of the Kalman filter
# in case of external position

ekf_initialized = False;
init_ekf = False;

def ext_pos_callback(ext_point_stmp):
    global ekf_initialized
    global init_ekf
    
    if (not ekf_initialized) and init_ekf:
        # initialize kalman filter
        if (rospy.get_param('stabilizer/estimator') == 2):
            x = ext_point_stmp.point.x
            y = ext_point_stmp.point.x
            z = ext_point_stmp.point.x
            
            print("Initializing the KF: [", x, " ", y, " ", z, "]")
            
            rospy.set_param("kalman/initialX", x)
            rospy.set_param("kalman/initialY", y)
            rospy.set_param("kalman/initialZ", z)
            update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])
            
            ekf_initialized = True


if __name__ == '__main__':
    rospy.init_node('Setup_node')

    print("Vehicle Setup")

    est = 2
    ctr = 2

    # Read the parameters
    cf_id = rospy.get_param('~cf', 'cf1')
    comm_lev = rospy.get_param('~comm_lev', 1);
    estimator = rospy.get_param('~Estimator', 'EKF')
    controller = rospy.get_param('~Controller', 'Mellinger')
    req_reset = rospy.get_param('~ResEstimator', True)

    print("Selecting CF: ", cf_id)
    print('Selecting Commander Level: ', comm_lev)
    print("Selecting Estimator: ", estimator)
    print("Selecting Controller: ", controller)

    # Subscribe to the external position topic, in case it is necessary.
    ext_pos_topic = "/" + cf_id + "/external_position"
    print("Subscribing to the external position topic: ", ext_pos_topic)
    rospy.Subscriber(ext_pos_topic, PointStamped, ext_pos_callback)

    # Create CF object
    cf = crazyflie.Crazyflie("/" + cf_id, "/tf")

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    # Select the controller level
    while (cf.getParam("commander/enHighLevel") != comm_lev):
        cf.setParam("commander/enHighLevel", comm_lev)
    print("Correctly set ", cf.getParam("commander/enHighLevel"), 
        " commander level")

    # Map the estimator name to index
    if (estimator == 'EKF'):
        est = 2
    if (estimator == 'CMP'):
        est = 1
    if (estimator == 'USC'):
        est = 3
    # Set the estimator
    while (cf.getParam("stabilizer/estimator") != est):
        cf.setParam("stabilizer/estimator", est) # 1)Complementary 2)EKF 3)USC

    print("Correctly set ", cf.getParam("stabilizer/estimator")," estimator")
    time.sleep(1)

    # If Kalman reset the estimator
    if (estimator == 'EKF' and req_reset):
        cf.setParam("kalman/resetEstimation", 1)
        cf.setParam("kalman/resetEstimation", 0)
        init_ekf = True
    
    # Map the controller name to index
    if (controller == 'PID'):
        ctr = 1
    if (controller == 'Mellinger'):
        ctr = 2
    # Set the controller
    while (cf.getParam("stabilizer/controller") != ctr):
        cf.setParam("stabilizer/controller", ctr) # 1)PID  2)Mellinger
    print("Correctly set ", cf.getParam("stabilizer/controller")," controller")
    time.sleep(3)

    rate = rospy.Rate(1)
    while (init_ekf and not ekf_initialized and not rospy.is_shutdown()):
        print("Waiting for filter initialization...")
        rate.sleep()
        rospy.spin()

        
    
