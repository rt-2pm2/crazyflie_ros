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
    req_reset = rospy.get_param('~ResEstimator', 0)

    print("Selecting CF: ", cf_id)
    print('Selecting Commander Level: ', comm_lev)
    print("Selecting Estimator: ", estimator)
    print("Selecting Controller: ", controller)

    # Create CF object
    cf = crazyflie.Crazyflie(cf_id, "/tf")


    while (cf.getParam("commander/enHighLevel") != comm_lev):
        cf.setParam("commander/enHighLevel", comm_lev)

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

    print("Correctly selected ", cf.getParam("stabilizer/estimator")," estimator")
    time.sleep(1)

    # If Kalman reset the estimator
    if (estimator == 'EKF' and req_reset):
        cf.setParam("kalman/resetEstimation", 1)
        cf.setParam("kalman/resetEstimation", 0)

    # Map the controller name to index
    if (controller == 'PID'):
        ctr = 1
    if (controller == 'Mellinger'):
        ctr = 2
    # Set the controller
    while (cf.getParam("stabilizer/controller") != ctr):
        cf.setParam("stabilizer/controller", ctr) # 1)PID  2)Mellinger
    print("Correctly selected ", cf.getParam("stabilizer/controller")," estimator")
    time.sleep(3)



    
