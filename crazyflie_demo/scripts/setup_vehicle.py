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

def set_ctrl_gains(cf, gains):
    print('Setting Control Gains...')
    for (name, k) in gains.items():
        print(name + " --> " +  str(k))
        while (cf.getParam("ctrlDD_par/" + name) != k):
            cf.setParam("ctrlDD_par/" + name, k)
            update_params(["ctrlDD_par/" + name])
    print("DONE!\n")

def set_est_gains(cf, gains):
    print('Setting Estimator Gains...')
    for (name, k) in gains.items():
        print(name + " --> " +  str(k))
        while (cf.getParam("estimatorDD_par/" + name) != k):
            cf.setParam("estimatorDD_par/" + name, k)
            update_params(["estimatorDD_par/" + name])
    print("DONE!\n")

def ext_pos_callback(ext_point_stmp):
    global ekf_initialized
    global init_ekf
    
    if (not ekf_initialized) and init_ekf:
        # initialize kalman filter
        if (rospy.get_param('stabilizer/estimator') == 2):
            x = ext_point_stmp.point.x
            y = ext_point_stmp.point.y
            z = ext_point_stmp.point.z
            
            rospy.loginfo("Initializing the KF: [" + str(x) + " " + 
                    str(y) + " " + str(z) + "]")
            
            rospy.set_param("kalman/initialX", x)
            rospy.set_param("kalman/initialY", y)
            rospy.set_param("kalman/initialZ", z)
            rospy.sleep(0.5)
            update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])
            
            ekf_initialized = True
            rospy.loginfo("Setup of Vehicle " + cf_id + " Complete!\n")


if __name__ == '__main__':
    rospy.init_node('Setup_node')

    rospy.loginfo("Vehicle Setup")

    est = 2
    ctr = 2
    cmode = 1

    # Read the parameters
    cf_id = rospy.get_param('~cf', 'cf1')
    comm_lev = rospy.get_param('~comm_lev', 1);
    estimator = rospy.get_param('~Estimator', 'EKF')
    controller = rospy.get_param('~Controller', 'Mellinger')
    req_reset = rospy.get_param('~ResEstimator', True)
    stabMode = rospy.get_param('~stabMode', '1')

    rospy.loginfo("Selecting CF: " + str(cf_id))
    rospy.loginfo('Selecting Commander Level: ' + str(comm_lev))
    rospy.loginfo("Selecting Estimator: " + str(estimator))
    rospy.loginfo("Selecting Controller: " + str(controller))
    rospy.loginfo("Selecting StabMode: " + str(stabMode))

    # Subscribe to the external position topic, in case it is necessary.
    ext_pos_topic = "/" + cf_id + "/external_position"
    rospy.loginfo("Subscribing to the external position topic: " + 
            str(ext_pos_topic))
    rospy.Subscriber(ext_pos_topic, PointStamped, ext_pos_callback)

    # Create CF object
    rospy.sleep(1.0)
    cf = crazyflie.Crazyflie("/" + cf_id, "/tf")

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    rospy.sleep(1.0)
    # Select the controller level
    while (cf.getParam("commander/enHighLevel") != comm_lev):
        cf.setParam("commander/enHighLevel", comm_lev)
    rospy.loginfo("Correctly set " + str(cf.getParam("commander/enHighLevel")) + 
        " commander level")

    # Map the estimator name to index
    if (estimator == 'EKF'):
        est = 2
    if (estimator == 'CMP'):
        est = 1
    if (estimator == 'DD'):
        est = 3

    if (estimator == 'DD'):
        EstimationGains = {
                'K_x': 0.005, 
                'K_x_d': 0.1,  
                'K_y': 0.005,
                'K_y_d': 0.1,  
                'K_a2d0': 0.002,
                'K_a2d1': 0.002,
                'K_a2d2': 0.002,
                'K_a2d3': 0.002,
                'K_b2d0': 0.0001,
                'K_b2d1': 0.0001,
                'K_b2d2': 0.0001,
                'K_b2d3': 0.0001,
                'K_b2d6': 0.0001,
                'K_b2d7': 0.0001,
                'K_b2d8': 0.0001,
                'K_b2d9': 0.0001,
                'K_b2d10': 0.0001,
                'K_b2d11': 0.0001,
                'K_b2d12': 0.0001,
                'K_b2d13': 0.0001,
                'K_b2d14': 0.0001,
                'K_b2d15': 0.0001,
                }
        set_est_gains(cf, EstimationGains)
        time.sleep(1.0)


    # Set the estimator
    while (cf.getParam("stabilizer/estimator") != est):
        cf.setParam("stabilizer/estimator", est)

    rospy.loginfo("Correctly set " + str(cf.getParam("stabilizer/estimator")) + 
            " estimator")
    time.sleep(1)

    # If Kalman reset the estimator
    if (estimator == 'EKF' and req_reset):   
        rospy.set_param("kalman/resetEstimation", 1)
        update_params(["kalman/resetEstimation"])
        rospy.sleep(0.5)
        update_params(["kalman/resetEstimation"])
        rospy.sleep(0.5)
        rospy.set_param("kalman/resetEstimation", 0)
        update_params(["kalman/resetEstimation"])
        rospy.sleep(0.5)
        update_params(["kalman/resetEstimation"])
        rospy.sleep(0.5)
        init_ekf = True
    
    # Map the controller name to index
    if (controller == 'PID'):
        ctr = 1
    if (controller == 'Mellinger'):
        ctr = 2
    if (controller == 'DD'):
        ctr = 4
    if (controller == 'EXT' or controller == "Ext"):
        ctr = 5


    if (controller == 'DD'):
        ControlGains = {
                'Kxy': -4.0,
                'Kxy_d': -4.0,
                'Kz': -25.0,
                'Kz_d': -10.0,
                'Katt': -16.0,
                'Katt_d': -8.0,
                'Kyaw': 34.0,
                'Kyaw_d': 12.0
        }
        set_ctrl_gains(cf, ControlGains)
        time.sleep(1.0)


    # Set the controller
    while (cf.getParam("stabilizer/controller") != ctr):
        cf.setParam("stabilizer/controller", ctr) # 1)PID  2)Mellinger
        update_params(["stabilizer/controller"])
        rospy.sleep(0.2)

    rospy.loginfo("Correctly set " + str(cf.getParam("stabilizer/controller")) + 
            " controller")

    
    if ctr != 5:
        # Setting the flight mode on the Crazyfly
        if (stabMode == 0):
            # Rates
            rospy.loginfo("Set Rates Control Mode")
            cmode = 0
        if (stabMode == 1):
            # Angle
            rospy.loginfo("Set Angle Control Mode")
            cmode = 1

        while (cf.getParam("flightmode/stabModeRoll") != cmode):
            rospy.loginfo("Setting flightmode/stabModeRoll = " + str(cmode))
            cf.setParam("flightmode/stabModeRoll", cmode)
            update_params(["flightmode/stabModeRoll"])
            rospy.sleep(0.2)
        while (cf.getParam("flightmode/stabModePitch") != cmode):
            cf.setParam("flightmode/stabModePitch", cmode) 
            update_params(["flightmode/stabModePitch"])
            rospy.sleep(0.2)
            rospy.loginfo("Setting flightmode/stabModePitch = " + str(cmode))


        
    time.sleep(1)

    rate = rospy.Rate(1)
    while (init_ekf and not ekf_initialized and not rospy.is_shutdown()):
        rospy.loginfo("Waiting for filter initialization...")
        rate.sleep()
        rospy.spin()

        
    
