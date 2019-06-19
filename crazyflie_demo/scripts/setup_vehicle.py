#!/usr/bin/env python

# Python script to start a crazyflie mission.
# 
# The script expects the name of the file as a parameter. It is also possible
# to specify the frequency of the thread publishing the position of the 
# 'ghost', that is the simulation of the expected trajectory.
# 
# Precisely the script will:
#     Load a trajectory from file
#     Upload the trajectory on the crazyflie
#     Ask the crazyflie to takeoff
#     Send the command to start the mission(trajectory)
#     Start a thread that simulate the trjectory execution
#     Ask the crazyflie to land after the end of the mission
# 
# 

import rospy
import crazyflie
import time
import uav_trajectory
from threading import Thread
from crazyflie_demo.msg import Trajectory 
from tf.transformations import euler_from_matrix


if __name__ == '__main__':
    rospy.init_node('Setup_node')

    print("Starting Vehicle Setup")

    cf = crazyflie.Crazyflie("cf1", "/tf")

    while (cf.getParam("commander/enHighLevel") != 1): 
        cf.setParam("commander/enHighLevel", 1)

    while (cf.getParam("stabilizer/estimator") != 3):
        cf.setParam("stabilizer/estimator", 3) # 1)Complementary 2)EKF 3)USC

    while (cf.getParam("stabilizer/controller") != 2):
        cf.setParam("stabilizer/controller", 2) # 1)PID  2)Mellinger 
    time.sleep(3)

    cf.setParam("kalman/resetEstimation", 1)
    cf.setParam("kalman/resetEstimation", 0)

    
