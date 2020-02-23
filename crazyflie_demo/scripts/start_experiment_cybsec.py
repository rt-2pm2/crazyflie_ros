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

import numpy as np
import rospy
import crazyflie
import time

from crazyflie_driver.srv import UpdateParams

import uav_trajectory
from threading import Thread
from crazyflie_demo.msg import Trajectory 
from geometry_msgs.msg import Vector3 
from tf.transformations import euler_from_matrix

from crazyflie_demo.msg import ExperimentLog 


def req_takeoff(cf):
    cf.takeoff(targetHeight = 0.5, duration = 2.0)
    cf.takeoff(targetHeight = 0.5, duration = 2.0)
    time.sleep(2.5)

def switch_ctrl_module(cf, ctrl_id):
    cf.setParam("stabilizer/controller", ctrl_id)
    update_params(["stabilizer/controller"])


def req_landing(cf):
    rospy.loginfo("Landing")
    cf.land(targetHeight = 0.05, duration = 2.0)
    time.sleep(0.1)
    cf.land(targetHeight = 0.0, duration = 2.0)
    time.sleep(2)


if __name__ == '__main__':
    rospy.init_node('Captain')

    rospy.loginfo("Starting Node Captain")

    cf = crazyflie.Crazyflie("cf2", "/tf")

    rospy.wait_for_service('/cf2/update_params')
    rospy.loginfo("Commander found update_params service")
    update_params = rospy.ServiceProxy('/cf2/update_params', UpdateParams)

    req_takeoff(cf) 

    cf.goTo(goal = [0, 0.2, 0.0], yaw=0.0, duration = 2.0, relative = True)
    cf.goTo(goal = [0, 0.2, 0.0], yaw=0.0, duration = 2.0, relative = True)
    for i in range(50):
        if (np.random.rand() > 0.5):
            switch_ctrl_module(cf, 1)
        else:
            switch_ctrl_module(cf, 2)
        time.sleep(0.2)

    time.sleep(2.0)
    cf.goTo(goal = [0, -0.2, 0.0], yaw=0.0, duration = 2.0, relative = True)
    cf.goTo(goal = [0, -0.2, 0.0], yaw=0.0, duration = 2.0, relative = True)
    for i in range(50):
        if (np.random.rand() > 0.5):
            switch_ctrl_module(cf, 1)
        else:
            switch_ctrl_module(cf, 2)
        time.sleep(0.2)

    ####### END MISSION
    req_landing(cf)
 
    cf.stop()

