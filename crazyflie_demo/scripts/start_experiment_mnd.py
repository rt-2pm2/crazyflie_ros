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

# Trajectory Publisher
ghost_pub = rospy.Publisher('cf2/ghost_trajectory', Vector3, queue_size=10)

experiment_pub = rospy.Publisher('/experiment_switches', ExperimentLog, queue_size = 5) 



# Global Variables
enabled_distortion = 0
malicious_anchor = 0
enabled_module = 0
exp_msg = ExperimentLog()

exp_msg.enabled_distortion = enabled_distortion
exp_msg.malicious_anchor = malicious_anchor
exp_msg.enabled_module = enabled_module


# This was publishing Trajectory messages.
def rep_trajectory(trajectory, start_position, freq):
        timeSpan = trajectory.duration; 

        r = rospy.Rate(freq)

        print("Running at freq. = ", r)
        start_time = rospy.get_time() 
        curr_time = start_time
        print("Current time: ", curr_time)
        print("Start time: ", start_time)
        print("Expected end time: ", start_time + timeSpan)
        end_time = start_time + timeSpan

        msg = Vector3()

        # Publishing Loop
        while (curr_time < end_time):
            # Evaluate the trajectory
            rep_trj = trajectory.eval(curr_time - start_time)

            msg.x = rep_trj.pos[0]
            msg.y = rep_trj.pos[1]
            msg.z = rep_trj.pos[2]

            # Pubblish the evaluated trajectory
            ghost_pub.publish(msg)

            # Wait the next loop
            # Take the time
            curr_time = curr_time + 0.5



def req_takeoff(cf):
    cf.takeoff(targetHeight = 0.5, duration = 2.0)
    cf.takeoff(targetHeight = 0.5, duration = 2.0)
    time.sleep(2.5)

def req_start_trj(cf):
    rospy.loginfo("Starting Trajectory")
    cf.startTrajectory(0, timescale=1.0)
    cf.startTrajectory(0, timescale=1.0)
    time.sleep(traj.duration)

def switch_mnd_module(cf, flag):
    global exp_msg

    if (flag):
        print("Activating the Malicious Node Detector")
        cf.setParam("mnd_param/activate", 1)
        exp_msg.enabled_module = 1;
    else:
        print("Deactivating the Malicious Node Detector")
        cf.setParam("mnd_param/activate", 0)
        exp_msg.enabled_module = 0;

    update_params(["mnd_param/activate"])
    update_params(["mnd_param/activate"])
   
    now_time = rospy.get_rostime()
    exp_msg.header.seq = exp_msg.header.seq + 1
    exp_msg.header.stamp.secs = now_time.secs
    exp_msg.header.stamp.nsecs = now_time.nsecs
    experiment_pub.publish(exp_msg)

    rospy.sleep(0.2) 


def switch_distortion(flag):
    global exp_msg

    if (flag):
        print("Enabling Distortion...")
        rospy.set_param("/Dummy_Anchors/enable_distortion", True)
        exp_msg.enabled_distortion = 1
    else:
        print("Disabling Distortion...")
        rospy.set_param("/Dummy_Anchors/enable_distortion", False)
        exp_msg.enabled_distortion = 0
    
    exp_msg.header.seq = exp_msg.header.seq + 1
    now_time = rospy.get_rostime()
    exp_msg.header.stamp.secs = now_time.secs
    exp_msg.header.stamp.nsecs = now_time.nsecs
    experiment_pub.publish(exp_msg)


def req_landing(cf):
    rospy.loginfo("Landing")
    cf.land(targetHeight = 0.05, duration = 2.0)
    time.sleep(0.1)
    cf.land(targetHeight = 0.0, duration = 2.0)
    time.sleep(2)

def uploadTrajToDrone(cf, traj):
    rospy.loginfo("Uploading Trajectory...")
     
    cf.uploadTrajectory(0, 0, traj)
    rospy.loginfo("Trajectory duration: " + str(traj.duration))
    time.sleep(1)

    return traj

if __name__ == '__main__':
    rospy.init_node('Captain')

    rospy.loginfo("Starting Node Captain")

    cf = crazyflie.Crazyflie("cf2", "/tf")

    rospy.wait_for_service('/cf2/update_params')
    rospy.loginfo("Commander found update_params service")
    update_params = rospy.ServiceProxy('/cf2/update_params', UpdateParams)

    frequency = rospy.get_param('freq_ghost', 2.0);

    ## Load Trajectory File
    file_name = rospy.search_param('trajectory_file')
    if (file_name):
        trj_file = rospy.get_param(file_name) 
        print("Trajectory file parameter found! ", trj_file)
    else:
        rospy.signal_shutdown("Trjectory file not found!")
    traj = uav_trajectory.Trajectory()
    traj.loadcsv(trj_file)

    ## Upload the trajectory file to drone
#    uploadTrajToDrone(cf, traj)

    ####### START MISSION
    now_time = rospy.get_rostime()
    exp_msg.header.stamp.secs = now_time.secs
    exp_msg.header.stamp.nsecs = now_time.nsecs
    experiment_pub.publish(exp_msg)

#    req_takeoff(cf) 

    ## Follow Trajectory 1
#    t = Thread(target=rep_trajectory,
#            args=(traj,[0.0, 0.0, 0.0], frequency)).start() 

#    req_start_trj(cf) 

    ## Enable Distortion
    rospy.set_param("/Dummy_Anchors/distortion_value", 0.7)
    
    switch_distortion(True)   
    time.sleep(10)
    switch_distortion(False)
    time.sleep(10)

    switch_mnd_module(cf, True)   
    switch_distortion(True)   
    time.sleep(10)
    switch_distortion(False)
    time.sleep(10)

    ####### END MISSION
#    req_landing(cf)
 
#    cf.stop()

