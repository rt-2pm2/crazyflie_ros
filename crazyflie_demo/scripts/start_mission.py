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
import uav_trajectory
from threading import Thread
from crazyflie_demo.msg import Trajectory 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_matrix

# Trajectory Publisher
ghost_pub = rospy.Publisher('ghost_trajectory', Odometry, queue_size=10)

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

        msg = Odometry()

        # Publishing Loop
        while (curr_time < end_time):
            # Evaluate the trajectory
            rep_trj = trajectory.eval(curr_time - start_time)

            msg.pose.pose.position.x = rep_trj.pos[0]
            msg.pose.pose.position.y = rep_trj.pos[1]
            msg.pose.pose.position.z = rep_trj.pos[2]

            msg.twist.twist.linear.x = rep_trj.vel[0]
            msg.twist.twist.linear.y = rep_trj.vel[1]
            msg.twist.twist.linear.z = rep_trj.vel[2]

            #msg.accx = rep_trj.acc[0]
            #msg.accy = rep_trj.acc[1]
            #msg.accz = rep_trj.acc[2]

            ## Conver the Rotation matrix to euler angles
            #R = rep_trj.R
            #(roll, pitch, yaw) = euler_from_matrix(R)

            #msg.r = roll * 180 / np.pi
            #msg.p = pitch * 180 / np.pi
            #msg.y = yaw * 180 / np.pi

            # Pubblish the evaluated trajectory
            ghost_pub.publish(msg)

            # Wait the next loop
            r.sleep()
            # Take the time
            curr_time = rospy.get_time()


if __name__ == '__main__':
    rospy.init_node('Node_commander')

    rospy.loginfo("Starting Node Commander")

    cf = crazyflie.Crazyflie("cf2", "/tf")

    file_name = rospy.search_param('trajectory_file')
    if (file_name):
        trj_file = rospy.get_param(file_name) 
        print("Trajectory file parameter found! ", trj_file)
    else:
        rospy.signal_shutdown("Trjectory file not found!")


    frequency = rospy.get_param('freq_ghost', 30.0);

    rospy.loginfo("Uploading Trajectory...")
    traj = uav_trajectory.Trajectory()
    traj.loadcsv(trj_file) 
    cf.uploadTrajectory(0, 0, traj)
    rospy.loginfo("Trajectory duration: " + str(traj.duration))
    time.sleep(3)

    cf.takeoff(targetHeight = 0.5, duration = 3.0)
    time.sleep(4.0)
    
    #cf.goTo(goal = [0, 0.5, 1.10], yaw=0.0, duration = 2.0, relative = False)
    #cf.goTo(goal = [0, 0.5, 1.10], yaw=0.0, duration = 2.0, relative = False)
    #time.sleep(3.0)

    rospy.loginfo("Starting Trajectory")
    #cf.startTrajectory(0, timescale=1.0)
    cf.startTrajectory(0, timescale=1.0)
    t = Thread(target=rep_trajectory, args=(traj,[0.0, 0.5, 1.10], frequency)).start()

    #time.sleep(traj.duration)
 
    #cf.stop()
    rospy.loginfo("Landing")
    #cf.land(targetHeight = 0.05, duration = 2.0)
    #time.sleep(0.1)
    #cf.land(targetHeight = 0.05, duration = 2.0)
    #time.sleep(2)

