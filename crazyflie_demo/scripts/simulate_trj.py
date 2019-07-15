#!/usr/bin/env python

# This script loads a trajectory file and simulates the execution publishing a 
# custom trajectory message that is converted in odometry by another node

import rospy
import crazyflie
import time
import uav_trajectory
from threading import Thread
from crazyflie_demo.msg import Trajectory 
from tf.transformations import euler_from_matrix

# Trajectory Publisher
ghost_pub = rospy.Publisher('ghost_trajectory', Trajectory, queue_size=10)

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

        msg = Trajectory()

        # Publishing Loop
        while (curr_time < end_time):
            # Evaluate the trajectory
            rep_trj = trajectory.eval(curr_time - start_time)
            
            msg.px = rep_trj.pos[0]
            msg.py = rep_trj.pos[1]
            msg.pz = rep_trj.pos[2]

            msg.vx = rep_trj.vel[0]
            msg.vy = rep_trj.vel[1]
            msg.vz = rep_trj.vel[2]

            msg.accx = rep_trj.acc[0]
            msg.accy = rep_trj.acc[1]
            msg.accz = rep_trj.acc[2]

            # Conver the Rotation matrix to euler angles
            R = rep_trj.R
            (roll, pitch, yaw) = euler_from_matrix(R)

            msg.r = roll
            msg.p = pitch
            msg.y = yaw

            # Pubblish the evaluated trajectory
            ghost_pub.publish(msg)

            # Wait the next loop
            r.sleep()
            # Take the time
            curr_time = rospy.get_time()
   

if __name__ == '__main__':
    rospy.init_node('Ghost_Node')

    start_point = rospy.get_param('start_p', [0.0, 0.0 ,0.0])
    file_name = rospy.search_param('trj_file')
    if (file_name):
        trj_file = rospy.get_param(file_name) 
        print("Trajectory file found! ", trj_file)
    else:
        rospy.loginfo("Expecting parameter 'trj_file'")
        rospy.loginfo("rosrun plot_ghost.py _trj_file:='nameofthefile'")
        rospy.signal_shutdown("Trjectory file not found!")

    
    frequency = rospy.get_param('freq', 30.0);

    
    print("Loading trajectory from: ", trj_file)
    traj = uav_trajectory.Trajectory()
    traj.loadcsv(trj_file)

    t = Thread(target=rep_trajectory, args=(traj,start_point, frequency)).start()
 
