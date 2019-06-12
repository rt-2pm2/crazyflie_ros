#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory
from threading import Thread
from crazyflie_demo.msg import Trajectory 

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

            # Pubblish the evaluated trajectory
            ghost_pub.publish(msg)

            # Wait the next loop
            r.sleep()
            # Take the time
            curr_time = rospy.get_time()
   

if __name__ == '__main__':
    rospy.init_node('Ghost_Node')

    v = rospy.search_param('file')
    if (v):
        trj_file = rospy.get_param(v)
    else:
        rospy.signal_shutdown("Trjectory file not found!")

    
    print("Loading trajectory from: ", trj_file)
    traj = uav_trajectory.Trajectory()
    traj.loadcsv(trj_file)
    
    print("Trajecotry duration: ", traj.duration)

    t = Thread(target=rep_trajectory, args=(traj,[0,0,0], 10)).start()

