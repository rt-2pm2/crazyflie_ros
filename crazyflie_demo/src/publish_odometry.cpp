/**
 * @file publish_odometry.cpp
 *
 * @author rt-2pm2
 *
 *
 * This ROS node subscribe to the topic regarding the generated 
 * trajectory and * converts the message into an odometry ROS 
 * message that can be plotted with rviz.
 *
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <crazyflie_demo/Trajectory.h>

// DATA -------------------------------------------------------------------------
// Pubblication variables 
static ros::Publisher odom_pub;
static tf::TransformBroadcaster odom_broadcaster;

//
// Odometry: Topic 
static float trj_pos[3];
static float trj_vel[3];
static float trj_acc[3];

// Odometry: quaternion
static geometry_msgs::Quaternion odom_quat;

// Odometry: TF 
static geometry_msgs::TransformStamped odom_trans;

// Odometry Topic
static nav_msgs::Odometry odom;

// CALLBACK ---------------------------------------------------------------------

void trj_callback(const boost::shared_ptr<crazyflie_demo::Trajectory const>& msg);

// MAIN -------------------------------------------------------------------------
int main(int argc, char** argv) {

	// Initialize the ROS stack
	ros::init(argc, argv, "odometry_publisher");
	ros::NodeHandle nh;

	// OUTPUTS:
	// 1) Advertise topic
	odom_pub = nh.advertise<nav_msgs::Odometry> ("vehicle_od", 20);

	// Initialize the header part of the odometry TF packet 
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	// Initialize the header part of the odometry topic message
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";

	// INPUT
	// Subscribe to the topic produced by the node sending the trajectory
	ros::Subscriber trj_sub = nh.subscribe("/ghost_trajectory", 10, trj_callback);	

	while (nh.ok()) {
		ros::spin();
	}
}


/* 
 * Topic Callback
 * Whenever I receive a new Trajectory message, update the odometry.
 */
void trj_callback(const boost::shared_ptr<crazyflie_demo::Trajectory const>& msg) {
	// Take the time
	ros::Time current_time = ros::Time::now();

	// Fetch the ROS message
	trj_pos[0] = msg->px;	
	trj_pos[1] = msg->py;	
	trj_pos[2] = msg->pz;	

	trj_vel[0] = msg->vx;	
	trj_vel[1] = msg->vy;	
	trj_vel[2] = msg->vz;	

	trj_acc[0] = msg->accx;	
	trj_acc[1] = msg->accy;	
	trj_acc[2] = msg->accz;	

	// Update Tranformation Message	
	odom_trans.header.stamp = current_time;
	// Update the odometry transformation packet with
	// the information received via ROS topic
	odom_quat = tf::createQuaternionMsgFromYaw(0.0);

	// Position
	odom_trans.transform.translation.x = trj_pos[0];
	odom_trans.transform.translation.y = trj_pos[1];
	odom_trans.transform.translation.z = trj_pos[2];
	// Orientation
	odom_trans.transform.rotation = odom_quat;

	// Update Topic Message
	odom.header.stamp = current_time;
	// Pose part of the odometry message
	odom.pose.pose.position.x = trj_pos[0];
	odom.pose.pose.position.y = trj_pos[1];
	odom.pose.pose.position.z = trj_pos[2];
	odom.pose.pose.orientation = odom_quat;

	// Twist part of the odometry message 
	odom.twist.twist.linear.x = trj_vel[0];
	odom.twist.twist.linear.y = trj_vel[1];
	odom.twist.twist.linear.z = trj_vel[2];

	// Send the transform
	odom_broadcaster.sendTransform(odom_trans);

	// Pubblish the odometry message
	odom_pub.publish(odom);
}

