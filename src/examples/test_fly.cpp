/*
Parker Conroy
ARLab @ University of Utah
Nov 2012

This code runs a test of the hover and psudo-hover state. The psudo-hover state is designed to hover the craft without engaging the hover state. The hover state (state #3) that is actually closed loop control where the craft's velocity is measured with the bottom camera.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

	geometry_msgs::Twist twist_msg;
	geometry_msgs::Twist twist_msg_hover;
	geometry_msgs::Twist twist_msg_pshover;
	std_msgs::Empty emp_msg;
	

int main(int argc, char** argv)
{

	printf("Manual Test Node Starting");
	ros::init(argc, argv,"ARDrone_manual_test");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);

	ros::Publisher pub_empty_land;
	ros::Publisher pub_twist;
	ros::Publisher pub_empty_takeoff;
	ros::Publisher pub_empty_reset;
	double time;

//hover message
			twist_msg_hover.linear.x=0.0; 
			twist_msg_hover.linear.y=0.0;
			twist_msg_hover.linear.z=0.0;
			twist_msg_hover.angular.x=0.0; 
			twist_msg_hover.angular.y=0.0;
			twist_msg_hover.angular.z=0.0;  

//psudo-hover message
			twist_msg_pshover.linear.x=0.00001; // lowest value for psudo hover to work
			twist_msg_pshover.linear.y=0.0;
			twist_msg_pshover.linear.z=0.0;
			twist_msg_pshover.angular.x=0.0; 
			twist_msg_pshover.angular.y=0.0;
			twist_msg_pshover.angular.z=0.0; 

//command message
			float fly_time=7.0;
			float land_time=5.0;
			float kill_time =2.0;	

			twist_msg.linear.x=0.0; 
			twist_msg.linear.y=-0.0001;
			twist_msg.linear.z=0.0;
			twist_msg.angular.x=0.0; 
			twist_msg.angular.y=0.0;
			twist_msg.angular.z=0.0;


	
    pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
	pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */

	
	time =(double)ros::Time::now().toSec();	
	ROS_INFO("Starting ARdrone_test loop");


while (ros::ok()) {
		
		if (	 (double)ros::Time::now().toSec()< time+5.0){
		
			pub_empty_takeoff.publish(emp_msg); //launches the drone
				pub_twist.publish(twist_msg_hover); //drone is flat
			ROS_INFO("Taking off");
			}//takeoff

		else if (	 (double)ros::Time::now().toSec()> time+fly_time+land_time+kill_time){
		
			ROS_INFO("Closing Node");
			pub_empty_reset.publish(emp_msg); //kills the drone		
			break; 
			
			}//kill node

		else if (	 ((double)ros::Time::now().toSec()> time+fly_time+land_time)){
		
			pub_twist.publish(twist_msg_hover); //drone is flat
			pub_empty_land.publish(emp_msg); //lands the drone
			ROS_INFO("Landing");
			}//land
		
		else
			{	
			pub_twist.publish(twist_msg_pshover);
			ROS_INFO("Psudo-Hovering (w/o CV)");
			}//fly according to desired twist

	ros::spinOnce();
	loop_rate.sleep();

}//ros::ok

}//main
