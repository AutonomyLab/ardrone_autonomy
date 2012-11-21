/*
Parker Conroy
ARLab @ University of Utah
Nov 2012

This code actuates the ARdrone from a generic joystick message. It is open loop.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

	
	double joy_x_,joy_y_,joy_z_;
	double joy_x,joy_y,joy_z;
	int new_msg;
	float forget =0.99;
	//double msg_time, old_msg_time;
	double joy_x_old,joy_y_old,joy_z_old;
	int seq, seq_old;
	geometry_msgs::Twist twist_msg;
	std_msgs::Empty emp_msg;
	sensor_msgs::Joy joy_msg_in;

	
		void joy_callback(const sensor_msgs::Joy& joy_msg_in)
{
		//Take in point
	joy_x_=joy_msg_in.axes[0];
	joy_y_=joy_msg_in.axes[1];
	joy_z_=joy_msg_in.axes[2];
	//msg_time=(double)ros::Time::now().toNSec();
    new_msg=1;
}
	
float map(float value, float in_min, float in_max, float out_min, float out_max) {
  return (float)((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}	


int main(int argc, char** argv)
{
	ROS_INFO("Node Starting");
	ros::init(argc, argv,"ARDrone_fly_from_joy");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);

	ros::Publisher pub_twist;
	ros::Publisher pub_empty;
	ros::Subscriber joy_sub;
	joy_x_old=0;
	joy_y_old=0;
	joy_z_old=0;

    pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
	pub_empty = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
	joy_sub = node.subscribe("/joy", 1, joy_callback);
	
//	ROS_INFO("Publishing Launch Command");
	
	

    ROS_INFO("Publishing Twist Message");
 	while (ros::ok()) {
//pub_empty.publish(emp_msg); //launches the drone

joy_x=map(joy_x_,-100,100,-1,1);
joy_y=map(joy_y_,-100,100,-1,1);
joy_z=map(joy_z_,-100,100,-1,1);

//Make a zone in the joystick where no controls are used
if (fabs(joy_x)<0.1) {joy_x =0;}
else {joy_x=joy_x*forget+joy_x_old*(1-forget);}

if (fabs(joy_y)<0.1) {joy_y =0;}
else {joy_y=joy_y*forget+joy_y_old*(1-forget);}

if (fabs(joy_z)<0.1) {joy_z =0;}
else {joy_z=joy_z*forget+joy_z_old*(1-forget);}


//ROS_INFO("time diff: %f",msg_time-old_msg_time);

      twist_msg.linear.x=joy_x;
	  twist_msg.linear.y=joy_y;	
	  twist_msg.linear.z=0.0;//THRUST AND YAW ARE DISABLED
	  twist_msg.angular.z=0.0;	


		joy_x_old=joy_x;
		joy_y_old=joy_y;
		joy_z_old=joy_z;
		new_msg=0;
		pub_twist.publish(twist_msg);
         
         ros::spinOnce();
		loop_rate.sleep();

			}//ros::ok
ROS_ERROR("ROS::ok failed- Node Closing");

}//main
