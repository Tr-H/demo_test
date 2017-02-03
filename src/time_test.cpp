

#include "ros/ros.h"
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
geometry_msgs::PoseStamped pose_status;
//geometry_msgs::PoseStamped pose_status2;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "time_test");
		ros::NodeHandle n;
	ros::Rate loop_rate(0.5);
	while(ros::ok())
	{
		//geometry_msgs::PoseStamped pose_status;

	pose_status.header.stamp  = ros::Time::now();   
	// pose_status.header.stamp.sec = ros::Time::now().toSec();
		ROS_INFO("	%f		%f",(float)pose_status.header.stamp.sec,(float)pose_status.header.stamp.nsec);	
		   loop_rate.sleep();
	}
}
