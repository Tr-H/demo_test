#include <stdlib.h>
#include "ros/ros.h"
#include "demo_test/pos_data.h"
#include "demo_test/pos_write_data.h"
#include "demo_test/pos_status.h"
#include <demo_test/demo_code.h>
#include <string.h>
#include <boost/thread.hpp>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <stdlib.h>
#include <time.h>
#include <geometry_msgs/PoseStamped.h>
geometry_msgs::PoseStamped pose_mocap;
geometry_msgs::PoseStamped pose_status;
demo_test::pos_status pos_status_temp;
demo_test::demo_code pos_status_temp1;
demo_test::pos_data xyz;
FILE *fd2; 
FILE *fd1;
void statusdata_cb(const demo_test::demo_code& pos_status1)
{	
	memcpy(&pos_status_temp1, &pos_status1, sizeof(pos_status1));
	pose_status.header.stamp  = ros::Time::now();
	fprintf(fd2,"%f %f %f %f %f\n",(float)pose_status.header.stamp.sec,
								(float)pose_status.header.stamp.nsec,
								(float)pos_status_temp1.x_d,
								(float)pos_status_temp1.y_d,
								(float)pos_status_temp1.z_d);
}
void mocapdata_cb(const demo_test::pos_data& pos)
{	
	 memcpy(&xyz, &pos, sizeof(pos));
    pose_mocap.header.stamp  = ros::Time::now();
    float x= xyz.pos[0];
    float y= xyz.pos[2];
    float z= -xyz.pos[1];
	fprintf(fd1,"%f %f %f %f %f\n",(float)pose_mocap.header.stamp.sec,
								(float)pose_mocap.header.stamp.nsec,
								x,
								y,
								z);
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "data_record");
		ros::NodeHandle n;
		    //open file first time
	fd1	= fopen("catkin_ws/src/demo_test/src/optitrack_data.txt","w");
    fd2 = fopen("catkin_ws/src/demo_test/src/pose_data.txt","w");
    if(!fd1)
	    {
	        ROS_ERROR("open file 'optitrack_data' error\n");
	        return -1;
	    }
	
	
    if(!fd2)
	    {
	        ROS_ERROR("open file 'pose_data' error\n");
	        return -1;
	    } 

	ros::Subscriber sub_mocap = n.subscribe("demo_udp", 1000, &mocapdata_cb); 
	ros::Subscriber sub_status = n.subscribe("/mavros/uav_poseset/code", 1000, &statusdata_cb); 
	ros::AsyncSpinner s(3);
    s.start();
    ROS_INFO("record starting!");
	ros::spin();
	fclose(fd1);  	   
	fclose(fd2);  	   

}
