#include <stdlib.h>
#include "ros/ros.h"
#include "demo_test/pos_data.h"
#include "demo_test/pos_write_data.h"
#include "demo_test/pos_status.h"
#include "demo_test/attitude_feedback.h"
#include "demo_test/velocity_feedback.h"
#include "demo_test/body_attitude.h"
#include "demo_test/position_setpoint.h"
#include <string.h>
#include <cstring>
#include <boost/thread.hpp>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <stdlib.h>
#include <time.h>
#include <geometry_msgs/PoseStamped.h>
#include <demo_test/demo_code.h>
#include "checksum.h"
using namespace std;
boost::mutex pos_mutex;
boost::mutex pos_sp_mutex;
boost::mutex send_mutex;
boost::mutex type_mutex;
boost::mutex freq_mutex;
demo_test::pos_data xyz;
demo_test::demo_code uav_code;
demo_test::pos_status pos_status1;
ros::Publisher mocap_send_pub;
ros::Publisher code_send_pub;
ros::Publisher status_upd_pub;
geometry_msgs::PoseStamped pose;
//geometry_msgs::PoseStamped last_pose;
float send_currentpos_freq = 20;
float send_desirepos_freq = 20;
float send_type_freq = 20;
int global_id = 1;
float ref_data[4];
float global_i = 1;
typedef struct _pos_type_temp
	{
		int type;
		bool flage_position;
		bool flage_velocity;
		bool flage_altitude;
		bool flage_attitude;
		bool flage_climb;
	}pos_sp_type_temp;
 pos_sp_type_temp global_type_temp;
int i_number =0;

void udpcallback(const demo_test::pos_data& pos)
{
   ROS_INFO_ONCE("|UAV%d| :: [ MOCAP ] DATE RECEIVED !!",global_id);
    boost::mutex::scoped_lock lock(pos_mutex);
    memcpy(&xyz, &pos, sizeof(pos));

      pose.header.stamp = ros::Time::now();
	 // pose.header.frame_id = "fcu";
	  pose.pose.position.x = (double)xyz.pos[0 + 3*(global_id-1)];
  	  pose.pose.position.y = (double)(xyz.pos[1 + 3*(global_id-1)]);
	  pose.pose.position.z = (double)(xyz.pos[2 + 3*(global_id-1)]);
	  pose.pose.orientation.x = (double)(xyz.q[0 + 4*(global_id-1)]);
	  pose.pose.orientation.y = (double)(xyz.q[1 + 4*(global_id-1)]);
	  pose.pose.orientation.z = (double)(xyz.q[2 + 4*(global_id-1)]);
	  pose.pose.orientation.w = (double)(xyz.q[3 + 4*(global_id-1)]);
    lock.unlock();
}

void writecallback(const demo_test::pos_write_data& pos1)
{   	
	ROS_INFO_THROTTLE(1,"|UAV%d| :: rewrite command !!",global_id);
	boost::mutex::scoped_lock possp_lock(pos_sp_mutex);
			 ref_data[0]=pos1.pos_d[0];
			 ref_data[1]=pos1.pos_d[1];
			 ref_data[2]=pos1.pos_d[2];
		     ref_data[3]=pos1.pos_d[3];
    	possp_lock.unlock();
    	boost::mutex::scoped_lock type_lock(type_mutex);
		    global_type_temp.type= pos1.type;
			global_type_temp.flage_position = (pos1.flag[0]==1)? true:false;
			global_type_temp.flage_velocity = (pos1.flag[1]==1)? true:false;
			global_type_temp.flage_altitude = (pos1.flag[2]==1)? true:false;
			global_type_temp.flage_attitude = (pos1.flag[3]==1)? true:false;
			global_type_temp.flage_climb = (pos1.flag[4]==1)? true:false;
			//write_has_chang=1;
		type_lock.unlock();
}

void code_send()
{		
	ROS_INFO_ONCE("|UAV%d| :: [ COMMAND ]  SENDING!!",global_id);
		global_type_temp.type=1;
		 global_type_temp.flage_position=true;
		 global_type_temp.flage_velocity=true;
		 global_type_temp.flage_altitude=true;
		 global_type_temp.flage_attitude=true;
		 global_type_temp.flage_climb=true;
		uint8_t test_sum = 0;
		
				ros::Rate loop_rate(12);
					while(ros::ok())	
						{
							boost::mutex::scoped_lock possp_lock(pos_sp_mutex);
								uav_code.x_d = ref_data[0];
								uav_code.y_d = ref_data[1];
								uav_code.z_d = ref_data[2];
							    uav_code.yaw_d = ref_data[3];
							possp_lock.unlock();
							boost::mutex::scoped_lock type_lock(type_mutex);
								boost::mutex::scoped_lock lock(pos_mutex);
									// if (global_type_temp.type == 4)
									// {
									// 	uav_code.x_d = pose.pose.position.x;
									// 	uav_code.y_d = pose.pose.position.y;
									// }
									if (global_type_temp.type == 4 && uav_code.z_d + pose.pose.position.y  >= -0.10f)
									{	
										ROS_INFO("|UAV%d| :: has been desire position, trun into 'NORMAL' mode",global_id);
										global_type_temp.type = 0;
									}
									if (global_type_temp.type == 2 && pose.pose.position.y <= 0.11f)
									{
										ROS_INFO("|UAV%d| :: has landed, trun into 'IDEL' mode",global_id);
										global_type_temp.type = 1;
									}
								lock.unlock();
							    uav_code.type = (uint8_t)global_type_temp.type;
							    test_sum = 0;
							    if(global_type_temp.flage_position)
							    	test_sum = test_sum | 0x01;
							    if(global_type_temp.flage_velocity)
							    	test_sum = test_sum | 0x02;
							    if(global_type_temp.flage_altitude)
							    	test_sum = test_sum | 0x04;
							    if(global_type_temp.flage_attitude)
							    	test_sum = test_sum | 0x08;
							    if(global_type_temp.flage_climb)
							    	test_sum = test_sum | 0x10;
							    uav_code.flag = (uint8_t)test_sum;
							type_lock.unlock();
							code_send_pub.publish(uav_code);
						loop_rate.sleep();
						}
}
void mocap_send()	
{	
	ros::Rate loop_rate(55);
	while(ros::ok())
	{
		boost::mutex::scoped_lock lock(pos_mutex);
			if (ros::Time::now() - pose.header.stamp <= ros::Duration(0.5))
			{
		//  pose.pose.position.x=10;// - global_i / 22;
		// pose.pose.position.y=-2;// + global_i / 22;
		//  pose.pose.position.z=-3;//+ global_i / 12;

		// global_i ++; 
				ROS_INFO_ONCE("|UAV%d| :: [ MOCAP ] DATE SENDING !!",global_id);
				mocap_send_pub.publish(pose);
			}
			else
			{
				ROS_INFO_THROTTLE(4,"|UAV%d| :: mocap data timeout!! check ros_client",global_id);
			}
		lock.unlock();
	loop_rate.sleep();
	}
	
}


void status_publish_thrd()
{
	ros::Rate loop_pos_status(100);
	while(ros::ok())
		{	
			boost::mutex::scoped_lock possp_lock(pos_sp_mutex);
				pos_status1.dx = ref_data[0];
				pos_status1.dy = ref_data[1];
				pos_status1.dz = ref_data[2];
			    pos_status1.d_yaw = ref_data[3];
			possp_lock.unlock();

			boost::mutex::scoped_lock pos_lock(pos_mutex);
			    pos_status1.pos_x = xyz.pos[0 + 3*(global_id-1)];
			    pos_status1.pos_y = xyz.pos[2 + 3*(global_id-1)];
			    pos_status1.pos_z = -xyz.pos[1 + 3*(global_id-1)];
			    pos_status1.id = global_id;
		    pos_lock.unlock();
			
			boost::mutex::scoped_lock type_lock(type_mutex);
				pos_status1.type = global_type_temp.type;
				pos_status1.flag[0] = (int)((global_type_temp.flage_position)? 1:0);
				pos_status1.flag[1] = (int)((global_type_temp.flage_velocity)? 1:0);
				pos_status1.flag[2] = (int)((global_type_temp.flage_altitude)? 1:0);
				pos_status1.flag[3] = (int)((global_type_temp.flage_attitude)? 1:0);
				pos_status1.flag[4] = (int)((global_type_temp.flage_climb)? 1:0);
		    type_lock.unlock();

			boost::mutex::scoped_lock freq_lock(freq_mutex);
				pos_status1.send_currentpos_freq=send_currentpos_freq;
				pos_status1.send_desirepos_freq=send_desirepos_freq;
				pos_status1.send_type_freq=send_type_freq;
			freq_lock.unlock();
			status_upd_pub.publish(pos_status1);
			loop_pos_status.sleep();
		}
}
int main(int argc, char **argv)
{	char * node_name;
	
	
	// char * raw_mocapsend_name;
	// char * raw_codesend_name;
	// char * strings;
	node_name = new char[sizeof("send_node")];
	memset(node_name, 0, sizeof("send_node"));
	strcpy(node_name,"send_node");
	// strings = new char[sizeof("/")];
	// memset(strings, 0, sizeof("/"));
	// strcpy(strings,"/");
	char * raw_uav_status_name;
	raw_uav_status_name = new char[sizeof("/pos_status_topic")];
	memset(raw_uav_status_name, 0, sizeof("/pos_status_topic"));
	strcpy(raw_uav_status_name,"/pos_status_topic");
	char * raw_uav_rewrite_name;	
	raw_uav_rewrite_name = new char[sizeof("/pos_write_topic")];
	memset(raw_uav_rewrite_name, 0, sizeof("/pos_write_topic"));
	strcpy(raw_uav_rewrite_name,"/pos_write_topic");
	
	char * uav_status_name = strcat(raw_uav_status_name,argv[2]);
	char * uav_rewrite_name = strcat(raw_uav_rewrite_name,argv[2]);
	global_id = atoi(argv[2]);
	ros::init(argc,argv,node_name);
	ros::NodeHandle nh(argv[1]);
	ros::Subscriber sub = nh.subscribe("/demo_udp", 10, &udpcallback);
	ros::Subscriber sub2 = nh.subscribe(uav_rewrite_name,10, &writecallback);
	mocap_send_pub = nh.advertise<geometry_msgs::PoseStamped>("mocap/pose",10);
	code_send_pub = nh.advertise<demo_test::demo_code>("uav_poseset/code",10);
	status_upd_pub = nh.advertise<demo_test::pos_status>(uav_status_name,10);
	 
	boost::thread thrd(&mocap_send);
	boost::thread thrd2(&code_send);
	boost::thread thrd3(&status_publish_thrd);
	ros::AsyncSpinner s(3);
	s.start();
	 ros::Rate loop_rate(10);
	while(ros::ok())
	{
		ROS_INFO_ONCE("|UAV%d| :: [ mavros_send ] RUNING!!",global_id);
		loop_rate.sleep();
	}
	return 0;
}