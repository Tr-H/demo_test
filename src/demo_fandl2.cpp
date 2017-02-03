/*
formation_shape_pub.msg have not been used. 
*/
#include <stdlib.h>
#include "ros/ros.h"
#include "demo_test/pos_data.h"
#include "demo_test/pos_write_data2.h"
#include "demo_test/pos_status.h"
#include "demo_test/formation_shape.h"
#include <string.h>
#include <boost/thread.hpp>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <stdlib.h>
#include <time.h>
#include "math.h"
#include"checksum.h"
using namespace boost;
using namespace std;

demo_test::pos_status raw_status_data[4];
demo_test::pos_write_data2 pos[4];
demo_test::pos_write_data2 pos_will_write[4];
ros::Publisher chatter_pub[4];
int gloabl_follow1_id;
int global_leader_id; 
boost::mutex pos_status_mutex;
boost::mutex forma_shape_mutex;
bool robot_run_flag[4];
typedef struct _formation_shape
	{
		float delta_x;
		float delta_y;
		float delta_z;
		float delta_yaw;
	}formation_shape_temp;
 formation_shape_temp forma_shape[4];
 ros::Publisher formation_shape_pub;
void pos_status_Callback(const demo_test::pos_status& pos_status1)
	{
			boost::mutex::scoped_lock lock(pos_status_mutex);
			int id=0;
			id = pos_status1.id-1;

			robot_run_flag[id]=true;
			memcpy(&raw_status_data[id], &pos_status1, sizeof(pos_status1));
			pos[id].header.stamp = raw_status_data[id].header.stamp;
			pos[id].pos_d[0]=raw_status_data[id].dx;
			pos[id].pos_d[1]=raw_status_data[id].dy;
			pos[id].pos_d[2]=raw_status_data[id].dz;
			pos[id].pos_d[3]=raw_status_data[id].d_yaw;
			pos[id].type=raw_status_data[id].type;
			pos[id].send_currentpos_freq=raw_status_data[id].send_currentpos_freq;
			pos[id].send_desirepos_freq=raw_status_data[id].send_desirepos_freq;
			pos[id].send_type_freq=raw_status_data[id].send_type_freq;
			pos[id].flag[0]=raw_status_data[id].flag[0];
			pos[id].flag[1]=raw_status_data[id].flag[1];
			pos[id].flag[2]=raw_status_data[id].flag[2];
			pos[id].flag[3]=raw_status_data[id].flag[3];
			pos[id].flag[4]=raw_status_data[id].flag[4];
	}
void formation_shape_rev_function(const demo_test::formation_shape& formation_shape_rev_temp)
{
	int id = 0;
	boost::mutex::scoped_lock lock(forma_shape_mutex);
	for (id = 0;id <4;id ++)
	{
	forma_shape[id].delta_x = formation_shape_rev_temp.delta_x[id];
	forma_shape[id].delta_y = formation_shape_rev_temp.delta_y[id];
	forma_shape[id].delta_z = formation_shape_rev_temp.delta_z[id];
	forma_shape[id].delta_yaw = formation_shape_rev_temp.delta_yaw[id];
	}
}

void follower_thread(const int id)
{
ros::Rate loop_rate(120);
float x;
float y;
float z;
float yawy;
ROS_INFO("robot<%d> follower thread .......start",id+1);
while(ros::ok())
	{
		
		ROS_INFO_THROTTLE(2,"robot<%d> follower thread ......runing",id+1);
		boost::mutex::scoped_lock lock(pos_status_mutex);
		boost::mutex::scoped_lock lock_shape(forma_shape_mutex);
		 x = 2*(raw_status_data[global_leader_id-1].q4*raw_status_data[global_leader_id-1].q3 + raw_status_data[global_leader_id-1].q1*raw_status_data[global_leader_id-1].q2);
	     y = 1-2*(raw_status_data[global_leader_id-1].q2*raw_status_data[global_leader_id-1].q2+raw_status_data[global_leader_id-1].q3*raw_status_data[global_leader_id-1].q3);
		 yawy = -atan2(x,y);
			    // printf("%.2f\n",yawy);
		pos_will_write[id].header.stamp = pos[id].header.stamp;
		pos_will_write[id].pos_d[0] = raw_status_data[global_leader_id-1].pos_x + forma_shape[id].delta_x*cosf(yawy+1.57) - forma_shape[id].delta_y*sinf(yawy+1.57);
		pos_will_write[id].pos_d[1] = raw_status_data[global_leader_id-1].pos_y + forma_shape[id].delta_x*sinf(yawy+1.57) + forma_shape[id].delta_y*cosf(yawy+1.57);
		pos_will_write[id].pos_d[2] = raw_status_data[global_leader_id-1].pos_z + forma_shape[id].delta_z;
		pos_will_write[id].pos_d[3] = yawy;//pos[global_leader_id-1].pos_d[3] + forma_shape[id].delta_yaw;
		lock_shape.unlock();
		if(pos[id].type != 3)
			{
				ROS_INFO("============================================");
				ROS_INFO("robot<%d> has jump out the 'FORMATION' mode",id+1);
				ROS_INFO("============================================");
				break;
			}
		pos_will_write[id].type = pos[id].type;
		pos_will_write[id].send_currentpos_freq = pos[id].send_currentpos_freq;
		pos_will_write[id].send_desirepos_freq = pos[id].send_desirepos_freq;
		pos_will_write[id].send_type_freq = pos[id].send_type_freq;
		pos_will_write[id].flag[0]=pos[id].flag[0];
		pos_will_write[id].flag[1]=pos[id].flag[1];
		pos_will_write[id].flag[2]=pos[id].flag[2];
		pos_will_write[id].flag[3]=pos[id].flag[3];
		pos_will_write[id].flag[4]=pos[id].flag[4];
		ROS_INFO_THROTTLE(0.5,"robot<%d> dx : %.2f |dy : %.2f |dz :%.2f |dyaw :%.2f",id+1,pos_will_write[id].pos_d[0],pos_will_write[id].pos_d[1],pos_will_write[id].pos_d[2],pos_will_write[id].pos_d[3]);
		lock.unlock();
		chatter_pub[id].publish(pos_will_write[id]);
		loop_rate.sleep();

	}
		ROS_INFO("robot<%d> follower thread ......finish",id+1);
}



int main(int argc, char **argv)
	{

	
	int number_follower;
		std::string ref_status;
		std::string ref_status1;
		char* ref_status2;
		ros::init(argc, argv, "demo_fandl");
		ros::NodeHandle n;
	if(argc != 1)	
	{global_leader_id = atoi(argv[1]);}
	else
	{ROS_INFO("usage:[rosrun demo_test demo_fandl idofleader !]");
	return 0;}
	thread_group follower_threads;
	int i=0;
	for (i=0;i<4;i++)
		{robot_run_flag[i]=false;}
	for (i=0;i<4;i++)
		{
			forma_shape[i].delta_x = 0;
			forma_shape[i].delta_y = 0;
			forma_shape[i].delta_z = 0;
			forma_shape[i].delta_yaw = 0;
		}
		/***************************/
		/*
		the delta_xyz has been define as the position error 
		between follower and leader while the leader's yaw is -1.57
		*/
		forma_shape[1].delta_x = 0.8f;
		forma_shape[1].delta_y = 0.8f;
		//forma_shape[1].delta_x = 1.2f;
		forma_shape[0].delta_x = -0.8f;
		forma_shape[0].delta_y = 0.8f;
		/***************************/

		char* publish_name[4];

			publish_name[0] = new char[sizeof("pos_write_topic1")];
			memset(publish_name[0], 0, sizeof("pos_write_topic1"));
			strcpy(publish_name[0],"pos_write_topic1");
			publish_name[1] = new char[sizeof("pos_write_topic2")];
			memset(publish_name[1], 0, sizeof("pos_write_topic2"));
			strcpy(publish_name[1],"pos_write_topic2");
			publish_name[2] = new char[sizeof("pos_write_topic3")];
			memset(publish_name[2], 0, sizeof("pos_write_topic3"));
			strcpy(publish_name[2],"pos_write_topic3");
			publish_name[3] = new char[sizeof("pos_write_topic4")];
			memset(publish_name[3], 0, sizeof("pos_write_topic4"));
			strcpy(publish_name[3],"pos_write_topic4");
		

		ros::Subscriber sub1 = n.subscribe("pos_status_topic1", 20, pos_status_Callback);
		ros::Subscriber sub2 = n.subscribe("pos_status_topic2", 20, pos_status_Callback);
		ros::Subscriber sub3 = n.subscribe("pos_status_topic3", 20, pos_status_Callback);
		ros::Subscriber sub4 = n.subscribe("pos_status_topic4", 20, pos_status_Callback);
		ros::Subscriber sub5 = n.subscribe("formation_shape_rewrite", 20, formation_shape_rev_function);
		ros::AsyncSpinner s(12);
    	s.start();
	ROS_INFO("waiting for receive the locaton message..5s..");
		ros::Duration(1).sleep();
	ROS_INFO("waiting for receive the locaton message..4s..");
		ros::Duration(1).sleep();
	ROS_INFO("waiting for receive the locaton message..3s..");
		ros::Duration(1).sleep();
	ROS_INFO("waiting for receive the locaton message..2s..");
		ros::Duration(1).sleep();
	ROS_INFO("waiting for receive the locaton message..1s..");

	while( !(((!robot_run_flag[0]) || raw_status_data[0].type == 3 || global_leader_id == 1) 
			 && ((!robot_run_flag[1]) || raw_status_data[1].type == 3 || global_leader_id == 2) 
				 && ((!robot_run_flag[2]) || raw_status_data[2].type == 3 || global_leader_id == 3) 
					 && ((!robot_run_flag[3]) || raw_status_data[3].type == 3 || global_leader_id == 4)) && ros::ok())
	{
		ROS_INFO_ONCE("check robot mode...start..");
		for(i=0;i<4;i++)
		{
			if (robot_run_flag[i] && pos[i].type !=3 && i != global_leader_id-1 )
			{	//printf("%d",i);
				pos[i].type = 3;
				if( chatter_pub[i] != NULL)
				chatter_pub[i].publish(pos[i]);
				else
				chatter_pub[i] = n.advertise<demo_test::pos_write_data2>(publish_name[i], 40);
	
			}
		}
	}
	formation_shape_pub = n.advertise<demo_test::formation_shape>("forma_shape_pub",10);
		ROS_INFO("check robot mode...finish..");
				ros::Duration(1).sleep();
		ROS_INFO("robots status:");
		for (i=0;i<4;i++)
		{
			ROS_INFO_ONCE("==============================");
		 if (robot_run_flag[i])
		 {
		 	ROS_INFO("   robot    id   :          %d",i+1);
		 	if(global_leader_id == (i+1))
			ROS_INFO("formation status :      leader");
			else
			ROS_INFO("formation status :    follower");
		    ROS_INFO("------------------------------");

		 }
		}
		ROS_INFO("==============================");
				ros::Duration(1).sleep();

		int id_follow_1;
		float min_r_lead_follow = 10000;
		float r;
		for (i=0;i<4;i++)
		{
			if(i != (global_leader_id-1) )
			{
				r=sqrt(pow((raw_status_data[global_leader_id].pos_x-raw_status_data[i].pos_x),2)+pow((raw_status_data[global_leader_id].pos_y-raw_status_data[i].pos_y),2));
				if(r < min_r_lead_follow)
				{
					id_follow_1 = i+1;
				}
			}
		}
		gloabl_follow1_id = id_follow_1;
		
		for (i=0;i<4;i++)
		{
			if(robot_run_flag[i] && global_leader_id != (i+1))
				follower_threads.create_thread(bind(&follower_thread,i));
		}
		demo_test::formation_shape formation_shape_rev_temp1;
		ros::Rate loop_rate_main(80);
		while(ros::ok())
		{	
			boost::mutex::scoped_lock lock_shape(forma_shape_mutex);
			for (i = 0;i <4;i ++)
				{
				formation_shape_rev_temp1.delta_x[i] = forma_shape[i].delta_x;
				formation_shape_rev_temp1.delta_y[i] = forma_shape[i].delta_y;
				formation_shape_rev_temp1.delta_z[i] = forma_shape[i].delta_z;
				formation_shape_rev_temp1.delta_yaw[i] = forma_shape[i].delta_yaw;
				}
				lock_shape.unlock();
			formation_shape_pub.publish(formation_shape_rev_temp1);
			ROS_INFO_THROTTLE(3,"main thread runing.");
			loop_rate_main.sleep();
		}
	}