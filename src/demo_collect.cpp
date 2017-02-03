#include <stdlib.h>
#include "ros/ros.h"
#include "demo_test/pos_data.h"
#include "demo_test/pos_write_data.h"
#include "demo_test/pos_status.h"
#include "demo_test/quadrotor_collect.h"
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
#include"checksum.h"
using namespace std;
boost::mutex pos_status_mutex;
boost::mutex pos_sp_mutex;
boost::mutex send_mutex;
boost::mutex type_mutex;
boost::mutex freq_mutex;
demo_test::quadrotor_collect q_collect;
typedef struct _q_temp
	{
		bool vaul;
		float send_currentpos_freq;
		float send_desirepos_freq;
		float send_type_freq;

	}q_temp;
 q_temp q1_collect_temp;
 q_temp q2_collect_temp;
 q_temp q3_collect_temp;
 q_temp q4_collect_temp;
 q_temp q5_collect_temp;
 q_temp q6_collect_temp;

void pos_status_Callback(const demo_test::pos_status& pos_status1)
	{		//demo_test::pos_status raw_status_data;
			boost::mutex::scoped_lock lock(pos_status_mutex);
			//memcpy(&raw_status_data, &pos_status1, sizeof(pos_status1));
			//ROS_INFO("%d",raw_status_data.id);
				if((int)pos_status1.id == 1)
				{
					//ROS_INFO("aaaa");
					q1_collect_temp.vaul = true;
					q1_collect_temp.send_currentpos_freq = pos_status1.send_currentpos_freq;
					q1_collect_temp.send_desirepos_freq = pos_status1.send_desirepos_freq;
					q1_collect_temp.send_type_freq = pos_status1.send_type_freq;
				}
				if((int)pos_status1.id == 2)
				{
					q2_collect_temp.vaul = true;
					q2_collect_temp.send_currentpos_freq = pos_status1.send_currentpos_freq;
					q2_collect_temp.send_desirepos_freq = pos_status1.send_desirepos_freq;
					q2_collect_temp.send_type_freq = pos_status1.send_type_freq;
				}
				if((int)pos_status1.id == 3)
				{
					q3_collect_temp.vaul = true;
					q3_collect_temp.send_currentpos_freq = pos_status1.send_currentpos_freq;
					q3_collect_temp.send_desirepos_freq = pos_status1.send_desirepos_freq;
					q3_collect_temp.send_type_freq = pos_status1.send_type_freq;
				}
				if((int)pos_status1.id == 4)
				{
					q4_collect_temp.vaul = true;
					q4_collect_temp.send_currentpos_freq = pos_status1.send_currentpos_freq;
					q4_collect_temp.send_desirepos_freq = pos_status1.send_desirepos_freq;
					q4_collect_temp.send_type_freq = pos_status1.send_type_freq;
				}
				if((int)pos_status1.id == 5)
				{
					q5_collect_temp.vaul = true;
					q5_collect_temp.send_currentpos_freq = pos_status1.send_currentpos_freq;
					q5_collect_temp.send_desirepos_freq = pos_status1.send_desirepos_freq;
					q5_collect_temp.send_type_freq = pos_status1.send_type_freq;
				}
				if((int)pos_status1.id == 6)
				{
					q6_collect_temp.vaul = true;
					q6_collect_temp.send_currentpos_freq = pos_status1.send_currentpos_freq;
					q6_collect_temp.send_desirepos_freq = pos_status1.send_desirepos_freq;
					q6_collect_temp.send_type_freq = pos_status1.send_type_freq;
				}
			
			lock.unlock();
	}
int main(int argc, char **argv)
{	
	q1_collect_temp.vaul = false;
	q1_collect_temp.send_currentpos_freq = 0;
	q1_collect_temp.send_desirepos_freq = 0;
	q1_collect_temp.send_type_freq = 0;

	q2_collect_temp.vaul = false;
	q2_collect_temp.send_currentpos_freq = 0;
	q2_collect_temp.send_desirepos_freq = 0;
	q2_collect_temp.send_type_freq = 0;

	q3_collect_temp.vaul = false;
	q3_collect_temp.send_currentpos_freq = 0;
	q3_collect_temp.send_desirepos_freq = 0;
	q3_collect_temp.send_type_freq = 0;

	q4_collect_temp.vaul = false;
	q4_collect_temp.send_currentpos_freq = 0;
	q4_collect_temp.send_desirepos_freq = 0;
	q4_collect_temp.send_type_freq = 0;

	q5_collect_temp.vaul = false;
	q5_collect_temp.send_currentpos_freq = 0;
	q5_collect_temp.send_desirepos_freq = 0;
	q5_collect_temp.send_type_freq = 0;

	q6_collect_temp.vaul = false;
	q6_collect_temp.send_currentpos_freq = 0;
	q6_collect_temp.send_desirepos_freq = 0;
	q6_collect_temp.send_type_freq = 0;

	ros::init(argc, argv, "demo_collect");
    ros::NodeHandle n3;
	ros::Subscriber sub = n3.subscribe("pos_status_topic1", 2000, pos_status_Callback);
	ros::Subscriber sub2 = n3.subscribe("pos_status_topic2", 2000, pos_status_Callback);
	float num_quadrotor;
	float sum_currentpos_freq;
	float sum_desirepos_freq;
	float sum_type_freq;
	
	ros::Publisher chatter_pub = n3.advertise<demo_test::quadrotor_collect>("freq_send", 1000);

//ROS_INFO("AAAA");
   ros::Rate loop_rate(30);
	while(ros::ok())
	{	ros::spinOnce();	
		num_quadrotor=0;
			 sum_currentpos_freq=0;
			 sum_desirepos_freq=0;
			 sum_type_freq=0;
			boost::mutex::scoped_lock lock(pos_status_mutex);
				if(q1_collect_temp.vaul == true)
				{
					num_quadrotor=num_quadrotor+1;
					sum_currentpos_freq=sum_currentpos_freq+q1_collect_temp.send_currentpos_freq;
					sum_desirepos_freq=sum_desirepos_freq+q1_collect_temp.send_desirepos_freq;
					sum_type_freq=sum_type_freq+q1_collect_temp.send_type_freq;
				}
				if(q2_collect_temp.vaul == true)
				{
					num_quadrotor=num_quadrotor+1;
					sum_currentpos_freq=sum_currentpos_freq+q2_collect_temp.send_currentpos_freq;
					sum_desirepos_freq=sum_desirepos_freq+q2_collect_temp.send_desirepos_freq;
					sum_type_freq=sum_type_freq+q2_collect_temp.send_type_freq;
				}
				if(q3_collect_temp.vaul == true)
				{
					num_quadrotor=num_quadrotor+1;
					sum_currentpos_freq=sum_currentpos_freq+q3_collect_temp.send_currentpos_freq;
					sum_desirepos_freq=sum_desirepos_freq+q3_collect_temp.send_desirepos_freq;
					sum_type_freq=sum_type_freq+q3_collect_temp.send_type_freq;
				}
				if(q4_collect_temp.vaul == true)
				{
					num_quadrotor=num_quadrotor+1;
					sum_currentpos_freq=sum_currentpos_freq+q4_collect_temp.send_currentpos_freq;
					sum_desirepos_freq=sum_desirepos_freq+q4_collect_temp.send_desirepos_freq;
					sum_type_freq=sum_type_freq+q4_collect_temp.send_type_freq;
				}
				if(q5_collect_temp.vaul == true)
				{
					num_quadrotor=num_quadrotor+1;
					sum_currentpos_freq=sum_currentpos_freq+q5_collect_temp.send_currentpos_freq;
					sum_desirepos_freq=sum_desirepos_freq+q5_collect_temp.send_desirepos_freq;
					sum_type_freq=sum_type_freq+q5_collect_temp.send_type_freq;
				}
				if(q6_collect_temp.vaul == true)
				{
					num_quadrotor=num_quadrotor+1;
					sum_currentpos_freq=sum_currentpos_freq+q6_collect_temp.send_currentpos_freq;
					sum_desirepos_freq=sum_desirepos_freq+q6_collect_temp.send_desirepos_freq;
					sum_type_freq=sum_type_freq+q6_collect_temp.send_type_freq;
				}
				q_collect.num=(int)num_quadrotor;
				q_collect.sum_currentpos_freq=sum_currentpos_freq/num_quadrotor;
				q_collect.sum_desirepos_freq=sum_desirepos_freq/num_quadrotor;
				q_collect.sum_type_freq=sum_type_freq/num_quadrotor;
				chatter_pub.publish(q_collect);
			lock.unlock();
			loop_rate.sleep();

	}




}