/* 输入参数:四旋翼编号（1,2）*/

#include <stdlib.h>
#include "ros/ros.h"
#include "demo_test/pos_status.h"
#include "demo_test/pos_write_data.h"
#include "string.h"
#include "stdio.h"
#include "sys/types.h"
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <boost/thread.hpp>
#include "math.h"

#define write_freq 10
#define pai 3.14

demo_test::pos_status raw_status_data;
demo_test::pos_write_data traj_data;

boost::mutex pos_status_mutex;
boost::mutex traj_mutex;

struct timespec time1={0, 0};
struct timespec time2={0, 0};

char* status_topic_name;
char* write_topic_name;

typedef struct
{
	float x;
	float y;
	float z;
}pos_data;
pos_data  pos;

float pos_init[4];
float ref_data[4];
float err[3];
unsigned int type = 1;
bool sub_flag = false;

void pos_status_Callback(const demo_test::pos_status& pos_status1)
{
	boost::mutex::scoped_lock lock(pos_status_mutex);
	memcpy(&raw_status_data, &pos_status1, sizeof(pos_status1));
	//if(raw_status_data.type==0)  //normal状态进行轨迹规划
	//{		
		pos.x = raw_status_data.pos_x;
		pos.y = raw_status_data.pos_y;
		pos.z = raw_status_data.pos_z;
	
		ref_data[0] = raw_status_data.dx;
		ref_data[1] = raw_status_data.dy;
		ref_data[2] = raw_status_data.dz;
		ref_data[3] = raw_status_data.d_yaw;

		type = raw_status_data.type;

		traj_data.send_currentpos_freq = raw_status_data.send_currentpos_freq;
		traj_data.send_desirepos_freq = raw_status_data.send_desirepos_freq;
		traj_data.send_type_freq = raw_status_data.send_type_freq;
        traj_data.flag[0]=raw_status_data.flag[0];
		traj_data.flag[1]=raw_status_data.flag[1];
		traj_data.flag[2]=raw_status_data.flag[2];
		traj_data.flag[3]=raw_status_data.flag[3];
		traj_data.flag[4]=raw_status_data.flag[4];
		
		lock.unlock();
		
		err[0] = pos.x - ref_data[0];
		err[1] = pos.y - ref_data[1]; 
		err[2] = pos.z - ref_data[2]; 

		if(fabs(err[0])<0.05 && fabs(err[1])<0.05 && fabs(err[2])<0.05 && !sub_flag && raw_status_data.type==0)   
		{
			sub_flag = true;
			pos_init[0] = pos.x;
			pos_init[1] = pos.y;
			pos_init[2] = pos.z;
			pos_init[3] = ref_data[3];
		} 
	//}
	
}

int main(int argc, char **argv)
{	

	ros::init(argc, argv, "traj_write");
	ros::NodeHandle node;
	memset(&pos, 0, sizeof(pos));
	memset(&pos_init, 0, sizeof(pos_init));
	memset(&ref_data, 0, sizeof(ref_data));
	memset(&traj_data, 0, sizeof(traj_data));
	memset(&err, 0, sizeof(err));
	int global_id = 0;

    ROS_INFO("Ready to write the desired trajectory!!!");
	if(argc != 2)
	{
        ROS_ERROR("missing arguments\n");
    //    return -1;
    }

	global_id = atoi(argv[1]);	

	if (global_id==1)
	{
		status_topic_name = new char[sizeof("pos_status_topic1")];
		memset(status_topic_name, 0, sizeof("pos_status_topic1"));
		strcpy(status_topic_name,"pos_status_topic1");

		write_topic_name = new char[sizeof("pos_write_topic1")];
		memset(write_topic_name, 0, sizeof("pos_write_topic1"));
		strcpy(write_topic_name,"pos_write_topic1");
   	}
	if (global_id==2)
   	{
		status_topic_name = new char[sizeof("pos_status_topic2")];
		memset(status_topic_name, 0, sizeof("pos_status_topic2"));
		strcpy(status_topic_name,"pos_status_topic2");

		write_topic_name = new char[sizeof("pos_write_topic2")];
		memset(write_topic_name, 0, sizeof("pos_write_topic2"));
		strcpy(write_topic_name,"pos_write_topic2");
    }

	
	ros::Subscriber pos_status_sub = node.subscribe(status_topic_name, 2000, pos_status_Callback);	
	ros::Publisher pos_sp_pub = node.advertise<demo_test::pos_write_data>(write_topic_name, 4000);

	ros::AsyncSpinner s(1);
	s.start();
	
	ros::Rate loop_rate(write_freq);
	
	static bool time_get = true;
	float t1 = 0;
	float t2 = 0;
	float t = 0;
	
	//sub_flag = true;
	
	while(ros::ok())
	{
		if (sub_flag == true)
		{	
			boost::mutex::scoped_lock lock(traj_mutex);	
	
			clock_gettime(CLOCK_MONOTONIC, &time1);     //时间函数
			if(time_get == true)
			{
				clock_gettime(CLOCK_MONOTONIC, &time2);
				t2 = (float)(time2.tv_sec + time2.tv_nsec*0.000000001);
				time_get = false;
			}	
	
			t1 =(float)(time1.tv_sec + time1.tv_nsec*0.000000001);

			t = t1 - t2;
			
			ROS_INFO("t1=%.4lf \n t2=%.4lf \n t=%.4lf \n",t1,t2,t);
		
			if(t<5)	
			{
				ref_data[0] = pos_init[0] + t*0.08;
				ref_data[1] = pos_init[1];
				ref_data[2] = pos_init[2];
				ref_data[3] = pos_init[3];
			}		
			else
			{
				if(t<10)
				{
					ref_data[0] = pos_init[0] + 5*0.08;
					ref_data[1] = pos_init[1] + (t-5)*0.08;
					ref_data[2] = pos_init[2];
					ref_data[3] = pos_init[3] + pai/2;	
				}
				else
				{
					if(t<=15)
					{
						ref_data[0] = pos_init[0] + 5*0.08-(t-10)*0.08;
						ref_data[1] = pos_init[1] + 5*0.08-(t-10)*0.08;
						ref_data[2] = pos_init[2];
						ref_data[3] = pos_init[3] +pai/2 + pai/4 ;
					}
					else
					{
						type = 2;
					}
				
				}
			}
			
			traj_data.pos_d[0] = ref_data[0];
			traj_data.pos_d[1] = ref_data[1];
			traj_data.pos_d[2] = ref_data[2];
			traj_data.pos_d[3] = ref_data[3];
			traj_data.type = type;
				
			ROS_INFO("pos_init x=%.4f, y=%.4f, z=%.4f yaw=%.4f\n",pos_init[0], pos_init[1], pos_init[2],pos_init[3]);
			ROS_INFO("position setpoint x_d=%.4f, y_d=%.4f, z_d=%.4f, yaw_d=%.4f \n",ref_data[0],ref_data[1],ref_data[2],ref_data[3]);	
			pos_sp_pub.publish(traj_data);
			
			lock.unlock();
			
		} 
		
		if(t > 15 && type == 2 )
		{
			break;
		}
		
		loop_rate.sleep();
	}

	return 0;

}
