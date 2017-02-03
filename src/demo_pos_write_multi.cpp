#include <stdlib.h>
#include "ros/ros.h"
#include "demo_test/pos_data.h"
#include "demo_test/pos_write_data.h"
#include "demo_test/pos_status.h"
#include <string.h>
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
boost::mutex pos_mutex;
demo_test::pos_data xyz;

char *status_topic_name;
char *write_topic_name;
int subflag=0;
//float ssss;
class myStatusRewrite
{
public:	
	demo_test::pos_write_data pos;
	demo_test::pos_status raw_status_data;
	bool flag_dpos;
	bool flag_type;
	bool flag_freq;
	bool flag_flag;
	int id;
	char ** type;
	int argc
	myStatusRewrite() :
		n_rewrite("~")
	{
		flag_dpos = false;
		flag_type = false;
		flag_freq = false;
		flag_flag = false;
		id = 0;
		argc = 0;
	};
	
	void initialize()
	{	char * raw_uav_status_name;
		raw_uav_status_name = new char[sizeof("/pos_status_topic")];
		memset(raw_uav_status_name, 0, sizeof("/pos_status_topic"));
		strcpy(raw_uav_status_name,"/pos_status_topic");
		char * status_topic_name = strcat(raw_uav_status_name,itoa(uav_id));
		ros::Subscriber sub = n_rewrite.subscribe(status_topic_name, 20, &myStatusRewrite::pos_status_Callback,this);
		ros::AsyncSpinner s(3);
		s.start();
	}
private:
	ros::NodeHandle n_rewrite;
void pos_status_Callback(const demo_test::pos_status& pos_status1)
	{
			boost::mutex::scoped_lock lock(pos_status_mutex);
			memcpy(&raw_status_data, &pos_status1, sizeof(pos_status1));
			pos.pos_d[0]=raw_status_data.dx;
			pos.pos_d[1]=raw_status_data.dy;
			pos.pos_d[2]=raw_status_data.dz;
			pos.pos_d[3]=raw_status_data.d_yaw;
			pos.type=raw_status_data.type;
			pos.send_currentpos_freq=raw_status_data.send_currentpos_freq;
			pos.send_desirepos_freq=raw_status_data.send_desirepos_freq;
			pos.send_type_freq=raw_status_data.send_type_freq;
			pos.flag[0]=raw_status_data.flag[0];
			pos.flag[1]=raw_status_data.flag[1];
			pos.flag[2]=raw_status_data.flag[2];
			pos.flag[3]=raw_status_data.flag[3];
			pos.flag[4]=raw_status_data.flag[4];
			//subflag=1;
			status_rewrite_thread(id,argc)
			lock.unlock();
	}

void udpcallback(const demo_test::pos_data& pos)
	{
		    boost::mutex::scoped_lock lock(pos_mutex);
		    memcpy(&xyz, &pos, sizeof(pos));
		    lock.unlock();
	}
void status_rewrite_thread(int uav_id,int argc)
	{	
		float ref_data[7];
		int ref_data_int[6];
		std::string ref_status;
		char * raw_uav_rewrite_name;	
		raw_uav_rewrite_name = new char[sizeof("/pos_write_topic")];
		memset(raw_uav_rewrite_name, 0, sizeof("/pos_write_topic"));
		strcpy(raw_uav_rewrite_name,"/pos_write_topic");
		char * write_topic_name = strcat(raw_uav_rewrite_name,itoa(uav_id));
		ros::Publisher chatter_pub = n_rewrite.advertise<demo_test::pos_write_data>(write_topic_name, 40);
		ros::Rate loop_rate(60);
		ros::Subscriber sub = n_rewrite.subscribe(status_topic_name, 20, pos_status_Callback);
		ROS_INFO("[UAV %d]Sending START\n",uav_id);
		if (!strcmp(type[2],"dpos"))
			{
				if(argc!=7)
				    {
						ROS_ERROR("[UAV %d]missing arguments\n",uav_id);
						return -1;
				    }
			    ref_data[0] = atof(type[3]);
			    ref_data[1] = atof(type[4]);
			    ref_data[2] = atof(type[5]);
			    ref_data[3] = atof(type[6]);
				
		    }
		else
///////////////////////////////////////////////////////////////////////////////////////////////
			if (!strcmp(type[2],"type"))
				{
					if(argc!=4)
					    {
							ROS_ERROR("[UAV %d]missing arguments\n",uav_id);
							return -1;
					    }
					ref_status = type[3];
					if(ref_status.compare("idel")==0)
						ref_data_int[0]=1;
					else
						if(ref_status.compare("land")==0)
							ref_data_int[0]=2;
						else
							if(ref_status.compare("formation")==0)
								ref_data_int[0]=3;
							else
								if(ref_status.compare("takeoff")==0)
									ref_data_int[0]=4;
								else
									ref_data_int[0]=0;
						
				}
			else
///////////////////////////////////////////////////////////////////////////////////////////////
				if (!strcmp(type[2],"freq"))
					{
						if(argc!=6)
						    {
								ROS_ERROR("[UAV %d]missing arguments\n",uav_id);
								return -1;
						    }
						ref_data[4]=atof(type[3]);
						if (ref_data[4]<20)
							{
								ROS_ERROR("[UAV %d]pos freq cant <20",uav_id);
								ref_data[4]=20;
							}
						ref_data[5]=atof(type[4]);
						ref_data[6]=atof(type[5]);
					
					}
				else
///////////////////////////////////////////////////////////////////////////////////////////////
					if (!strcmp(type[2],"flag"))
					    {
							if(argc!=8)
							    {
									ROS_ERROR("[UAV %d]missing arguments\n",uav_id);
									return -1;
							    }
							ref_data_int[1]=atoi(type[3]);
							ref_data_int[2]=atoi(type[4]);
							ref_data_int[3]=atoi(type[5]);
							ref_data_int[4]=atoi(type[6]);
							ref_data_int[5]=atoi(type[7]);						
					    }
					else
///////////////////////////////////////////////////////////////////////////////////////////////
						{
							ROS_ERROR("[UAV %d]wrong command!\n",uav_id);
							return -1;
						}
	int timer1 =0;
		while(ros::ok())
			{
				ros::spinOnce();
				boost::mutex::scoped_lock lock(pos_status_mutex);
				//step 1 : publish rewrite pos topic 	
					if (subflag==1)
						{
							if (!strcmp(argv[2],"dpos"))
								{
									pos.pos_d[0]=ref_data[0];
									pos.pos_d[1]=ref_data[1];
									pos.pos_d[2]=ref_data[2];
									pos.pos_d[3]=ref_data[3];
								}
							if (!strcmp(argv[2],"type"))
								{
									pos.type=ref_data_int[0];
								}
							if (!strcmp(argv[2],"freq"))
								{	
									pos.send_currentpos_freq=ref_data[4];
									pos.send_desirepos_freq=ref_data[5];
									pos.send_type_freq=ref_data[6];
								}
							if (!strcmp(argv[2],"flag"))
								{	
									pos.flag[0]=ref_data_int[1];
									pos.flag[1]=ref_data_int[2];
									pos.flag[2]=ref_data_int[3];
									pos.flag[3]=ref_data_int[4];
									pos.flag[4]=ref_data_int[5];
								}
							chatter_pub.publish(pos);
							timer1 ++;
						}
		    	//step 2 : count the times of publish
				    if (timer1 == 5)
						{
							ROS_INFO("[UAV %d]Send 5 times, waiting for checkout..",uav_id);
				    		timer1=0;
				    	}
				//step 3 : check the status of gcs, check fail back to [ step 1 ]  ,check success go to [ step 4 ]
					if(raw_status_data.dx==pos.pos_d[0] && 
						raw_status_data.dy==pos.pos_d[1] && 
							raw_status_data.dz==pos.pos_d[2] && 
								raw_status_data.d_yaw==pos.pos_d[3] && 
									raw_status_data.type==pos.type && 
										raw_status_data.send_currentpos_freq==pos.send_currentpos_freq &&
											raw_status_data.send_desirepos_freq == pos.send_desirepos_freq &&
												raw_status_data.send_type_freq == pos.send_type_freq &&
													raw_status_data.flag[0]==pos.flag[0] &&
													raw_status_data.flag[1]==pos.flag[1] &&
													raw_status_data.flag[2]==pos.flag[2] &&
													raw_status_data.flag[3]==pos.flag[3] &&
													raw_status_data.flag[4]==pos.flag[4] &&
														subflag==1)
						{
							switch((int)pos.type)
								{
									case 0:
										ref_status1="normal";
										break;
									case 1:
										ref_status1="idel";
										break;
									case 2:
										ref_status1="land";
										break;
									case 3:
										ref_status1="formation";
										break;
									case 4:
										ref_status1="takeoff";
										break;
									default:
										break;
								}	
							ref_status2 = (char*)ref_status1.data();	
				//step 4 : ros info output 
							ROS_INFO("[UAV %d]send successful....",uav_id);	  			
							ROS_INFO("[UAV %d]now",uav_id);
							ROS_INFO("[UAV %d]fly type is: %s",uav_id,ref_status2);	  			
							ROS_INFO("[UAV %d]desire position set point is %.2f %.2f %.2f",uav_id,pos.pos_d[0],pos.pos_d[1],pos.pos_d[2]);	  			
							ROS_INFO("[UAV %d]desire yaw is %.2f",uav_id,pos.pos_d[3]);	  			
							ROS_INFO("[UAV %d]send_currentpos_freq is %.1fHZ   send_desirepos_freq is %.1f HZ   send_type_freq is %.1f HZ",uav_id,pos.send_currentpos_freq,pos.send_desirepos_freq,pos.send_type_freq);	  			
							ROS_INFO("[UAV %d]flag is %d %d %d %d %d",uav_id,pos.flag[0],pos.flag[1],pos.flag[2],pos.flag[3],pos.flag[4]);
							ROS_INFO("[UAV %d]rewrite programm end",uav_id);  			
							lock.unlock();
							loop_rate.sleep();
							//sleep(60);
							break;
				
						}
				lock.unlock();
				loop_rate.sleep();
			}
			subflag=0;
	}

};

int main(int argc, char **argv)
	{
		int global_id;
		bool flag_all_rewrite = false;
		//----------------------------------------------------------------
		if (!strcmp(argv[1],"all"))
		{flag_all_rewrite = true;
		global_id = 0;}
		else
		{flag_all_rewrite = false;
		global_id = atoi(argv[1]);}
		ros::Subscriber sub = nh.subscribe("/demo_udp", 10, &udpcallback);
		ros::AsyncSpinner s(3);
		s.start();
		while(ros::ok())
		{

		}
		myStatusRewrite mSR = new myStatusRewrite();
		mSR.id = global_id;
		mSR.argc = argc;
		mSR.initialize();
		//----------------------------------------------------------------

		int i_argc;
		    for (i_argc=0;i_argc<argc;i_argc++)
			    {
			    	    ROS_INFO("argv:%s",argv[i_argc]);
			    }
	    	ROS_INFO("argc:%d",argc);

	}