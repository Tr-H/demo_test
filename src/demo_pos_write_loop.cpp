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
#include <iostream>
#include "checksum.h"
using namespace std;
boost::mutex pos_status_mutex;
float ref_data[7];
int ref_data_int[6];
char* ref_status=0;
char* ref_status1=0;
int subflag=0;
demo_test::pos_write_data pos;
demo_test::pos_status raw_status_data;
void pos_status_Callback(const demo_test::pos_status& pos_status1)
	{
			boost::mutex::scoped_lock lock(pos_status_mutex);
			memcpy(&raw_status_data, &pos_status1, sizeof(pos_status1));
			pos.pos_d[0]=raw_status_data.pos_d[0];
			pos.pos_d[1]=raw_status_data.pos_d[1];
			pos.pos_d[2]=raw_status_data.pos_d[2];
			pos.pos_d[3]=raw_status_data.pos_d[3];
			pos.type[0]=raw_status_data.type[0];
			pos.send_currentpos_freq=raw_status_data.send_currentpos_freq;
			pos.send_desirepos_freq=raw_status_data.send_desirepos_freq;
			pos.send_type_freq=raw_status_data.send_type_freq;
			pos.flag[0]=raw_status_data.flag[0];
			pos.flag[1]=raw_status_data.flag[1];
			pos.flag[2]=raw_status_data.flag[2];
			pos.flag[3]=raw_status_data.flag[3];
			pos.flag[4]=raw_status_data.flag[4];
			subflag=1;
			lock.unlock();
	}
void main(int argc, char **argv)
{	

	ros::init(argc, argv, "demo_pos_write");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<demo_test::pos_write_data>("pos_write_topic", 1000);
	ros::Subscriber sub = n.subscribe("pos_status_topic", 1000, pos_status_Callback);
	int subflag=0;
	char strmain;
	while(ros::ok())
	{
	
			 cin>>strmain;
				if (!strcmp(strmain,"dpos"))
						{   
							char strtemp,str_store[10];
							cin.get(str_store,4);
							cin>>strtemp;
							if(!strcmp(strtemp,13))
						    {
								ref_data[0] = atof(str_store[0]);
							    ref_data[1] = atof(str_store[1]);
							    ref_data[2] = atof(str_store[2]);
							    ref_data[3] = atof(str_store[3]);
						    }
						    else
						    {
							    printf("missing arguments\n");
								return -1;
							}
					    
						
				  }else
	///////////////////////////////////////////////////////////////////////////////////////////////

				  if (!strcmp(strmain,"type"))
						{
							char strtemp,str_store[10];
							cin.get(str_store,1);
							cin>>strtemp;
							if(!strcmp(strtemp,13))
						    {	
						    	ref_status = new char[sizeof(str_store[0])];
								memset(ref_status, 0, sizeof(str_store[0]));
								strcpy(ref_status,str_store[0]);
							 if(!strcmp(ref_status,"idel"))
								ref_data_int[0]=1;
								else
									if(!strcmp(ref_status,"land"))
									ref_data_int[0]=2;
									else
										if(!strcmp(ref_status,"takeoff"))
										ref_data_int[0]=3;
										else
										ref_data_int[0]=0;
						    }
						    else
						    {
							    printf("missing arguments\n");
								return -1;
							}
					    
							
					}else
	///////////////////////////////////////////////////////////////////////////////////////////////
				  if (!strcmp(strmain,"freq")){
						 
							char strtemp,str_store[10];
							cin.get(str_store,3);
							cin>>strtemp;
							if(!strcmp(strtemp,13))
						    {
								ref_data[4] = atof(str_store[0]);
							    ref_data[5] = atof(str_store[1]);
							    ref_data[6] = atof(str_store[2]);
						    }
						    else
						    {
							    printf("missing arguments\n");
								return -1;
							}

						
					
					}else
	///////////////////////////////////////////////////////////////////////////////////////////////
			
				  if (!strcmp(strmain,"flag"))
				  {
							char strtemp,str_store[10];
							cin.get(str_store,5);
							cin>>strtemp;
							if(!strcmp(strtemp,13))
						    {
							    ref_data_int[1]=atoi(str_store[0]);
								ref_data_int[2]=atoi(str_store[1]);
								ref_data_int[3]=atoi(str_store[2]);
								ref_data_int[4]=atoi(str_store[3]);
								ref_data_int[5]=atoi(str_store[4]);	
						    }
						    else
						    {
							    printf("missing arguments\n");
								return -1;
							}
						
				  }else
	///////////////////////////////////////////////////////////////////////////////////////////////
				  {
					printf("wrong command!\n");
					return -1;
				}
   		int timer1 =0;
	ROS_INFO("Sending START");
			ros::Rate loop_rate(20);
			while(ros::ok()){
					boost::mutex::scoped_lock lock(pos_status_mutex);
			if (subflag==1)
				{
					if (!strcmp(strmain,"dpos"))
						{
							pos.pos_d[0]=ref_data[0];
							pos.pos_d[1]=ref_data[1];
							pos.pos_d[2]=ref_data[2];
							pos.pos_d[3]=ref_data[3];
							}
					if (!strcmp(strmain,"type"))
						{
							pos.type[0]=ref_data_int[0];
							}
					if (!strcmp(strmain,"freq"))
						{	pos.send_currentpos_freq=ref_data[4];
							pos.send_desirepos_freq=ref_data[5];
							pos.send_type_freq=ref_data[6];
							}
					if (!strcmp(strmain,"flag"))
						{	pos.flag[0]=ref_data_int[1];
							pos.flag[1]=ref_data_int[2];
							pos.flag[2]=ref_data_int[3];
							pos.flag[3]=ref_data_int[4];
							pos.flag[4]=ref_data_int[5];
							}
					chatter_pub.publish(pos);
					timer1 ++;
			}
		    if (timer1 == 5)
			{ROS_INFO("Send 5 times, waiting for checkout..");
		    timer1=0;}
			if(raw_status_data.pos_d[0]==pos.pos_d[0] && 
				raw_status_data.pos_d[1]==pos.pos_d[1] && 
					raw_status_data.pos_d[2]==pos.pos_d[2] && 
						raw_status_data.pos_d[3]==pos.pos_d[3] && 
							raw_status_data.type[0]==pos.type[0] && 
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
									switch(pos.type[0])
									{
									case 0:
									ref_status1 = new char[sizeof("normal")];
									memset(ref_status1, 0, sizeof("normal"));
									strcpy(ref_status1,"normal");
									break;
									case 1:
									ref_status1 = new char[sizeof("idel")];
									memset(ref_status1, 0, sizeof("idel"));
									strcpy(ref_status1,"idel");
									break;
									case 2:
									ref_status1 = new char[sizeof("land")];
									memset(ref_status1, 0, sizeof("land"));
									strcpy(ref_status1,"land");
									break;
									case 3:
									ref_status1 = new char[sizeof("takeoff")];
									memset(ref_status1, 0, sizeof("takeoff"));
									strcpy(ref_status1,"takeoff");
									break;
									default:
									break;
									}		
			printf("\nsend successful....\nnow\n   desire position set point is %.2f %.2f %.2f \n	desire yaw is %.2f \n	fly type is: %s\n  send_currentpos_freq is %.1fHZ   send_desirepos_freq is %.1f HZ   send_type_freq is %.1f HZ\n  flag is %d %d %d %d %d\nrewrite programm end\n",pos.pos_d[0],pos.pos_d[1],pos.pos_d[2],pos.pos_d[3],ref_status1,pos.send_currentpos_freq,pos.send_desirepos_freq,pos.send_type_freq,pos.flag[0],pos.flag[1],pos.flag[2],pos.flag[3],pos.flag[4]);		
				  			

					break;
				
				}
			lock.unlock();
			ros::spinOnce();
			loop_rate.sleep();

			}
	}
	return 0;
	}
