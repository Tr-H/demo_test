#include <stdlib.h>
#include "ros/ros.h"
#include "demo_test/formation_shape.h"
#include "demo_test/formation_data.h"
#include "demo_test/formation_shape_pub.h"
#include "demo_test/pos_write_team.h"
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
float ref_data[7];
int ref_data_int[6];
char *status_topic_name;
char *write_topic_name;
int subflag=0;
float ssss;
demo_test::formation_data pos;
demo_test::formation_shape_pub raw_status_data;
void pos_status_Callback(const demo_test::formation_shape_pub& pos_status1)
	{
			boost::mutex::scoped_lock lock(pos_status_mutex);
			memcpy(&raw_status_data, &pos_status1, sizeof(pos_status1));
			lock.unlock();
			subflag=1;
	}
int main(int argc, char **argv)
	{	
	    //int global_id; 
		std::string ref_status;
		std::string ref_status1;
		char* ref_status2;
		ros::init(argc, argv, "demo_team_pos_write");
		ros::NodeHandle n1;
	//----------------------------------------------------------------
		//global_id = atoi(argv[1]);
	// char * raw_uav_status_name;
	// char * raw_uav_rewrite_name;
	// raw_uav_status_name = new char[sizeof("/pos_status_topic")];
	// memset(raw_uav_status_name, 0, sizeof("/pos_status_topic"));
	// strcpy(raw_uav_status_name,"/pos_status_topic");	
	// raw_uav_rewrite_name = new char[sizeof("/pos_write_topic")];
	// memset(raw_uav_rewrite_name, 0, sizeof("/pos_write_topic"));
	// strcpy(raw_uav_rewrite_name,"/pos_write_topic");
	// char * status_topic_name = strcat(raw_uav_status_name,argv[1]);
	// char * write_topic_name = strcat(raw_uav_rewrite_name,argv[1]);
		
	//----------------------------------------------------------------

		int i_argc;
	    for (i_argc=0;i_argc<argc;i_argc++)
		    {
		    	    ROS_INFO("argv:%s",argv[i_argc]);
		    }
	    ROS_INFO("argc:%d",argc);
		ros::Publisher chatter_pub = n1.advertise<demo_test::formation_data>("formation_pos_rewrite", 40);
		ros::Rate loop_rate(60);
		ros::Subscriber sub = n1.subscribe("forma_status_pub", 20, pos_status_Callback);
		ROS_INFO("Sending START");

		if (!strcmp(argv[1],"dpos"))
			{
				if(argc!=6)
				    {
						ROS_ERROR("missing arguments\n");
						return -1;
				    }
			    ref_data[0] = atof(argv[2]);
			    ref_data[1] = atof(argv[3]);
			    ref_data[2] = atof(argv[4]);
			    ref_data[3] = atof(argv[5]);
				
		    }
		else
///////////////////////////////////////////////////////////////////////////////////////////////
// 			if (!strcmp(argv[1],"type"))
// 				{
// 					if(argc!=4)
// 					    {
// 							ROS_ERROR("missing arguments\n");
// 							return -1;
// 					    }
// 					ref_status = argv[3];
// 					if(ref_status.compare("idel")==0)
// 						ref_data_int[0]=1;
// 					else
// 						if(ref_status.compare("land")==0)
// 							ref_data_int[0]=2;
// 						else
// 							if(ref_status.compare("formation")==0)
// 								ref_data_int[0]=3;
// 							else
// 								if(ref_status.compare("takeoff")==0)
// 									ref_data_int[0]=4;
// 								else
// 									ref_data_int[0]=0;
						
// 				}
// 			else
// ///////////////////////////////////////////////////////////////////////////////////////////////
// 				if (!strcmp(argv[1],"freq"))
// 					{
// 						if(argc!=6)
// 						    {
// 								ROS_ERROR("missing arguments\n");
// 								return -1;
// 						    }
// 						ref_data[4]=atof(argv[3]);
// 						if (ref_data[4]<20)
// 							{
// 								ROS_ERROR("pos freq cant <20");
// 								ref_data[4]=20;
// 							}
// 						ref_data[5]=atof(argv[4]);
// 						ref_data[6]=atof(argv[5]);
					
// 					}
// 				else
// ///////////////////////////////////////////////////////////////////////////////////////////////
// 					if (!strcmp(argv[1],"flag"))
// 					    {
// 							if(argc!=8)
// 							    {
// 									ROS_ERROR("missing arguments\n");
// 									return -1;
// 							    }
// 							ref_data_int[1]=atoi(argv[3]);
// 							ref_data_int[2]=atoi(argv[4]);
// 							ref_data_int[3]=atoi(argv[5]);
// 							ref_data_int[4]=atoi(argv[6]);
// 							ref_data_int[5]=atoi(argv[7]);						
// 					    }
					// else
///////////////////////////////////////////////////////////////////////////////////////////////
						{
							ROS_ERROR("wrong command!\n");
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
							if (!strcmp(argv[1],"dpos"))
								{
									pos.dpos[0]=ref_data[0];
									pos.dpos[1]=ref_data[1];
									pos.dpos[2]=ref_data[2];
									if(((fabs(ref_data[3]-raw_status_data.team_pos_yaw)<(2*3.1415f - fabs(ref_data[3]-raw_status_data.team_pos_yaw)))?fabs(ref_data[3]-raw_status_data.team_pos_yaw):(2*3.1415f - fabs(ref_data[3]-raw_status_data.team_pos_yaw))) < 0.785f)
										{
											pos.yaw=ref_data[3];
										}
									else
										{
											ROS_ERROR("yaw change is too fast!\n");
											return -1;
										}
								}
							// if (!strcmp(argv[1],"type"))
							// 	{
							// 		pos.type=ref_data_int[0];
							// 	}
							// if (!strcmp(argv[1],"freq"))
							// 	{	
							// 		pos.send_currentpos_freq=ref_data[4];
							// 		pos.send_desirepos_freq=ref_data[5];
							// 		pos.send_type_freq=ref_data[6];
							// 	}
							// if (!strcmp(argv[1],"flag"))
							// 	{	
							// 		pos.flag[0]=ref_data_int[1];
							// 		pos.flag[1]=ref_data_int[2];
							// 		pos.flag[2]=ref_data_int[3];
							// 		pos.flag[3]=ref_data_int[4];
							// 		pos.flag[4]=ref_data_int[5];
							// 	}
							// 	pos.header.stamp = ros::Time::now();
							chatter_pub.publish(pos);
							timer1 ++;
						}
		    	//step 2 : count the times of publish
				    if (timer1 == 5)
						{
							ROS_INFO("Send 5 times, waiting for checkout..");
				    		timer1=0;
				    	}
				//step 3 : check the status of gcs, check fail back to [ step 1 ]  ,check success go to [ step 4 ]
					if(raw_status_data.team_pos_x==pos.dpos[0] && 
						raw_status_data.team_pos_y==pos.dpos[1] && 
							raw_status_data.team_pos_z==pos.dpos[2] && 
								raw_status_data.team_pos_yaw==pos.yaw && 
									// raw_status_data.type==pos.type && 
									// 	raw_status_data.send_currentpos_freq==pos.send_currentpos_freq &&
									// 		raw_status_data.send_desirepos_freq == pos.send_desirepos_freq &&
									// 			raw_status_data.send_type_freq == pos.send_type_freq &&
									// 				raw_status_data.flag[0]==pos.flag[0] &&
									// 				raw_status_data.flag[1]==pos.flag[1] &&
									// 				raw_status_data.flag[2]==pos.flag[2] &&
									// 				raw_status_data.flag[3]==pos.flag[3] &&
									// 				raw_status_data.flag[4]==pos.flag[4] &&
														subflag==1)
						{
							// switch((int)pos.type)
							// 	{
							// 		case 0:
							// 			ref_status1="normal";
							// 			break;
							// 		case 1:
							// 			ref_status1="idel";
							// 			break;
							// 		case 2:
							// 			ref_status1="land";
							// 			break;
							// 		case 3:
							// 			ref_status1="formation";
							// 			break;
							// 		case 4:
							// 			ref_status1="takeoff";
							// 			break;
							// 		default:
							// 			break;
							// 	}	
							// ref_status2 = (char*)ref_status1.data();	
				//step 4 : ros info output 
							ROS_INFO("send successful....");	  			
							ROS_INFO("now");
							// ROS_INFO("fly type is: %s",ref_status2);	  			
							ROS_INFO("desire position set point is %.2f %.2f %.2f",pos.dpos[0],pos.dpos[1],pos.dpos[2]);	  			
							ROS_INFO("desire yaw is %.2f",pos.yaw);	  			
							// ROS_INFO("send_currentpos_freq is %.1fHZ   send_desirepos_freq is %.1f HZ   send_type_freq is %.1f HZ",pos.send_currentpos_freq,pos.send_desirepos_freq,pos.send_type_freq);	  			
							// ROS_INFO("flag is %d %d %d %d %d",pos.flag[0],pos.flag[1],pos.flag[2],pos.flag[3],pos.flag[4]);
							// ROS_INFO("rewrite programm end");  			
							lock.unlock();
							loop_rate.sleep();
							//sleep(60);
							break;
				
						}
				lock.unlock();
				loop_rate.sleep();

			}
		return 0;
	}
