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
#define VEL_SP 0.25
#define P_POS 1.4
demo_test::pos_status raw_status_data;
demo_test::pos_write_data pos;
boost::mutex pos_status_mutex;
float MINDIS = VEL_SP/P_POS;
int i_number = 0;
int i_number_read_main = 0;
void pos_status_Callback(const demo_test::pos_status& pos_status1)
	{
		
		boost::mutex::scoped_lock lock(pos_status_mutex);
		memcpy(&raw_status_data, &pos_status1, sizeof(pos_status1));

			pos.type=raw_status_data.type;
			pos.send_currentpos_freq=raw_status_data.send_currentpos_freq;
			pos.send_desirepos_freq=raw_status_data.send_desirepos_freq;
			pos.send_type_freq=raw_status_data.send_type_freq;
			pos.flag[0]=raw_status_data.flag[0];
			pos.flag[1]=raw_status_data.flag[1];
			pos.flag[2]=raw_status_data.flag[2];
			pos.flag[3]=raw_status_data.flag[3];
			pos.flag[4]=raw_status_data.flag[4];
		
	}

int main(int argc, char **argv)
	{
		float d_pos[3];

		ros::init(argc, argv, "demo_read_data");
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe("pos_status_topic1", 2000, pos_status_Callback);
		ros::Publisher chatter_pub = n.advertise<demo_test::pos_write_data>("pos_write_topic1", 4000);
			ROS_INFO("***************welcome*******************");
			ROS_INFO("**** MINDIS:%.3f *** VEL_SP:%.3f ******",MINDIS,VEL_SP);
		    ROS_INFO("*****************************************");

	if(argc<=1)
	    {
	        ROS_INFO("usage error!\nusage: ./read_file  data.bin\n");
	        return -1;
	    }
    //int fd;
    //open file first time
    FILE *fd = fopen(argv[1],"r");
    if(!fd)
	    {
	        ROS_INFO("open file error\n");
	        return -1;
	    }
	float xyz[4];
    int count = 0;
    
    while(4 ==fscanf(fd,"%f %f %f %f",&xyz[0],&xyz[1],&xyz[2],&xyz[3]))
	    {
	          i_number++;
	            ROS_INFO("data: i=%d, x=%.4f, y=%.4f, z=%.4f, yaw=%.4f\n", i_number, xyz[0], xyz[1], xyz[2], xyz[3]);
	       
	    }
    if(fclose(fd)==-1)
	    {
	        ROS_INFO("close file error\n");
	    }
	    ROS_INFO("*****************************************");
	     sleep(1);
		ROS_INFO("reading  MAP_DATA  ........ ");
	/*----------------------------------------------------------------------------------------------------*/
	    sleep(3); //wait 
	    ROS_INFO("reading  MAP_DATA  .done ");
	     sleep(1);
	/*----------------------------------------------------------------------------------------------------*/
	    ROS_INFO("*****************************");
	    float *_data_map = (float *)malloc(sizeof(float) * i_number * 4);
	    if (_data_map == 0) 
	    {	
	        ROS_INFO("malloc wrong \n");
	        return -1;
	    }
	    memset(_data_map, 0x00, sizeof(float) * i_number * 4);

	    
	    fd = fopen(argv[1],"r");
	    if(!fd)
		    {
		        ROS_INFO("open file error\n");
		        return -1;
		    }
		 count = 0;
		 int i_number_read = 0;
	    while(4 ==fscanf(fd,"%f %f %f %f",&xyz[0],&xyz[1],&xyz[2],&xyz[3]))
		    {
		          
		          *(_data_map+(i_number_read*4))=xyz[0];
		          *(_data_map+(i_number_read*4)+1)=xyz[1];
		          *(_data_map+(i_number_read*4)+2)=xyz[2];
		          *(_data_map+(i_number_read*4)+3)=xyz[3];
		       	i_number_read++;
		    }
	    if(fclose(fd)==-1)
		    {
		        ROS_INFO("close file error\n");
		    }


	    int j;
	    
	    for(j=0;j<i_number * 4;j++)
	    {
	    	ROS_INFO("%.4f ",*(_data_map+j));
	    }
	     sleep(1);
	     ROS_INFO("*****************************");
	    ROS_INFO("loading  MAP_DATA  ........ ");
	/*----------------------------------------------------------------------------------------------------*/
	    sleep(3); //wait 
	    ROS_INFO("loading  MAP_DATA  .done ");
	    ROS_INFO("mission of MAP_DATA is runing.....");
	    sleep(1);
	/*----------------------------------------------------------------------------------------------------*/
	ros::AsyncSpinner s(2);
    s.start();
	ros::Rate loop_rate(150);

	ROS_INFO("i_number:........%d..........",i_number);
	ROS_INFO("*****************************");
	ROS_INFO("phase : ........ %d .........",i_number_read_main);
	ROS_INFO("*****************************");
	float delta_xyz[3];
	float map_delta_xyz[3];
	float delta_r=0;
	float map_delta_r=0;
	float old_xyz[3];
	float new_old_xyz[3];
	float new_old_r=0;
	float map_old_xyz[3];
	float map_old_r=0;
	float old_yaw = raw_status_data.att_yaw;
	float global_d_x =raw_status_data.dx;
	float global_d_y =raw_status_data.dy;
	float global_d_z =raw_status_data.dz;
	float global_d_yaw =raw_status_data.d_yaw;
	bool start_run = false;
	while(ros::ok() && i_number >1)
		{	
			boost::mutex::scoped_lock lock(pos_status_mutex);
			
		delta_xyz[0] = raw_status_data.dx  - raw_status_data.pos_x;
		delta_xyz[1] = raw_status_data.dy  - raw_status_data.pos_y;
		delta_xyz[2] = raw_status_data.dz  - raw_status_data.pos_z;
		delta_r = sqrt(delta_xyz[0]*delta_xyz[0] + delta_xyz[1]*delta_xyz[1] + delta_xyz[2]*delta_xyz[2]);
		if (i_number_read_main == 0 && delta_r <= 0.06 && start_run == false)
		{
			global_d_x = (float)(*(_data_map));
			global_d_y = (float)(*(_data_map+1));
			global_d_z = (float)(*(_data_map+2));
			global_d_yaw = (float)(*(_data_map+3));
			pos.pos_d[0] = global_d_x;
			pos.pos_d[1] = global_d_y;
			pos.pos_d[2] = global_d_z;
			pos.pos_d[3] = global_d_yaw;
			chatter_pub.publish(pos);
			old_xyz[0]= raw_status_data.pos_x;
			old_xyz[1]= raw_status_data.pos_y;
			old_xyz[2]= raw_status_data.pos_z;
			old_yaw= raw_status_data.att_yaw;
			map_old_xyz[0] = (float)(*(_data_map)) - old_xyz[0];
			map_old_xyz[1] = (float)(*(_data_map+1)) - old_xyz[1];
			map_old_xyz[2] = (float)(*(_data_map+2)) - old_xyz[2];
			map_old_r = sqrt(map_old_xyz[0]*map_old_xyz[0] + map_old_xyz[1]*map_old_xyz[1] + map_old_xyz[2]*map_old_xyz[2]);
			//i_number_read_main ++;
			start_run = true;
		}


			map_delta_xyz[0] = (float)(*(_data_map+(i_number_read_main*4)))  - raw_status_data.pos_x;
			map_delta_xyz[1] = (float)(*(_data_map+(i_number_read_main*4)+1))  - raw_status_data.pos_y;
			map_delta_xyz[2] = (float)(*(_data_map+(i_number_read_main*4)+2))  - raw_status_data.pos_z;
			map_delta_r = sqrt(map_delta_xyz[0]*map_delta_xyz[0] + map_delta_xyz[1]*map_delta_xyz[1] + map_delta_xyz[2]*map_delta_xyz[2]);
			//global_d_yaw = (float)(*(_data_map+(i_number_read_main*4)+3));
			new_old_xyz[0] = raw_status_data.pos_x - old_xyz[0];
			new_old_xyz[1] = raw_status_data.pos_y - old_xyz[1];
			new_old_xyz[2] = raw_status_data.pos_z - old_xyz[2];
			new_old_r = sqrt(new_old_xyz[0]*new_old_xyz[0] + new_old_xyz[1]*new_old_xyz[1] + new_old_xyz[2]*new_old_xyz[2]);

			if (start_run && i_number_read_main >= 0 && (map_delta_r <= 0.05 || (map_old_r!=0 && new_old_r >= map_old_r && (map_delta_r < new_old_r && map_old_r > map_delta_r)) ))
			{
					ROS_INFO("*****************************");
					ROS_INFO("phase : ........ %d .........",i_number_read_main+1);
					ROS_INFO("*****************************");
				i_number_read_main ++;
				old_xyz[0]= raw_status_data.pos_x;
				old_xyz[1]= raw_status_data.pos_y;
				old_xyz[2]= raw_status_data.pos_z;
				map_old_xyz[0] = (float)(*(_data_map+(i_number_read_main*4))) - old_xyz[0];
				map_old_xyz[1] = (float)(*(_data_map+(i_number_read_main*4)+1)) - old_xyz[1];
				map_old_xyz[2] = (float)(*(_data_map+(i_number_read_main*4)+2)) - old_xyz[2];
				map_old_r = sqrt(map_old_xyz[0]*map_old_xyz[0] + map_old_xyz[1]*map_old_xyz[1] + map_old_xyz[2]*map_old_xyz[2]);
				old_yaw= raw_status_data.att_yaw;
			}

			if (i_number_read_main >=0 && i_number_read_main < i_number-1 && map_delta_r > 0.05 && start_run)
			{
				global_d_x = map_delta_xyz[0]/map_delta_r*MINDIS + raw_status_data.pos_x;
				global_d_y = map_delta_xyz[1]/map_delta_r*MINDIS + raw_status_data.pos_y;
				global_d_z = map_delta_xyz[2]/map_delta_r*MINDIS + raw_status_data.pos_z;
				if(map_old_r != 0 && i_number_read_main ==0 )
				global_d_yaw = old_yaw + ((float)(*(_data_map+(i_number_read_main*4)+3)) - old_yaw) * new_old_r / map_old_r;
				if(map_old_r != 0 && i_number_read_main >0 )
				global_d_yaw = (float)(*(_data_map+(i_number_read_main*4)-1)) + ((float)(*(_data_map+(i_number_read_main*4)+3)) - (float)(*(_data_map+(i_number_read_main*4)-1)) ) * new_old_r / map_old_r;
				
				pos.pos_d[0] = global_d_x;
				pos.pos_d[1] = global_d_y;
				pos.pos_d[2] = global_d_z;

					while(global_d_yaw >= 3.15 || global_d_yaw <= -3.15 )
						{
							if(global_d_yaw > 0)
								{
									global_d_yaw = global_d_yaw - 2*3.14;
									//break;
								}
							else
								{
									global_d_yaw = global_d_yaw + 2*3.14;
									
								}
						}
				pos.pos_d[3] = global_d_yaw;
				chatter_pub.publish(pos);
			}

			if(i_number_read_main == i_number-1)
			{
			global_d_x = (float)(*(_data_map+(i_number_read_main*4)));
			global_d_y = (float)(*(_data_map+(i_number_read_main*4)+1));
			global_d_z = (float)(*(_data_map+(i_number_read_main*4)+2));
			global_d_yaw = (float)(*(_data_map+(i_number_read_main*4)+3));
			pos.pos_d[0] = global_d_x;
			pos.pos_d[1] = global_d_y;
			pos.pos_d[2] = global_d_z;	
			pos.pos_d[3] = global_d_yaw;
			chatter_pub.publish(pos);
			i_number_read_main ++;
			}

			if(i_number_read_main == i_number)
			{
				break;
			}
			lock.unlock();
		
			loop_rate.sleep();
		}
		ROS_INFO("*****************************");
		ROS_INFO("phase : ........END..........");
		ROS_INFO("*****************************");
	    return 0;

	}