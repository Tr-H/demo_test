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
#define P_POS 1.6
demo_test::pos_status raw_status_data;
demo_test::pos_write_data pos;
boost::mutex pos_status_mutex;
float VEL_SP_X = 0;
float VEL_SP_Y = 0;
float VEL_SP_Z = 0;
//float MINDIS = VEL_SP/P_POS;
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

		ros::init(argc, argv, "demo_read_data2");
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe("/pos_status_topic2", 2000, pos_status_Callback);
		ros::Publisher chatter_pub = n.advertise<demo_test::pos_write_data>("/pos_write_topic2", 4000);
			ROS_INFO("***************welcome*******************");
			ROS_INFO("**** MINDIS: *** VEL_SP: ******");
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
	float xyz[5];
    int count = 0;
    
    while(5 ==fscanf(fd,"%f %f %f %f %f",&xyz[0],&xyz[1],&xyz[2],&xyz[3],&xyz[4]))
	    {
	          i_number++;
	            ROS_INFO("data: i=%d, x=%.4f, y=%.4f, z=%.4f, yaw=%.4f, time=%.4f\n", i_number, xyz[0], xyz[1], xyz[2], xyz[3], xyz[4]);
	       
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
	    float *_data_map = (float *)malloc(sizeof(float) * i_number * 5);
	    if (_data_map == 0) 
	    {	
	        ROS_INFO("malloc wrong \n");
	        return -1;
	    }
	    memset(_data_map, 0x00, sizeof(float) * i_number * 5);

	    
	    fd = fopen(argv[1],"r");
	    if(!fd)
		    {
		        ROS_INFO("open file error\n");
		        return -1;
		    }
		 count = 0;
		 int i_number_read = 0;
	    while(5 ==fscanf(fd,"%f %f %f %f %f",&xyz[0],&xyz[1],&xyz[2],&xyz[3],&xyz[4]))
		    {
		          
		          *(_data_map+(i_number_read*5))=xyz[0];
		          *(_data_map+(i_number_read*5)+1)=xyz[1];
		          *(_data_map+(i_number_read*5)+2)=xyz[2];
		          *(_data_map+(i_number_read*5)+3)=xyz[3];
		          *(_data_map+(i_number_read*5)+4)=xyz[4];
		       	i_number_read++;
		    }
	    if(fclose(fd)==-1)
		    {
		        ROS_INFO("close file error\n");
		    }


	    int j;
	    
	    for(j=0;j<i_number;j++)
	    {
	    	ROS_INFO(" x=%.4f, y=%.4f, z=%.4f, yaw=%.4f, time=%.4f\n", *(_data_map+j*5),*(_data_map+j*5+1), *(_data_map+j*5+2),*(_data_map+j*5+3),*(_data_map+j*5+4));
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
	float map_old_xyz[3];
	float map_old_r=0;
	float new_old_xyz[3];
	float new_old_r=0;
	float old_yaw = raw_status_data.att_yaw;
	float global_d_x =raw_status_data.dx;
	float global_d_y =raw_status_data.dy;
	float global_d_z =raw_status_data.dz;
	float global_d_yaw =raw_status_data.d_yaw;
	bool start_run = false;
	struct timespec time1={0, 0};
	struct timespec time2={0, 0};
	float delta_t = 0;
	float t1 = 0;
	float t2 = 0;
	float ip = 0;
	float cosa = 1;
	float trac_e_x = 0;
	float trac_e_y = 0;
	float trac_e_z = 0;
	float cos_xyz[3];
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
			clock_gettime(CLOCK_MONOTONIC, &time2);
			t2 = (float)(time2.tv_sec + time2.tv_nsec*0.000000001);
		}

		clock_gettime(CLOCK_MONOTONIC, &time1);
		t1 = (float)(time1.tv_sec + time1.tv_nsec*0.000000001);
		delta_t = t1 - t2;

			map_delta_xyz[0] = (float)(*(_data_map+(i_number_read_main*5)))  - raw_status_data.pos_x;
			map_delta_xyz[1] = (float)(*(_data_map+(i_number_read_main*5)+1))  - raw_status_data.pos_y;
			map_delta_xyz[2] = (float)(*(_data_map+(i_number_read_main*5)+2))  - raw_status_data.pos_z;
			map_delta_r = sqrt(map_delta_xyz[0]*map_delta_xyz[0] + map_delta_xyz[1]*map_delta_xyz[1] + map_delta_xyz[2]*map_delta_xyz[2]);
			//global_d_yaw = (float)(*(_data_map+(i_number_read_main*4)+3));
			ip = map_old_xyz[0]*map_delta_xyz[0] + map_old_xyz[1]*map_delta_xyz[1] + map_old_xyz[2]*map_delta_xyz[2];
			while (start_run && i_number_read_main >= 0 && i_number_read_main < i_number-1 && ( delta_t >= (float)(*(_data_map+(i_number_read_main*5)+4)) || map_delta_r <= 0.03 || (ip <=0 && map_delta_r < 0.06) ))
			{

					ROS_INFO("*****************************");
					ROS_INFO("phase : ... %d ...%.4f.%.4f.%.4f..",i_number_read_main+1,VEL_SP_X,VEL_SP_Y,VEL_SP_Z);
					ROS_INFO("*****************************");
				old_xyz[0]= (float)(*(_data_map+(i_number_read_main*5)));
				old_xyz[1]= (float)(*(_data_map+(i_number_read_main*5)+1));
				old_xyz[2]= (float)(*(_data_map+(i_number_read_main*5)+2));
				i_number_read_main ++;
				//*****************************************************************************************************************
				if(i_number_read_main != 0)
					{
							 VEL_SP_X = map_old_xyz[0]/((float)(*(_data_map+(i_number_read_main*5)+4)) - (float)(*(_data_map+(i_number_read_main*5)-1)) );
							 VEL_SP_Y = map_old_xyz[1]/((float)(*(_data_map+(i_number_read_main*5)+4)) - (float)(*(_data_map+(i_number_read_main*5)-1)) );
							 VEL_SP_Z = map_old_xyz[2]/((float)(*(_data_map+(i_number_read_main*5)+4)) - (float)(*(_data_map+(i_number_read_main*5)-1)) );
					}
					else
					{
						 	 VEL_SP_X = map_old_xyz[0]/((float)(*(_data_map+(i_number_read_main*5)+4)) - t2);
							 VEL_SP_Y = map_old_xyz[1]/((float)(*(_data_map+(i_number_read_main*5)+4)) - t2);
							 VEL_SP_Z = map_old_xyz[2]/((float)(*(_data_map+(i_number_read_main*5)+4)) - t2);
					}
				//*****************************************************************************************************************
				map_old_xyz[0] = (float)(*(_data_map+(i_number_read_main*5))) - old_xyz[0];
				map_old_xyz[1] = (float)(*(_data_map+(i_number_read_main*5)+1)) - old_xyz[1];
				map_old_xyz[2] = (float)(*(_data_map+(i_number_read_main*5)+2)) - old_xyz[2];
				map_old_r = sqrt(map_old_xyz[0]*map_old_xyz[0] + map_old_xyz[1]*map_old_xyz[1] + map_old_xyz[2]*map_old_xyz[2]);
				map_delta_xyz[0] = (float)(*(_data_map+(i_number_read_main*5)))  - raw_status_data.pos_x;
				map_delta_xyz[1] = (float)(*(_data_map+(i_number_read_main*5)+1))  - raw_status_data.pos_y;
				map_delta_xyz[2] = (float)(*(_data_map+(i_number_read_main*5)+2))  - raw_status_data.pos_z;
				map_delta_r = sqrt(map_delta_xyz[0]*map_delta_xyz[0] + map_delta_xyz[1]*map_delta_xyz[1] + map_delta_xyz[2]*map_delta_xyz[2]);
				//global_d_yaw = (float)(*(_data_map+(i_number_read_main*4)+3));
				new_old_xyz[0] = raw_status_data.pos_x - old_xyz[0];
				new_old_xyz[1] = raw_status_data.pos_y - old_xyz[1];
				new_old_xyz[2] = raw_status_data.pos_z - old_xyz[2];
				new_old_r = sqrt(new_old_xyz[0]*new_old_xyz[0] + new_old_xyz[1]*new_old_xyz[1] + new_old_xyz[2]*new_old_xyz[2]);
				old_yaw= raw_status_data.att_yaw;
				ip = map_old_xyz[0]*map_delta_xyz[0] + map_old_xyz[1]*map_delta_xyz[1] + map_old_xyz[2]*map_delta_xyz[2];
				
			}
				if(map_old_r!=0 && map_delta_r!=0)
				cosa = ip/(map_delta_r*map_old_r);
				if (cosa>1)
					{cosa = 1;
						ROS_INFO("cos ERROR!");}
				if (cosa<-1)
					{cosa = -1;
						ROS_INFO("cos ERROR!");}
					if(map_old_r!=0 && map_delta_r!=0)
					{cos_xyz[0] = map_old_xyz[0]*(cosa*map_delta_r)/map_old_r;
					cos_xyz[1] = map_old_xyz[1]*(cosa*map_delta_r)/map_old_r;
					cos_xyz[2] = map_old_xyz[2]*(cosa*map_delta_r)/map_old_r;
					trac_e_x = map_delta_xyz[0]-cos_xyz[0];
					trac_e_y = map_delta_xyz[1]-cos_xyz[1];
					trac_e_z = map_delta_xyz[2]-cos_xyz[2];}
					else
					{
						trac_e_x = 0;
						trac_e_y = 0;
						trac_e_z = 0;
					}
					ROS_INFO_THROTTLE(0.5,"cosa: %.4f",cosa);
					ROS_INFO_THROTTLE(0.5,"e_x: %.4f e_y: %.4f e_z: %.4f",trac_e_x,trac_e_y,trac_e_z);
					ROS_INFO_THROTTLE(0.5,"delta_r: %.4f i_number_read_main: %d",delta_r,i_number_read_main);
					if(start_run)
					ROS_INFO_THROTTLE(0.5,"start_run: true");
					else
					ROS_INFO_THROTTLE(0.5,"start_run: false");

					ROS_INFO_THROTTLE(0.5," x %.2f y %.2f z %.2f ",raw_status_data.pos_x,raw_status_data.pos_y,raw_status_data.pos_z);
					ROS_INFO_THROTTLE(0.5,"dx %.2fdy %.2fdz %.2f dyaw %.2f",raw_status_data.dx,raw_status_data.dy,raw_status_data.dz,raw_status_data.d_yaw);


			if (i_number_read_main >=0 && i_number_read_main < i_number-1 && map_delta_r > 0.03 && start_run)
			{
				global_d_x = VEL_SP_X/P_POS + raw_status_data.pos_x +trac_e_x;
				global_d_y = VEL_SP_Y/P_POS + raw_status_data.pos_y +trac_e_y;
				global_d_z = VEL_SP_Z/P_POS + raw_status_data.pos_z +trac_e_z;
				if(new_old_r <= map_old_r && map_old_r != 0)
				{
					if(i_number_read_main ==0 )
						global_d_yaw = old_yaw + ((float)(*(_data_map+(i_number_read_main*5)+3)) - old_yaw) * new_old_r / map_old_r;
					if(i_number_read_main >0 )
						global_d_yaw = (float)(*(_data_map+(i_number_read_main*5)-2)) + ((float)(*(_data_map+(i_number_read_main*5)+3)) - (float)(*(_data_map+(i_number_read_main*5)-2)) ) * new_old_r / map_old_r;
				}
				else
				{
					global_d_yaw = (float)(*(_data_map+(i_number_read_main*5)+3));
				}
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
			global_d_x = (float)(*(_data_map+(i_number_read_main*5)));
			global_d_y = (float)(*(_data_map+(i_number_read_main*5)+1));
			global_d_z = (float)(*(_data_map+(i_number_read_main*5)+2));
			global_d_yaw = (float)(*(_data_map+(i_number_read_main*5)+3));
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