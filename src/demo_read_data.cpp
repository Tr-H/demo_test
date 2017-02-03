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
demo_test::pos_status raw_status_data;
boost::mutex pos_status_mutex;

float delta_x;
float delta_y;
float delta_z;
float desire_x =0;
float desire_y =0;
float desire_z =0;
float desire_r_x =0;
float desire_r_y =0;
float desire_r_z =0;
float old_x = 0;
float old_y = 0;
float old_z = 0;
float delta_old_x = 0;
float delta_old_y = 0;
float delta_old_z = 0;
float delta_old_rr = 0;
float delta_r =0;
float desire_yaw;
float dist_r=0;
float vel_ff_x=0;
float vel_ff_y=0;
float vel_ff_z=0;
bool vel_ff_zero_flag = true;
bool vel_ff_flag = false;
bool vel_ff_int_flag = false;
bool new_dpos_flag = false;

void pos_status_Callback(const demo_test::pos_status& pos_status1)
	{
			boost::mutex::scoped_lock lock(pos_status_mutex);
			memcpy(&raw_status_data, &pos_status1, sizeof(pos_status1));
			delta_x =raw_status_data.dx  - raw_status_data.pos_x;
			delta_y =raw_status_data.dy  - raw_status_data.pos_y;
			delta_z =raw_status_data.dz  - raw_status_data.pos_z;
			delta_old_x = raw_status_data.pos_x - old_x;
			delta_old_y = raw_status_data.pos_y - old_y;
			delta_old_z = raw_status_data.pos_z - old_z;
			delta_old_rr = delta_old_x*delta_old_x + delta_old_y*delta_old_y + delta_old_z*delta_old_z;
			delta_r = delta_x*delta_x+delta_y*delta_y+delta_z*delta_z;
			if( ( delta_r< (0.08 * 0.08) || ( dist_r!=0 && delta_old_rr >= dist_r && (delta_r <= delta_old_rr && delta_r <= dist_r ) ) )
					 && new_dpos_flag == false )
			{
				new_dpos_flag = true;
				vel_ff_flag = true;
				vel_ff_zero_flag = false;
				vel_ff_int_flag = true;
			}

			if(delta_r >= 0.5*0.5 && delta_old_rr >= 0.5*0.5)
			{
				vel_ff_zero_flag = true;
				vel_ff_flag = false;
			}
			/*else
			{
				new_dpos_flag = false;
			}*/
			//subflag=1;
			//lock.unlock();
	}

//catkin_ws/src/demo_test/src/test.txt
int main(int argc, char **argv)
	{
		ros::init(argc, argv, "demo_read_data");
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe("pos_status_topic1", 2000, pos_status_Callback);
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
    int i_number = 0;
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
	ros::Rate loop_rate(100);
	i_number_read = 0;
	ROS_INFO("i_number:........%d..........",i_number);
	ROS_INFO("*****************************");
	ROS_INFO("phase : ........ %d .........",i_number_read);
	ROS_INFO("*****************************");
	while(ros::ok())
		{	
			boost::mutex::scoped_lock lock(pos_status_mutex);

			if(n.hasParam("my_quadrotor1/x_d") && n.hasParam("my_quadrotor1/y_d") && n.hasParam("my_quadrotor1/z_d") && n.hasParam("my_quadrotor1/yaw_d") 
				&& n.hasParam("my_quadrotor1/vel_ff_x") && n.hasParam("my_quadrotor1/vel_ff_y") && n.hasParam("my_quadrotor1/vel_ff_z")) 
			{

				if(vel_ff_flag && i_number_read > 0 && i_number_read < i_number-1)
				{
					vel_ff_x = (float)( (double)(*(_data_map+(i_number_read*4)))   -   (double)(*(_data_map+(i_number_read*4-4))) );
					vel_ff_y = (float)( (double)(*(_data_map+(i_number_read*4)+1)) -   (double)(*(_data_map+(i_number_read*4-3))) );
					vel_ff_z = (float)( (double)(*(_data_map+(i_number_read*4)+2)) -   (double)(*(_data_map+(i_number_read*4-2))) );
					n.setParam("my_quadrotor1/vel_ff_x", (double)vel_ff_x);
					n.setParam("my_quadrotor1/vel_ff_y", (double)vel_ff_y);
					n.setParam("my_quadrotor1/vel_ff_z", (double)vel_ff_z);
				}
				
				if( ( (i_number_read == 0 || i_number_read >= i_number) || vel_ff_zero_flag ) && vel_ff_int_flag)
				{
					vel_ff_x = 0;
					vel_ff_y = 0;
					vel_ff_z = 0;
					n.setParam("my_quadrotor1/vel_ff_x",(double)vel_ff_x);
					n.setParam("my_quadrotor1/vel_ff_y",(double)vel_ff_y);
					n.setParam("my_quadrotor1/vel_ff_z",(double)vel_ff_z);
					if(i_number_read >= i_number
						&& raw_status_data.vel_ff_x == 0 
					&& raw_status_data.vel_ff_y == 0 
					&& raw_status_data.vel_ff_z == 0 )
						{break;}

				}

				if(new_dpos_flag && i_number_read < i_number)
				{	
					
					raw_status_data.pos_x
					n.setParam("my_quadrotor1/x_d", (double)(*(_data_map+(i_number_read*4))));
					n.setParam("my_quadrotor1/y_d", (double)(*(_data_map+(i_number_read*4)+1)));
					n.setParam("my_quadrotor1/z_d", (double)(*(_data_map+(i_number_read*4)+2)));
					n.setParam("my_quadrotor1/yaw_d", (double)(*(_data_map+(i_number_read*4)+3)));
					
					
					ROS_INFO("sending new data... %d ..",i_number_read);
				}
				if(raw_status_data.dx == (*(_data_map+(i_number_read*4)))
					&& raw_status_data.dy == (*(_data_map+(i_number_read*4)+1))
					&& raw_status_data.dz == (*(_data_map+(i_number_read*4)+2))
					&& raw_status_data.d_yaw == (*(_data_map+(i_number_read*4)+3))
					&& raw_status_data.vel_ff_x == vel_ff_x 
					&& raw_status_data.vel_ff_y == vel_ff_y 
					&& raw_status_data.vel_ff_z == vel_ff_z 
					)
				{
					
					desire_r_x = raw_status_data.pos_x - raw_status_data.dx;
					desire_r_y = raw_status_data.pos_y - raw_status_data.dy;
					desire_r_z = raw_status_data.pos_z - raw_status_data.dz;


					dist_r = desire_r_x*desire_r_x + desire_r_y*desire_r_y +desire_r_z*desire_r_z;
					old_x = raw_status_data.pos_x;
					old_y = raw_status_data.pos_y;
					old_z = raw_status_data.pos_z;
				i_number_read++;
				ROS_INFO("*****************************");
				ROS_INFO("phase : ........ %d .........",i_number_read);
				ROS_INFO("*****************************");
				new_dpos_flag = false;
				vel_ff_flag = false;
				}
				if( vel_ff_zero_flag
					&& raw_status_data.vel_ff_x == 0 
					&& raw_status_data.vel_ff_y == 0 
					&& raw_status_data.vel_ff_z == 0 )
				vel_ff_zero_flag = false;

			}
			else
			{
				ROS_INFO("can not find ros param, please check....");
				 sleep(2);
			}
			lock.unlock();
			
			loop_rate.sleep();
		}
		ROS_INFO("*****************************");
		ROS_INFO("phase : ........END..........");
		ROS_INFO("*****************************");
	    return 0;

	}