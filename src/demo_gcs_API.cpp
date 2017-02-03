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
#include "checksum.h"
using namespace std;
#define BAUDRATE B57600//B57600
#define MODEMDEVICE "/dev/ttyUSB0"
#define MODEMDEVICE1 "/dev/ttyUSB1"
#define MODEMDEVICE2 "/dev/ttyUSB2"
#define MODEMDEVICE3 "/dev/ttyUSB3"
/*char *MODEMDEVICE="/dev/tty";
//= "/dev/ttyUSB0";
char* USBport;*/
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;
boost::mutex pos_mutex;
boost::mutex pos_sp_mutex;
boost::mutex send_mutex;
boost::mutex type_mutex;
boost::mutex freq_mutex;
demo_test::pos_data xyz;
demo_test::pos_data xyz_old;
demo_test::pos_status pos_status1;
demo_test::attitude_feedback attitude_feedback1;
demo_test::velocity_feedback velocity_feedback1;
demo_test::body_attitude body_attitude1;
demo_test::position_setpoint position_setpoint1;
ros::Publisher status_upd_sub;
ros::Publisher attitude_feedback_sub;
ros::Publisher velocity_feedback_sub;
ros::Publisher body_attitude_sub;
ros::Publisher position_setpoint_sub;
float ref_data[4];
float vel_ff_x = 0;
float vel_ff_y = 0;
float vel_ff_z = 0;
std::string ref_status1;
char *ref_status2;
char *ref_status;
char *ref_name;
char *MODEMDEVICE_NAME;
char *status_publish_name;
char *rewrite_topic_name;
char *velocity_feedback_name;
char *attitude_feedback_name;
char *body_attitude_name;
char *position_setpoint_name;
float send_currentpos_freq=22;
float send_desirepos_freq=12;
float send_type_freq=5;
int write_has_chang=0;
int global_id;

bool write_att=true;

unsigned char  BUF[256];
unsigned char  BUF1[256];
unsigned char (*temp_BUF)[256];
//unsigned char  buf[256]; 
unsigned char  buf[32];
unsigned char  buf_now[32];
unsigned char  buf_point; 
const uint8_t *BUFF_TEMP;
bool read_finish_flag=false;
int buff_i=0;
int temp_buff_i=0;
float global_roll=0;
float global_pitch=0;
float global_yaw=0;
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
 // global_type_temp.type[6] = {1,1,1,1,1,1};
class myGroundCS 
{
public:
	myGroundCS();
	//~myGroundCS();
	struct timespec time1;
	struct timespec time2;
	//static bool flag = true;
	int send_write(int fd,char *buffer,int length);
	int open_port(const char * tty_port);
	int setup_port(int fd);
	void udpcallback(const demo_test::pos_data& pos);
 	void writecallback(const demo_test::pos_write_data& pos1);
	void send_current_pos(const int& fd);
	void send_desire_pos(const int& fd);
	// void send_desire_type(const int& fd);
	void receive_attitude(const int& fd);
	void status_publish_thrd();
	void receive_unpackage();
	bool setup_port_flag;
	int fd_setup_port;

private:
};
myGroundCS::myGroundCS()
{    int i;
	 timespec time1 = {0, 0};
	 timespec time2 = {0, 0};
 	 setup_port_flag=true;
		//setup_port
			for(i=0;i<6;i++)
			{
		 global_type_temp.type=1;
		 global_type_temp.flage_position=true;
		 global_type_temp.flage_velocity=true;
		 global_type_temp.flage_altitude=true;
		 global_type_temp.flage_attitude=true;
		 global_type_temp.flage_climb=true;
	        }
		 
	 fd_setup_port=myGroundCS::open_port(MODEMDEVICE_NAME);
		
	if (fd_setup_port < 0)
	{
		setup_port_flag=false;
	}
	else
	{
		if (myGroundCS::setup_port(fd_setup_port) < 0)
		{
			setup_port_flag=false;
		}
		else
		{
			
		ROS_INFO("%s is open...\n", MODEMDEVICE_NAME);
		}
    }

}


int myGroundCS::open_port(const char * tty_port)
{
    int fd;
    fd = open(tty_port, O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        ROS_ERROR("open tty error\n");
        return (-1);
    }
    else
    {
        ROS_INFO("open %s successfully\n", tty_port);
    }
    if (!isatty(fd))
    {
        ROS_ERROR("%s is not a serial port\n", tty_port);
        return (-1);
    }
    return (fd);
}

int myGroundCS::setup_port(int fd)
{
    struct termios config;
    if (tcgetattr(fd, &config) < 0)
    {
        ROS_WARN("get serial pot attribute error\n");
        return (-1);
    }
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
	#ifdef OLCUC
	  config.c_oflag &= ~OLCUC;
	#endif
	#ifdef ONOEOT
	  config.c_oflag &= ~ONOEOT;
	#endif
	  config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	  config.c_cflag &= ~(CSIZE | PARENB);
	  config.c_cflag |= CS8;
	  config.c_cc[VMIN] = 0;
	  config.c_cc[VTIME] = 0; 
	   if (cfsetispeed(&config, BAUDRATE) < 0 || cfsetospeed(&config, BAUDRATE) < 0)
	    {
			ROS_WARN("\nERROR: Could not set desired baud rate of %d Baud\n", BAUDRATE);
			return (-1);
	    }
	   if (tcsetattr(fd, TCSAFLUSH, &config) < 0)
	    {
			fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
			return (-1);
	    }
	    return 1;
}


void myGroundCS::udpcallback(const demo_test::pos_data& pos)
{
   
    boost::mutex::scoped_lock lock(pos_mutex);
    memcpy(&xyz, &pos, sizeof(pos));
    lock.unlock();
}
void myGroundCS::writecallback(const demo_test::pos_write_data& pos1)
{   	boost::mutex::scoped_lock possp_lock(pos_sp_mutex);
			 ref_data[0]=pos1.pos_d[0];
			 ref_data[1]=pos1.pos_d[1];
			 ref_data[2]=pos1.pos_d[2];
		     ref_data[3]=pos1.pos_d[3];
		     vel_ff_x = 0;
		     vel_ff_y = 0;
		     vel_ff_z = 0;
    	possp_lock.unlock();
	 // ref_data[4]=pos1.pos_d[4];
    	boost::mutex::scoped_lock freq_lock(freq_mutex);
		   	 send_currentpos_freq=pos1.send_currentpos_freq;
			 send_desirepos_freq=pos1.send_desirepos_freq;
			 send_type_freq=pos1.send_type_freq;
	 	freq_lock.unlock();
    	 
    	boost::mutex::scoped_lock type_lock(type_mutex);
		    global_type_temp.type= pos1.type;
			global_type_temp.flage_position = (pos1.flag[0]==1)? true:false;
			global_type_temp.flage_velocity = (pos1.flag[1]==1)? true:false;
			global_type_temp.flage_altitude = (pos1.flag[2]==1)? true:false;
			global_type_temp.flage_attitude = (pos1.flag[3]==1)? true:false;
			global_type_temp.flage_climb = (pos1.flag[4]==1)? true:false;
			write_has_chang=1;
		type_lock.unlock();
}

// to be complete------------------
void myGroundCS::send_current_pos(const int& fd)
{
    //to be complete
    char buffer[20]={0xFE,12,0,1,50,232};
	unsigned int seq=1;
	float temp_freq;
	typedef struct _mocap_data
		{
			float x;
			float y;
			float z;
		}mocap_data;
	mocap_data _pos_data;
	uint8_t ck[2];
   	int write_n;
	
	float delay_time;
		while(ros::ok())
			{
			ros::Rate loop_rate_send_current(send_currentpos_freq);	
			temp_freq=send_currentpos_freq;	
				while(ros::ok())
				{
					if(temp_freq == send_currentpos_freq)
						{
							boost::mutex::scoped_lock lock(pos_mutex);
							_pos_data.x = xyz.pos[0+3*(global_id-1)];
							_pos_data.y = xyz.pos[2+3*(global_id-1)];
							_pos_data.z = -xyz.pos[1+3*(global_id-1)];
							lock.unlock();
						//ROS_INFO("sending position_feedback to uav1: x=%.4f, y=%.4f, z=%.4f", _pos_data.x, _pos_data.y, _pos_data.z);
							ROS_DEBUG("pos :%d",seq);  
						  	memcpy(buffer+6, &_pos_data, sizeof(_pos_data));
							buffer[2]=seq++;		
							buffer[18]=88;
							uint16_t crcTmp = crc_calculate((uint8_t *)(buffer+1),18);
	
							ck[0] = (uint8_t)(crcTmp & 0xFF);
							ck[1] = (uint8_t)(crcTmp >> 8);
							buffer[18]=ck[0];
							buffer[19]=ck[1];

							write_n = 0;
							boost::mutex::scoped_lock send_lock(send_mutex);
							write_n=write(fd,buffer,20);
							//write_n=send_write(fd,buffer,20);	
							send_lock.unlock();
							if(write_n!=20)
								{
									ROS_WARN("send current position feedback error\n");
									///////////////////
								}
						}
					else
						break;
						loop_rate_send_current.sleep();
				}
			}
}

//-------------------------------------------------------------------------------
//
void myGroundCS::send_desire_pos(const int& fd)
{
    char buffer2[26]={0xFE,18,0,1,50,233};
	unsigned int seq2=1;
	float temp_freq;
	typedef struct _pos_sp_data
		{
			float x_d;
			float y_d;
			float z_d;
	        float yaw_d;
	        uint8_t type;
	        uint8_t flag;
		}pos_sp_data;
	pos_sp_data _xyz_sp;
	
	uint8_t ck2[2];
	int write_n2;
	float delay_time;
	uint8_t test_sum;
	while(ros::ok())
	{
	ros::Rate loop_rate_send_desire(send_desirepos_freq);
		temp_freq = send_desirepos_freq;
		while(ros::ok())
			{
				if(temp_freq == send_desirepos_freq)
					{	
						boost::mutex::scoped_lock possp_lock(pos_sp_mutex);
							_xyz_sp.x_d = ref_data[0];
							_xyz_sp.y_d = ref_data[1];
							_xyz_sp.z_d = ref_data[2];
						    _xyz_sp.yaw_d = ref_data[3];
						possp_lock.unlock();
						boost::mutex::scoped_lock type_lock(type_mutex);
						    _xyz_sp.type = (uint8_t)global_type_temp.type;
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
						    _xyz_sp.flag = (uint8_t)test_sum;
						   // ROS_INFO("sum : %02X",test_sum);
						type_lock.unlock();
						// _xyz_sp.type = ref_data[4];
					 // ROS_INFO("sending position setpoint to uav1: x_d=%.4f, y_d=%.4f, z_d=%.4f, yaw_d=%.4f\n", _xyz_sp.x_d, _xyz_sp.y_d, _xyz_sp.z_d, _xyz_sp.yaw_d);
						ROS_DEBUG("dpos: %d",seq2);  
					 	memcpy(buffer2+6, &_xyz_sp, sizeof(_xyz_sp));
						buffer2[2]=seq2++;		
						buffer2[24]=127;//22
						uint16_t crcTmp2 = crc_calculate((uint8_t *)(buffer2+1),24);//22
			
						ck2[0] = (uint8_t)(crcTmp2 & 0xFF);
						ck2[1] = (uint8_t)(crcTmp2 >> 8);
						buffer2[24]=ck2[0];//22
						buffer2[25]=ck2[1];//23
						write_n2 = 0;
						boost::mutex::scoped_lock send_lock(send_mutex);
						write_n2=write(fd,buffer2,26);//24
						//write_n2=send_write(fd,buffer2,24);	
						send_lock.unlock();
					if(write_n2!=26)
						{
							ROS_WARN("send desire position setpoint  error\n");
							//return -1;
						}
					}
				else 
					break;
				loop_rate_send_desire.sleep();
			}
	}
}
// //////////////////////////////////////////////////////////////////////
/*void myGroundCS::send_desire_type(const int& fd)
{
    char buffer2[17]={0xFE,9,0,1,50,234};
	unsigned int seq2=1;
	float temp_freq;
	typedef struct _pos_type
		{
			int type;
			bool flage_position;
			bool flage_velocity;
			bool flage_altitude;
			bool flage_attitude;
			bool flage_climb;
		}pos_sp_type;
	pos_sp_type _type_d;
	
	uint8_t ck2[2];
	int write_n2;
	// float delay_time;
	while(ros::ok())
	{
	ros::Rate loop_rate_send_type(send_type_freq);
		temp_freq = send_type_freq;
		while(ros::ok())
			{
				//delay_time=1000000/send_desirepos_freq-20;
				if(temp_freq == send_type_freq)
					{	
						boost::mutex::scoped_lock type_lock(type_mutex);
							_type_d.type = global_type_temp.type;
							_type_d.flage_position = global_type_temp.flage_position;
							_type_d.flage_velocity = global_type_temp.flage_velocity;
						    _type_d.flage_altitude = global_type_temp.flage_altitude;
							_type_d.flage_attitude = global_type_temp.flage_attitude;
							_type_d.flage_climb = global_type_temp.flage_climb;
						type_lock.unlock();
					 // ROS_INFO("sending position setpoint to uav1: x_d=%.4f, y_d=%.4f, z_d=%.4f, yaw_d=%.4f\n", _xyz_sp.x_d, _xyz_sp.y_d, _xyz_sp.z_d, _xyz_sp.yaw_d);
						ROS_DEBUG("type: %d",seq2);  
					 	memcpy(buffer2+6, &_type_d, sizeof(_type_d));
						buffer2[2]=seq2++;		
						buffer2[15]=212;//
						uint16_t crcTmp2 = crc_calculate((uint8_t *)(buffer2+1),15);
			
						ck2[0] = (uint8_t)(crcTmp2 & 0xFF);
						ck2[1] = (uint8_t)(crcTmp2 >> 8);
						buffer2[15]=ck2[0];
						buffer2[16]=ck2[1];
						write_n2 = 0;
						boost::mutex::scoped_lock send_lock(send_mutex);
						write_n2=write(fd,buffer2,17);
						//write_n2=send_write(fd,buffer2,24);	
						send_lock.unlock();
					if(write_n2!=17)
						{
							ROS_WARN("send desire type  error\n");
							//return -1;
						}
					}
				else 
					break;
				loop_rate_send_type.sleep();
			}
	}
}*/
////////////////////////////////////////////////////////////////////////
void myGroundCS::receive_attitude(const int& fd)
{	int i;
	float jiao,speed;
	int nread;
	int nread_now;
	int i_temp;
	uint8_t ck[2];
	float *temp;
		float roll,pitch,yaw,rollspeed,pitchspeed,yawspeed;
		float x,y,z,vx,vy,vz;
		float PI=3.1415926;
		float body_roll_rate;
		float body_pitch_rate;
		float body_yaw_rate;
		float trust;
		float yaw_rate;
		float afx,afy,afz;
	//ros::Rate loop_read(200);
	while(ros::ok())
	{	  

		//boost::mutex::scoped_lock send_lock(send_mutex);
		if((nread_now=read(fd,buf_now, sizeof(buf_now))) < sizeof(buf_now))
		{
			/*if(nread_now < 5)
			{	
				usleep(6000);
				printf("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n%d",nread_now);
			}*/
			usleep(600);
		}
		
		if(nread_now != 0)
		{
		for (i=0;i<nread;i++)
		{	
			if((buf[i] == 0xFE && buf[i+3] == 0x01 && i < nread-3) ||(buf[i] == 0xFE && buf_now[i-nread+3] == 0x01 && i >= nread-3))
						{
							memcpy(BUF1,BUF,sizeof(BUF));
							//boost::function0< void> f = boost::bind(&myGroundCS::receive_unpackage,this);
							boost::thread thrd(boost::bind(&myGroundCS::receive_unpackage,this));
							BUF[0] = buf[i];
							// printf("BUF:");
							// for(i_temp=0;i_temp < buff_i;i_temp++)
							// {
							// 	printf("%02X",BUF[i_temp]);
							// }
							// printf("\n");
							buff_i=0;
						}
					else
						{	
							buff_i++;
							
							BUF[buff_i] = buf[i];
						}
		}
		memcpy(buf,buf_now,sizeof(buf_now));
		nread=nread_now;
		}
		
	}
}
////////////////////////////////////////////////////////////////////////
void myGroundCS::receive_unpackage()
{
		int i;
	float jiao,speed;
	int nread;
	//int buff_i;
	uint8_t ck[2];
	float *temp;
		float roll,pitch,yaw,rollspeed,pitchspeed,yawspeed;
		float x,y,z,vx,vy,vz;
		float PI=3.1415926;
		float body_roll_rate;
		float body_pitch_rate;
		float body_yaw_rate;
		float trust;
		float yaw_rate;
		float afx,afy,afz;

			temp_BUF = &BUF1;
			 if((*temp_BUF)[1]==0x25 )
			            	{	(*temp_BUF)[43]=0X16;
			            		BUFF_TEMP = (const uint8_t*)&(*temp_BUF)[1];
			            		uint16_t crcTmp = crc_calculate(BUFF_TEMP,43);
								ck[0] = (uint8_t)(crcTmp & 0xFF);
								ck[1] = (uint8_t)(crcTmp >> 8);
									//ROS_INFO("CK0 %02X CK1 %02X BUF34 %02X BUF35 %02X",ck[0],ck[1],BUF[43],BUF[44]);
			            			if(ck[1]==(*temp_BUF)[44])
			            			switch((*temp_BUF)[5])//判别msgid
											{	
												//
												case 0x53:                       
											    i=6;						
											    temp = (float*)&(*temp_BUF)[i+20];								
											    jiao=*temp;
											    // body_roll_rate = jiao*180/PI;
											    body_roll_rate=jiao;
											    temp = (float*)&(*temp_BUF)[i+24];									
											    jiao=*temp;
											    // body_pitch_rate=jiao*180/PI;									
											    body_pitch_rate=jiao;
											    temp = (float*)&(*temp_BUF)[i+28];									
											    jiao=*temp;
											    // body_yaw_rate=jiao*180/PI;									
											    body_yaw_rate=jiao;
											    temp = (float*)&(*temp_BUF)[i+32];							
											    speed=*temp;
											    trust=speed;									
											    // (*temp_BUF)[0]=0X00;
											    body_attitude1.body_roll_rate=body_roll_rate;
											    body_attitude1.body_pitch_rate=body_pitch_rate;
											    body_attitude1.body_yaw_rate=body_yaw_rate;
											    body_attitude1.trust=trust;
											    body_attitude_sub.publish(body_attitude1);
											    //ROS_INFO("CK0 %02X CK1 %02X BUF34 %02X BUF35 %02X",ck[0],ck[1],(*temp_BUF)[43],(*temp_BUF)[44]);
											    ROS_INFO("\n--------------------seq:33-----------------%02X\n",(*temp_BUF)[2]);
												/*printf("\nroll:%f pitch:%f yaw:%f \n",roll,pitch,yaw);
												printf("rollspeed:%f pitchspeed:%f yawspeed:%f \n",rollspeed,pitchspeed,yawspeed);
								*/				//ROS_INFO("b_roll_Rate: %.2f b_pitch_Rate: %.2f b_yaw_Rate: %.2f trust: %.4f",body_roll_rate,body_pitch_rate,body_yaw_rate,trust);
												break;
												default:
												break;
											}
								//read_finish_flag=false;
			            	}
			 if((*temp_BUF)[1]==0x33 )
			            	{
			            		(*temp_BUF)[57]=0X8C;
			            		BUFF_TEMP = (const uint8_t*)&(*temp_BUF)[1];
			            		uint16_t crcTmp = crc_calculate(BUFF_TEMP,57);
								ck[0] = (uint8_t)(crcTmp & 0xFF);
								ck[1] = (uint8_t)(crcTmp >> 8);
									//ROS_INFO("CK0 %02X CK1 %02X BUF34 %02X BUF35 %02X",ck[0],ck[1],BUF[57],BUF[58]);
								if(ck[1]==(*temp_BUF)[58])
			            			switch((*temp_BUF)[5])//判别msgid
											{
												case 0x55:                       
											    i=6;						
											    temp = (float*)&(*temp_BUF)[i+4];								
											    jiao=*temp;
											    x = jiao;
											    temp = (float*)&(*temp_BUF)[i+8];									
											    jiao=*temp;
											    y = jiao;									
											    temp = (float*)&(*temp_BUF)[i+12];									
											    jiao=*temp;
											    z = jiao;									
											    temp = (float*)&(*temp_BUF)[i+16];							
											    speed=*temp;
											    vx=speed;
											    temp = (float*)&(*temp_BUF)[i+20];							
											    speed=*temp;
											    vy=speed;
											    temp = (float*)&(*temp_BUF)[i+24];							
											    speed=*temp;
											    vz=speed;
											    temp = (float*)&(*temp_BUF)[i+28];							
											    speed=*temp;
											    afx=speed;
											    temp = (float*)&(*temp_BUF)[i+32];							
											    speed=*temp;
											    afy=speed;
											    temp = (float*)&(*temp_BUF)[i+36];							
											    speed=*temp;
											    afz=speed;
											    temp = (float*)&(*temp_BUF)[i+40];							
											    speed=*temp;
											    yaw=speed;
											    temp = (float*)&(*temp_BUF)[i+44];							
											    speed=*temp;
											    yaw_rate=speed;

											    position_setpoint1.x_d=(float)x;
											    position_setpoint1.y_d=(float)y;
											    position_setpoint1.z_d=(float)z;
											    position_setpoint1.x_speed_d=(float)vx;
											    position_setpoint1.y_speed_d=(float)vy;
											    position_setpoint1.z_speed_d=(float)vz;											    
											    position_setpoint1.x_a_d=(float)afx;
											    position_setpoint1.y_a_d=(float)afy;
											    position_setpoint1.z_a_d=(float)afz;
											    position_setpoint1.yaw_d=(float)yaw;
											    position_setpoint1.yaw_rate_d=(float)yaw_rate;
											    position_setpoint_sub.publish(position_setpoint1);
											    // (*temp_BUF)[0]=0X00;
											    ROS_INFO("\n--------------------seq:44-----------------%02X\n",(*temp_BUF)[2]);
								// 				/*printf("\nroll:%f pitch:%f yaw:%f \n",roll,pitch,yaw);
								// 				printf("rollspeed:%f pitchspeed:%f yawspeed:%f \n",rollspeed,pitchspeed,yawspeed);
								// */				//ROS_INFO("b_roll_Rate: %.2f b_pitch_Rate: %.2f b_yaw_Rate: %.2f trust: %.4f",body_roll_rate,body_pitch_rate,body_yaw_rate,trust);
								// 				ROS_INFO("X: %.2f Y: %.2f Z: %.2f ",x,y,z);
								// 				ROS_INFO("vX: %.2f vY: %.2f vZ: %.2f ",vx,vy,vz);
								// 				ROS_INFO("afX: %.2f afY: %.2f afZ: %.2f ",afx,afy,afz);
								// 				ROS_INFO("yaw: %.2f yaw_rate: %.2f ",yaw,yaw_rate);

												break;
												default:
												break;
											}
											//read_finish_flag=false;
			            	}
			 if((*temp_BUF)[1]==0x1c )
			            	{	(*temp_BUF)[34]=0X27;
			            		BUFF_TEMP = (const uint8_t*)&(*temp_BUF)[1];
			            		uint16_t crcTmp = crc_calculate(BUFF_TEMP,34);
								ck[0] = (uint8_t)(crcTmp & 0xFF);
								ck[1] = (uint8_t)(crcTmp >> 8);
									if(ck[1]==(*temp_BUF)[35])
											{	
												
												switch((*temp_BUF)[5])//判别msgid
												{
													case 0x1E: 
													//ROS_INFO("CK0 %02X CK1 %02X BUF34 %02X BUF35 %02X",ck[0],ck[1],(*temp_BUF)[34],(*temp_BUF)[35]);                      
												    i=6;						
												    temp = (float*)&(*temp_BUF)[i+4];								
												    jiao=*temp;
												    //roll=jiao*180/PI;
												    roll=jiao;
												    temp = (float*)&(*temp_BUF)[i+8];									
												    jiao=*temp;
												    //pitch=jiao*180/PI;									
												    pitch=jiao;
												    temp = (float*)&(*temp_BUF)[i+12];									
												    jiao=*temp;
												    // yaw=jiao*180/PI;									
												    yaw=jiao;
												    temp = (float*)&(*temp_BUF)[i+16];							
												    speed=*temp;
												    // rollspeed=speed*180/PI;									
												    rollspeed=speed;
												    temp = (float*)&(*temp_BUF)[i+20];
												    speed=*temp;
												    // pitchspeed=speed*180/PI;
												    pitchspeed=speed;
												    temp = (float*)&(*temp_BUF)[i+24];
												    speed=*temp;	
												    // yawspeed=speed*180/PI;  
												    yawspeed=speed;
												    
												    attitude_feedback1.roll=(float)roll;
												    attitude_feedback1.pitch=(float)pitch;
												    attitude_feedback1.yaw=(float)yaw;
												    //boost::mutex::scoped_lock pos_lock1(pos_mutex);
												    write_att=false;
												    global_roll=(float)roll;
												     global_pitch=(float)pitch;
												      global_yaw=(float)yaw;
												      write_att=true;
												      //pos_lock1.unlock();
												    attitude_feedback1.speed_roll=(float)rollspeed;
												    attitude_feedback1.speed_pitch=(float)pitchspeed;
												    attitude_feedback1.speed_yaw=(float)yawspeed;
												    attitude_feedback_sub.publish(attitude_feedback1);
												    // (*temp_BUF)[0]=0X00;
												    ROS_INFO("\n--------------------seq:11-----------------%02X\n",(*temp_BUF)[2]);
													/*printf("\nroll:%f pitch:%f yaw:%f \n",roll,pitch,yaw);
													printf("rollspeed:%f pitchspeed:%f yawspeed:%f \n",rollspeed,pitchspeed,yawspeed);
									*/				//read_finish_flag=false;
													break;
													default:
													//read_finish_flag=false;
													break;
												}
											}
									else{
											(*temp_BUF)[34]=0XB9;
						            		BUFF_TEMP = (const uint8_t*)&(*temp_BUF)[1];
						            		uint16_t crcTmp = crc_calculate(BUFF_TEMP,34);
											ck[0] = (uint8_t)(crcTmp & 0xFF);
											ck[1] = (uint8_t)(crcTmp >> 8);
											
											if(ck[1]==(*temp_BUF)[35])
											switch((*temp_BUF)[5])//判别msgid
											{
												
												case 0x20:
												//ROS_INFO("CK0 %02X CK1 %02X BUF34 %02X BUF35 %02X",ck[0],ck[1],(*temp_BUF)[34],(*temp_BUF)[35]);
												i=6;						
												temp = (float*)&(*temp_BUF)[i+4];								
												x=*temp;
												temp = (float*)&(*temp_BUF)[i+8];									
												y=*temp;									
												temp = (float*)&(*temp_BUF)[i+12];									
												z=*temp;									
												temp = (float*)&(*temp_BUF)[i+16];							
												vx=*temp;									
												temp = (float*)&(*temp_BUF)[i+20];
												vy=*temp;
												temp = (float*)&(*temp_BUF)[i+24];	
												vz=*temp;

												velocity_feedback1.x=(float)x;
												velocity_feedback1.y=(float)y;
												velocity_feedback1.z=(float)z;
												velocity_feedback1.vx=(float)vx;
												velocity_feedback1.vy=(float)vy;
												velocity_feedback1.vz=(float)vz;								    
												velocity_feedback_sub.publish(velocity_feedback1);
												// (*temp_BUF)[0]=0X00;
											    ROS_INFO("\n--------------------seq:22-----------------%02X\n",(*temp_BUF)[2]);
												/*printf("x:%f y:%f z:%f \n",x,y,z);
												printf("vx:%f vy:%f vz:%f \n",vx,vy,vz); */
												//read_finish_flag=false;
												break;
												default:
												//read_finish_flag=false;
												break;
											}
										}

									}
}
////////////////////////////////////////////////////////////////////////

void myGroundCS::status_publish_thrd()
{

	ros::Rate loop_pos_status(100);
	while(ros::ok())
		{
			boost::mutex::scoped_lock possp_lock(pos_sp_mutex);
				pos_status1.dx = ref_data[0];
				pos_status1.dy = ref_data[1];
				pos_status1.dz = ref_data[2];
			    pos_status1.d_yaw = ref_data[3];
			    pos_status1.vel_ff_x = vel_ff_x;
			    pos_status1.vel_ff_y = vel_ff_y;
			    pos_status1.vel_ff_z = vel_ff_z;
			possp_lock.unlock();

			boost::mutex::scoped_lock pos_lock(pos_mutex);
			    pos_status1.pos_x = xyz.pos[0];
			    pos_status1.pos_y = xyz.pos[2];
			    pos_status1.pos_z = -xyz.pos[1];
			    pos_status1.id = global_id;
			    if(write_att)
			    {
			    pos_status1.att_yaw = global_yaw;
			    pos_status1.att_roll = global_roll;
			    pos_status1.att_pitch = global_pitch;
			    }
			    //pos_status1.name = ref_name;
		    pos_lock.unlock();
			
			boost::mutex::scoped_lock type_lock(type_mutex);
				pos_status1.type = global_type_temp.type;
				/*if (global_type_temp.flage_position[0])
					pos_status1.flag[1] = 1;
				else
					pos_status1.flag[1] = 0;*/
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
			status_upd_sub.publish(pos_status1);

			loop_pos_status.sleep();
		}
}


//////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{

//--------------------------------------------------------------------------
	global_id = atoi(argv[6]);

   char* param_xd;
   char* param_yd;
   char* param_zd;
   char* param_yawd;
   char* param_vel_ff_x;
   char* param_vel_ff_y;
   char* param_vel_ff_z;

   if (global_id==1)
	   {
		   	param_xd = new char[sizeof("my_quadrotor1/x_d")];
			memset(param_xd, 0, sizeof("my_quadrotor1/x_d"));
			strcpy(param_xd,"my_quadrotor1/x_d");
			param_yd = new char[sizeof("my_quadrotor1/y_d")];
			memset(param_yd, 0, sizeof("my_quadrotor1/y_d"));
			strcpy(param_yd,"my_quadrotor1/y_d");
			param_zd = new char[sizeof("my_quadrotor1/z_d")];
			memset(param_zd, 0, sizeof("my_quadrotor1/z_d"));
			strcpy(param_zd,"my_quadrotor1/z_d");
			param_yawd = new char[sizeof("my_quadrotor1/yaw_d")];
			memset(param_yawd, 0, sizeof("my_quadrotor1/yaw_d"));
			strcpy(param_yawd,"my_quadrotor1/yaw_d");

			param_vel_ff_x = new char[sizeof("my_quadrotor1/vel_ff_x")];
			memset(param_vel_ff_x, 0, sizeof("my_quadrotor1/vel_ff_x"));
			strcpy(param_vel_ff_x,"my_quadrotor1/vel_ff_x");
			param_vel_ff_y = new char[sizeof("my_quadrotor1/vel_ff_y")];
			memset(param_vel_ff_y, 0, sizeof("my_quadrotor1/vel_ff_y"));
			strcpy(param_vel_ff_y,"my_quadrotor1/vel_ff_y");
			param_vel_ff_z = new char[sizeof("my_quadrotor1/vel_ff_z")];
			memset(param_vel_ff_z, 0, sizeof("my_quadrotor1/vel_ff_z"));
			strcpy(param_vel_ff_z,"my_quadrotor1/vel_ff_z");

			MODEMDEVICE_NAME = new char[sizeof(MODEMDEVICE)];
			memset(MODEMDEVICE_NAME, 0, sizeof(MODEMDEVICE));
			strcpy(MODEMDEVICE_NAME,MODEMDEVICE);
			status_publish_name = new char[sizeof("pos_status_topic1")];
			memset(status_publish_name, 0, sizeof("pos_status_topic1"));
			strcpy(status_publish_name,"pos_status_topic1");
			rewrite_topic_name = new char[sizeof("pos_write_topic1")];
			memset(rewrite_topic_name, 0, sizeof("pos_write_topic1"));
			strcpy(rewrite_topic_name,"pos_write_topic1");
			velocity_feedback_name = new char[sizeof("velocity_feedback1")];
			memset(velocity_feedback_name, 0, sizeof("velocity_feedback1"));
			strcpy(velocity_feedback_name,"velocity_feedback1");	
			attitude_feedback_name = new char[sizeof("attitude_feedback1")];
			memset(attitude_feedback_name, 0, sizeof("attitude_feedback1"));
			strcpy(attitude_feedback_name,"attitude_feedback1");
			body_attitude_name = new char[sizeof("body_attitude1")];
			memset(body_attitude_name, 0, sizeof("body_attitude1"));
			strcpy(body_attitude_name,"body_attitude1");			
			position_setpoint_name = new char[sizeof("position_setpoint1")];
			memset(position_setpoint_name, 0, sizeof("position_setpoint1"));
			strcpy(position_setpoint_name,"position_setpoint1");
	   }
   if (global_id==2)
	   {
		   	param_xd = new char[sizeof("my_quadrotor2/x_d")];
			memset(param_xd, 0, sizeof("my_quadrotor2/x_d"));
			strcpy(param_xd,"my_quadrotor2/x_d");
			param_yd = new char[sizeof("my_quadrotor2/y_d")];
			memset(param_yd, 0, sizeof("my_quadrotor2/y_d"));
			strcpy(param_yd,"my_quadrotor2/y_d");
			param_zd = new char[sizeof("my_quadrotor2/z_d")];
			memset(param_zd, 0, sizeof("my_quadrotor2/z_d"));
			strcpy(param_zd,"my_quadrotor2/z_d");
			param_yawd = new char[sizeof("my_quadrotor2/yaw_d")];
			memset(param_yawd, 0, sizeof("my_quadrotor2/yaw_d"));
			strcpy(param_yawd,"my_quadrotor2/yaw_d");

			param_vel_ff_x = new char[sizeof("my_quadrotor2/vel_ff_x")];
			memset(param_vel_ff_x, 0, sizeof("my_quadrotor2/vel_ff_x"));
			strcpy(param_vel_ff_x,"my_quadrotor2/vel_ff_x");
			param_vel_ff_y = new char[sizeof("my_quadrotor2/vel_ff_y")];
			memset(param_vel_ff_y, 0, sizeof("my_quadrotor2/vel_ff_y"));
			strcpy(param_vel_ff_y,"my_quadrotor2/vel_ff_y");
			param_vel_ff_z = new char[sizeof("my_quadrotor2/vel_ff_z")];
			memset(param_vel_ff_z, 0, sizeof("my_quadrotor2/vel_ff_z"));
			strcpy(param_vel_ff_z,"my_quadrotor2/vel_ff_z");
			MODEMDEVICE_NAME = new char[sizeof(MODEMDEVICE1)];
			memset(MODEMDEVICE_NAME, 0, sizeof(MODEMDEVICE1));
			strcpy(MODEMDEVICE_NAME,MODEMDEVICE1);
			status_publish_name = new char[sizeof("pos_status_topic2")];
			memset(status_publish_name, 0, sizeof("pos_status_topic2"));
			strcpy(status_publish_name,"pos_status_topic2");
			rewrite_topic_name = new char[sizeof("pos_write_topic2")];
			memset(rewrite_topic_name, 0, sizeof("pos_write_topic2"));
			strcpy(rewrite_topic_name,"pos_write_topic2");
			velocity_feedback_name = new char[sizeof("velocity_feedback2")];
			memset(velocity_feedback_name, 0, sizeof("velocity_feedback2"));
			strcpy(velocity_feedback_name,"velocity_feedback2");	
			attitude_feedback_name = new char[sizeof("attitude_feedback2")];
			memset(attitude_feedback_name, 0, sizeof("attitude_feedback2"));
			strcpy(attitude_feedback_name,"attitude_feedback2");	
			body_attitude_name = new char[sizeof("body_attitude2")];
			memset(body_attitude_name, 0, sizeof("body_attitude2"));
			strcpy(body_attitude_name,"body_attitude2");
			position_setpoint_name = new char[sizeof("position_setpoint2")];
			memset(position_setpoint_name, 0, sizeof("position_setpoint2"));
			strcpy(position_setpoint_name,"position_setpoint2");
	   }
 
   if (global_id==3)
	   {
		   	param_xd = new char[sizeof("my_quadrotor3/x_d")];
			memset(param_xd, 0, sizeof("my_quadrotor3/x_d"));
			strcpy(param_xd,"my_quadrotor3/x_d");
			param_yd = new char[sizeof("my_quadrotor3/y_d")];
			memset(param_yd, 0, sizeof("my_quadrotor3/y_d"));
			strcpy(param_yd,"my_quadrotor3/y_d");
			param_zd = new char[sizeof("my_quadrotor3/z_d")];
			memset(param_zd, 0, sizeof("my_quadrotor3/z_d"));
			strcpy(param_zd,"my_quadrotor3/z_d");
			param_yawd = new char[sizeof("my_quadrotor3/yaw_d")];
			memset(param_yawd, 0, sizeof("my_quadrotor3/yaw_d"));
			strcpy(param_yawd,"my_quadrotor3/yaw_d");

			param_vel_ff_x = new char[sizeof("my_quadrotor3/vel_ff_x")];
			memset(param_vel_ff_x, 0, sizeof("my_quadrotor3/vel_ff_x"));
			strcpy(param_vel_ff_x,"my_quadrotor3/vel_ff_x");
			param_vel_ff_y = new char[sizeof("my_quadrotor3/vel_ff_y")];
			memset(param_vel_ff_y, 0, sizeof("my_quadrotor3/vel_ff_y"));
			strcpy(param_vel_ff_y,"my_quadrotor3/vel_ff_y");
			param_vel_ff_z = new char[sizeof("my_quadrotor3/vel_ff_z")];
			memset(param_vel_ff_z, 0, sizeof("my_quadrotor3/vel_ff_z"));
			strcpy(param_vel_ff_z,"my_quadrotor3/vel_ff_z");
			MODEMDEVICE_NAME = new char[sizeof(MODEMDEVICE2)];
			memset(MODEMDEVICE_NAME, 0, sizeof(MODEMDEVICE2));
			strcpy(MODEMDEVICE_NAME,MODEMDEVICE2);
			status_publish_name = new char[sizeof("pos_status_topic3")];
			memset(status_publish_name, 0, sizeof("pos_status_topic3"));
			strcpy(status_publish_name,"pos_status_topic3");
			rewrite_topic_name = new char[sizeof("pos_write_topic3")];
			memset(rewrite_topic_name, 0, sizeof("pos_write_topic3"));
			strcpy(rewrite_topic_name,"pos_write_topic3");
			velocity_feedback_name = new char[sizeof("velocity_feedback3")];
			memset(velocity_feedback_name, 0, sizeof("velocity_feedback3"));
			strcpy(velocity_feedback_name,"velocity_feedback3");	
			attitude_feedback_name = new char[sizeof("attitude_feedback3")];
			memset(attitude_feedback_name, 0, sizeof("attitude_feedback3"));
			strcpy(attitude_feedback_name,"attitude_feedback3");	
			body_attitude_name = new char[sizeof("body_attitude3")];
			memset(body_attitude_name, 0, sizeof("body_attitude3"));
			strcpy(body_attitude_name,"body_attitude3");
			position_setpoint_name = new char[sizeof("position_setpoint3")];
			memset(position_setpoint_name, 0, sizeof("position_setpoint3"));
			strcpy(position_setpoint_name,"position_setpoint3");
	   }
	   if (global_id==4)
	   {
		   	param_xd = new char[sizeof("my_quadrotor4/x_d")];
			memset(param_xd, 0, sizeof("my_quadrotor4/x_d"));
			strcpy(param_xd,"my_quadrotor4/x_d");
			param_yd = new char[sizeof("my_quadrotor4/y_d")];
			memset(param_yd, 0, sizeof("my_quadrotor4/y_d"));
			strcpy(param_yd,"my_quadrotor4/y_d");
			param_zd = new char[sizeof("my_quadrotor4/z_d")];
			memset(param_zd, 0, sizeof("my_quadrotor4/z_d"));
			strcpy(param_zd,"my_quadrotor4/z_d");
			param_yawd = new char[sizeof("my_quadrotor4/yaw_d")];
			memset(param_yawd, 0, sizeof("my_quadrotor4/yaw_d"));
			strcpy(param_yawd,"my_quadrotor4/yaw_d");

			param_vel_ff_x = new char[sizeof("my_quadrotor4/vel_ff_x")];
			memset(param_vel_ff_x, 0, sizeof("my_quadrotor4/vel_ff_x"));
			strcpy(param_vel_ff_x,"my_quadrotor4/vel_ff_x");
			param_vel_ff_y = new char[sizeof("my_quadrotor4/vel_ff_y")];
			memset(param_vel_ff_y, 0, sizeof("my_quadrotor4/vel_ff_y"));
			strcpy(param_vel_ff_y,"my_quadrotor4/vel_ff_y");
			param_vel_ff_z = new char[sizeof("my_quadrotor4/vel_ff_z")];
			memset(param_vel_ff_z, 0, sizeof("my_quadrotor4/vel_ff_z"));
			strcpy(param_vel_ff_z,"my_quadrotor4/vel_ff_z");
			MODEMDEVICE_NAME = new char[sizeof(MODEMDEVICE3)];
			memset(MODEMDEVICE_NAME, 0, sizeof(MODEMDEVICE3));
			strcpy(MODEMDEVICE_NAME,MODEMDEVICE3);
			status_publish_name = new char[sizeof("pos_status_topic4")];
			memset(status_publish_name, 0, sizeof("pos_status_topic4"));
			strcpy(status_publish_name,"pos_status_topic4");
			rewrite_topic_name = new char[sizeof("pos_write_topic4")];
			memset(rewrite_topic_name, 0, sizeof("pos_write_topic4"));
			strcpy(rewrite_topic_name,"pos_write_topic4");
			velocity_feedback_name = new char[sizeof("velocity_feedback4")];
			memset(velocity_feedback_name, 0, sizeof("velocity_feedback4"));
			strcpy(velocity_feedback_name,"velocity_feedback4");	
			attitude_feedback_name = new char[sizeof("attitude_feedback4")];
			memset(attitude_feedback_name, 0, sizeof("attitude_feedback4"));
			strcpy(attitude_feedback_name,"attitude_feedback4");	
			body_attitude_name = new char[sizeof("body_attitude4")];
			memset(body_attitude_name, 0, sizeof("body_attitude4"));
			strcpy(body_attitude_name,"body_attitude4");
			position_setpoint_name = new char[sizeof("position_setpoint4")];
			memset(position_setpoint_name, 0, sizeof("position_setpoint4"));
			strcpy(position_setpoint_name,"position_setpoint4");
	   }
//--------------------------------------------------------------------------
	ros::init(argc, argv, "demo_gcs_API");
    ros::NodeHandle n;
    myGroundCS myGCS;
    while(ros::ok())
	    {
			if(argc != 7)
			    {
			        ROS_ERROR("missing arguments\n");
			        //return -1;
			    }
		    else
			    {
				
				    if(!myGCS.setup_port_flag)
					    {   
					    	ROS_ERROR("PLEASE REPLUGIN USB ");
					    	myGCS.fd_setup_port=myGCS.open_port(MODEMDEVICE_NAME);
								if (myGCS.fd_setup_port < 0)
									{
										myGCS.setup_port_flag=false;
									}
								else
									{
										if (myGCS.setup_port(myGCS.fd_setup_port) < 0)
											{
												myGCS.setup_port_flag=false;
											}
										else
											{
											ROS_INFO("%s is open...\n", MODEMDEVICE_NAME);
											myGCS.setup_port_flag=true;
											}
								    }
						
						}
					else
						{
							break;
						}
				}
				sleep(2);
	    }
    memset(&xyz, 0,sizeof(xyz));
    memset(&xyz_old, 0,sizeof(xyz_old));

    //ros init
    ROS_INFO("aaaaa");
    ros::Subscriber sub = n.subscribe("demo_udp", 1000, &myGroundCS::udpcallback,&myGCS);
    ros::Subscriber sub2 = n.subscribe(rewrite_topic_name, 4000, &myGroundCS::writecallback,&myGCS);
    //read trajectory data from file argv[1]
    ROS_INFO("argv[0]:%s",argv[0]);
    ROS_INFO("argv[1]:%s",argv[1]);
    ROS_INFO("argv[2]:%s",argv[2]);
    ROS_INFO("argv[3]:%s",argv[3]);
    ROS_INFO("argv[4]:%s",argv[4]);
    ROS_INFO("argv[5]:%s",argv[5]);
    ROS_INFO("argv[6]:%s",argv[6]);
    ROS_INFO("argv[7]:%s",argv[7]);
    //transfor init data
    boost::mutex::scoped_lock possp_lock(pos_sp_mutex);
	    ref_data[0] = atof(argv[1]);
	    ref_data[1] = atof(argv[2]);
	    ref_data[2] = atof(argv[3]);
	    ref_data[3] = atof(argv[4]);

		ref_name = new char[sizeof(argv[5])];
		memset(ref_name, 0, sizeof(argv[5]));
		strcpy(ref_name,argv[5]);

		global_id = atoi(argv[6]);

	possp_lock.unlock();
    //motion capture postion data
    demo_test::pos_data temp_xyz;
    memset(&temp_xyz, 0, sizeof(temp_xyz));
    
    int i = 0;
    double s1; 
    double s2;
    double s3;
    double s4;
 
	n.setParam(param_xd, atof(argv[1]));
	n.setParam(param_yd, atof(argv[2]));
	n.setParam(param_zd, atof(argv[3]));
	n.setParam(param_yawd, atof(argv[4]));
	n.setParam(param_vel_ff_x, vel_ff_x);
	n.setParam(param_vel_ff_y, vel_ff_y);
	n.setParam(param_vel_ff_z, vel_ff_z);

//system("rosrun dynamic_reconfigure dynparam set_from_parameters demo_gcs_API my_quadrotor1/x_d 1");
	//status publish init
	status_upd_sub=n.advertise<demo_test::pos_status>(status_publish_name,2000);
	/////
	attitude_feedback_sub=n.advertise<demo_test::attitude_feedback>(attitude_feedback_name,4000);
	velocity_feedback_sub=n.advertise<demo_test::velocity_feedback>(velocity_feedback_name,4000);
	body_attitude_sub=n.advertise<demo_test::body_attitude>(body_attitude_name,4000);
	position_setpoint_sub=n.advertise<demo_test::position_setpoint>(position_setpoint_name,4000);
	//send thread init

	boost::thread send_current(boost::bind(&myGroundCS::send_current_pos,&myGCS,myGCS.fd_setup_port));
	boost::thread send_desire(boost::bind(&myGroundCS::send_desire_pos,&myGCS,myGCS.fd_setup_port));
	// boost::thread send_type(boost::bind(&myGroundCS::send_desire_type,&myGCS,myGCS.fd_setup_port));
	

	//receive thread init
	boost::thread receive_attitude(boost::bind(&myGroundCS::receive_attitude,&myGCS,myGCS.fd_setup_port));


	boost::thread status_publish(&myGroundCS::status_publish_thrd,&myGCS);
    
    ros::AsyncSpinner s(3);
    s.start();
    double ss;

    ros::Rate loop_rate(10);
    while(ros::ok())
		{				
			boost::mutex::scoped_lock lock_dpos(pos_sp_mutex);
			boost::mutex::scoped_lock lock_pos(pos_mutex);
			boost::mutex::scoped_lock lock_type(type_mutex);
			if (write_has_chang==0)
				{
					if (n.getParam(param_xd,ss))
						{
							ROS_DEBUG("Got x_d: %.2f",(double)ss);
							ref_data[0]=(float)ss;
						}
					else
						{
							ROS_ERROR("x_dFAIL!!");
							n.setParam(param_xd, (double)ref_data[0]);
						}

					if (n.getParam(param_yd,ss))
						{
							ROS_DEBUG("Got y_d: %.2f",(double)ss);
							ref_data[1]=(float)ss;
						}
					else
						{
							ROS_ERROR("y_d FAIL!!");
							n.setParam(param_yd, (double)ref_data[1]);
						}

					if (n.getParam(param_zd,ss))
						{
							ROS_DEBUG("Got z_d: %.2f",(double)ss);
							ref_data[2]=(float)ss;
						}
					else
						{
							ROS_ERROR("z_d FAIL!!");
							n.setParam(param_zd, (double)ref_data[2]);
						}

					if (n.getParam(param_yawd,ss))
						{
							ROS_DEBUG("Got yaw_d: %.2f",(double)ss);
							ref_data[3]=(float)ss;
						}
					else
						{
							ROS_ERROR("yaw_d FAIL!!");
							n.setParam(param_yawd, (double)ref_data[3]);
						}
					if (n.getParam(param_vel_ff_x,ss))
						{
							ROS_DEBUG("Got vel_ff_x: %.2f",(double)ss);
							vel_ff_x=(float)ss;
						}
					else
						{
							ROS_ERROR("vel_ff_x FAIL!!");
							n.setParam(param_vel_ff_x, (double)vel_ff_x);
						}
					if (n.getParam(param_vel_ff_y,ss))
						{
							ROS_DEBUG("Got vel_ff_y: %.2f",(double)ss);
							vel_ff_y=(float)ss;
						}
					else
						{
							ROS_ERROR("vel_ff_y FAIL!!");
							n.setParam(param_vel_ff_y, (double)vel_ff_y);
						}
					if (n.getParam(param_vel_ff_z,ss))
						{
							ROS_DEBUG("Got vel_ff_z: %.2f",(double)ss);
							vel_ff_z=(float)ss;
						}
					else
						{
							ROS_ERROR("vel_ff_z FAIL!!");
							n.setParam(param_vel_ff_z, (double)vel_ff_z);
						}
				}
			else
				{
					n.setParam(param_xd, (double)ref_data[0]);
					n.setParam(param_yd, (double)ref_data[1]);
				    n.setParam(param_zd, (double)ref_data[2]);
					n.setParam(param_yawd, (double)ref_data[3]);
					n.setParam(param_vel_ff_x, vel_ff_x);
					n.setParam(param_vel_ff_y, vel_ff_y);
					n.setParam(param_vel_ff_z, vel_ff_z);
					write_has_chang=0;
				}
			switch((int)global_type_temp.type)
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
					default:
						break;
				}	
				ref_status2 = (char*)ref_status1.data();	
    		ROS_INFO_THROTTLE(1,"RUNING... \n");
			ROS_INFO_THROTTLE(1,"type : %s \n",ref_status2);
			ROS_INFO_THROTTLE(1,"sending position setpoint to uav<%d>: x_d=%.4f, y_d=%.4f, z_d=%.4f, yaw_d=%.4f\n",global_id,ref_data[0],ref_data[1], ref_data[2], ref_data[3]);
			ROS_INFO_THROTTLE(1,"sending position_feedback to uav<%d>: x=%.4f, y=%.4f, z=%.4f\n",global_id, xyz.pos[0+3*(global_id-1)], xyz.pos[2+3*(global_id-1)], -xyz.pos[1+3*(global_id-1)]);  
			ROS_INFO_THROTTLE(1,"sending freq:  feedback = %.1f HZ, setpoint= %.1f HZ, type&flag= %.1f HZ\n", send_currentpos_freq, send_desirepos_freq,send_type_freq);     
			lock_dpos.unlock();
			lock_pos.unlock();
			lock_type.unlock();
			loop_rate.sleep();
		}
    return 0;
}
