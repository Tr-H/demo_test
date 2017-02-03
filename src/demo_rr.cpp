#include "ros/ros.h"
#include "std_msgs/String.h"

#include <boost/thread.hpp>
#include "checksum.h"

#include <stdio.h>
#include <strings.h>
#include <sys/types.h>
#include <fcntl.h> 
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#define BAUDRATE B57600
#define MODEMDEVICE "/dev/ttyUSB0"
#define PI 3.1415926 

static unsigned char  BUF[256];
static unsigned char  buf[256];  

class myGroundCS 
{
public:
    myGroundCS();
    int open_port(const char * tty_port);
    int setup_port(int fd);
    void recieve(const int& fd); 
    void recieve1(const int& fd); 
    
    void jiexi();
    void jiexi1();
    
    bool setup_port_flag;
    int fd_setup_port; 
    unsigned char jiaoyan,jiaoyan1; 
    int i;
    int num_1;
    int num_2;
    
    float jiao,speed;
    float *temp;
    const uint8_t *temp1;
    float roll,pitch,yaw,rollspeed,pitchspeed,yawspeed;
    float x,y,z,vx,vy,vz;

private:
};
myGroundCS::myGroundCS()
{
    setup_port_flag=true;
    fd_setup_port=myGroundCS::open_port(MODEMDEVICE);
    if (fd_setup_port < 0)
    {
        setup_port_flag=false;
    }else{
    if (myGroundCS::setup_port(fd_setup_port) < 0)
    {
        setup_port_flag=false;
    }else
    {
    printf("%s is open...\n", MODEMDEVICE);
    }
    }
    int i=0;
    
    

}

int myGroundCS::open_port(const char * tty_port)
{
    int fd;
    fd = open(tty_port, O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        printf("open tty error\n");
        return (-1);
    }
    else
    {
        printf("open %s successfully\n", tty_port);
    }
    if (!isatty(fd))
    {
        printf("%s is not a serial port\n", tty_port);
        return (-1);
    }
    return (fd);
}

int myGroundCS::setup_port(int fd)
{
    struct termios config;
    if (tcgetattr(fd, &config) < 0)
    {
        printf("get serial pot attribute error\n");
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
        printf("\nERROR: Could not set desired baud rate of %d Baud\n", BAUDRATE);
        return (-1);
    }
   if (tcsetattr(fd, TCSAFLUSH, &config) < 0)
    {
        fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
        return (-1);
    }
    return 1;
}

void myGroundCS::jiexi()
{                      
        i=6;           
        temp = (float*)&BUF[i+4];               
        jiao=*temp;
        roll=jiao*180/PI;
        temp = (float*)&BUF[i+8];                 
        jiao=*temp;
        pitch=jiao*180/PI;                  
        temp = (float*)&BUF[i+12];                  
        jiao=*temp;
        yaw=jiao*180/PI;                  
        temp = (float*)&BUF[i+16];              
        speed=*temp;
        rollspeed=speed*180/PI;                 
        temp = (float*)&BUF[i+20];
        speed=*temp;
        pitchspeed=speed*180/PI;
        temp = (float*)&BUF[i+24];
        speed=*temp;  
        yawspeed=speed*180/PI;            
        fflush(stdout);                             
}

void myGroundCS::jiexi1()
{                     
      i=6;           
      temp = (float*)&BUF[i+4];               
      x=*temp;
      temp = (float*)&BUF[i+8];                 
      y=*temp;                  
      temp = (float*)&BUF[i+12];                  
      z=*temp;                  
      temp = (float*)&BUF[i+16];              
      vx=*temp;                 
      temp = (float*)&BUF[i+20];
      vy=*temp;
      temp = (float*)&BUF[i+24];  
      vz=*temp;
      fflush(stdout);                             
}

void myGroundCS::recieve(const int& fd)
{
  int num_1=0;
  while(1)
  {        
    if(BUF[0]==0XFE)
    {             
      BUF[34]=0x27;
      temp1 = (const uint8_t*)&BUF[1];
      uint16_t crcTmp = crc_calculate(temp1,34);
      uint8_t ck[2];
      //ck[0] = (uint8_t)(crcTmp & 0xFF);
      ck[1] = (uint8_t)(crcTmp >> 8);
      if(BUF[35]==ck[1])
      {         
        jiexi();

        printf("\nnum_1:%d\n",num_1++);   
        printf("\nseq:%02X\n",BUF[2]);
        printf("\nroll:%f pitch:%f yaw:%f \n",roll,pitch,yaw);
        printf("rollspeed:%f pitchspeed:%f yawspeed:%f \n",rollspeed,pitchspeed,yawspeed);
        //printf("x:%f y:%f z:%f \n",x,y,z);
        //printf("vx:%f vy:%f vz:%f \n",vx,vy,vz);          
      }
      BUF[0]=0;//无重复数据，一秒10个左右
      fflush(stdout);//刷新屏幕     
    } 
  } 
}

void myGroundCS::recieve1(const int& fd)
{
  int num_2=0;
  while(1)
  {        
    if(BUF[0]==0XFE)
    {             
      BUF[34]=0xB9;
      temp1 = (const uint8_t*)&BUF[1];
      uint16_t crcTmp = crc_calculate(temp1,34);
      uint8_t ck[2];
      //ck[0] = (uint8_t)(crcTmp & 0xFF);
      ck[1] = (uint8_t)(crcTmp >> 8);
      if(BUF[35]==ck[1])
      {         
        jiexi1();
        printf("\nnum_2:%d\n",num_2++);   
        printf("\nseq:%02X\n",BUF[2]);
        //printf("\nroll:%f pitch:%f yaw:%f \n",roll,pitch,yaw);
        //printf("rollspeed:%f pitchspeed:%f yawspeed:%f \n",rollspeed,pitchspeed,yawspeed);
        printf("x:%f y:%f z:%f \n",x,y,z);
        printf("vx:%f vy:%f vz:%f \n",vx,vy,vz);          
      }
      BUF[0]=0;//无重复数据，一秒10个左右
      fflush(stdout);//刷新屏幕     
    } 
  } 
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "demo_gcs9");
  ros::NodeHandle n;
  ROS_INFO("recieve begin!");

  int nread=0,ii=0;
  myGroundCS myGCS;
    if(!myGCS.setup_port_flag)
    {  
        return -1;
    }
  //tcflush(myGCS.fd_setup_port, TCIOFLUSH);
boost::thread recieve(boost::bind(&myGroundCS::recieve,&myGCS,myGCS.fd_setup_port));//new pthread
boost::thread recieve1(boost::bind(&myGroundCS::recieve1,&myGCS,myGCS.fd_setup_port));//new pthread
ros::AsyncSpinner s(3);
s.start();
//ros::Rate loop_rate(1);

  while (ros::ok())
  {

    nread=read(myGCS.fd_setup_port,buf,1);       
    if(nread==1&&buf[0]==0xFE)
    {                      
        BUF[0]=*buf;
        ii=0;
    }
    else if(nread==1&&buf[0]!=0xFE)
    {
      ii++;
      if(ii>=100)
      {
          ii=100;
      }                       
      BUF[ii]=*buf;
    } 
    /*for(i=0;i<100;i++)
    {
      printf("%02x\t",BUF[i]);
    }*/                     
  }
  close (myGCS.fd_setup_port);
  return 0;
}