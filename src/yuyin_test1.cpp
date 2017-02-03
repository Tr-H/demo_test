#include "ros/ros.h"
#include <stdio.h>  
#include <sys/types.h>  
#include <sys/socket.h>  
#include <string.h>  
#include <stdlib.h>  
#include <sys/un.h>  
#include <pthread.h>  
#include <arpa/inet.h>
#define MCAST_PORT 9988
#define INADDR_ANY2 "192.168.1.100"
const char *sendData = "hello test1\n";	
int main(int argc, char* argv[])
{
	ros::init(argc, argv, "demo_yuyin");
	ros::NodeHandle n1;
	struct sockaddr_in server_addr;
	memset(&server_addr,0,sizeof(server_addr));
	server_addr.sin_family=AF_INET;             // 设置域为IPV4  
    server_addr.sin_addr.s_addr=inet_addr(INADDR_ANY2);     // 绑定到 INADDR_ANY 地址  
    server_addr.sin_port=htons(MCAST_PORT);           // 注意端口转换  
    int sockfd = socket(AF_INET,SOCK_STREAM,0);
    if (sockfd < 0)
    {
    	ROS_INFO("SOCKET ERROR!");
    	return -1;
    }
    int sock_bind = bind(sockfd,(struct sockaddr *)(&server_addr),sizeof(server_addr));
    if (sock_bind < 0)
    {	
    	ROS_INFO("BIND ERROR!");
    	return -1;
    }
    int sock_listen = listen(sockfd,4);
    if (sock_listen < 0)
    {	
    	ROS_INFO("LISTEN ERROR!");
    	return -1;
    }
   ROS_INFO("wait for server connection");
    int newfd = accept(sockfd,NULL,NULL);
    if (newfd < 0)
    {
    	ROS_INFO("accept ERROR");
    	return -1;
    }
    int recv_num,recv_num_total = 0;
    char recv_buf[50];
    char code_forward[50]="00\n";
    char code_back[50]="01\n";
    char code_left[50]="02\n";
    char code_right[50]="03\n";
    char code_up[50]="04\n";
    char code_down[50]="05\n";
    // ros::Rate loop_rate(200);
    while(ros::ok())
    {
	// loop_rate.sleep();
    memset(recv_buf,0,sizeof(recv_buf));
    recv_num=recv(newfd,recv_buf,50,0);  
     if (recv_num<0)  
            ROS_INFO("receive wrong\n");  
        else if(recv_num>0)  
        {  
            recv_num_total+=recv_num;  
            ROS_INFO("client:connect successed reive: %s \nabout %d num of date\n",recv_buf,recv_num_total);  
            sync();  
           
            send(newfd,sendData, strlen(sendData), 0);
        }  
        else  
        {  
            ROS_INFO("interrapte of connection\n");  
            ROS_INFO("wait for client connection\n");  
            newfd=accept(sockfd,NULL,NULL);  
        }
        
        if (strcmp(recv_buf,code_forward) == 0) 
        {
            ROS_INFO("code_forward runing");
        }
        if (strcmp(recv_buf,code_back) == 0) 
        {
            ROS_INFO("code_back runing");
        }
        if (strcmp(recv_buf,code_left) == 0) 
        {
            ROS_INFO("code_left runing");
        }
        if (strcmp(recv_buf,code_right) == 0) 
        {
            ROS_INFO("code_right runing");
        }
        if (strcmp(recv_buf,code_up) == 0) 
        {
            ROS_INFO("code_up runing");
        }
        if (strcmp(recv_buf,code_down) == 0) 
        {
            ROS_INFO("code_down runing");
        }
    usleep(200);
    }

}