#include <stdio.h>
#include <winsock2.h>

#pragma comment(lib,"ws2_32.lib")

int main(int argc, char* argv[])
{
    //初始化WSA
    WORD sockVersion = MAKEWORD(2,2);
    WSADATA wsaData;
    if(WSAStartup(sockVersion, &wsaData)!=0)
    {
        return 0;
    }

    //创建套接字
    SOCKET slisten = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(slisten == INVALID_SOCKET)
    {
        printf("socket error !");
        return 0;
    }

    //绑定IP和端口
    sockaddr_in sin;
    sin.sin_family = AF_INET;
    sin.sin_port = htons(9988);//监听9988端口
    sin.sin_addr.S_un.S_addr = inet_addr("192.168.1.104");//绑定到本机Ip
	//开始绑定
    if(bind(slisten, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR)
    {
        printf("bind error !");
    }

    //开始监听
    if(listen(slisten, 5) == SOCKET_ERROR)
    {
        printf("listen error !");
        return 0;
    }

    //循环接收数据
    SOCKET sClient;
    sockaddr_in remoteAddr;
    int nAddrlen = sizeof(remoteAddr);
	char revData[255]={0}; 
    while (true)
    {
        printf("等待客户端连接...\n");
		if(sClient = accept(slisten, (SOCKADDR *)&remoteAddr, &nAddrlen))
		{
			if(sClient == INVALID_SOCKET)
			{
				printf("客户端连接错误，accept error !");
				continue;
			 }
			printf("接受到一个客户端连接：%s \r\n", inet_ntoa(remoteAddr.sin_addr));
        
    /*    //接收数据
        int ret = recv(sClient, revData, 255, 0);        
        if(ret > 0)
        {
            revData[ret] = 0x00;
            printf(revData);
        }*/
		//接收数据2
			int recv_num,recv_num_total=0;
			char recv_buf[50];
			while(true)
			{   

				
				//char* mymsg="ABC";
				//gets(mymsg);
				//send(sClient, mymsg, strlen(mymsg), NULL);
				//recv(sClient, revData, 255, NULL);            //一直接收客户端socket的send操作
				//printf("***Client***    %s\n", revData);
				
				
				memset(recv_buf,0,sizeof(recv_buf));		// 清空一下recv_buf缓存区
				recv_num=recv(sClient,recv_buf,26,0);
				/*if (recv_num<0)
					printf("服务器端接收失败！\n");*/
				if(recv_num>0)
				{
					recv_num_total+=recv_num;
					printf("服务器端：调用接收成功！内容为：%s\n共收到%d个字节的数据。\n",recv_buf,recv_num_total);
					 //发送数据
        			////char * sendData = "你好，TCP客户端！\n";
		
                   //// send(sClient, sendData, strlen(sendData), 0);
            
				}
				
			}
        //发送数据
        char * sendData = "你好，TCP客户端！\n";
		
        send(sClient, sendData, strlen(sendData), 0);
			
		}
	}
	printf("服务器端已经断开\n");
	closesocket(sClient);
    closesocket(slisten);
    WSACleanup();     
    return 0;
}