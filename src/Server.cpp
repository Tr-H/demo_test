#include <stdio.h>
#include <winsock2.h>

#pragma comment(lib,"ws2_32.lib")

int main(int argc, char* argv[])
{
    //��ʼ��WSA
    WORD sockVersion = MAKEWORD(2,2);
    WSADATA wsaData;
    if(WSAStartup(sockVersion, &wsaData)!=0)
    {
        return 0;
    }

    //�����׽���
    SOCKET slisten = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(slisten == INVALID_SOCKET)
    {
        printf("socket error !");
        return 0;
    }

    //��IP�Ͷ˿�
    sockaddr_in sin;
    sin.sin_family = AF_INET;
    sin.sin_port = htons(9988);//����9988�˿�
    sin.sin_addr.S_un.S_addr = inet_addr("192.168.1.104");//�󶨵�����Ip
	//��ʼ��
    if(bind(slisten, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR)
    {
        printf("bind error !");
    }

    //��ʼ����
    if(listen(slisten, 5) == SOCKET_ERROR)
    {
        printf("listen error !");
        return 0;
    }

    //ѭ����������
    SOCKET sClient;
    sockaddr_in remoteAddr;
    int nAddrlen = sizeof(remoteAddr);
	char revData[255]={0}; 
    while (true)
    {
        printf("�ȴ��ͻ�������...\n");
		if(sClient = accept(slisten, (SOCKADDR *)&remoteAddr, &nAddrlen))
		{
			if(sClient == INVALID_SOCKET)
			{
				printf("�ͻ������Ӵ���accept error !");
				continue;
			 }
			printf("���ܵ�һ���ͻ������ӣ�%s \r\n", inet_ntoa(remoteAddr.sin_addr));
        
    /*    //��������
        int ret = recv(sClient, revData, 255, 0);        
        if(ret > 0)
        {
            revData[ret] = 0x00;
            printf(revData);
        }*/
		//��������2
			int recv_num,recv_num_total=0;
			char recv_buf[50];
			while(true)
			{   

				
				//char* mymsg="ABC";
				//gets(mymsg);
				//send(sClient, mymsg, strlen(mymsg), NULL);
				//recv(sClient, revData, 255, NULL);            //һֱ���տͻ���socket��send����
				//printf("***Client***    %s\n", revData);
				
				
				memset(recv_buf,0,sizeof(recv_buf));		// ���һ��recv_buf������
				recv_num=recv(sClient,recv_buf,26,0);
				/*if (recv_num<0)
					printf("�������˽���ʧ�ܣ�\n");*/
				if(recv_num>0)
				{
					recv_num_total+=recv_num;
					printf("�������ˣ����ý��ճɹ�������Ϊ��%s\n���յ�%d���ֽڵ����ݡ�\n",recv_buf,recv_num_total);
					 //��������
        			////char * sendData = "��ã�TCP�ͻ��ˣ�\n";
		
                   //// send(sClient, sendData, strlen(sendData), 0);
            
				}
				
			}
        //��������
        char * sendData = "��ã�TCP�ͻ��ˣ�\n";
		
        send(sClient, sendData, strlen(sendData), 0);
			
		}
	}
	printf("���������Ѿ��Ͽ�\n");
	closesocket(sClient);
    closesocket(slisten);
    WSACleanup();     
    return 0;
}