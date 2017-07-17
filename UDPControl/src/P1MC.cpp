/* Author: Howard Dong */

//
// *********     P1 UDP Control & Receive      *********
//
//
//

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>


#include <stdio.h>
#include <signal.h>
#include <sys/mman.h>
#include <memory.h>
#include <sys/ioctl.h>  
#include <sys/types.h>  
#include <sys/stat.h>  
#include <fcntl.h>  
#include <linux/rtc.h>  
#include <errno.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <sys/time.h>  
#include <unistd.h>  
#include <math.h>
#include <pthread.h>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/stat.h>
#include <arpa/inet.h>
int ID = 0;

int getch()
{
#ifdef __linux__
	struct termios oldt, newt;
	int ch;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
#elif defined(_WIN32) || defined(_WIN64)
	return _getch();
#endif
}


void catch_signal(int sig)
{

}

int kbhit(void)
{
#ifdef __linux__
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
#elif defined(_WIN32) || defined(_WIN64)
	return _kbhit();
#endif
}


/////////////////////////////////////Socket Init /////////////////////////////////////////////
typedef struct {
	char robotID;
	float OutputPacket[5];//the content of this packet
	float InputPacket[160];
	bool ReceiveExecuted;//updated means that receiver can receive information
	bool SendExecuted;//true means that receiver has received the information,and the sender can send;false means the sender is sending
	int outputSocket;
	int inputSocket;
	int port;
	char buffer[256];
	float fPos;
	float fVel;
	float fGoal;
	bool SocketEnable;
}Broadcast;
/*
Output Packet Structure
FLOAT[0]:1-ZERO,2-STAND,3-WALK
FLOAT[1]~FLOAT[3] to be defined

Input Packet Structure
FLOAT0: time
FLOAT1:  dxl_1_Goal
FLOAT2:  dxl_1_Pos
FLOAT3:  dxl_1_Vel
FLOAT4:  dxl_2_Goal
FLOAT5:  dxl_2_Pos
FLOAT6:  dxl_2_Vel
FLOAT7:  dxl_3_Goal
FLOAT8:  dxl_3_Pos
FLOAT9:  dxl_3_Vel
FLOAT10: dxl_4_Goal
FLOAT11: dxl_4_Pos
FLOAT12: dxl_4_Vel
FLOAT13: dxl_5_Goal
FLOAT14: dxl_5_Pos
FLOAT15: dxl_5_Vel
FLOAT16: dxl_6_Goal
FLOAT17: dxl_6_Pos
FLOAT18: dxl_6_Vel
FLOAT19: ati_1_Fx
FLOAT20: ati_1_Fy
FLOAT21: ati_1_Fz
FLOAT22: ati_1_Tx
FLOAT23: ati_1_Ty
FLOAT24: ati_1_Tz
FLOAT25: ati_2_Fx
FLOAT26: ati_2_Fy
FLOAT27: ati_2_Fz
FLOAT28: ati_2_Tx
FLOAT29: ati_2_Ty
FLOAT30: ati_2_Tz
FLOAT31: mti_Yaw
FLOAT32: mti_Pitch
FLOAT33: mti_Roll
*/

bool bIsRecData = false;
Broadcast broadcast;
int outputPort=51655;
int inputPort=51658;
int UDPSocket(int port){
	int sockfd;							//定义socket套接字
	struct sockaddr_in sin; 					//定义网络套接字地址结构
	bzero(&sin,sizeof(sin));						//地址结构清零
	sin.sin_family = AF_INET;					//指定使用的通讯协议族
	sin.sin_addr.s_addr = htonl(INADDR_ANY);			//指定接受任何连接
	sin.sin_port = htons(port);					//指定监听的端口
	sockfd = socket(AF_INET,SOCK_DGRAM,0);			//创建UDP套接字
	bind(sockfd,(struct sockaddr *)&sin,sizeof(sin));		//给套接字绑定端口	
	return sockfd;
}

int UDPSend(int sock,float* buffer,int size,char* IP,int port){
	struct sockaddr_in address;

	memset(&address, 0, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_port = htons(port);
	address.sin_addr.s_addr = inet_addr(IP);
	int count=sendto(sock,buffer,size,0,(struct sockaddr *)&address,sizeof(address));
	return count;
}

int UDPRcv(int sock,float* buffer,int size){
	struct sockaddr_in sin;
	int sinlen = sizeof(sin);	
	int count=recvfrom(sock,buffer,size,0, (struct sockaddr *)&sin,(socklen_t*)&sinlen);
	return count;
}

void SetSocketOption(int socket){
	struct linger {
		u_short l_onoff;
		u_short l_linger;
	};
	linger m_sLinger;
	m_sLinger.l_onoff=1;	//(在closesocket()调用,但是还有数据没发送完毕的时候容许逗留)	
	m_sLinger.l_linger=5;	//(容许逗留的时间为5秒)

	int fBroadcast=1;
//	setsockopt(socket,SOL_SOCKET,SO_LINGER,(const char*)&m_sLinger,sizeof(linger));
	setsockopt(socket,SOL_SOCKET,SO_BROADCAST,&fBroadcast,sizeof(fBroadcast));   
}

void InitComm()
{
	broadcast.port=51655;


	broadcast.outputSocket=UDPSocket(outputPort);
	broadcast.inputSocket=UDPSocket(inputPort);
	if(broadcast.outputSocket==-1){
		printf("Communication: Create Output socket error!\n");
		broadcast.SocketEnable=false;
	}
	else broadcast.SocketEnable=true;

	SetSocketOption(broadcast.outputSocket);
	SetSocketOption(broadcast.inputSocket);
}

void SendData()
{
	int count=-1;	
	count= UDPSend(broadcast.outputSocket,broadcast.OutputPacket,sizeof(broadcast.OutputPacket),"127.0.0.1",51654);	
	// count= UDPSend(broadcast.outputSocket,broadcast.OutputPacket,sizeof(broadcast.OutputPacket),"192.168.0.101",51654);	

	// count= UDPSend(broadcast.outputSocket,broadcast.OutputPacket,sizeof(broadcast.OutputPacket),"101.5.220.130",51654);	

	if(count ==-1)printf("Communication: UDPSend Error!\n"); 

}

//////////////////////////////////////////////////////////////////////////////////////////////



#define ESC_ASCII_VALUE                 0x1b
#define ENTER_ASCII_VALUE               0xd

FILE *fp;

int flag_comm_thread_stop = 0;


void* listen(void * data)
{
	printf("Communication: Listening..\n");

	if( ( fp = fopen( "./data.txt", "w" ) ) == NULL )
	{
		printf( "error opening data file ./data.txt\n" );
	}
	int count=-1;
	int i=0;
	broadcast.ReceiveExecuted=false;//default
	while(1)
	{		
		if(flag_comm_thread_stop == 0)
		{		
			count=UDPRcv(broadcast.inputSocket,broadcast.InputPacket,sizeof(broadcast.InputPacket));		
			//sleep(1);
			if(count!=-1)
			{
				if(broadcast.InputPacket[0]==-1000)
				{				
					printf(" listen thread exit safely!\n");	         
					pthread_exit(0); 
				}
				else
				{				
					broadcast.ReceiveExecuted=true;
					//printf("Communication: Rcv: %f %f %f %f\n",broadcast.InputPacket[0],broadcast.InputPacket[1],broadcast.InputPacket[2],broadcast.InputPacket[3]);
					for(int i=0;i<160;i++)
						fprintf(fp,"%f ",broadcast.InputPacket[i]);
					fprintf(fp,"\n");
				}
			}
			else
				printf("UDPRcv Error!\n");
		}
		else
		{
			printf(" listen thread exit safely!\n");	         
			pthread_exit(0); 
		}
	}
	return NULL;

}

void* send(void * data)
{
	printf("Communication: Sending..\n");
	int nFloat0=-1;

	while(1)
	{	
		printf("press ESC to exit!\n");
		std::cin >> nFloat0;
		ID++;
		if (getch() == ESC_ASCII_VALUE)
		{			
			if(bIsRecData)			
				fclose( fp );		
			nFloat0 = 255;
			broadcast.OutputPacket[0] = nFloat0;
			//printf("%f\n",broadcast.OutputPacket[0]);
			SendData();

			flag_comm_thread_stop = 1;
			printf(" send thread exit safely!\n");
			pthread_exit(0); 
		}
		
		else
		{	
			if(nFloat0 == 1)
			{
				nFloat0 = 1; // Straight forwards
				printf(" (Straight: %d) \n", nFloat0);
				// broadcast.OutputPacket[1] = 1; // Forward
				broadcast.OutputPacket[1] = 20; // Step amount
				broadcast.OutputPacket[2] = 0.15; // Step length
			}
			else if(nFloat0 == 2){
				nFloat0 = 2; // Straight backwards
				printf(" (Straight: %d) \n", nFloat0);
				// broadcast.OutputPacket[1] = 1; // Forward
				broadcast.OutputPacket[1] = 3; // Step amount
				broadcast.OutputPacket[2] = 0.15; // Step length
			}
			else if (nFloat0 == 3)
			{
				nFloat0 = 3; // Side left
				printf(" (Side: %d) \n", nFloat0);
				// broadcast.OutputPacket[1] = 1; // Going left
				broadcast.OutputPacket[1] = 11; // Step amount. Needs to be odd
				broadcast.OutputPacket[2] = 0.07; // Step width

			}
			else if (nFloat0 == 4)
			{
				nFloat0 = 4; // Side right
				printf(" (Side: %d) \n", nFloat0);
				// broadcast.OutputPacket[1] = 1; // Going left
				broadcast.OutputPacket[1] = 11; // Step amount. Needs to be odd
				broadcast.OutputPacket[2] = 0.07; // Step width

			}
			else if (nFloat0 == 5)
			{
				nFloat0 = 5; // Curve
				printf(" (Curve: %d) \n", nFloat0);
				broadcast.OutputPacket[1] = 3; // X
				broadcast.OutputPacket[2] = -1; // Y
				broadcast.OutputPacket[3] = -30.00001; // Psi. Care for singuarity. Add 0.00001
			}
			else if (nFloat0 == 6)
			{
				nFloat0 = 6; // Circle clockwise
				printf(" (Circle: %d) \n", nFloat0);
				broadcast.OutputPacket[1] = 0.2; // radius
				broadcast.OutputPacket[2] = 1.57; // psi_all positive
				broadcast.OutputPacket[3] = 0; // useless
			}
			else if (nFloat0 == 7)
			{
				nFloat0 = 7; // Circle anticlockwise
				printf(" (Circle: %d) \n", nFloat0);
				broadcast.OutputPacket[1] = 0.45; // radius
				broadcast.OutputPacket[2] = 1.57; // psi_all positive
				broadcast.OutputPacket[3] = 0; // useless
			}
			// else if (nFloat0 == 4){
			// 	nFloat0 = 5;
			// 	printf("Go (Walk: %d) \n", nFloat0);
			// }
			else if(nFloat0 == 8){
				nFloat0 = 8;
				printf(" Kicking \n");
				broadcast.OutputPacket[1] = 1; // leftKicking
				broadcast.OutputPacket[2] = 1; // strength
				broadcast.OutputPacket[3] = 0; // angle	
			}
			else if (nFloat0 == 9){
				nFloat0 = 9;
				printf("Stop: %d \n", nFloat0);
			}
			// else if (nFloat0 == 6){
			// 	nFloat0 = 7;
			// 	printf("Go (Walk: %d) \n", nFloat0);
			// }
			// else if (nFloat0 == 7){
			// 	nFloat0 = 8;
			// 	printf("Go (Walk: %d) \n", nFloat0);
			// }
			else if(nFloat0 == 10){
				nFloat0 = 10; // STAND
				printf("Standing\n");
			}
			else if(nFloat0 == 99){
				nFloat0 = 99;
				printf("Stopping now");
			}
			else{
				std::cout << nFloat0 << " non existing" << std::endl;
			}
			broadcast.OutputPacket[0] = nFloat0;
			broadcast.OutputPacket[4] = ID;
			std::cout << "ID: " << ID << std::endl;
			// printf("%f\n",broadcast.OutputPacket[0]);
			SendData();
		}
	}


}

int main(int argc, char*argv[])
{
	pthread_t th_comm_read,th_comm_send;
	void * retval;  

	int ch;
	char optstr[] = "l0123456789";
	while( ( ch = getopt( argc, argv, optstr ) ) != -1 )
	{
		if( ch == 'l' ) bIsRecData = true;
		else bIsRecData = false;
	} 

	InitComm();

	if(bIsRecData)
		pthread_create(&th_comm_read, NULL, listen, 0);
	
	pthread_create(&th_comm_send, NULL, send, 0);
	
	if(bIsRecData)
		pthread_join(th_comm_read, &retval);
	
	pthread_join(th_comm_send, &retval);

	//pause();

	return 0;
}
