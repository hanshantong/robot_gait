//##################################################
//# PROJECT: P1MC.CPP
/* Author: Howard Dong */
//##################################################
/*******************************************************************************
* Copyright (c) 2017, UBT CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of UBT nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#include <P1MC.h>
int walking = 0;
int prev_state = 0;
/////////////////////////////////////THREAD DEFINE    //////////////////////////////////////////

int nATI=0; //1:start ati thread
int nMTI=0; //1:start mti thread
int nDXL=0; //1:start DXL thread
int nDXL_UPBODY=0; //1:start DXL UPBODY MOTION thread  added by lichunjing 2017-06-15
int nUDP_CTRL=0; //1:start DXL UPBODY MOTION thread  added by lichunjing 2017-07-06
int nVISION_CTRL=0; //1:start DXL UPBODY MOTION thread  added by lichunjing 2017-07-06

/////////////////////////////////////END OF THREAD DEFINE //////////////////////////////////////

double time_passed, t_initial, t1,t2,t3,t4;

///////////////////////////////////// THREAD CONTROL  ///////////////////////////////////////////

bool flag_motion_thread_stop = 0;
bool flag_motion_upbody_thread_stop = 0;
bool flag_ati_thread_stop = 0;
bool flag_mti_thread_stop = 0;
bool flag_comm_thread_stop = 0;
bool flag_comm_vision_thread_stop = 0;
/////////////////////////////////////  ROBOTIS-H54  ///////////////////////////////////////////

//#define DXL_DEVICENAME                  "/dev/ttyUSB3"      // Check which port is being used on your controller
//#define DXL_BAUDRATE                    3000000             // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define ESC_ASCII_VALUE                 0x1b

#define MC_TIMER_PERIOD                 5000000             // 2ms

#define DXL_NUM                         12                   // number of dxl motors
#define DXL_NUM_UPBODY                  7                    // number of dxl motors of upbody  added by lichunjing 2017-06-15

#define GEAR_RATIO 501.9
#define DEGREE_TO_REG_VALUE_SF (250961.5 / 180.0)      //

#define CTRL_PERIOD_S   0.005

char dxl_port_name[50];
char dxl_upbody_port_name[50];   //added by lichunjing 2017-06-15
int ndxl_speed;
int ndxl_upbody_speed;

uint8_t    flag_dxl_mode = 0;   //0: torque mode,  3: position mode  
bool       flag_mode_changed = 0;

bool flag_go_zero[12] = {1,1,1,1,1,1,
	1,1,1,1,1,1};
	bool flag_demo_sin[2] = {0,0};

	int nGaitIndex=0;

/////////////////////////////////////  END OF ROBOTIS-H54  ///////////////////////////////////////////


/////////////////////////////////////  ATI-MODBUS  ///////////////////////////////////////////

//#define ATI_1_DEVICENAME                      "/dev/ttyUSB0"
//#define ATI_1_BAUDRATE                        1250000

//#define ATI_2_DEVICENAME                      "/dev/ttyUSB1"
//#define ATI_2_BAUDRATE                        1250000

#define ATI_NUM                               2

	char ati_1_port_name[50];
	char ati_2_port_name[50];
	int nati_speed;

/////////////////////////////////////  END OF ATI-MODBUS  //////////////////////////////////////////////




/////////////////////////////////////////  MTI-300  ////////////////////////////////////////////////////
//#define MTI300_DEVICENAME                      "/dev/ttyUSB2"
//#define MTI300_BAUDRATE                        115200

	char mti_port_name[50];
	int nmti_speed;

//////////////////////////////////////  END OF MTI-300  ////////////////////////////////////////////////

//////////////////////////////////////  GLOABLE  ////////////////////////////////////////////////

	struct gAti_Data    gati_data[2] = {0};

	struct gMti_Data    gmti_data;

	dxl_Actuator *dxl_actuator = new dxl_Actuator[DXL_NUM];
dxl_Actuator *dxl_actuator_upbody = new dxl_Actuator[DXL_NUM_UPBODY];  //added by lichunjing 2017-06-15

//////////////////////////////////////  END OF GLOABLE  ////////////////////////////////////////////////

bool loadConfig()
{	
	//load comm port configuration in ./comm.txt
	FILE *fp;
	char dev[50],memo[50];
	int nSpeed;
	if( ( fp = fopen( "./comm.txt", "r" ) ) == NULL )
	{
		printf( "error loading config file ./comm.txt\n" );
		return false;
	}
	else
	{
		for( int i = 0; i < 5; i++ )
		{
			fscanf( fp, "%s\t%d\t%s", dev,&nSpeed,memo);
			if (!strcmp(memo,"//Leg_Motor"))
			{
				strcpy(dxl_port_name,dev);
				ndxl_speed=nSpeed;
			}
			else if (!strcmp(memo,"//Left_ati"))
			{
				strcpy(ati_1_port_name,dev);
				nati_speed=nSpeed;
			}
			else if (!strcmp(memo,"//Right_ati"))
			{
				strcpy(ati_2_port_name,dev);
				nati_speed=nSpeed;
			}
			else if (!strcmp(memo,"//mti"))
			{
				strcpy(mti_port_name,dev);
				nmti_speed=nSpeed;
			}
			else if (!strcmp(memo,"//Upbody_Motor"))
			{
				strcpy(dxl_upbody_port_name,dev);
				ndxl_upbody_speed=nSpeed;
			}						
			//printf("%s %d\n",dev,nSpeed);			
		}
		fclose( fp );
	}

	int nID,ndir;
	float fZero;

	if( ( fp = fopen( "./JointSetup.txt", "r" ) ) == NULL )
	{
		printf( "error loading config file ./JointSetup.txt\n" );
		return false;
	}
	else
	{
		for( int i = 0; i < DXL_NUM; i++ )
		{
			fscanf( fp, "%d\t%f\t%d", &nID,&fZero,&ndir);						
			// printf("%d %f %d\n",nID,fZero,ndir);		
			dxl_actuator[i].SetID(nID);
			dxl_actuator[i].SetZeroPos(fZero);
			dxl_actuator[i].SetDir(ndir);
		}

		for( int i = 0; i < DXL_NUM_UPBODY; i++ )   //added by lichunjing 2017-06-15
		{
			fscanf( fp, "%d\t%f\t%d", &nID,&fZero,&ndir);						
			// printf("%d %f %d\n",nID,fZero,ndir);		
			dxl_actuator_upbody[i].SetID(nID);
			dxl_actuator_upbody[i].SetZeroPos(fZero);
			dxl_actuator_upbody[i].SetDir(ndir);
		}
		// printf("nID=%d\tfZero=%f\tndir=%d\n",dxl_actuator_upbody[0].id,dxl_actuator_upbody[0].zero_pos,dxl_actuator_upbody[0].nDir);
		fclose( fp );
	}


	float theta_min;
	float theta_max;
	float delta_theta;

	if( ( fp = fopen( "./JointRange.txt", "r" ) ) == NULL )
	{
		printf( "error loading JointRange file ./JointRange.txt\n" );
		return false;
	}
	else
	{
		for( int i = 0; i < DXL_NUM; i++ )
		{
			fscanf( fp, "%f\t%f\t%f", &theta_min,&theta_max,&delta_theta);						
		    //printf("%f %f %f\n",theta_min,theta_max,delta_theta);		
			dxl_actuator[i].set_theta_range(theta_min,theta_max);
			dxl_actuator[i].set_delta_theta_range(delta_theta);
		}
		fclose( fp );
	}



	int nValue;
	char cName[50];
	if( ( fp = fopen( "./config.txt", "r" ) ) == NULL )
	{
		printf( "error loading config file ./config.txt\n" );
		return false;
	}
	else
	{
		for( int i = 0; i < 6; i++ )
		{
			fscanf( fp, "%s\t%d", cName,&nValue);						
			if(!strcmp(cName,"ATI"))
			{
				nATI = nValue;
			}
			else if(!strcmp(cName,"MTI"))
			{
				nMTI = nValue;
			}
			else if(!strcmp(cName,"DXL"))
			{
				nDXL = nValue;
			}
			else if(!strcmp(cName,"DXL_UPBODY"))
			{
				nDXL_UPBODY = nValue;
			}
			else if(!strcmp(cName,"UDP_CTRL"))
			{
				nUDP_CTRL = nValue;
				//printf("nUDP_CTRL=%d\n",nUDP_CTRL);
			}
			else if(!strcmp(cName,"VISION_CTRL"))
			{
				nVISION_CTRL = nValue;
				//printf("nVISION_CTRL=%d\n",nVISION_CTRL);
			}												
		}
		fclose( fp );
	}	


	return true;
}

////////////////////////////////////  Socket ///////////////////////////////////////////////////////////
typedef struct {
	char robotID;
	float OutputPacket[160];//the content of this packet
	float InputPacket[4];
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

typedef struct {
	char robotID;
	float OutputPacket[7];//the content of this packet
	float InputPacket[6];
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
}Broadcast_Vision;
/*
Input Packet Structure
FLOAT[0]:1-ZERO,2-STAND,3-WALK
FLOAT[1]~FLOAT[3] to be defined

Output Packet Structure
FLOAT0: time
FLOAT1:  dxl_1_Goal
FLOAT2:  dxl_1_Pos
FLOAT3:  dxl_1_Vel
FLOAT4-36: dxl_2 - dxl_12  
FLOAT37: ati_1_Fx
FLOAT38: ati_1_Fy
FLOAT39: ati_1_Fz
FLOAT40: ati_1_Tx
FLOAT41: ati_1_Ty
FLOAT42: ati_1_Tz
FLOAT43: ati_2_Fx
FLOAT44: ati_2_Fy
FLOAT45: ati_2_Fz
FLOAT46: ati_2_Tx
FLOAT47: ati_2_Ty
FLOAT48: ati_2_Tz
FLOAT49: mti_Yaw
FLOAT50: mti_Pitch
FLOAT51: mti_Roll
FLOAT52-54: x_est, vx_est, ax_est
FLOAT54-57: y_est, vy_est, ay_est
*/
bool bIsSendData=false;
Broadcast broadcast;
Broadcast_Vision broadcast_vision;
float gait_packet[108];

int outputPort=51657;
int inputPort=51654;

int outputPort_Vision=51657;
int inputPort_Vision=51654;

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
	broadcast.port=51654;

	for(int i=0;i<4;i++)
		broadcast.InputPacket[i] = 0;


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

void InitComm_Vision_Ctrl()
{
	broadcast_vision.port=51654;

	for(int i=0;i<6;i++)
		broadcast_vision.InputPacket[i] = 0;


	broadcast_vision.outputSocket=UDPSocket(outputPort_Vision);
	broadcast_vision.inputSocket=UDPSocket(inputPort_Vision);

	if(broadcast_vision.outputSocket==-1){
		printf("Communication: Create Output socket error!\n");
		broadcast_vision.SocketEnable=false;
	}
	else broadcast_vision.SocketEnable=true;

	SetSocketOption(broadcast_vision.outputSocket);
	SetSocketOption(broadcast_vision.inputSocket);
}


void GetSocketData(float _nTime, dxl_Actuator *_dxl_actuator)
{
	broadcast.OutputPacket[0] = _nTime;
	for(int i=0;i<DXL_NUM;i++)
	{
		broadcast.OutputPacket[i*3+1] = _dxl_actuator[i].get_goal_theta();
		broadcast.OutputPacket[i*3+2] = _dxl_actuator[i].get_present_theta();
		broadcast.OutputPacket[i*3+3] = _dxl_actuator[i].goal_torque;
	}
	for(int i=0;i<2;i++)
	{
		broadcast.OutputPacket[i*6+37] = gati_data[i].Fx;
		broadcast.OutputPacket[i*6+38] = gati_data[i].Fy;
		broadcast.OutputPacket[i*6+39] = gati_data[i].Fz;
		broadcast.OutputPacket[i*6+40] = gati_data[i].Tx;
		broadcast.OutputPacket[i*6+41] = gati_data[i].Ty;
		broadcast.OutputPacket[i*6+42] = gati_data[i].Tz;		
	}
	broadcast.OutputPacket[49] = gmti_data.eulerangles.Roll;
	broadcast.OutputPacket[50] = gmti_data.eulerangles.Pitch;
	broadcast.OutputPacket[51] = gmti_data.eulerangles.Yaw;
	for(int i = 0; i < 108; i++){
		broadcast.OutputPacket[52+i] = gait_packet[i];
	}
	// broadcast.OutputPacket[100] = t1;
	// broadcast.OutputPacket[101] = t2;
	// broadcast.OutputPacket[102] = t3;
}

void SendData()
{
	if(bIsSendData)
	{
		int count=-1;
		count= UDPSend(broadcast.outputSocket,broadcast.OutputPacket,sizeof(broadcast.OutputPacket),"192.168.8.39",51656);	
		//printf("%d\n",count);
		if(count ==-1)printf("Communication: UDPSend Error!\n"); 
	}
}

void GetVisionData(float _nTime)
{
	broadcast_vision.OutputPacket[0] = _nTime;    //time
	broadcast_vision.OutputPacket[1] = gmti_data.eulerangles.Yaw;  //Robot_mti_yaw
	broadcast_vision.OutputPacket[2] = gmti_data.eulerangles.Pitch; //Robot_mti_pitch
	broadcast_vision.OutputPacket[3] = gmti_data.eulerangles.Roll;  //Robot_mti_roll
	broadcast_vision.OutputPacket[4] = 12;    //Odometry_dx
	broadcast_vision.OutputPacket[5] = 13;    //Odometry_dy
	broadcast_vision.OutputPacket[6] = 14;    //Odometry_dtheta

}

void SendVisionData()
{
	int count=-1;
	count= UDPSend(broadcast_vision.outputSocket,broadcast_vision.OutputPacket,sizeof(broadcast_vision.OutputPacket),"127.0.0.1",51656);	
		//printf("%d\n",count);
	if(count ==-1)printf("Communication: UDPSend Error!\n"); 
}

///////////////////////////////////    End of Socket ///////////////////////////////////////////////////


int getch()
{
	struct termios oldt, newt;
	int ch;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
}

int kbhit(void)
{
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
}

////////////////////////////////////////// rtc Timer  ///////////////////////////////////////////////////

int fd = 0;  
int timerInterval[MAX_NUM_TIMERS];
int timerMask[MAX_NUM_TIMERS];
int timerZeroPoint[MAX_NUM_TIMERS];

unsigned long counter;

void openTimer()
{
	fd = open ("/dev/rtc", O_RDONLY);

	if (fd ==  -1) {
		perror("/dev/rtc");
		exit(errno);
	}

	memset( timerInterval, 0, sizeof( int ) * MAX_NUM_TIMERS );
	memset( timerMask, 0, sizeof( int ) * MAX_NUM_TIMERS );
	memset( timerZeroPoint, 0, sizeof( int ) * MAX_NUM_TIMERS );

	counter = 0;
}

void InitTimer()
{
	int retval;
	retval = ioctl(fd, RTC_IRQP_SET, TIMER_RESOLUTION * 1024);
	if (retval == -1) {
		perror("ioctl");
		exit(errno);
	}

	fprintf(stderr, "\nPeriodic IRQ rate set to : %ldHz\n", TIMER_RESOLUTION * 1024);
	fflush(stderr);

	counter = 0;
	
	/* Enable periodic interrupts */
	retval = ioctl(fd, RTC_PIE_ON, 0);
	if (retval == -1) {
		perror("ioctl");
		exit(errno);
	}	
}

int delay( int duration )
{
	int retval;
	unsigned long data;
	for( int i = 0; i < duration; i++ )
	{
		retval = read(fd, &data, sizeof(unsigned long));
		if (retval == -1) {
			perror("read");
			exit(errno);
		}
	}
	return 0;
}

void enable( int timerID )
{
	if( timerID >= 0 && timerID < MAX_NUM_TIMERS )
		timerMask[timerID] = 1;
}

void disable( int timerID )
{
	if( timerID >= 0 && timerID < MAX_NUM_TIMERS )
		timerMask[timerID] = 0;
}

bool isRunning( int timerID )
{
	if( timerID >= 0 && timerID < MAX_NUM_TIMERS )
		return timerMask[timerID] == 1;
	return false;
}

int Round(float number)
{
	return (number >0.0) ? (number + 0.5) : (number - 0.5);
}

void setTimer( int timerID, int interval )
{
	int iv = Round(0.001*(float)interval*TIMER_RESOLUTION*1024);
	//printf("iv %d\n",iv);
	if( timerID >= 0 && timerID < MAX_NUM_TIMERS )
	{
		int retval;
		unsigned long data;

		retval = read( fd, &data, sizeof( unsigned long ) );
		counter += ( data >> 8 );
		timerInterval[timerID] = iv;
		timerZeroPoint[timerID] = counter%( timerInterval[timerID] );
		enable( timerID );
	}
}

// this function returns when one or more timers expire
void getNextTimerEvent(int * timerIDs, int& numberOfEvents)
{
	int retval;
	unsigned long data = 0;
	unsigned long time;
	numberOfEvents = 0;
	while( 1 )
	{
		#if 1
		struct timeval tv = {5, 0};
		fd_set readfds;

		FD_ZERO(&readfds);
		FD_SET(fd, &readfds);
		
		//The select will wait until an RTC interrupt happens. 
		retval = select(fd+1, &readfds, NULL, NULL, &tv);
		if (retval == -1) {
			perror("select");
			exit(errno);
		}
		//This read won't block
		#endif
		retval = read( fd, &data, sizeof( unsigned long ) );
		time = counter + (data >> 8);// now time contains the current counter value, and counter contains the previous one
		if( time < counter )
		{
			counter = time;
			time = counter + ( data >> 8 );
		}
		for( int i = 0; i < MAX_NUM_TIMERS; i++ )
		{
			if( timerMask[i] )
				if( ( time - timerZeroPoint[i] )%timerInterval[i] == 0 // on the beat
				 ||( time - timerZeroPoint[i] )/timerInterval[i] - ( counter  - timerZeroPoint[i] )/timerInterval[i] >= 1 ) // jump over
					timerIDs[numberOfEvents++] = i;
			}
			counter = time;
			if( numberOfEvents ) break;
		}	
	}

	unsigned long getTickCount()
	{
		int retval;
		unsigned long data;
		retval = read( fd, &data, sizeof( unsigned long ) );
		counter += ( data>>8 );
		return counter;
	}	

/////////////////////////////////////////rtc Timer  ////////////////////////////////////////////////////

/////////////////////////////////////////  MTI-300  ////////////////////////////////////////////////////
	void* sensorMTI(void* data) {

		PortHandler *mti300_portHandler;
		mti300_portHandler = new PortHandler(mti_port_name);

		mti300_portHandler->Port_init_mti(mti_port_name, nmti_speed);

		Mti300 *mti300;
		mti300 = new Mti300();

	//try 100 times to send gotoconfig instructions to enter config state of mti-300, if success, break out.
		mti300->gotoConfig(mti300_portHandler);

	// get device code
		mti300->get_dev_code(mti300_portHandler);

	// get device SN
		mti300->get_dev_SN(mti300_portHandler);   

	// get device buadrate
		mti300->get_dev_baud(mti300_portHandler);

	// get device output configration
		mti300->get_dev_output_config(mti300_portHandler);


	//goto measurement
		mti300->gotoMeasurement(mti300_portHandler);
// 
	//set port configration to block mode for 50 bytes everytime
		mti300_portHandler->setupPort_mti_continue_receive();

	//abort the first frame data
		mti300->rx_nonblock(mti300_portHandler, 35);    

		while(1)
		{
			if(flag_mti_thread_stop == 0)
			{
			//mti300->recv_data(mti300_portHandler);
				mti300->recv_data_auto_recovery(mti300_portHandler);
				mti300->update_to_gloable_var(&gmti_data);
			// mti300->print_data(mti300_portHandler);
			}
			else
			{
				printf("2. mti thread exit safely!\n");
				mti300_portHandler->closePort();
				pthread_exit(0); 
			}
		}

		return NULL;
	}
//////////////////////////////////////  END OF MTI-300  ////////////////////////////////////////////////	


/////////////////////////////////////  ATI-MODBUS  ///////////////////////////////////////////
	void* sensorATI_1(void* data) {

	// uint8_t RX_BUFFER[255];
	// uint8_t  Rx_result = 0;


	struct Calibration ati_1_cali;   //confirm added by li chunjing 2017-05-16

	string ati_SerialNumber="FT19717";
	string ati_CaliPartNumber="SI-2800-120";
	string ati_CalibFamilyId="Net";
	uint8_t ati_ForceUnits = 2;
	uint8_t ati_TorqueUnits = 3;

	for(uint16_t i=0;i<sizeof(ati_SerialNumber);i++)
		ati_1_cali.CalibSerialNumber[i]=ati_SerialNumber[i];
	ati_1_cali.CalibSerialNumber[7] = '\0';
	 //printf("CaliSerialNumber: %s\n",ati_1_cali.CalibSerialNumber);

	for(uint16_t i=0;i<sizeof(ati_CaliPartNumber);i++)
		ati_1_cali.CalibPartNumber[i]=ati_CaliPartNumber[i];
	ati_1_cali.CalibPartNumber[11] = '\0';
	 //printf("CalibPartNumber: %s\n",ati_1_cali.CalibPartNumber);

	for(uint16_t i=0;i<sizeof(ati_CalibFamilyId);i++)
		ati_1_cali.CalibFamilyId[i]=ati_CalibFamilyId[i];
	ati_1_cali.CalibFamilyId[3] = '\0';
	 //printf("CalibFamilyId: %s\n",ati_1_cali.CalibFamilyId);

	ati_1_cali.ForceUnits = ati_ForceUnits;
	ati_1_cali.TorqueUnits = ati_TorqueUnits;



	//port, packet,read and write handler initialize
	PortHandler *ati_portHandler;
	ati_portHandler = new PortHandler(ati_1_port_name);


	ati_portHandler->Port_init_ati(ati_1_port_name, nati_speed);


	Ati_Mini45 *ati_mini45;
	// ati_mini45 = new Ati_Mini45();
	ati_mini45 = new Ati_Mini45(ati_1_cali);

	//read and print calibration information   :by lichunjing 2017-03-04
	ati_mini45->read_calibration(ati_portHandler);
	ati_mini45->Print_Cali();

	printf("flag_SN_right: %d\n",ati_mini45->flag_SN_right);
	printf("flag_PN_right: %d\n",ati_mini45->flag_PN_right);
	printf("flag_FID_right: %d\n",ati_mini45->flag_FID_right);
	printf("flag_FU_right: %d\n",ati_mini45->flag_FU_right);
	printf("flag_TU_right: %d\n",ati_mini45->flag_TU_right);

	//read and print gage and offset information   :by lichunjing 2017-03-06
	ati_mini45->read_gage_offset(ati_portHandler);

	//write gage    by li chunjing 2017-03-06
	ati_mini45->write_gage(ati_portHandler);

	//start stream
	ati_mini45->start_stream(ati_portHandler);

	 //Set port baudrate and port config for receive 13 bytes everytime before read() function return.  by li chunjing  2017-03-08
	if (ati_portHandler->setupPort_ati_continue_receive()!=1)
	{
		printf("ATI: Failed to change the baudrate and port config for receiving continus data!\n");
	}


	while(1)
	{
		if(flag_ati_thread_stop == 0)
		{
			//ati_mini45->update_ft_data(ati_portHandler);
			ati_mini45->update_ft_data_restart_stream_mode(ati_portHandler);
			ati_mini45->update_to_gloable_var(gati_data);
			// std::cout << "Left" << std::endl;
			// ati_mini45->print_ft();
			if(ati_mini45->check_error_counter >= 5)
			{
				ati_mini45->check_error_counter = 0;
				ati_mini45->stop_stream(ati_portHandler);

				ati_portHandler->Port_init_ati(ati_1_port_name, nati_speed);

					//start stream
				ati_mini45->start_stream(ati_portHandler);

				 //Set port baudrate and port config for receive 13 bytes everytime before read() function return.  by li chunjing  2017-03-08
				if (ati_portHandler->setupPort_ati_continue_receive()!=1)
				{
					printf("ATI: Failed to change the baudrate and port config for receiving continus data!\n");
				}
			}			
		}
		else
		{
		    // ati_portHandler->writePort(STOP_STREAMING,sizeof(STOP_STREAMING));
			ati_mini45->stop_stream(ati_portHandler);
		    usleep(50000);    //time delay 100ms before close port
		    ati_portHandler->closePort();	
		    printf("3. ati thread exit safely!\n");
		    pthread_exit(0); 	
		}  
	}
	return NULL;
}
/////////////////////////////////////  END OF ATI-MODBUS  ///////////////////////////////////////////

/////////////////////////////////////  ATI-MODBUS  ///////////////////////////////////////////
void* sensorATI_2(void* data) {

	// uint8_t RX_BUFFER[255];
	// uint8_t  Rx_result = 0;

	struct Calibration ati_2_cali;   //confirm added by li chunjing 2017-05-16

	string ati_SerialNumber="FT19716";
	string ati_CaliPartNumber="SI-2800-120";
	string ati_CalibFamilyId="Net";
	uint8_t ati_ForceUnits = 2;
	uint8_t ati_TorqueUnits = 3;

	for(uint16_t i=0;i<sizeof(ati_SerialNumber);i++)
		ati_2_cali.CalibSerialNumber[i]=ati_SerialNumber[i];
	ati_2_cali.CalibSerialNumber[7] = '\0';
	 //printf("CaliSerialNumber: %s\n",ati_2_cali.CalibSerialNumber);

	for(uint16_t i=0;i<sizeof(ati_CaliPartNumber);i++)
		ati_2_cali.CalibPartNumber[i]=ati_CaliPartNumber[i];
	ati_2_cali.CalibPartNumber[11] = '\0';
	 //printf("CalibPartNumber: %s\n",ati_2_cali.CalibPartNumber);

	for(uint16_t i=0;i<sizeof(ati_CalibFamilyId);i++)
		ati_2_cali.CalibFamilyId[i]=ati_CalibFamilyId[i];
	ati_2_cali.CalibFamilyId[3] = '\0';
	 //printf("CalibFamilyId: %s\n",ati_2_cali.CalibFamilyId);

	ati_2_cali.ForceUnits = ati_ForceUnits;
	ati_2_cali.TorqueUnits = ati_TorqueUnits;



	//port, packet,read and write handler initialize
	PortHandler *ati_portHandler;
	ati_portHandler = new PortHandler(ati_2_port_name);


	ati_portHandler->Port_init_ati(ati_2_port_name, nati_speed);


	Ati_Mini45 *ati_mini45;
	// ati_mini45 = new Ati_Mini45();
	ati_mini45 = new Ati_Mini45(ati_2_cali);

	//read and print calibration information   :by lichunjing 2017-03-04
	ati_mini45->read_calibration(ati_portHandler);
	ati_mini45->Print_Cali();

	printf("flag_SN_right: %d\n",ati_mini45->flag_SN_right);
	printf("flag_PN_right: %d\n",ati_mini45->flag_PN_right);
	printf("flag_FID_right: %d\n",ati_mini45->flag_FID_right);
	printf("flag_FU_right: %d\n",ati_mini45->flag_FU_right);
	printf("flag_TU_right: %d\n",ati_mini45->flag_TU_right);

	// if( (ati_mini45->flag_SN_right && ati_mini45->flag_PN_right && ati_mini45->flag_FID_right && ati_mini45->flag_FU_right && ati_mini45->flag_TU_right) == 0){
	// 	printf("Sensor didn't start properly");
	// }

	//read and print gage and offset information   :by lichunjing 2017-03-06
	ati_mini45->read_gage_offset(ati_portHandler);

	//write gage    by li chunjing 2017-03-06
	ati_mini45->write_gage(ati_portHandler);

	//start stream
	ati_mini45->start_stream(ati_portHandler);

	 //Set port baudrate and port config for receive 13 bytes everytime before read() function return.  by li chunjing  2017-03-08
	if (ati_portHandler->setupPort_ati_continue_receive()!=1)
	{
		printf("ATI: Failed to change the baudrate and port config for receiving continus data!\n");
	}


	while(1)
	{
		if(flag_ati_thread_stop == 0)
		{
			//ati_mini45->update_ft_data(ati_portHandler);
			ati_mini45->update_ft_data_restart_stream_mode(ati_portHandler);
			ati_mini45->update_to_gloable_var(gati_data+1);
			//std::cout << "Right" << std::endl;
			// ati_mini45->print_ft();
			if(ati_mini45->check_error_counter >= 5)
			{
				ati_mini45->check_error_counter = 0;
				ati_mini45->stop_stream(ati_portHandler);

				ati_portHandler->Port_init_ati(ati_1_port_name, nati_speed);

					//start stream
				ati_mini45->start_stream(ati_portHandler);

				 //Set port baudrate and port config for receive 13 bytes everytime before read() function return.  by li chunjing  2017-03-08
				if (ati_portHandler->setupPort_ati_continue_receive()!=1)
				{
					printf("ATI: Failed to change the baudrate and port config for receiving continus data!\n");
				}
			}			
		}
		else
		{
		    // ati_portHandler->writePort(STOP_STREAMING,sizeof(STOP_STREAMING));
			ati_mini45->stop_stream(ati_portHandler);
		    usleep(50000);    //time delay 100ms before close port
		    ati_portHandler->closePort();	
		    printf("3. ati thread exit safely!\n");
		    pthread_exit(0); 	
		}  
	}
	return NULL;
}
/////////////////////////////////////  END OF ATI-MODBUS  ///////////////////////////////////////////


////////////////////////////////////////  DXL MOTOR THREAD  /////////////////////////////////////////
void* motion(void* data) 
{
//timer variable set
	int numOfTimerEvents = 0;
	int popupTimers[MAX_NUM_TIMERS];
	openTimer();
	InitTimer();
	setTimer(0,5);//5 ms timer

	float time,time_zero;

	bool bIsWalkStart=false;

	//set pid parameter 
	int32_t outMin = -300;
	int32_t outMax = 300;

    int32_t Velocity_Max = 16613;   //33.1*501.9 = 16612.89
    int32_t Velocity_Min = -16613;

    float error_theta_max_torque_degree = 0.6; //8
    float Kp = outMax / DEGREE_TO_REG_VALUE_SF / error_theta_max_torque_degree;

    float error_theta_max_torque_degree_knee = 0.6; //8
    float Kp_knee = outMax / DEGREE_TO_REG_VALUE_SF / error_theta_max_torque_degree_knee;


    float time_to_max_torque_s = 2.0;  //10.0 time to take from 1 degree error to full scale torque through integrator 
    float Ki = outMax * CTRL_PERIOD_S / DEGREE_TO_REG_VALUE_SF / time_to_max_torque_s;

    float negtive_torque_max_velocity = 600;
    float Kd1 = negtive_torque_max_velocity / Velocity_Max;
    float Instructtion_velocity_max_torque = 60;    //instruction velocity to generate 300 torque
    float Kd2 = outMax / Instructtion_velocity_max_torque / DEGREE_TO_REG_VALUE_SF ;



	// //port, packet,read and write handler initialize
    // PortHandler *dxl_portHandler;
    // dxl_portHandler = new PortHandler(dxl_port_name);

    // dxl_PacketHandler *dxl_packetHandler;
    // dxl_packetHandler = new dxl_PacketHandler();

    // dxl_GroupSyncRead *dxl_groupSyncRead;
    // dxl_groupSyncRead = new dxl_GroupSyncRead(dxl_portHandler, dxl_packetHandler);

    // dxl_GroupSyncWrite *dxl_groupSyncWrite;
    // dxl_groupSyncWrite = new dxl_GroupSyncWrite(dxl_portHandler, dxl_packetHandler);


    // dxl_GroupSyncWrite *dxl_groupSyncWrite_position;
    // dxl_groupSyncWrite_position = new dxl_GroupSyncWrite(dxl_portHandler, dxl_packetHandler,ADDR_GOAL_POSITION,LEN_PRESENT_POSITION);    


	//initilize PID controller
    Pid_Controller *pid_controller = new Pid_Controller[DXL_NUM];


    // // added by lichunjing 2017-06-14
    // Robot_Model *robot_model;
    // robot_model = new Robot_Model();

    // robot_model->update_phi();
    // robot_model->update_trans_matrix();
    // robot_model->update_inv_trans_matrix();

    // // printf("C00=%f C01=%f C02=%f C10=%f C11=%f C12=%f C20=%f C21=%f C22=%f\n",robot_model->C_inv[0].t[0][0], robot_model->C_inv[0].t[0][1], robot_model->C_inv[0].t[0][2], robot_model->C_inv[0].t[1][0], robot_model->C_inv[0].t[1][1], robot_model->C_inv[0].t[1][2], robot_model->C_inv[0].t[2][0], robot_model->C_inv[0].t[2][1], robot_model->C_inv[0].t[2][2]);

    // robot_model->update_joint_position();

    // int i = 8;
    // // printf("x0=%f y0=%f z0=%f\n",robot_model->pos_joint[i].x, robot_model->pos_joint[i].y, robot_model->pos_joint[i].z);
    // // printf("x8=%f y8=%f z8=%f\n",robot_model->pos_joint[8].x, robot_model->pos_joint[8].y, robot_model->pos_joint[8].z);
    
    // robot_model->update_com_position();
    // robot_model->calculate_gravity_compsation();

    // printf("gravity_comp_1=%f\tgravity_comp_2=%f\n",robot_model->gravity_comp[1],robot_model->gravity_comp[10]);

    // robot_model->matrix_test();

    // return 0;
    // // added by lichunjing 2017-06-14

    int32_t minTorque = -850;
    int32_t maxTorque =  850;

    for(uint16_t i=0;i<DXL_NUM;i++)
    {
    	if( (i == 3) || (i == 9) ){
    		pid_controller[i].PID_setgains(Kp_knee, Ki, Kd1, Kd2);
    	}
    	else{
    		pid_controller[i].PID_setgains(Kp, Ki, Kd1, Kd2);
    	}	
    	if( (i == 0) || (i == 6) ){
    		pid_controller[i].PID_setoutMin(-300);
    		pid_controller[i].PID_setoutMax(300);
    	}
    	else{
    		pid_controller[i].PID_setoutMin(minTorque);
    		pid_controller[i].PID_setoutMax(maxTorque);
    	}
    }

    Motion_Ctrl *motion_ctrl;
    motion_ctrl = new Motion_Ctrl(0);
    motion_ctrl->read_para();
    int mode = 4; // 0 is walk, 1 is ssp, 2 is ssp with sine, 3 is dsp with sine
    Robot *h13 = new Robot(mode);

	//initilize port
    // dxl_portHandler->Port_init(dxl_port_name, ndxl_speed);

// initilize motor opration mode 
 //    if (flag_dxl_mode == 0)
 //    {
 //    	for(int i=0;i<DXL_NUM;i++)
 //    	{			
 //    		dxl_actuator[i].Set_Mode_Torque(dxl_portHandler, dxl_packetHandler);
 //    		dxl_actuator[i].SyncRead_init(dxl_groupSyncRead);
 //    	}
 //    }	
 //    else if(flag_dxl_mode == 3)
 //    {
 //    	for(int i=0;i<DXL_NUM;i++)
 //    	{
 //    		dxl_actuator[i].Set_Mode_Position(dxl_portHandler, dxl_packetHandler);
 //    		dxl_actuator[i].SyncRead_init(dxl_groupSyncRead);
 //    	}
 //    }

	// // //read initial position and velocity
	// dxl_actuator->SyncRead_Send(dxl_packetHandler, dxl_groupSyncRead);  //select one of SyncRead_Send function is OK
	// for(int i=0;i<DXL_NUM;i++)
	// {
	// 	dxl_actuator[i].Update_Pos_Vel(dxl_packetHandler, dxl_groupSyncRead);
	// 	dxl_actuator[i].start_theta = dxl_actuator[i].get_present_theta();
	// 	dxl_actuator[i].traj_theta = dxl_actuator[i].get_present_theta();
	// 	//(dxl_actuator+i)->goal_position = (dxl_actuator+i)->present_position;   //initial goal positon set as goal position 
	// } 
	while(1)
	{
		if(flag_motion_thread_stop == 0)
		{
			getNextTimerEvent( popupTimers, numOfTimerEvents );
			for( int i = 0; i < numOfTimerEvents; i++ )
			{
				switch( popupTimers[i] )
				{
					case 0:
					time=getTickCount();
					time/=1024*TIMER_RESOLUTION;                        	                        

					// if(flag_mode_changed == 1)
					// {
					// 	if (flag_dxl_mode == 0)
					// 	{
					// 		for(int i=0;i<DXL_NUM;i++)
					// 		{
					// 			dxl_actuator[i].Set_Mode_Torque(dxl_portHandler, dxl_packetHandler);
					// 		}
					// 	}								
					// 	else if(flag_dxl_mode == 3)
					// 	{
					// 		for(int i=0;i<DXL_NUM;i++)
					// 		{
					// 			dxl_actuator[i].Set_Mode_Position(dxl_portHandler, dxl_packetHandler);
					// 		}
					// 	}
					// 	flag_dxl_mode = 0;
					// }
					// clipTime(&time_passed, &t_initial);
						    //read present position and velocity
							// dxl_actuator->SyncRead_Send(dxl_packetHandler, dxl_groupSyncRead);  //select one of SyncRead_Send function is OK
							// clipTime(&time_passed, &t_initial);
							// t1 = time_passed;
							// for(int i=0;i<DXL_NUM;i++)
							// {
							// 	dxl_actuator[i].Update_Pos_Vel(dxl_packetHandler, dxl_groupSyncRead);
							// 	dxl_actuator[i].check_theta_range();	
							// 	dxl_actuator[i].check_delta_theta_range();
							// 	if(dxl_actuator[i].flag_theta_InRange == 0)
							// 	{
							// 		printf("dxl_actuator:%d theta out of range!\n", i);
							//    		// return 0;
							// 	}
							// 	if(dxl_actuator[i].flag_delta_theta_InRange == 0)
							// 	{
							// 		printf("dxl_actuator:%d delta theta out of range!\n", i);
							//    		// return 0;
							// 	}							

							// }

							// for(int i=0;i<DXL_NUM;i++)
							// {						
							// 	if(dxl_actuator[i].get_present_theta() != dxl_actuator[i].get_previous_theta())
							// 	{
							// 		printf("dxl_actuator[%d]:\t tehta:%f\t\tdelta_theta:%f\n",i, dxl_actuator[i].get_present_theta(), abs(dxl_actuator[i].get_present_theta()-dxl_actuator[i].get_previous_theta()));
							// 	}									
							// }

	                        //on start ,go back to 0 position in speed 15 degree/s     by lichunjing   2017-03-10
							for(int i=0;i<DXL_NUM;i++)     
							{
								if(flag_go_zero[i] == 1)
								{
									if(h13->goto_zero(dxl_actuator+i))
									{
										flag_go_zero[i] = 0;
									}	                                   	   		
								}  	            	         
							}

	                        // printf("present_theta[0]: %f\t present_theta[1]: %f\t \n",(dxl_actuator)->present_theta,(dxl_actuator+1)->present_theta);
	                        // printf("goal_theta[0]: %f\t goal_theta[1]: %f\t  goal_theta[2]: %f\t \n",(dxl_actuator)->goal_theta,(dxl_actuator+1)->goal_theta,(dxl_actuator+2)->goal_theta);                                 
//====================================================================GAIT==================================================       

	                        //UDP socket control, InputPacket FLOAT[0]:1-ZERO,2-STAND,3-WALK;FLOAT[1]~FLOAT[3] to be defined
	                        if(broadcast.InputPacket[0] == 10)//Stand
	                        {
	                        	if(!bIsWalkStart)
	                        	{
	                        		time_zero=time;
	                        		bIsWalkStart=true;
	                        	}
	                        	prev_state = 0;
	                        	h13->stand(dxl_actuator, DXL_NUM);

	                        }
	                        else if(broadcast.InputPacket[0] == 1)//Straight
	                        {
	                        	if(!bIsWalkStart)
	                        	{
	                        		time_zero=time;
	                        		bIsWalkStart=true;
	                        	}
	                        	if(walking == 0 && (prev_state != 1)){
	                        		//===============================
	                        		h13->erase_trajectories();

	                        		int forward  = (int) (broadcast.InputPacket[1]);
	                        		int leftFootStart = 1;
	                        		int step_amount = (int) (broadcast.InputPacket[2]);
	                        		double step_length = (float) (broadcast.InputPacket[3]);
	                        		std::cout << "Generated walk with: forward" << forward << "," << step_amount << " steps and " << step_length << " step length" << std::endl;
	                        		h13->calc_trajectories_walk_straight(forward, leftFootStart, step_amount, step_length);
									//====================================         	                        		
	                        	}
	                        	else{
	                        		walking = h13->walk(dxl_actuator, gait_packet);
												// Log the data and send via udp
	                        		GetSocketData(time-time_zero,dxl_actuator);
	                        		SendData();
	                        	}	  
	                        	prev_state = 1;
	                        }
						    else if(broadcast.InputPacket[0] == 2)//Side
						    {
						    	if(!bIsWalkStart)
						    	{
						    		time_zero=time;
						    		bIsWalkStart=true;
						    	}
						    	if(walking == 0 && (prev_state != 2)){
                        		//=================
						    		h13->erase_trajectories();

						    		int leftside            = (int) (broadcast.InputPacket[1]);
								  //  If left side then Rightfoot Start first.
									int leftFootStart       = 1 - leftside;  //left foot swing, right foot static first if == 0
									int step_amount = (int) (broadcast.InputPacket[2]);

									double step_width = (double) (broadcast.InputPacket[3]);
									std::cout << "Generated walk with: left" << leftside << "," << step_amount << " steps and " << step_width << " step width" << std::endl;
									h13->calc_trajectories_walk_side(leftside, leftFootStart, step_amount, step_width);
									//==================
								}
								else{
									walking = h13->walk(dxl_actuator, gait_packet);
													// Log the data and send via udp
									GetSocketData(time-time_zero,dxl_actuator);
									SendData();
								}
								prev_state = 2;
							}       
	                        else if(broadcast.InputPacket[0] == 3)//Curve
	                        {
	                        	if(walking == 0 && (prev_state != 3)){
	                    			//====================================                            
	                        		h13->erase_trajectories();

	                        		if(!bIsWalkStart)
	                        		{
	                        			time_zero=time;
	                        			bIsWalkStart=true;
	                        		}

	                        		int forward  = 1;
	                        		int leftFootStart = 1;
	                        		double x_goal = (double) (broadcast.InputPacket[1]);
	                        		double y_goal = (double) (broadcast.InputPacket[2]);
	                        		double psi_goal = (double) (broadcast.InputPacket[3]*pi/180);
									std::cout << "Generated curve gait with: " << x_goal << "," << y_goal << "," << psi_goal << std::endl;

	                        		h13->calc_trajectories_walk_curve(forward, leftFootStart, x_goal, y_goal, psi_goal);
									//==================================
	                        		
	                        	}
	                        	else{
	                        		walking = h13->walk(dxl_actuator, gait_packet);
												// Log the data and send via udp
	                        		GetSocketData(time-time_zero,dxl_actuator);
	                        		SendData();
	                        	}
	                        	prev_state = 3;
	                        }
	                        else if(broadcast.InputPacket[0] == 4)//Círcling
	                        {
	                        	if(walking == 0 && (prev_state != 4)){
	                        	//==================
	                        		h13->erase_trajectories();

	                        		if(!bIsWalkStart)
	                        		{
	                        			time_zero=time;
	                        			bIsWalkStart=true;
	                        		}

	                        		double step_width = 0.1;
	                        		int clockwise = (int) (broadcast.InputPacket[1]);

	                        		double degree = pi/2;
	                        		double x_start = 0;
	                        		double y_start = 0;
	                        		double x_stop  = (double) (broadcast.InputPacket[2]);
	                        		double y_stop  = (double) (broadcast.InputPacket[3]);
	                        		double radius = 1;
	                        		double x_center = 1;
	                        		double y_center = 0;
									std::cout << "Generated curve gait with: " << x_stop << "," << y_stop << ", Radius: " << radius << std::endl;

	                        		h13->calc_trajectories_walk_circling(clockwise, radius, step_width, x_start, y_start, x_stop, y_stop, x_center, y_center);

								//======================
	                        		
	                        	}
	                        	else{
	                        		walking = h13->walk(dxl_actuator, gait_packet);
												// Log the data and send via udp
	                        		GetSocketData(time-time_zero,dxl_actuator);
	                        		SendData();
	                        	}
	                        	prev_state = 4;
	                        }
	                        else if(broadcast.InputPacket[0] == 5)//Círcling
	                        {
	                        	// h13->stop(dxl_actuator, gait_packet);
								if((prev_state != 5)){
	                        	//==================
	                        		// h13->erase_trajectories();

	                        		if(!bIsWalkStart)
	                        		{
	                        			time_zero=time;
	                        			bIsWalkStart=true;
	                        		}

	                        		double step_width = 0.1;
	                        		std::cout << "erase and walking " << std::endl;
	                        		h13->stopGait();
	                        		
								//======================
	                        		
	                        	}
	                        	else{
	                        		
	                        		walking = h13->walk(dxl_actuator, gait_packet);
												// Log the data and send via udp
	                        		GetSocketData(time-time_zero,dxl_actuator);
	                        		SendData();
	                        	}
	                        	prev_state = 5;
	                        }
	                        else{
	                        	// h13->stop
	                        }

//==========================================================GAIT END =========================
                            //printf("traj_theta[0]: %f\t traj_theta[1]: %f\t  traj_theta[2]: %f\t \n",(dxl_actuator)->traj_theta,(dxl_actuator+1)->traj_theta,(dxl_actuator+2)->traj_theta);

	                        //PID control of motor
	                        for(int i=0;i<DXL_NUM;i++)
	                        {
	                        	pid_controller[i].PID_setrefValue(dxl_actuator[i].goal_position);
	                        	pid_controller[i].PID_setfbackValue(dxl_actuator[i].present_position);
	                        	pid_controller[i].PID_setfbackValue_d(dxl_actuator[i].present_velocity);
	                        	pid_controller[i].PID_run();
	                        }



	                        for(int i=0;i<DXL_NUM;i++)
	                        {
	                        	if(dxl_actuator[0].comm_result == 0){
	                        		dxl_actuator[i].goal_torque = (int32_t)(pid_controller[i].PID_getOut());
                                	// std::cout << "Read and write success" << std::endl;
	                        	}
	                        	else{
	                        		dxl_actuator[i].goal_torque = (int32_t)(0);
	                        		printf("Comm results of %d was unsuccessful: %d \n", i, dxl_actuator[0].comm_result);
	                        	}

	                        	if ((dxl_actuator[i].flag_theta_InRange == 0) || (dxl_actuator[i].flag_delta_theta_InRange == 0)){
	                        		for(int k=0;k<DXL_NUM;k++)
	                        		{
	                        			dxl_actuator[k].goal_torque = (int32_t)(0);
	                        		}				
	                        		flag_motion_thread_stop = 1;					
	                        		printf("Theta %d In range: Theta %d, DeltaTheta %d \n", i, dxl_actuator[i].flag_theta_InRange, dxl_actuator[i].flag_delta_theta_InRange);
	                        	}

                                 dxl_actuator[i].goal_torque = (int32_t)(0);
// 
                                // std::cout << i << ": " << dxl_actuator[i].goal_position << std::endl;
								//if(dxl_actuator[i].goal_torque>=300)
						      		//printf("%d,%d\n",i,dxl_actuator[i].goal_torque);
	                        }
	                        // if(broadcast.InputPacket[0] == 3)//WALK
	                        {
							    //SOFTENING ANKLE
	                        	if(h13->parameters.control_leg_length == 1){
	                        		int32_t tx, ty;
	                        		h13->update_ati(gati_data);
	                        		if(h13->leftIsSupportLeg){
	                        			if(h13->in_foot_landing_control_phase == 1){
	                        				h13->softening_ankle(0, &tx, &ty);
	                        				dxl_actuator[10].goal_torque = -tx;
	                        				dxl_actuator[11].goal_torque = ty;
	                        			}
	                        		}
	                        		else{
	                        			if(h13->in_foot_landing_control_phase == 1){
	                        				h13->softening_ankle(1, &tx, &ty);
	                        				dxl_actuator[4].goal_torque = tx;
	                        				dxl_actuator[5].goal_torque = ty;
	                        			}
	                        		}
	                        	}
	                        }
	                        int32_t tx, ty;
	                        h13->update_ati(gati_data);
										// h13->softening_ankle(0, &tx, &ty);
										// dxl_actuator[10].goal_torque = -tx;
										// dxl_actuator[11].goal_torque = ty;


										// h13->softening_ankle(1, &tx, &ty);
										// dxl_actuator[4].goal_torque = tx;
										// dxl_actuator[5].goal_torque = ty;

							//=======================
							 // dxl_actuator[3].goal_torque = (int32_t)(120);

	      //                    //write goal torque to motor
	      //                   if (flag_dxl_mode == 0)
	      //                   {
	      //                   	for(int i=0;i<DXL_NUM;i++)
	      //                   	{
	      //                   		dxl_actuator[i].SyncWrite_init(dxl_groupSyncWrite);
	      //                   	}
							//     dxl_actuator->SyncWrite_Send_Goal_Torque(dxl_groupSyncWrite, dxl_packetHandler);   //select one of function is OK
							// }								
							// else if(flag_dxl_mode == 3)
							// {
							// 	for(int i=0;i<DXL_NUM;i++)
							// 	{
							// 		dxl_actuator[i].SyncWrite_Set_Position(dxl_groupSyncWrite_position,dxl_packetHandler);
							// 	}
							// 	dxl_actuator->SyncWrite_Send_Goal_Position(dxl_groupSyncWrite_position, dxl_packetHandler); 
							// }
							break;
						}
					}

				}
				else
				{			
			// 		for(int i=0;i<DXL_NUM;i++)
			// 		{
			// 			dxl_actuator[i].goal_torque = 0;
			// 		}
			// //write 0 torque to stop motor
			// 		for(int i=0;i<DXL_NUM;i++)
			// 		{
			// 			dxl_actuator[i].SyncWrite_init(dxl_groupSyncWrite);
			// 	// Read current position
			// 	// Set to position control
			// 	// Set goal position to current position
			// 		}
			// dxl_actuator->SyncWrite_Send_Goal_Torque(dxl_groupSyncWrite, dxl_packetHandler);   //select one of function is OK

			// // Disable Dynamixel Torque
			// for(int i=0;i<DXL_NUM;i++)
			// {
			// 	dxl_actuator[i].Torque_Disable(dxl_portHandler, dxl_packetHandler);

			// }

	        usleep(100000);    //time delay 100ms before close port
	        // dxl_portHandler->closePort();

	        printf("4. motion thread exit safely!\n");

	        pthread_exit(0); 
	    }
	}
	return NULL;
}
////////////////////////////////////////  END OF DXL MOTOR THREAD  /////////////////////////////////////////

////////////////////////////////////////  DXL UP BODY MOTION THREAD  /////////////////////////////////////////
void* motion_upbody(void* data) 
{

	PortHandler *dxl_portHandler;
	dxl_portHandler = new PortHandler(dxl_upbody_port_name);

	dxl_PacketHandler *dxl_packetHandler;
	dxl_packetHandler = new dxl_PacketHandler();

		//initilize port
	dxl_portHandler->Port_init(dxl_upbody_port_name, ndxl_upbody_speed);


	for(int i=0;i<DXL_NUM_UPBODY;i++)
	{
		dxl_actuator_upbody[i].Set_Mode_Position(dxl_portHandler, dxl_packetHandler);	
	}

	for(int i=0;i<5;i++)
	{
		dxl_actuator_upbody[i].goto_theta_mode_position(dxl_portHandler, dxl_packetHandler, 0.0);
	}

	dxl_actuator_upbody[5].goto_theta_mode_position(dxl_portHandler, dxl_packetHandler, 0.0);
	dxl_actuator_upbody[6].goto_theta_mode_position(dxl_portHandler, dxl_packetHandler, 25.0);


	while(1)
	{
		usleep(1000000);

		if(flag_motion_upbody_thread_stop == 0)
		{

		}
		else
		{
			for(int i=0;i<DXL_NUM_UPBODY;i++)
			{
				dxl_actuator_upbody[i].Torque_Disable(dxl_portHandler, dxl_packetHandler);
			}
			printf("6. motion upbody thread exit safely!\n");
			pthread_exit(0);
		}
	}

	// printf("1111111111111111111\n");
	// return 0;
}
////////////////////////////////////////  DXL UP BODY MOTION THREAD  /////////////////////////////////////////

void* thread_control(void* data) 
{
	printf("press ESC to exit!\n");
	if (getch() == ESC_ASCII_VALUE)
	{   
		printf("catch esc!\n");
		flag_motion_thread_stop = 1;
		flag_motion_upbody_thread_stop = 1;
		flag_ati_thread_stop = 1;
		flag_mti_thread_stop = 1;
		flag_comm_thread_stop = 1;
	}
	printf("1. ctrl thread exit safely!\n");
	pthread_exit(0); 
}

///////////////////////////////////  Socket Thread  ///////////////////////////////////////////////////////

void* listen(void * data)
{
	printf("Communication: Listening..\n");
	int count=-1;
	int i=0;
	broadcast.ReceiveExecuted=false;//default

	while(1)
	{
		if(flag_comm_thread_stop == 0)
		{
			count=UDPRcv(broadcast.inputSocket,broadcast.InputPacket,sizeof(broadcast.InputPacket));		
			if(count!=-1)
			{
				broadcast.ReceiveExecuted=true;
				//printf("Communication: Rcv: %f\n",broadcast.InputPacket[0]);
			}
			else
				printf("UDPRcv Error!\n");
			if(broadcast.InputPacket[0] == 255)   //catch esc signal from UDP app   added by li chunjing 2017-06-01
			{
				printf("5. comm thread exit safely!\n");

				GetSocketData(-1000,dxl_actuator);
				SendData();		
				flag_motion_thread_stop = 1;
				flag_motion_upbody_thread_stop = 1;
				flag_ati_thread_stop = 1;
				flag_mti_thread_stop = 1;
				flag_comm_thread_stop = 1;   
				pthread_exit(0); 
			}
		}
		else
		{
			printf("5. comm thread exit safely!\n");

			pthread_exit(0); 			
		}
	}
	return NULL;

}


void* listen_vision_ctrl(void * data)
{
	printf("VISION_CTRL: Listening..\n");
	int count=-1;
	int i=0;
	broadcast_vision.ReceiveExecuted=false;//default

	while(1)
	{
		if(flag_comm_vision_thread_stop == 0)
		{

			// count=UDPRcv_int(broadcast.inputSocket, command_type,sizeof(command_type));		
			// if(count!=-1)
			// {
			// 	broadcast.ReceiveExecuted=true;
			// 	//printf("Communication: Rcv: %f\n",broadcast.InputPacket[0]);
			// }
			// else
			// 	printf("UDPRcv Error!\n");

			// 	printf("command_type:%d  ", command_type[0]);


			count=UDPRcv(broadcast_vision.inputSocket,broadcast_vision.InputPacket,sizeof(broadcast_vision.InputPacket));		
			if(count!=-1)
			{
				broadcast_vision.ReceiveExecuted=true;
				//printf("Communication: Rcv: %f\n",broadcast.InputPacket[0]);
			}
			else
				printf("UDPRcv Error!\n");
			if(broadcast_vision.InputPacket[0] == 255)   //catch esc signal from UDP app   added by li chunjing 2017-06-01
			{
				printf("1. comm thread exit safely!\n");

				GetVisionData(-1000);
				SendVisionData();		
				flag_motion_thread_stop = 1;
				flag_motion_upbody_thread_stop = 1;
				flag_ati_thread_stop = 1;
				flag_mti_thread_stop = 1;
				flag_comm_thread_stop = 1;
				flag_comm_vision_thread_stop = 1;   
				pthread_exit(0); 
			}

			for(int i=0;i<5;i++)
			{
				printf("Packet[%d]:%f  ", i, broadcast_vision.InputPacket[i]);
			}
			printf("\n");

			printf("111111111\n");

			GetVisionData(111);	
			SendVisionData();

			printf("22222222222\n");
		}
		else
		{
			printf("1. comm thread exit safely!\n");

			pthread_exit(0); 			
		}
	}
	return NULL;

}

////////////////////////////////// End of Socket Thread ///////////////////////////////////////////////////

// int main(int argc, char*argv[])
// {
//     Robot *h13 = new Robot(4);
//     h13->drawTrajectory();
// }
int main(int argc, char*argv[])
{
	pthread_t th_ctrl, th_sr_ati_1, th_sr_ati_2, th_sr_mti, th_mt, th_mt_upbody, th_comm, th_vision_ctrl;
	void * retval;

	int ch;
	char optstr[] = "l0123456789";
	while( ( ch = getopt( argc, argv, optstr ) ) != -1 )
	{
		if( ch == 'l' ) bIsSendData = true;
		else bIsSendData = false;
	}

	loadConfig();  

	for(int i=0;i<DXL_NUM;i++)
		printf("%f %d\n",dxl_actuator[i].zero_pos,dxl_actuator[i].nDir); 

	//InitComm();
    //init();
	if(nATI == 1)
	{
		pthread_create(&th_sr_ati_1, NULL, sensorATI_1, 0);
		pthread_create(&th_sr_ati_2, NULL, sensorATI_2, 0);
	}

	if(nMTI ==1)
	{
		pthread_create(&th_sr_mti, NULL, sensorMTI, 0);
	}

	if(nDXL == 1)
	{
		pthread_create(&th_mt, NULL, motion, 0);
	}

	if(nDXL_UPBODY == 1)
	{
		pthread_create(&th_mt_upbody, NULL, motion_upbody, 0);
	}

	if(nUDP_CTRL == 1)
	{
		InitComm();
		pthread_create(&th_comm, NULL, listen, 0); 
	}

	if(nVISION_CTRL == 1)
	{
		InitComm_Vision_Ctrl();
		pthread_create(&th_vision_ctrl, NULL, listen_vision_ctrl, 0); 
	}

    //pthread_create(&th_ctrl, NULL, thread_control, 0);
    //pthread_create(&th_comm, NULL, listen, 0); 

    //pthread_join(th_ctrl, &retval);   //note: order of thread exit is important!

	if(nATI == 1)
	{
		pthread_join(th_sr_ati_1, &retval);
		pthread_join(th_sr_ati_2, &retval);
	}

	if(nMTI ==1)
	{
		pthread_join(th_sr_mti, &retval);
	}

	if(nDXL == 1)
	{
		pthread_join(th_mt, &retval);
	} 

	if(nDXL_UPBODY == 1)
	{
		pthread_join(th_mt_upbody, &retval);
	}

	if(nUDP_CTRL == 1)
	{
		pthread_join(th_comm, &retval);
	}

	if(nVISION_CTRL == 1)
	{
		pthread_join(th_vision_ctrl, &retval); 
	}

    //pthread_join(th_comm, &retval);

	printf("7. main thread exit safely!\n");
	return 0;
}
