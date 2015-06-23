///////////////////////////////////////////////////
//接口板程序
//单片机型号:PIC18F45K80 TQFP 
//电压:5.0V  晶振频率:8.000MHZ  PLL:开启
//作者: 李文华
//时间:2014年04月18日
///////////////////////////////////////////////////
#include <PIC18F45K80.h>
#include <htc.h>
#include "CAN.H"
#include "SIN.H"
#include "Detint.H"
#include "mcu.h"
#include "string.h"
//#include "stdlib.h"
//#include "time.h"




#define true 1
#define false 0

#define TIME500MS 100 //500ms定时 计数值(非定时器)
#define TIME1S 2 //1s定时 计数值(非定时器)
#define TIME30S 60 //30s定时 计数值(非定时器)
#define MAXADBUFNUM 10 //最大的AD数据缓冲区数目(计算平均值用)
#define FLASHCONT 20 //信号灯闪灯时间计数

#define STA_MECH_ON 0X01 //开机状态
#define STA_CAN_OK 0X02 //CAN 通信正常
#define STA_MASTER_OK 0X03 //主板数据就绪正常
#define STA_LAMP_OK 0X04 //接收到新的灯色命令状态
#define STA_MASTER_ERR 0X05 //主机错误状态

#define CAN_TIMEOUT 100 //CAN总线通信超时计数
#define MASTER_TIMEOUT 250 //主板通信超时计数

#define MAX_DATA_LENS 16 //串口数据包最大包长(含包长字节本身)

#define YEAR	  15//程序版本信息中的年
#define MONTH	  3//程序版本信息中的月
#define DAY		  5//程序版本信息中的日
const unsigned char board_version[5]={MYTYPE,YEAR,MONTH,DAY,0};//板卡程序版本

bit time5ms = false; //5ms定时标志位
bit time10ms = false; //10ms定时标志位

bit CANInt = false; //CAN中断标志位
bit ctrl_coil_bind=false;//线圈绑定控制位
bit det_result_send = false; //检测结果是否已经发送过(脉冲型检测使用)



unsigned int eeprom_addr = ADDR_ID;
unsigned char eeprom_num = NUM_ID;

unsigned char my_addr = 0x00; //暂存板卡的地址
unsigned char my_config = 0x00; //板卡的配置数据
unsigned char t1_cont = 0; //T1定时器中断次数计数器
unsigned char sta = 0x00; //板卡工作状态指示

unsigned char can_time_cont = 0x00,master_time_cont = 0x00; //CAN通信超时计数，主板超时计数

unsigned char t500ms_count =0;
/////////////////////////////////////////////////////////
unsigned char send_buffer[MAX_DATA_LENS];//串口1发送数据缓冲变量
unsigned char recv_buffer[(MAX_DATA_LENS+4)];//串口1接收数据缓冲变量


unsigned char recv1_byte_temp=0x00;//串口1接收字节缓冲变量
unsigned char recv1_data_p=0x00;//串口1接收数据指针
unsigned char uart1_timeout_counter;

bit recv1_data_ok=false;//接收到正确的数据包

///////////////////////////////////////////////////////
//CAN通信协议解析用到的定义
#define CANNOREPLY 0x00 //帧不需要回复
#define CANRQREPLY 0x01  //帧请求重发
#define CANCONFIRM 0x02 //帧接收确认
#define CANFBACK 0x03  //备用

#define CANREADID 0X00 //读取模块/板卡的ID
#define CANWRITEID 0X01 //写入模块/板卡的ID



#define CAN_COM_R_DET_RESULT  0X02  //主控板读取接口板16个通道检测结果命令
#define CAN_COM_R_SPEED_1TO4  0X03  //主控板读取接口板1-4组测速平均速度
#define CAN_COM_R_SPEED_5TO8  0X04  //主控板读取接口板5-8组测速平均速度
#define CAN_COM_R_SPEED_9TO12  0X05  //主控板读取接口板9-12组测速平均速度
#define CAN_COM_R_SPEED_13TO16  0X06  //主控板读取接口板13-16组测速平均速度
#define CAN_COM_R_COIL_BIND_1TO4  0X07 //主控板读取接口板1-4组测速通道的绑定情况
#define CAN_COM_R_COIL_BIND_5TO8  0x08 //主控板读取接口板5-8组测速通道的绑定情况
#define CAN_COM_R_COIL_DIST_1TO4  0X09 //主控板读取接口板1-4组测速通道的距离
#define CAN_COM_R_COIL_DIST_5TO8  0x0A //主控板读取接口板5-8组测速通道的距离
#define CAN_COM_R_COIL_DIST_9TO12 0x0b //主控板读取接口板9-12组测速通道的距离
#define CAN_COM_R_COIL_DIST_13TO16 0x0c //主控板读取接口板13-16组测速通道的距离
#define CAN_COM_W_COIL_BIND_1TO4  0X0d  //主控板写入接口板1-4组测速通道的绑定情况
#define CAN_COM_W_COIL_BIND_5TO8  0x0e  //主控板写入接口板5-8组测速通道的绑定情况
#define CAN_COM_W_COIL_DIST_1TO4  0X0f  //主控板写入接口板1-4组测速通道的距离
#define CAN_COM_W_COIL_DIST_5TO8  0x10  //主控板写入接口板5-8组测速通道的距离
#define CAN_COM_W_COIL_BIND_9TO12  0X11  //主控板写入接口板9-12组测速通道的绑定情况
#define CAN_COM_W_COIL_BIND_13TO16 0X12  //主控板写入接口板13-16组测速通道的绑定情况
#define CAN_COM_W_COIL_DIST_9TO12  0X13  //主控板写入接口板9-12组测速通道的距离
#define CAN_COM_W_COIL_DIST_13TO16 0x14  //主控板写入接口板13-16组测速通道的距离
#define CAN_COM_R_COIL_BIND_9TO12  0X15  //主控板读取接口板9-12组测速通道的绑定情况
#define CAN_COM_R_COIL_BIND_13TO16 0x16  //主控板读取接口板13-16组测速通道的绑定情况

#define CAN_COM_R_DET_SET  0x19//主控板写入检测方式设置
#define CAN_COM_W_DET_SET  0x1A//主控板写入检测方式设置
#define CAN_COM_C_COIL_BIND 0x1b//主控板下发写入线圈绑定情况的控制字

#define CANCOMRVER    0xff //读取程序版本

unsigned char CANRid[4]={0,0,0,0}; 
unsigned char CANRdata[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},CANRdlc = 8; 
unsigned char CANSid[4]={0,0,0,0}; 
unsigned char CANSdata[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},CANSdlc = 8; 
unsigned char f_r_mod = CANNOREPLY; //帧回复状态

/////////////////////////////////////////////////////////////////
//接口板参数相关变量

unsigned char det_work_mode = 0x00;//存储检测工作方式(存在型，脉冲型)

unsigned char det_ch_state[8] ={0,0,0,0,0,0,0,0};//存储32个通道的接口板工作状态
unsigned char det_coil_bind[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//存储线圈绑定数据
unsigned long int  det_coil_bind_grp=0x00000000;//线圈绑定组别
unsigned char det_coil_dist[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//存储线圈绑定距离
unsigned char CH_A[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//存储前线圈编号
unsigned char CH_B[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//存储后线圈编号

unsigned long int det_result=0x00000000;//32个通道车辆检测结果

unsigned char det_average_speed[16]={0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14};//存储8组线圈的平均速度
unsigned long int det_coil_bind_state=0x00000000;//接收到的线圈绑定情况
unsigned int det_speed_timer=0x0000;//各组线圈测速计时状态
unsigned char det_coil_time[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//存储8组线圈车辆通过时间
unsigned char det_coil_speed[16][10]={0x00};//存储16组线圈检测到的车速
unsigned int det_speed_time_ok=0x0000;//检测车辆时间填充完成标志
unsigned char fill_time_num[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//检测车辆速度时间计数
unsigned int speed_time_sum=0x0000;//检测时间之和
unsigned char det_average_speed_time=0x00;//最终取得的车速检测平均时间




////////////////////////////////////////////////////////////////
//端口初始化
void PORTInit(void)
{
	ANCON1 = 0X00;
	ANCON0 = 0X00;
	//ANCON2 = 0X00;
	
	RBIE=false;

	
	TRISA &= 0xF0;//LED_CAN(RA0),CHA0,CHA1,CHA2(RA1-RA3)为输出端
	TRISB &= 0xef;//LED_RUN(RB4)为输出端
	TRISC |= 0x0f;//MO0-MO3(RC0-RC3)为输入端
	TRISD = 0x0f;//RD0-RD3为输入端RD4-RD7为输出端

}

/////////////////////////////////
void IntManager(void)
{
	//允许全局中断
	INTCON |= 0xc0;

	
	//定时器0开启中断并启动赋初值
	TMR0IE = 1;  //开TMR0中断
	TMR0IF = 0;  //清标志位
	T08BIT = 0; //将TMR0设为16位定时器
	T0CS   = 0; //将TMR0设置为内部时钟定时器
	//赋初值
	TMR0H  = 0xd8; //  5ms
	TMR0L  = 0xef;
	//启动TMR0定时器
	TMR0ON = 1;
	//定时器1开启中断并启动赋初值
	TMR1IE = 1;  //开TMR1中断
	TMR1IF = 0;  //清标志位
	T1CON  = 0x00;//设置T1
	//赋初值
	TMR1H  =  0xd8; //
	TMR1L  = 0xef;
	//启动TMR0定时器
	T1CON  |= 0x01;
	
}
/////////////////////////////////////////
//串口1初始化
void Uart1Init()
{

//	SPBRGH1=0x00;//波特率发生器寄存器赋初值34
	SPBRG1=0x0c;//即波特率为115200bps;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	TXSTA1|=0x20;//BRG置1，高波特率，发送使能位使能
//	BAUDCON1|=0x08;//16位波特率发生器
	BAUDCON1&=0xf7;//8位波特率发生器
	TXSTA1&=0xa7;//异步模式
	RCSTA1|=0x90;//串口使能,接收使能

	
	//串口1 接收中断使能
	PIE1 |=0x20;
}
//////////////////////////////////////////////
//串口1发送单个字节
void Uart1SendByte(unsigned char byte)
{
	while(!TX1IF);
	TXREG1=byte;
}
/////////////////////////////////////////////////
//串口1发送数据包(需装入的参数不包括包头和校验)
void Uart1SendData(void)
{
	unsigned char i,j;
	unsigned char temp[4];
	Uart1SendByte(0x02);//数据包的包头

	temp[0] = det_result;
	temp[1] = det_result>>8;
	temp[2]=det_result>>16;
	temp[3]=det_result>>24;
	
	for(i=0;i<4;i++)
		{
			Uart1SendByte(temp[i]);
		}
}
////////////////////////////////////////////////
//串口1接收数据包
void Uart1RecvData()//
{
unsigned char i=0,j=0;
if(RC1IF)
{
	recv_buffer[recv1_data_p]=RCREG1;//读取接收缓冲区的数据
	recv1_data_p++;
	RC1IF=false;
	uart1_timeout_counter=0;
	if(recv1_data_p<=3)//定义数据长度未包括包头和包长3个字节,+4
	{		
		if(recv_buffer[0]==0xaa)//包头字节
		{
			if(recv1_data_p>2)
			{
				if(recv_buffer[1]==0x55)
				{	
					if(0x02==recv_buffer[2])
					{
						//LED_CAN = !LED_CAN;
						recv1_data_ok=true;//接收到正确完整数据包标志位置位
						
					//	Uart1SendData();
						recv1_data_p=0;
					}
					else
					{
						recv1_data_p=0;
					}				
				}					
				else
				{
					recv1_data_p=0;
				}
			}
		}
		else
		{
			recv1_data_p=0;
		}
	}
	else
	{
		recv1_data_p=0;
	}
	
}

}
//////////////////////////////////////////////
void Uart1TimeOut()
{
	uart1_timeout_counter++;
	if(uart1_timeout_counter==199)
		{
			memset(recv_buffer, 0x00, 20);
			recv1_data_p=0;
		}
	if(uart1_timeout_counter>200)
		{
		uart1_timeout_counter=200;
		}
}

	

////////////////////////////////////////////////////////////////
//定时器0中断服务
void TMR0IntServer(void)
{
	unsigned char i;
	// 5ms
	TMR0H  = 0xd8; //
	TMR0L  = 0xef;
	
	time5ms = true;
/* 	
	TMR0H  = 0xb1; // 10ms
	TMR0L  = 0xdf;
	time10ms = true;
	*/for(i=0;i<16;i++)
		{
			if(det_speed_timer&(0x0001<<i))
				{
					det_coil_time[i]++;
				}
		}
		
}
/////////////////////////////////////////////////////
//定时器1中断服务
void TMR1IntServer(void)
{	
	TMR1H  = 0xb1; // 10ms
	TMR1L  = 0xdf;
/*	
	TMR1H  = 0x3c; // 25ms
	TMR1L  = 0xaf;
*/	
	t1_cont++;

	if(t1_cont>=FLASHCONT)
		{
			t1_cont = 0;
				
		}
	
}
///////////////////////////////////////////////////////////////
//中断入口服务
void interrupt IntServer(void)
{

	//定时器0中断判断
	if(TMR0IE && TMR0IF)
		{

			TMR0IF = 0;
			TMR0IntServer();
		}
	//定时器1中断判断
	if(TMR1IE && TMR1IF)
		{

			TMR1IF = 0;
			TMR1IntServer();
		}

	//发生CAN中断
	if((CANSTAT & 0x0e)!= 0x00)
		{
			CANInt=true;
		}
	if(RC1IE&&RC1IF)//串口1中断
		{
			Uart1RecvData();
		}

}

///////////////////////////////////////////////////////////////////////////////////////
//发送测试数据
void CANSendTest(void)
{
	CANSdlc = 8;
	CANSdata[0]=det_coil_bind_grp;
	CANSdata[1]=det_coil_bind_grp>>8;
	CANSdata[2]=det_coil_bind_grp>>16;
	CANSdata[3]=det_coil_bind_grp>>24;
	CANSdata[4]=det_coil_bind_grp&0x000000ff;
	CANSdata[5]=det_coil_bind_grp>>8&0x000000ff;
	CANSdata[6]=det_coil_bind_grp>>16&0x000000ff;
	CANSdata[7]=det_coil_bind_grp>>24&0x000000ff;
	CANSend(CANSid,CANSdata,CANSdlc);
}
void CANSendTest2(unsigned long int  temp)
{
	CANSdlc =4;
	CANSdata[0]=temp;
	CANSdata[1]=temp>>8;
	CANSdata[2]=temp>>16;
	CANSdata[3]=temp>>24;		
	CANSend(CANSid,CANSdata,CANSdlc);
}

void CANSendTest3(unsigned char  tempt[16])
{
	CANSdlc =7;
	CANSdata[0]=tempt[0];
	CANSdata[1]=tempt[1];
	CANSdata[2]=tempt[2];
	CANSdata[3]=tempt[3];		
	CANSdata[4]=tempt[4];
	CANSdata[5]=tempt[5];
	CANSdata[6]=tempt[6];		
	CANSend(CANSid,CANSdata,CANSdlc);
}
void CANSendTest4(unsigned char  tempt)
{
	CANSdlc =1;
	CANSdata[0]=tempt;
	CANSend(CANSid,CANSdata,CANSdlc);
}

	
///////////////////////////////////////////////////////////////////////////////////////
//发送错误指示
void CANSendError(void)
{
	CANSdlc = 8;
	CANSdata[0]=0xff;
	CANSdata[1]=0xff;
	CANSdata[2]=0xff ;
	CANSdata[3]=0xff;
	CANSdata[4]=0xff;
	CANSdata[5]=0xff;
	CANSdata[6]=0xff;
	CANSdata[7]=0xff;
	CANSend(CANSid,CANSdata,CANSdlc);
}

///////////////////////////////////////////////////////////////////////
//线圈绑定验证是否合格
bit CoilBindErrSet(void)
{
	unsigned char i;
	unsigned char coil_bind_eror=0x00;
	unsigned char bind_num1=0x00;
	unsigned char bind_num2=0x00;
	unsigned long int tempt;
	if(!ctrl_coil_bind)
		{
			CANSendError();
			return 0;
		}
	for(i=0;i<4;i++)
		{
		bind_num1=(CANRdata[i+1]&0x0f)+((CANRdata[5]>>(i*2))&0x01)*16;
		bind_num2=((CANRdata[i+1]&0xf0)>>4)+((CANRdata[5]>>(i*2+1))&0x01)*16;
		//CANSendTest2(bind_num1);
		//CANSendTest2(bind_num2);
		//CANSendTest2(det_coil_bind_state);
		if(((det_coil_bind_state>>bind_num1)&0x00000001)||((det_coil_bind_state>>bind_num2)&0x00000001)||(bind_num1==bind_num2))
			//检测线圈绑定是否有重复
			{
			
				coil_bind_eror=true;
				CANRdata[i+1]=0xff;
				CANRdata[0]=0x1b;
				CANSend(CANSid,CANRdata,CANRdlc);
				break;
			}
		else
			{
				if(bind_num1<=15)
				{
					det_coil_bind_state|=((0x00000001<<bind_num1)&0x0000ffff);		//将绑定的线圈位置1，标记为已绑定
					
				//	CANSendTest2(det_coil_bind_state);
				}
				else
				{
					det_coil_bind_state|=0x00010000<<(bind_num1-16);
				}
				if(bind_num2<=15)
				{
					det_coil_bind_state|=((0x00000001<<bind_num2)&0x0000ffff);		//将绑定的线圈位置1，标记为已绑定
					
					//CANSendTest2(det_coil_bind_state);
				}
				else
				{
					det_coil_bind_state|=0x00010000<<(bind_num2-16);
				}
			//	CANSendTest2(det_coil_bind_state);
			//	det_coil_bind[i]=CANRdata[i+1];
				coil_bind_eror=false;
			//	CANSendTest();
			}
		}
	
	if(coil_bind_eror)												//绑定不合格返回0，合格返回1
		{
			return 0;
		}
	else
		{
			return 1;
		}
}

//////////////////////////////////////////////////////////////////////////////////
//CAN数据确认回复
//将刚收到的CAN数据再原包打回
//更改ID以及回复模式
void CANSendConfirm(void)
{
	unsigned char i;

	for(i=0;i<8;i++)
		{
			CANSdata[i] = CANRdata[i];
		}
	CANSdata[0] &= 0x3f; //将回复模式设置为00 不需要回复
	CANSdlc = CANRdlc;
	CANSend(CANSid,CANSdata,CANSdlc);
}
//////////////////////////////////////////////////////////////////////////////////
//CAN发送板的ID给主板
void CANSendID(unsigned char sm)
{

	CANSdlc = NUM_ID + 1;

	if(sm == CANREADID)
		CANSdata[0] = CANREADID; //不需要回复,发送的是ID
	else
		CANSdata[0] = CANWRITEID; //不需要回复,发送的是ID

	//从EEPROM中读取ID信息
	eeprom_addr = ADDR_ID;
	eeprom_num = NUM_ID;
	EEPROMRead(&eeprom_addr,&eeprom_num,(CANSdata+1));
	CANSend(CANSid,CANSdata,CANSdlc);
}
//////////////////////////////////////////////////////////////////////////////////
//CAN发送板的ID给主板 将ID信息写入到EEPROM 根据帧请求回复ID信息
void CANWriteID(unsigned char fm)
{		
	eeprom_addr = ADDR_ID;
	eeprom_num = NUM_ID;
	EEPROMWrite(&eeprom_addr,&eeprom_num,(CANRdata+1));
	if(fm==CANRQREPLY)
		{
			CANSendID(CANWRITEID);
		}
}

//////////////////////////////////////////////////////////////////////////////////
//读取检测结果,并发送给主控板
void CANReadDetResult()
{
	CANSdata[0] = CAN_COM_R_DET_RESULT; //将回复模式设置为00 不需要回复
	CANSdlc = 5;

	CANSdata[1] = det_result;
	CANSdata[2] = det_result>>8;
	CANSdata[3]=det_result>>16;
	CANSdata[4]=det_result>>24;
	
/*	

	
	CANSdata[1] =	dt_int[0];
	CANSdata[2] =	dt_int[1];
	CANSdata[3] =	dt_int[2];
	CANSdata[4] =	dt_int[3];
*/	CANSend(CANSid,CANSdata,CANSdlc);
	if(!(det_work_mode & 0x01))
		{ //脉冲型检测 
			det_result_send = true; //为脉冲型检测提供标志
		}

}
////////////////////////////////////////////////////////////////////////////////////
//读取1－4通道平均速度
void CANReadSpeed1TO4(void)
{
	CANSdata[0]=CAN_COM_R_SPEED_1TO4;
	CANSdlc=5;
	
	CANSdata[1]=det_average_speed[0];
	CANSdata[2]=det_average_speed[1];
	CANSdata[3]=det_average_speed[2];
	CANSdata[4]=det_average_speed[3];
	CANSend(CANSid,CANSdata,CANSdlc);
}
////////////////////////////////////////////////////////////////////////////////////
//读取5-8通道平均速度
void CANReadSpeed5TO8(void)
{
	CANSdata[0]=CAN_COM_R_SPEED_5TO8;
	CANSdlc=5;

	CANSdata[1]=det_average_speed[4];
	CANSdata[2]=det_average_speed[5];
	CANSdata[3]=det_average_speed[6];
	CANSdata[4]=det_average_speed[7];
	CANSend(CANSid,CANSdata,CANSdlc);
}
////////////////////////////////////////////////////////////////////////////////////
//读取9－12通道平均速度
void CANReadSpeed9TO12(void)
{
	CANSdata[0]=CAN_COM_R_SPEED_9TO12;
	CANSdlc=5;
	
	CANSdata[1]=det_average_speed[8];
	CANSdata[2]=det_average_speed[9];
	CANSdata[3]=det_average_speed[10];
	CANSdata[4]=det_average_speed[11];
	CANSend(CANSid,CANSdata,CANSdlc);
}
////////////////////////////////////////////////////////////////////////////////////
//读取13-16通道平均速度
void CANReadSpeed13TO16(void)
{
	CANSdata[0]=CAN_COM_R_SPEED_13TO16;
	CANSdlc=5;

	CANSdata[1]=det_average_speed[12];
	CANSdata[2]=det_average_speed[13];
	CANSdata[3]=det_average_speed[14];
	CANSdata[4]=det_average_speed[15];
	CANSend(CANSid,CANSdata,CANSdlc);
}
/////////////////////////////////////////////////////////////////////////////////////
//读取1－4组线圈绑定情况 并发送给主板
void CANReadCoilBind1To4()
{
	CANSdata[0]=CAN_COM_R_COIL_BIND_1TO4;
	CANSdlc=6;
	CANSdata[1]=det_coil_bind[0];
	CANSdata[2]=det_coil_bind[1];
	CANSdata[3]=det_coil_bind[2];
	CANSdata[4]=det_coil_bind[3];
	CANSdata[5]=det_coil_bind_grp;
	CANSend(CANSid,CANSdata,CANSdlc);

}
/////////////////////////////////////////////////////////////////////////////////////
//读取5-8组线圈绑定情况 并发送给主板
void CANReadCoilBind5To8()
{
	CANSdata[0]=CAN_COM_R_COIL_BIND_5TO8;
	CANSdlc=6;
	CANSdata[1]=det_coil_bind[4];
	CANSdata[2]=det_coil_bind[5];
	CANSdata[3]=det_coil_bind[6];
	CANSdata[4]=det_coil_bind[7];
	CANSdata[5]=det_coil_bind_grp>>8;
	CANSend(CANSid,CANSdata,CANSdlc);
}
/////////////////////////////////////////////////////////////////////////////////////
//读取9-12组线圈绑定情况 并发送给主板
void CANReadCoilBind9To12()
{
	CANSdata[0]=CAN_COM_R_COIL_BIND_9TO12;
	CANSdlc=6;
	CANSdata[1]=det_coil_bind[8];
	CANSdata[2]=det_coil_bind[9];
	CANSdata[3]=det_coil_bind[10];
	CANSdata[4]=det_coil_bind[11];
	CANSdata[5]=det_coil_bind_grp>>16;
	CANSend(CANSid,CANSdata,CANSdlc);
}

/////////////////////////////////////////////////////////////////////////////////////
//读取13-16组线圈绑定情况 并发送给主板
void CANReadCoilBind13To16()
{
	CANSdata[0]=CAN_COM_R_COIL_BIND_13TO16;
	CANSdlc=6;
	CANSdata[1]=det_coil_bind[12];
	CANSdata[2]=det_coil_bind[13];
	CANSdata[3]=det_coil_bind[14];
	CANSdata[4]=det_coil_bind[15];
	CANSdata[5]=(det_coil_bind_grp&0xff000000)>>24;
	CANSend(CANSid,CANSdata,CANSdlc);
}
///////////////////////////////////////////////////////////////////////////////////
//读取1－4组线圈绑定距离 并发送给主板
void CANReadCoilDist1TO4()
{
	CANSdata[0]=CAN_COM_R_COIL_DIST_1TO4;
	CANSdlc=5;
	eeprom_addr=ADDR_COIL_DIST;
	eeprom_num=NUM_COIL_DIST/4;
	EEPROMRead(&eeprom_addr,&eeprom_num,(CANSdata+1));
	CANSend(CANSid,CANSdata,CANSdlc);

}
///////////////////////////////////////////////////////////////////////////////////
//读取5-8组线圈绑定距离 并发送给主板
void CANReadCoilDist5TO8()
{
	CANSdata[0]=CAN_COM_R_COIL_DIST_5TO8;
	CANSdlc=5;
	eeprom_addr=ADDR_COIL_DIST+4;
	eeprom_num=NUM_COIL_DIST/4;
	EEPROMRead(&eeprom_addr,&eeprom_num,(CANSdata+1));
	CANSend(CANSid,CANSdata,CANSdlc);
}
///////////////////////////////////////////////////////////////////////////////////
//读取9－12组线圈绑定距离 并发送给主板
void CANReadCoilDist9TO12()
{
	CANSdata[0]=CAN_COM_R_COIL_DIST_9TO12;
	CANSdlc=5;
	eeprom_addr=ADDR_COIL_DIST+8;
	eeprom_num=NUM_COIL_DIST/4;
	EEPROMRead(&eeprom_addr,&eeprom_num,(CANSdata+1));
	CANSend(CANSid,CANSdata,CANSdlc);

}
///////////////////////////////////////////////////////////////////////////////////
//读取13-16组线圈绑定距离 并发送给主板
void CANReadCoilDist13TO16()
{
	CANSdata[0]=CAN_COM_R_COIL_DIST_13TO16;
	CANSdlc=5;
	eeprom_addr=ADDR_COIL_DIST+12;
	eeprom_num=NUM_COIL_DIST/4;
	EEPROMRead(&eeprom_addr,&eeprom_num,(CANSdata+1));
	CANSend(CANSid,CANSdata,CANSdlc);
}

///////////////////////////////////////////////////////////////////////////////////
//主控板写入1-4通道线圈绑定情况
void CANWriteCoilBIND1TO4(unsigned char fm)
{
unsigned char i;
if(CoilBindErrSet())								//检查绑定信息是否合格
{
	eeprom_addr=ADDR_COIL_BIND;
	eeprom_num=NUM_COIL_BIND/4;
	EEPROMWrite(&eeprom_addr,&eeprom_num,(CANRdata+1));
	
	det_coil_bind[0]=CANRdata[1];
	det_coil_bind[1]=CANRdata[2];
	det_coil_bind[2]=CANRdata[3];
	det_coil_bind[3]=CANRdata[4];
	det_coil_bind_grp&=0xffffff00;
	det_coil_bind_grp+=CANRdata[5];
	eeprom_addr=ADDR_COIL_BIND_GRP;
	eeprom_num=NUM_COIL_BIND_GRP;
	EEPROMWrite(&eeprom_addr,&eeprom_num,&det_coil_bind_grp);
	if(fm==CANRQREPLY)
		{
			//回发当前配置给主板
			CANSdata[0]=CAN_COM_W_COIL_BIND_1TO4;
			CANSdlc=6;
			CANSdata[1]=det_coil_bind[0];
			CANSdata[2]=det_coil_bind[1];
			CANSdata[3]=det_coil_bind[2];
			CANSdata[4]=det_coil_bind[3];
			CANSdata[5]=det_coil_bind_grp;//&0x000000ff;
			CANSend(CANSid,CANSdata,CANSdlc);
		}	
	
	for(i=0;i<=3;i++)							//将绑定的前后线圈归类分组
		{
			CH_A[i]=(det_coil_bind[i]&0x0f)+((det_coil_bind_grp>>(i*2))&0x00000001)*16;
			CH_B[i]=((det_coil_bind[i]&0xf0)>>4)+(det_coil_bind_grp>>(i*2+1)&0x00000001)*16;
		}
}
}
///////////////////////////////////////////////////////////////////////////////////
//主控板写入5－8通道线圈绑定情况
void CANWriteCoilBIND5TO8(unsigned char fm)
{
	unsigned char i;
	if(CoilBindErrSet())
	{
		eeprom_addr=ADDR_COIL_BIND+4;
		eeprom_num=NUM_COIL_BIND/4;
		EEPROMWrite(&eeprom_addr,&eeprom_num,(CANRdata+1));
		det_coil_bind[4]=CANRdata[1];
		det_coil_bind[5]=CANRdata[2];
		det_coil_bind[6]=CANRdata[3];
		det_coil_bind[7]=CANRdata[4];
		det_coil_bind_grp&=0xffff00ff;
		det_coil_bind_grp+=CANRdata[5]<<8;
		eeprom_addr=ADDR_COIL_BIND_GRP;
		eeprom_num=NUM_COIL_BIND_GRP;
		EEPROMWrite(&eeprom_addr,&eeprom_num,&det_coil_bind_grp);
		if(fm==CANRQREPLY)
			{
				//回发当前配置给主板
				CANSdata[0]=CAN_COM_W_COIL_BIND_5TO8;
				CANSdlc=6;
				CANSdata[1]=det_coil_bind[4];
				CANSdata[2]=det_coil_bind[5];
				CANSdata[3]=det_coil_bind[6];
				CANSdata[4]=det_coil_bind[7];
				CANSdata[5]=(det_coil_bind_grp&0x0000ff00)>>8;
				CANSend(CANSid,CANSdata,CANSdlc);
			}	
		
		for(i=4;i<=7;i++)							//将绑定的前后线圈归类分组
			{
				CH_A[i]=(det_coil_bind[i]&0x0f)+((det_coil_bind_grp>>(i*2))&0x00000001)*16;
				CH_B[i]=((det_coil_bind[i]&0xf0)>>4)+(det_coil_bind_grp>>(i*2+1)&0x00000001)*16;
			}
	}
}
////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
//主控板写入9-12通道线圈绑定情况
void CANWriteCoilBIND9TO12(unsigned char fm)
{
	unsigned char i;
	unsigned long int tempt1;
if(CoilBindErrSet())								//检查绑定信息是否合格
{
	eeprom_addr=ADDR_COIL_BIND+8;
	eeprom_num=NUM_COIL_BIND/4;
	EEPROMWrite(&eeprom_addr,&eeprom_num,(CANRdata+1));
	det_coil_bind[8]=CANRdata[1];
	det_coil_bind[9]=CANRdata[2];
	det_coil_bind[10]=CANRdata[3];
	det_coil_bind[11]=CANRdata[4];
	det_coil_bind_grp&=0xff00ffff;
	tempt1=CANRdata[5]<<8;
	det_coil_bind_grp+=(tempt1<<8);
	eeprom_addr=ADDR_COIL_BIND_GRP;
	eeprom_num=NUM_COIL_BIND_GRP;
	EEPROMWrite(&eeprom_addr,&eeprom_num,&det_coil_bind_grp);
	if(fm==CANRQREPLY)
		{
			//回发当前配置给主板
			CANSdata[0]=CAN_COM_W_COIL_BIND_9TO12;
			CANSdlc=6;
			CANSdata[1]=det_coil_bind[8];
			CANSdata[2]=det_coil_bind[9];
			CANSdata[3]=det_coil_bind[10];
			CANSdata[4]=det_coil_bind[11];
			CANSdata[5]=(det_coil_bind_grp&0x00ff0000)>>16;
			CANSend(CANSid,CANSdata,CANSdlc);
		}	
	for(i=8;i<=11;i++)							//将绑定的前后线圈归类分组
		{
			CH_A[i]=(det_coil_bind[i]&0x0f)+((det_coil_bind_grp>>(i*2))&0x00000001)*16;
			CH_B[i]=((det_coil_bind[i]&0xf0)>>4)+(det_coil_bind_grp>>(i*2+1)&0x00000001)*16;
		}
}
}
///////////////////////////////////////////////////////////////////////////////////
//主控板写入13-16通道线圈绑定情况
void CANWriteCoilBIND13TO16(unsigned char fm)
{
	unsigned char i;
	unsigned long int tempt2;
	if(CoilBindErrSet())
	{
		eeprom_addr=ADDR_COIL_BIND+12;
		eeprom_num=NUM_COIL_BIND/4;
		EEPROMWrite(&eeprom_addr,&eeprom_num,(CANRdata+1));
		det_coil_bind[12]=CANRdata[1];
		det_coil_bind[13]=CANRdata[2];
		det_coil_bind[14]=CANRdata[3];
		det_coil_bind[15]=CANRdata[4];
		det_coil_bind_grp&=0x00ffffff;
		tempt2=CANRdata[5]<<8;
		tempt2<<=16;
		det_coil_bind_grp+=tempt2;
		eeprom_addr=ADDR_COIL_BIND_GRP;
		eeprom_num=NUM_COIL_BIND_GRP;
		EEPROMWrite(&eeprom_addr,&eeprom_num,&det_coil_bind_grp);
		if(fm==CANRQREPLY)
			{
				//回发当前配置给主板
				CANSdata[0]=CAN_COM_W_COIL_BIND_13TO16;
				CANSdlc=6;
				CANSdata[1]=det_coil_bind[12];
				CANSdata[2]=det_coil_bind[13];
				CANSdata[3]=det_coil_bind[14];
				CANSdata[4]=det_coil_bind[15];
				CANSdata[5]=(det_coil_bind_grp&0xff000000)>>24;
				CANSend(CANSid,CANSdata,CANSdlc);
			}	
		for(i=12;i<=15;i++)							//将绑定的前后线圈归类分组
			{
				CH_A[i]=(det_coil_bind[i]&0x0f)+((det_coil_bind_grp>>(i*2))&0x00000001)*16;
				CH_B[i]=((det_coil_bind[i]&0xf0)>>4)+(det_coil_bind_grp>>(i*2+1)&0x00000001)*16;
			}
	}
}
////////////////////////////////////////////////////////////////////////////////////
//主板写入1－4组线圈绑定距离
void CANWriteCoilDist1TO4(unsigned char fm)
{
	eeprom_addr=ADDR_COIL_DIST;
	eeprom_num=NUM_COIL_DIST/4;
	EEPROMWrite(&eeprom_addr,&eeprom_num,(CANRdata+1));
	det_coil_dist[0]=CANRdata[1];
	det_coil_dist[1]=CANRdata[2];
	det_coil_dist[2]=CANRdata[3];
	det_coil_dist[3]=CANRdata[4];
	if(fm==CANRQREPLY)
			{
				//回发当前配置给主板
				CANSdata[0]=CAN_COM_W_COIL_DIST_1TO4;
				CANSdlc=5;
				eeprom_addr=ADDR_COIL_DIST;
				eeprom_num=NUM_COIL_DIST/4;
				EEPROMRead(&eeprom_addr,&eeprom_num,(CANSdata+1));
				CANSend(CANSid,CANSdata,CANSdlc);
			}	
}
////////////////////////////////////////////////////////////////////////////////////
//主板写入5-8组线圈绑定距离
void CANWriteCoilDist5TO8(unsigned char fm)
{
	eeprom_addr=ADDR_COIL_DIST+4;
	eeprom_num=NUM_COIL_DIST/4;
	EEPROMWrite(&eeprom_addr,&eeprom_num,(CANRdata+1));
	det_coil_dist[4]=CANRdata[1];
	det_coil_dist[5]=CANRdata[2];
	det_coil_dist[6]=CANRdata[3];
	det_coil_dist[7]=CANRdata[4];
	if(fm==CANRQREPLY)
			{
				//回发当前配置给主板
				CANSdata[0]=CAN_COM_W_COIL_DIST_5TO8;
				CANSdlc=5;
				eeprom_addr=ADDR_COIL_DIST+4;
				eeprom_num=NUM_COIL_DIST/4;
				EEPROMRead(&eeprom_addr,&eeprom_num,(CANSdata+1));
				CANSend(CANSid,CANSdata,CANSdlc);
			}	
}
////////////////////////////////////////////////////////////////////////////////////
//主板写入9-12组线圈绑定距离
void CANWriteCoilDist9TO12(unsigned char fm)
{
	eeprom_addr=ADDR_COIL_DIST+8;
	eeprom_num=NUM_COIL_DIST/4;
	EEPROMWrite(&eeprom_addr,&eeprom_num,(CANRdata+1));
	det_coil_dist[8]=CANRdata[1];
	det_coil_dist[9]=CANRdata[2];
	det_coil_dist[10]=CANRdata[3];
	det_coil_dist[11]=CANRdata[4];
	if(fm==CANRQREPLY)
			{
				//回发当前配置给主板
				CANSdata[0]=CAN_COM_W_COIL_DIST_9TO12;
				CANSdlc=5;
				eeprom_addr=ADDR_COIL_DIST+8;
				eeprom_num=NUM_COIL_DIST/4;
				EEPROMRead(&eeprom_addr,&eeprom_num,(CANSdata+1));
				CANSend(CANSid,CANSdata,CANSdlc);
			}	
}
////////////////////////////////////////////////////////////////////////////////////
//主板写入13-16组线圈绑定距离
void CANWriteCoilDist13TO16(unsigned char fm)
{
	eeprom_addr=ADDR_COIL_DIST+12;
	eeprom_num=NUM_COIL_DIST/4;
	EEPROMWrite(&eeprom_addr,&eeprom_num,(CANRdata+1));
	det_coil_dist[12]=CANRdata[1];
	det_coil_dist[13]=CANRdata[2];
	det_coil_dist[14]=CANRdata[3];
	det_coil_dist[15]=CANRdata[4];
	if(fm==CANRQREPLY)
			{
				//回发当前配置给主板
				CANSdata[0]=CAN_COM_W_COIL_DIST_13TO16;
				CANSdlc=5;
				eeprom_addr=ADDR_COIL_DIST+12;
				eeprom_num=NUM_COIL_DIST/4;
				EEPROMRead(&eeprom_addr,&eeprom_num,(CANSdata+1));
				CANSend(CANSid,CANSdata,CANSdlc);
			}	
}

///////////////////////////////////////////////////////////////////////////////
//主控板读取检测工作方式
void CANReadDetSet(void)
{
	CANSdata[0] = CAN_COM_W_DET_SET; 
	CANSdlc = 2;

	eeprom_addr = ADDR_DET_CFG;
	eeprom_num = NUM_DET_CFG;

	EEPROMRead(&eeprom_addr,&eeprom_num,&det_work_mode);
	 CANSdata[1]=det_work_mode;

	CANSend(CANSid,CANSdata,CANSdlc);

}

//////////////////////////////////////////////////////////////////////////////////
//主控板写入检测工作方式
void CANWriteDetSet(unsigned char fm)
{
	eeprom_addr = ADDR_DET_CFG;
	eeprom_num = NUM_DET_CFG;

	EEPROMWrite(&eeprom_addr,&eeprom_num,(CANRdata+1));
	det_work_mode = CANRdata[1];
	//回复判断
		if(fm==CANRQREPLY)
			{
				//回发当前配置给主板
				CANReadDetSet();
			}	
}

//////////////////////////////////////////////////////////////////////////////////
//主板写入线圈绑定控制字
//同时将原线圈绑定归0
void CANControlCoilBind(void)
{
	//unsigned char ctrl_coil_bind=0x00;
	//ctrl_coil_bind=CANRdata[2];
	unsigned char i;
	if(CANRdata[1]==0x00)
		{
			eeprom_addr=ADDR_COIL_BIND;
			eeprom_num=NUM_COIL_BIND;
			for(i=0;i<NUM_COIL_BIND;i++)
				{
					det_coil_bind[i]=0x00;	//原线圈绑定情况清0
				}
			EEPROMWrite(&eeprom_addr,&eeprom_num,det_coil_bind);
			det_coil_bind_grp=0x00000000;
			
			eeprom_addr=ADDR_COIL_BIND_GRP;
			eeprom_num=NUM_COIL_BIND_GRP;			
			EEPROMWrite(&eeprom_addr,&eeprom_num,det_coil_bind_grp);
			det_coil_bind_state=0x00000000;//将所有线圈的绑定标志位清0
			ctrl_coil_bind=true;		//线圈绑定写入使能
		}
	else
		{
			ctrl_coil_bind=false;
		}
}
void CANReadVersion()
{
	
	CANSdlc = 6;	
	CANSdata[0] =CANCOMRVER;
	CANSdata[1]=board_version[0];
	CANSdata[2]=board_version[1];
	CANSdata[3]=board_version[2];
	CANSdata[4]=board_version[3];
	CANSdata[5]=board_version[4];
	
	CANSend(CANSid,CANSdata,CANSdlc);
}

//////////////////////////////////////////////////////////////////////////////////
//CAN数据解析
void CANDATAAnalyzing(unsigned char id[],unsigned char dat[],unsigned char dlc)
{
unsigned char i;

	if((CANRid[0] != 0x09) || (CANRid[1] != 0x061)) //去除电源板
		{
		//	CANSendTest(CANRid);
			//解析帧的回复模式
			switch (((dat[0] & 0xc0)>>6))
				{
				case CANNOREPLY:
					f_r_mod = CANNOREPLY; //该帧不需要回复
					break;
				case CANRQREPLY:
					f_r_mod = CANRQREPLY;//该帧请求回复(回复的是板卡内部产生的数据)
					break;
				case CANCONFIRM:
					f_r_mod = CANCONFIRM;//该帧需要确认回复(回复的是主板发过来的数据以确认是否接收正确)
					break;
				case CANFBACK:
					f_r_mod = CANFBACK;//该帧不需要回复，备用
					break;
				default:
					f_r_mod = CANNOREPLY;//该帧不需要回复
					break;
				}
			if(dat[0]==CANCOMRVER)//主板读取板卡程序版本
					CANReadVersion();
					
			//判断是否是心跳报
			if(((id[1]>>5)&CAN_FMOD_HB)==CAN_FMOD_HB)
				{
					;
				}
			else
				{

						//数据要求确认回复
						if(f_r_mod ==CANCONFIRM)//回复确认
							{
								CANSendConfirm();
							}
						
						//解析该数据帧的具体用途
						switch ((dat[0] & 0x3f))
							{
							case CANREADID:
								//该帧用于读取板卡的ID信息
								if(f_r_mod ==CANRQREPLY) //有请求回复
									{
										CANSendID(CANREADID);
									}			
								break;
							case CANWRITEID:
								//该帧用于写入板卡的ID信息
								CANWriteID(f_r_mod);
								break;
							case CAN_COM_R_DET_RESULT: 
								//主机读取接口板检测结果
								CANReadDetResult(); 
								break;
							case CAN_COM_R_SPEED_1TO4:
								//主板读取1-4组平均速度
								CANReadSpeed1TO4();
								break;
							case CAN_COM_R_SPEED_5TO8:
								//主板读取5-8组平均速度
								CANReadSpeed5TO8();
								break;
							case CAN_COM_R_SPEED_9TO12:
								//主板读取9-12组平均速度
								CANReadSpeed9TO12();
								break;
							case CAN_COM_R_SPEED_13TO16:
								//主板读取13-16组平均速度
								CANReadSpeed13TO16();
								break;								
							case CAN_COM_R_COIL_BIND_1TO4:
								//主板读取1-4通道测速线圈绑定情况
								CANReadCoilBind1To4();
								break;
							case CAN_COM_R_COIL_BIND_5TO8:
								//主板读取5-8通道测速线圈绑定情况
								CANReadCoilBind5To8();
								break;
							case CAN_COM_R_COIL_DIST_1TO4:
								//主板读取1-4通道测速线圈的距离设置
								CANReadCoilDist1TO4();
								break;
							case CAN_COM_R_COIL_DIST_5TO8:
								//主板读取5-8通道测速线圈距离设置
								CANReadCoilDist5TO8();
								break;
							case CAN_COM_R_COIL_DIST_9TO12:
								//主板读取9-12通道测速线圈的距离设置
								CANReadCoilDist9TO12();
								break;
							case CAN_COM_R_COIL_DIST_13TO16:
								//主板读取13-16通道测速线圈距离设置
								CANReadCoilDist13TO16();
								break;
							case CAN_COM_W_COIL_BIND_1TO4:
								//主板写入1-4通道测速线圈绑定情况
								CANWriteCoilBIND1TO4(f_r_mod);
								break;
							case CAN_COM_W_COIL_BIND_5TO8:
								//主板写入5-8通道测速线圈绑定情况
								CANWriteCoilBIND5TO8(f_r_mod);	
								break;
							case CAN_COM_W_COIL_DIST_1TO4:
								//主板写入1-4通道测速线圈距离
								CANWriteCoilDist1TO4(f_r_mod);	
								break;
							case CAN_COM_W_COIL_DIST_5TO8:
								//主板读取5-8通道测速线圈距离
								CANWriteCoilDist5TO8(f_r_mod);	
								break;
							case CAN_COM_W_COIL_BIND_9TO12:
								//主控板写入接口板9-12组测速通道的绑定情况
								CANWriteCoilBIND9TO12(f_r_mod);
								break;
							case CAN_COM_W_COIL_BIND_13TO16:
								//主控板写入接口板13-16组测速通道的绑定情况
								CANWriteCoilBIND13TO16(f_r_mod);
								break;
							case CAN_COM_W_COIL_DIST_9TO12:
								//主控板写入接口板9-12组测速通道的距离
								CANWriteCoilDist9TO12(f_r_mod);	
								break;
							case CAN_COM_W_COIL_DIST_13TO16:
								//主控板写入接口板13-16组测速通道的距离
								CANWriteCoilDist13TO16(f_r_mod);	
								break;
							case CAN_COM_R_COIL_BIND_9TO12:
								//主控板读取接口板9-12组测速通道的绑定情况
								CANReadCoilBind9To12();
								break;
							case CAN_COM_R_COIL_BIND_13TO16:
								//主控板读取接口板13-16组测速通道的绑定情况
								CANReadCoilBind13To16();
								break;
							case CAN_COM_R_DET_SET:
								//主板读取检测工作方式
								CANReadDetSet();
								break;
							case CAN_COM_W_DET_SET:
								//主板写入检测工作方式
								CANWriteDetSet(f_r_mod);
								break;
							case CAN_COM_C_COIL_BIND:
								//主板下发写入线圈绑定情况控制字
								CANControlCoilBind();
								break;		
							default:
								//该帧表示的是未定义数据
								break;
							}

						master_time_cont = 0x00;//清零计数器
						sta = STA_MASTER_OK; //置于主机正确状态
				}
		}
}

//////////////////////////////////////////////////////////////////////
void GetAverageSpeed(void)
{
//	eeprom_addr=ADDR_COIL_BIND;
//	det_coil_bind_1to4
	unsigned char i,j;
	unsigned char min_speed=0xff;//存储最小速度
	unsigned char max_speed=0x00;//存储最大速度
	unsigned char dt_temp=0x00;
	unsigned long int bit_on=0x00000001;
	unsigned char bit_mov=0x00;		
	for(i=0;i<16;i++)
		{

			if(det_speed_timer&(0x0001<<i)) 					//检查该组线圈是否开始计时
				{
					if(CH_B[i]<=16)
					{
						bit_on=0x00000001;
						bit_mov=CH_B[i];
					}
					else
					{
						bit_on=0x00010000;
						bit_mov=CH_B[i]-16;
					}
					if(det_result&(bit_on<<bit_mov))				//检查后线圈是否有车通过
					{	
						det_speed_timer=det_speed_timer&(~(0x0001<<i));		//本组线圈停止计时
						det_coil_speed[i][fill_time_num[i]]=det_coil_time[i];
						fill_time_num[i]++;
						if(fill_time_num[i]>=10)
						{
							fill_time_num[i]=0;
							det_speed_time_ok|=(0x0001<<i);
						}
					}
				}
			else 
				{
					if(CH_A[i]<=16)
					{
						bit_on=0x00000001;
						bit_mov=CH_A[i];
					}
					else
					{
						bit_on=0x00010000;
						bit_mov=CH_A[i]-16;
					}
					if(det_result&(bit_on<<bit_mov))			//检查前线圈是否有车通过
					{
					//	CANSendTest4(CH_A[i],i);
						det_speed_timer|=(0x0001<<i);
					LED_RUN = !LED_RUN;
			//			dt_temp=CH_A[i];
			//			CANSendTest4(CH_A[i]);
			//			CANSendTest2(det_result);
			//			delayms(10);
			//			CANSendTest2(det_speed_timer);
			//			delayms(10);
					}		
				}
		}
	for(i=0;i<16;i++)
		{		
			if(det_speed_time_ok&(0x0001<<i))
				{
					for(j=0;j<10;j++)
						{
							speed_time_sum+=det_coil_speed[i][j];
							if(min_speed>det_coil_speed[i][j])
								{
									min_speed=det_coil_speed[i][j];
								}
							if(max_speed<det_coil_speed[i][j])
								{
									max_speed=det_coil_speed[i][j];
								}
	//						CANSendTest4(det_coil_speed[i][j],speed_time_sum);
						}
					det_average_speed_time=(speed_time_sum-(min_speed+max_speed))>>3;
					det_average_speed[i]=((det_coil_dist[i]*72)/det_average_speed_time);//单位换算
	//				CANSendTest4(det_speed_time_ok,det_average_speed_time);
	//				CANSendTest4(min_speed,max_speed);
					
					speed_time_sum=0x0000;
					min_speed=0xff;
					max_speed=0x00;
				//	det_speed_time_ok&=(~(0x01<<i));
				LED_RUN = !LED_RUN;

				}
		}

}
unsigned long int GetCHResult(void)
{  
	unsigned char i,j,det_temp=0;
	unsigned char dt_int[4]={0,0,0,0};
	unsigned long int result=0x00000000;
	for(i=0;i<=7;i++)
	{
		PORTD|=0xf0;//通道关闭
		PORTA&=0xf1;//通道地址清0
		PORTA|=(i<<1);//设置读取通道地址
	//	LED_CAN=CH_A2;
		PORTD&=0x0f;//通道打开
		det_temp=PORTD&0x0f;//读取通道状态
		if((det_temp&0x01)==0x01)	//获取1－8通道状态
		{
			dt_int[0]|=(0x01<<i);
		}
		else
		{
			dt_int[0]&=(~(0x01<<i));
		}
		if((det_temp&0x02)==0x02)	//获取9－16通道状态
		{
			dt_int[1]|=(0x01<<i);
		}
		else
		{
			dt_int[1]&=(~(0x01<<i));
		}
		if((det_temp&0x04)==0x04)	//获取17－24通道状态
		{
			dt_int[2]|=(0x01<<i);
		}
		else
		{
			dt_int[2]&=(~(0x01<<i));
		}
		if((det_temp&0x08)==0x08)	//获取25－32通道状态
		{
			dt_int[3]|=(0x01<<i);
		}
		else
		{
			dt_int[3]&=(~(0x01<<i));
		}
	}
	result=(dt_int[3]<<8)|dt_int[2];
	result=(result<<8)|dt_int[1];
	result=(result<<8)|dt_int[0];
	return result;
}

/////////////////////////////////////////////////////////////
//从端口读取检测结果，读32个通道的状态
void DetResultRead()
{
	static unsigned long int det_result_temp[3] = {0,0,0}; //临时存储检测结果
	static unsigned long int det_result_prev = 0x00000000; //存储上一次的检测结果

	
	det_result_temp[2] = det_result_temp[1];
	det_result_temp[1] = det_result_temp[0];
	det_result_temp[0] =GetCHResult();//读取端口状态

	if((det_result_temp[1] == det_result_temp[0])&&(det_result_temp[1] == det_result_temp[2]))
		{//连续三次一致才认为是有车
			if((det_work_mode&0x01) == 0X01)
				{ //存在型填充
					det_result = ~det_result_temp[2];
					//LED_CAN =  !LED_CAN;
				}
			else
				{ //脉冲型填充
					if(det_result_send)
						{
							det_result_send = false;
							det_result= ((det_result_prev ^ (~det_result_temp[2]))&(~det_result_prev)); //上升沿填充
							det_result_prev = ~det_result_temp[2];							
						}
					else
						{ //保护未被读取的上升沿
							det_result |= ((det_result_prev ^ (~det_result_temp[2]))&(~det_result_prev)); //上升沿填充
							det_result_prev = ~det_result_temp[2];	

						}
					
				}
			
		
		}
	

//	det_result=GetCHResult();
}

//////////////////////////////////////////////////////////////
//系统应用中的超时处理
void DoTimeout(void)
{	
	can_time_cont++;
	master_time_cont++;

	if(can_time_cont>= CAN_TIMEOUT)
		{
			//can总线通信超时
			can_time_cont = CAN_TIMEOUT;
			LED_CAN = true; //熄灭CAN指示LED灯

		}
	if(master_time_cont>=MASTER_TIMEOUT)
		{
			//主机异常超时
			master_time_cont = MASTER_TIMEOUT;
			sta = STA_MASTER_ERR; //将系统置于主机错误状态
		}	

}
void CoilInt(void)
{
	unsigned char i;
	eeprom_addr=ADDR_COIL_BIND;
	eeprom_num=NUM_COIL_BIND;
	EEPROMRead(&eeprom_addr,&eeprom_num,det_coil_bind);
	eeprom_addr=ADDR_COIL_BIND_GRP;
	eeprom_num=NUM_COIL_BIND_GRP;
	EEPROMRead(&eeprom_addr,&eeprom_num,&det_coil_bind_grp);
	eeprom_addr=ADDR_COIL_DIST;
	eeprom_num=NUM_COIL_DIST;
	EEPROMRead(&eeprom_addr,&eeprom_num,det_coil_dist);
	eeprom_addr = ADDR_DET_CFG;
	eeprom_num = NUM_DET_CFG;

	EEPROMRead(&eeprom_addr,&eeprom_num,&det_work_mode);

	for(i=0;i<16;i++)							//将绑定的前后线圈归类分组
		{
			CH_A[i]=(det_coil_bind[i]&0x0f)+((det_coil_bind_grp>>(i*2))&0x00000001)*16;
			CH_B[i]=((det_coil_bind[i]&0xf0)>>4)+(det_coil_bind_grp>>(i*2+1)&0x00000001)*16;
		}
}
void sepcialworkchick()
{
	unsigned char temp;
	temp=PORTC&0X0F;
	
}

//////////////////////////////////////////////////////////////

void main(void)
{
	unsigned char i,j=0;
	SWDTEN =false;//关闭看门狗
	delayms(500);	
	PORTInit();  //端口初始化
	
	LED_CAN = false;
    IntManager();
	Uart1Init();//串口1初始化
	CANInit(GetAddr(MYTYPE));
	CoilInt();//线圈数据初始化
	SWDTEN =true; //使能看门狗
	LED_CAN = true;
	while(1)
		{
			CANInt =false;					
			if(CANRecv(CANRid,CANRdata,&CANRdlc))//收到正确发给本板数据帧
			{
				LED_CAN = false;
				//解析数据帧
				CANDATAAnalyzing(CANRid,CANRdata,CANRdlc);
				LED_CAN = true;
			}	
			CLRWDT();//喂狗
			DetResultRead();//从端口读取检测结果
		if(recv1_data_ok==true)
			{
				Uart1SendData();				
				if(!(det_work_mode & 0x01))
					{ //脉冲型检测 
						det_result_send = true; //为脉冲型检测提供标志
					}
				recv1_data_ok=false;
			}		
			if(time5ms == true)   //5ms定时到
			{
			
				GetAverageSpeed();
				time5ms = false;	
				//DoTimeout();
				t500ms_count++;

				

				if(t500ms_count>=TIME500MS)
				{
					sepcialworkchick();
					t500ms_count=0;
					i++;
					//LED_RUN = !LED_RUN;
					j++;
					if(j>=8)
					{
						j=0;
					}
						
				}
			}
		}
}
