#include "TSC.H"




//CAN配置
#define TQSET 0xd1;  //用于BRGCON2设置 传输:2TQ 相位1:3TQ

#define CAN_SEND_TIMEOUT 1000 //CAN发送超时时间 1000相当于1.2ms左右。

/////////////////////////////////////////////////////
//设置板卡的过滤器和过滤器屏蔽器
//输入参数为板卡的地址(ID),详见详细设计通信部分、
//具体定义见 TSC.H
void CANFilterSet(unsigned char board_addr,unsigned char f_type)
{
	//过滤器0设置
	RXF0SIDH = 0;
	RXF0SIDH = (f_type<<5)|(CAN_ADDR_MASTER>>1);
	RXF0SIDL = 0;
	RXF0SIDL = (CAN_ADDR_MASTER<<7)|(CAN_FMOD_P2P<<5)|0x08|(board_addr>>4);
	RXF0EIDH = 0;
	RXF0EIDH = (board_addr<<4)|0x0f;
	RXF0EIDL = 0;
	RXF0EIDL = 0xF0|CAN_VER;

//////////////////////////////////////////////////
//////RXB0过滤器
		//过滤器1设置
	RXF1SIDH = RXF0SIDH;
	RXF1SIDL = RXF0SIDL;
	RXF1EIDH = RXF0EIDH;
	RXF1EIDL = RXF0EIDL;

		//过滤器2设置
		//将过滤器设置为可接收心跳报(心跳报为主板发给电源板)
	RXF2SIDH = 0x68;
	RXF2SIDL = 0x6a;
	RXF2EIDH = 0x0f;
	RXF2EIDL = 0xff;
	//0d020fff
//////////////////////////////////////////////////
//////RXB1过滤器

		//过滤器3设置
	RXF3SIDH = RXF0SIDH;
	RXF3SIDL = RXF0SIDL;
	RXF3EIDH = RXF0EIDH;
	RXF3EIDL = RXF0EIDL;

		//过滤器4设置
		//将过滤器设置为可接收心跳报(心跳报为主板发给电源板)
	RXF4SIDH = 0x68;
	RXF4SIDL = 0x6a;
	RXF4EIDH = 0x0f;
	RXF4EIDL = 0xff;

			//过滤器2设置
	RXF2SIDH = RXF0SIDH;
	RXF2SIDL = RXF0SIDL;
	RXF2EIDH = RXF0EIDH;
	RXF2EIDL = RXF0EIDL;

			//过滤器5设置
	RXF5SIDH = RXF0SIDH;
	RXF5SIDL = RXF0SIDL;
	RXF5EIDH = RXF0EIDH;
	RXF5EIDL = RXF0EIDL;

	//过滤器屏蔽器0设置
	RXM0SIDH = 0x1f;
	RXM0SIDL = 0x83;
	RXM0EIDH = 0xf0;
	RXM0EIDL = 0x00;

		//过滤器屏蔽器1设置
	RXM1SIDH = 0x1f;
	RXM1SIDL = 0xe3;
	RXM1EIDH = 0xf0;
	RXM1EIDL = 0x00;

}
/////////////////////////////////////////////////////
//CAN 通信错误清除
void CANErrClr(void)
{
	

}
/////////////////////////////////////////////////////
//设置本板的ID
//参数输入为 板卡地址board_addr 和 报文类型f_type
//目标地址统一为 CAN_ADDR_MASTER
void CANSetID(unsigned char board_addr,unsigned char f_type)
{
	//TXB0 ID设置
	TXB0SIDH = 0;
	TXB0SIDH = (f_type<<5)|(board_addr>>1);
	TXB0SIDL = 0;
	TXB0SIDL = (board_addr<<7)|(CAN_FMOD_P2P<<5)|0x08|(CAN_ADDR_MASTER>>4);
	TXB0EIDH = 0;
	TXB0EIDH = (CAN_ADDR_MASTER<<4)|0x0f;
	TXB0EIDL = 0;
	TXB0EIDL = 0xF0|CAN_VER;

	//TXB1 ID设置
	TXB1SIDH = TXB0SIDH;
	TXB1SIDL = TXB0SIDL;
	TXB1EIDH = TXB0EIDH;
	TXB1EIDL = TXB0EIDL;

	//TXB2 ID设置
	TXB2SIDH = TXB0SIDH;
	TXB2SIDL = TXB0SIDL;
	TXB2EIDH = TXB0EIDH;
	TXB2EIDL = TXB0EIDL;
}

/////////////////////////////////////////////////
//CAN初始化
//设置引脚输入输出 设置同步 设置波特率
void CANInit(unsigned char b_addr)
{
	TRISB |= 0x08;  //RB3/CANRX设置为输入
	TRISB &= 0xfb;  //RB2/CANTX设置为输出

	CANCON |=0x80; //进入CAN参数配置模式

	while(!(CANSTAT & 0x80)); //等待模式切换

	//进入配置模式成功
	ECANCON &= 0x3f; //使用传统CAN功能模式

	BRGCON1 = 0x00;  //同步SJW:1TQ  BRP:000000
	BRGCON2 = TQSET; //传输:2TQ 相位1:3TQ
	BRGCON3 = 0xc1;  //相位2:2TQ
	CIOCON  = 0x21;

	RXB0CON |= 0x06; //接收扩展帧，RXB0溢出时写入RXB1
	RXB1CON |= 0x06; //接收扩展帧
	
	CANSetID(b_addr,CAN_FTYPE_N1);
	CANFilterSet(b_addr,CAN_FTYPE_N1);
	CANErrClr();

	//初始化CAN 模块的I/O 控制寄存器
	CIOCON=0X00;



	CANCON &= 0x1f; //进入CAN正常工作模式
	while(CANSTAT&0XE0!=0){;}

	//PIR3=0X00; //清所有中断标志
	//PIE3=0X01; //使能接收缓冲器0 的接收中断
	//PIE5 =0xff; //开CAN中断

}

////////////////////////////////////////////////////
//CAN发送 
//输入参数为29位标识符和8个字节数据
//主要功能:
//监测3个发送缓冲区的状态，如果缓冲区一TXB0未就绪则使用发送缓冲区二TXB1
//如果发送缓冲区二TXB1未就绪 则使用发送缓冲区三TXB2
//返回值: 表示发送成功的缓冲区号 TXB0 = 0; TXB1 = 1;TXB2=2;
//发送不成功反回大于2的值;
unsigned char CANSend(unsigned char id[],unsigned char dat[],unsigned char dlc)
{
	unsigned int timeout_timer =0x0000;
	
	if(!(TXB0CON & 0x80)) //发送缓冲区0没有数据 可以放入要发送的数据
		{
			//写入29位标识符
			/*
			TXB0SIDH = id[0];
			TXB0SIDL = id[1] | 0x08;  //需使用29位扩展标识符
			TXB0EIDH = id[2];
			TXB0EIDL = id[3];
			*/

			//写入数据长度
			TXB0DLC = dlc;

			//写入8个字节数据
			TXB0D0 = dat[0];
			TXB0D1 = dat[1];
			TXB0D2 = dat[2];
			TXB0D3 = dat[3];
			TXB0D4 = dat[4];
			TXB0D5 = dat[5];
			TXB0D6 = dat[6];
			TXB0D7 = dat[7];

			//请求发送
			TXB0CON |= 0x08;
			while(TXB0CON&0x08)
				{
					timeout_timer++;
					if(timeout_timer >= CAN_SEND_TIMEOUT)
						{
							return 0xff;
						}
				}
			return 0;
		}
	
	if(!(TXB1CON & 0x80)) //发送缓冲区1没有数据 可以放入要发送的数据
		{
			//写入29位标识符
			/*
			TXB1SIDH = id[0];
			TXB1SIDL = id[1] | 0x08;  //需使用29位扩展标识符
			TXB1EIDH = id[2];
			TXB1EIDL = id[3];
			*/

			//写入数据长度
			TXB1DLC = dlc;

			//写入8个字节数据
			TXB1D0 = dat[0];
			TXB1D1 = dat[1];
			TXB1D2 = dat[2];
			TXB1D3 = dat[3];
			TXB1D4 = dat[4];
			TXB1D5 = dat[5];
			TXB1D6 = dat[6];
			TXB1D7 = dat[7];

			//请求发送
			TXB0CON |= 0x08;
			while(TXB0CON&0x08)
				{
					timeout_timer++;
					if(timeout_timer >= CAN_SEND_TIMEOUT)
						{
							return 0xff;
						}
				}
			return 1;
		}

	if(!(TXB2CON & 0x80)) //发送缓冲区2没有数据 可以放入要发送的数据
		{
			//写入29位标识符
			/*
			TXB2SIDH = id[0];
			TXB2SIDL = id[1] | 0x08;  //需使用29位扩展标识符
			TXB2EIDH = id[2];
			TXB2EIDL = id[3];
			*/

			//写入数据长度
			TXB2DLC = dlc;

			//写入8个字节数据
			TXB2D0 = dat[0];
			TXB2D1 = dat[1];
			TXB2D2 = dat[2];
			TXB2D3 = dat[3];
			TXB2D4 = dat[4];
			TXB2D5 = dat[5];
			TXB2D6 = dat[6];
			TXB2D7 = dat[7];

			//请求发送
			TXB2CON |= 0x08;
			while(TXB2CON&0x08)
				{
					timeout_timer++;
					if(timeout_timer >= CAN_SEND_TIMEOUT)
						{
							return 0xff;
						}
				}
			return 2;
		}
	return 0x03; //返回一个大于2的值
}


//////////////////////////////////////////////////
//判断是否接收到完整的CAN数据报文
//通过查询的方式来进行CAN数据报文的接收
bit CANRecv(unsigned char *id,unsigned char *dat,unsigned char *dlc)
{
	//判断发生何种中断，错误中断需要清除 并重新初始化CAN控制器

	if(RXB0IF ==1)
		{
			RXB0IF =0;
			RXB0CON &= 0x7f;
			id[0] = RXB0SIDH;
			id[1] = RXB0SIDL;
			id[2] = RXB0EIDH;
			id[3] = RXB0EIDL;

			*dlc =RXB0DLC;

			dat[0] = RXB0D0;
			dat[1] = RXB0D1;
			dat[2] = RXB0D2;
			dat[3] = RXB0D3;
			dat[4] = RXB0D4;
			dat[5] = RXB0D5;
			dat[6] = RXB0D6;
			dat[7] = RXB0D7;
			return 1;
		}
	if(RXB1IF ==1)
		{
			RXB1IF =0;
			RXB1CON &= 0x7f;
			id[0] = RXB1SIDH;
			id[1] = RXB1SIDL;
			id[2] = RXB1EIDH;
			id[3] = RXB1EIDL;

			*dlc =RXB1DLC;

			dat[0] = RXB1D0;
			dat[1] = RXB1D1;
			dat[2] = RXB1D2;
			dat[3] = RXB1D3;
			dat[4] = RXB1D4;
			dat[5] = RXB1D5;
			dat[6] = RXB1D6;
			dat[7] = RXB1D7;
			return 1;
			
		}
	if(TXB0IF ==1)
		{
			TXB0IF =0;
		}
	if(TXB1IF ==1)
		{
			TXB1IF =0;
 		}
	if(TXB2IF ==1)
		{
			TXB2IF =0;
		}
	//CAN发生错误，捕获错误号，并重新初始化CAN控制器
	if(ERRIF ==1)
		{
			ERRIF =0;
			//清除CAN缓冲区内部数据
			id[0] = 0;
			id[1] = 0;
			id[2] = 0;
			id[3] = 0;

			*dlc =0;

			dat[0] = 0;
			dat[1] = 0;
			dat[2] = 0;
			dat[3] = 0;
			dat[4] = 0;
			dat[5] = 0;
			dat[6] = 0;
			dat[7] = 0;

			CANInit(GetAddr(MYTYPE));
			//CAN通信错误，返回为0没有任何数据被接收处理
			return 0;
		}	
	return 0;

}

///////////////////////////////////////////////////
