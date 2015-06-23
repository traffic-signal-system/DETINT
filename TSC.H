//交通信号机头文件
//时间:2012年11月30日
//作者:陈宗仁
//

#define MYTYPE DETINT_BOARD


//信号机部分定义
#define POWER_BOARD 0x01  //电源板
#define LAMP_BOARD 0x02   //灯控板
#define CTRL_BOARD 0x03   //控制板
#define DET_BOARD 0x04    //检测器板
#define DETINT_BOARD 0x05 //接口板
#define FLASH_BOARD 0x06 //黄闪器
#define DISP_BOARD 0x07 //显示灯板



//CAN部分定义
#define CAN_BPS 500000    //CAN总线速率单位为 bps
#define CAN_VER 0x0f  	  //协议版本 协议草案 详见 详细设计协议版本分配

///////can FRAME MODE
#define CAN_FMOD_P2P 0x00 //点对点模式
#define CAN_FMOD_GB 0x01  //组播模式
#define CAN_FMOD_AB 0x02  //广播模式
#define CAN_FMOD_HB 0x03  //广播心跳模式
//CAN FRAME TYPE 指示该帧的实时性
#define CAN_FTYPE_H1 0x00 //最高优先级
#define CAN_FTYPE_H2 0x01 //第二优先级
#define CAN_FTYPE_H3 0x02 //第三优先级
#define CAN_FTYPE_N1 0x03 //正常一优先级
#define CAN_FTYPE_N2 0x04 //正常二优先级
#define CAN_FTYPE_L1 0x05 //低优先级
#define CAN_FTYPE_L2 0x06 //较低优先级
#define CAN_FTYPE_L3 0x07 //最低优先级
//CAN总线板卡地址定义
#define CAN_ADDR_MASTER 0x10  //主控板
#define CAN_ADDR_BMASTER1 0x11  //备用主控板1
#define CAN_ADDR_BMASTER2 0x12  //备用主控板2

#define CAN_ADDR_LAMP1 0x13  //灯控板1
#define CAN_ADDR_LAMP2 0x14  //灯控板2
#define CAN_ADDR_LAMP3 0x15  //灯控板3
#define CAN_ADDR_LAMP4 0x16  //灯控板4
#define CAN_ADDR_ELAMP1 0x17 //扩展灯控板1
#define CAN_ADDR_ELAMP2 0x18 //扩展灯控板2
#define CAN_ADDR_GBLAMP 0x19 //灯控板组播
#define CAN_ADDR_BLAMP1 0x1a //备用灯控板1
#define CAN_ADDR_BLAMP2 0x1b //备用灯控板2
#define CAN_ADDR_BLAMP3 0x1c //备用灯控板3
#define CAN_ADDR_BLAMP4 0x1d //备用灯控板4
#define CAN_ADDR_BLAMP5 0x1e //备用灯控板3
#define CAN_ADDR_BLAMP6 0x1f //备用灯控板4

#define CAN_ADDR_POWER 0x20 //电源板
#define CAN_ADDR_GBPOWER 0x21 //电源板组播
#define CAN_ADDR_BPOWER1 0x22 //备用电源板1
#define CAN_ADDR_BPOWER2 0x23 //备用电源板2

#define CAN_ADDR_DET1 0x24 //检测器板1
#define CAN_ADDR_DET2 0x25 //检测器板2
#define CAN_ADDR_GBDET 0x26 //检测器板组播
#define CAN_ADDR_BDET1 0x27 //备用检测器板1
#define CAN_ADDR_BDET2 0x28 //备用检测器板2

#define CAN_ADDR_DETINT1 0x29 //接口板1
#define CAN_ADDR_DETINT2 0x2a //接口板2
#define CAN_ADDR_GBDETINT 0x2b //接口板组播
#define CAN_ADDR_BDETINT1 0x2c //备用接口板1
#define CAN_ADDR_BDETINT2 0x2d //备用接口板2

#define CAN_ADDR_YFLASH 0x2E //黄闪器
#define CAN_ADDR_BYFLASH 0x2F //备用黄闪器

#define CAN_ADDR_CTRL 0x30 //控制模块
#define CAN_ADDR_BCTRL 0x31 //备用控制模块

#define CAN_ADDR_DISP 0x32 //显示灯板

/////////////////////////////////////////////////////////
//读取灯控板的地址
unsigned char GetLampBoardAddr(void)
{
	unsigned char addr = 0x00;
	unsigned int i=0;
	
	addr = (PORTB>>5) & 0x07;
	
	//延迟去抖动
	for(i=0;i<1000;i++) 
		{;}
	
	if(addr ==((PORTB>>5) & 0x07))
		{
			switch(addr)
				{
				case 0:
					return CAN_ADDR_LAMP1;
					break;
				case 1:
					return CAN_ADDR_LAMP2;
					break;
				case 2:
					return CAN_ADDR_LAMP3;
					break;
				case 3:
					return CAN_ADDR_LAMP4;
					break;
				case 4:
					return CAN_ADDR_ELAMP1;
					break;
				case 5:
					return CAN_ADDR_ELAMP2;
					break;
				case 6:
					return CAN_ADDR_BLAMP1;
					break;
				case 7:
					return CAN_ADDR_BLAMP2;
					break;
				default:
					break;			

				}
		}
	else
		{
			return 0xff;
		}
		return 0x00;
}
unsigned char GetDetIntBoardAddr(void)
{
	
	unsigned char addr = 0x00;
	unsigned int i=0;
	
	addr = PORTB& 0x03;
	
	//延迟去抖动
	for(i=0;i<1000;i++) 
		{;}
	if(addr ==( PORTB& 0x03))
		{
			switch(addr)
				{
					case 0:
						return CAN_ADDR_DETINT1;//00为第一块板
						break;
					case 1:
						return CAN_ADDR_DETINT2;//01为第二块板
						break;
					case 2:
						return CAN_ADDR_DETINT2; //10为第二块板
						break;
					case 3:
						return CAN_ADDR_DETINT1; //11为第一块板
						break;
					default:
						break;	
				}
		}
	else
		{
			return 0xff;
		}
		return 0x00;
}


////////////////////////////////////
//取得本板的具体地址
//参数输入为 板卡的类型
//
/*
#define POWER_BOARD 0x01  //电源板
#define LAMP_BOARD 0x02   //灯控板
#define CTRL_BOARD 0x03   //控制板
#define DET_BOARD 0x04    //检测器板
#define DETINT_BOARD 0x05 //接口板
*/
unsigned char GetAddr(unsigned char board_type)
{
	unsigned char rel;
	switch (board_type)
		{
		case POWER_BOARD:
			rel = CAN_ADDR_POWER;
			break;
		case LAMP_BOARD:
			//灯控板读取地址
			rel = GetLampBoardAddr();
			break;
		case CTRL_BOARD:
			//添加入控制板读取地址部分
			break;
		case DET_BOARD:
			//添加入检测板读取地址部分
			
			break;
		case DETINT_BOARD:
			rel = GetDetIntBoardAddr();
			//添加入接口板读取地址部分
			break;
		default:
			rel = 0;
			break;		

		}
	return rel;
}


















