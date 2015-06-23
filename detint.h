//接口板相关特殊定义
#define MY_ID 0x00

#define ADDR_ID 0x0000 //板卡ID存储首地址
#define NUM_ID  4      //板卡ID信息字节数 

#define ADDR_CONF 0x0004//板卡配置数据存储首地址
#define NUM_CONF  5     //板卡配置数据字节数

#define ADDR_COIL_BIND  0x0021 	//线圈绑定数据存储首地址
#define NUM_COIL_BIND  16 		//线圈绑定字节数
#define ADDR_COIL_BIND_GRP 0x0031//绑定线圈组别数据存储地址
#define NUM_COIL_BIND_GRP 4		//绑定线圈组别数据字节数
#define ADDR_COIL_DIST  0x0035	//线圈距离首地址
#define NUM_COIL_DIST   16     	// 线圈距离字节数
#define ADDR_DET_CFG 0x0045 	//检测工作方式
#define NUM_DET_CFG  1  		//检测工作方式字节数


//端口定义
#define DT_INT1 RD0
#define DT_INT2 RD1
#define DT_INT3 RD2
#define DT_INT4 RD2
#define CH_A0 RA1
#define CH_A1 RA2
#define CH_A2 RA3

#define CH_CS0 RD4
#define CH_CS1 RD5
#define CH_CS2 RD6
#define CH_CS3 RD7

#define LED_CAN RA0
#define LED_RUN RB4


#define MO0 RC0
#define MO1 RC1
#define MO2 RC2
#define MO3 RC3










