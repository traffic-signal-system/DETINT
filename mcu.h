


#pragma config XINST=OFF  //不使用扩展指令集
#pragma config FOSC=HS1   //使用外部高速晶振
//#pragma config FOSC=INTIO2   //使用内部振荡器
#pragma config WRTD = OFF //不保护EEPROM
//#pragma config WDTEN=OFF   //系统看门狗
#pragma config WDTEN=ON   //系统看门狗
#pragma config PLLCFG=OFF //不使用PLL
#pragma config SOSCSEL=DIG //不使用SOSC
#pragma config WDTPS=256 //1:256 1.024s益出

//振荡器配置
#define INTOSC1MHZ 0x32  //内部1MHZ振荡器
#define INTOSC2MHZ 0x42  //内部2MHZ振荡器
#define INTOSC4MHZ 0x52  //内部4MHZ振荡器
#define INTOSC8MHZ 0x62  //内部8MHZ振荡器
#define INTOSC16MHZ 0x72  //内部16MHZ振荡器

#define SYSOSC 8000000  //系统振荡器时钟

/////////////////////////////////////////////////////////////
//数据传输相关定义
#define DT_SMP 0 //在数据输出时间的中间采样数据
#define DT_CKE 0 //时钟从空闲转换到有效时发送
#define DT_SSPEN 0x20 //使能SPI并设置端口
#define DT_CKP 0x10 //时钟极性为 空闲时低电平
#define DT_MASTER_SSPM 0x02 //设置为主模式 将时钟设置为FOSC/64  =125K
#define DT_SLAVE_SSPM 0x04  //设置为从模式 SS启用


////////////////////////////////////////////////////////////


void delayms(unsigned int ms);
void delayus(unsigned int us);



////////////////////////////////////////////////////////////////
//读取EEPROM中的一个字节
//参数为起始存储地址，数据以及数据个数
void EEPROMRead(unsigned int *addr,unsigned char *num,unsigned char *dat)
{
	do
		{
			EEADRH = (*addr)>>8;
			EEADR  = (*addr);
			(*addr)++;
				
			EEDATA = 0;

			EECON1 &= 0x3f; //指向EEPROM数据存储块


			EECON1 |= 0x01;//执行一个EEPROM读操作
			asm("NOP");

			(*dat) = EEDATA;
			dat++;
		}
	while((*num)--);

}
///////////////////////////////////////////////////////////////
//写入EEPROM中的一组数据
//参数为起始存储地址，数据以及数据个数
void EEPROMWrite(unsigned int *addr,unsigned char *num,unsigned char *dat)
{
	INTCON &= 0x3f;//关闭中断
	while((*num)--)
		{
			EEADRH = (*addr)>>8;
			EEADR  = (*addr);
			(*addr)++;
			EEDATA = (*dat);
			dat++;	
			
			EECON1 &= 0x3f; //指向EEPROM数据存储块

			EECON1 |= 0x04;//执行一个EEPROM写操作

			

			//启动写操作序列 房误操作
			EECON2 = 0x55;
			EECON2 = 0xaa;

			EECON1 |= 0x02;//启动一个EEPROM写操作

			asm("NOP");
			while(EECON1 & 0x02);
			asm("NOP");
			EECON1 &= 0x04;//执行一个EEPROM写操作
					
		}

	INTCON |= 0xc0;//打开中断
}

//不适合做100us以下的定时，误差大
void delayus(unsigned int us)  
{

	unsigned int i;
	us=us-5;
	
	for(i=0;i<us;i++)
		{
			;
		}
	

}

///////////////////////////////////////////////////////////////
//粗略延时ms级别
void delayms(unsigned int ms)   
{
	unsigned int i;

	for(i=0;i<ms;i++)
		{
			delayus(997);
		}

}
