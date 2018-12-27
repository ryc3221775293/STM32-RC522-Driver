#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"

//////////////////////////////////////////////////////////////////////////////////	 

//RS485驱动 代码	   
									  
//////////////////////////////////////////////////////////////////////////////////


#ifdef EN_USART3_RX   	//如果使能了接收


//接收缓存区 	
u8 RS485A_RX_BUF[RS485_REC_LEN];  	//接收缓冲,最大64个字节.
//接收到的数据长度
u8 RS485A_RX_CNT=0;   		  	  
u8 RS485A_begin=0;	                //RS485 1包头标记复位
u8 RS485A_end=0;                    //RS485 1包尾标记复位
u8 RS485A_overflow=0;               //RS485 1溢出标记复位 
void USART3_IRQHandler(void)
{
	u8 res;	    
 
 	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //接收到数据
	{	 
	 			 
		res =USART_ReceiveData(USART3); 	//读取接收到的数据
		if((res==0x55)&&(RS485A_begin==0))             //接收到了包头0x55
				{
				RS485A_RX_CNT=0;
				RS485A_begin=1;
				}

		if((RS485A_RX_CNT<RS485_REC_LEN)&&(RS485A_begin==1))
		{
			RS485A_RX_BUF[RS485A_RX_CNT]=res;		//记录接收到的值
			RS485A_RX_CNT++;						//接收数据增加1 
		}
		if((res==0xaa)&&(RS485A_RX_CNT<RS485_REC_LEN)&&(RS485A_begin==1))  //接收到了包尾0xaa
		{	
		  RS485A_end=1;
		  RS485A_begin=0;
		}
		/*	if(RS485_RX_CNT>(RS485_REC_LEN))
			{
		  RS485_end=1;
		  RS485_begin=0;
		  RS485_overflow=1;
		 }
		  */ 
	}  											 
} 
#endif										 
//初始化IO 串口3
//pclk1:PCLK1时钟频率(Mhz)
//bound:波特率	  
void RS485A_Init(u32 bound)
{  
    GPIO_InitTypeDef GPIO_InitStructure;
  	USART_InitTypeDef USART_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);//使能GPIOC时钟
	//GPIO_PinRemapConfig(GPIO_PartialRemap_USART3  , ENABLE);//如果使用默认的功能，那就不用重映射了
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 //PC12端口配置
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                       //PC10 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     //复用推挽输出 
    GPIO_Init(GPIOB, &GPIO_InitStructure);                             

    //RX初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;                             //PC11 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                  //浮空输入 

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3,ENABLE);//复位串口3
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3,DISABLE);//停止复位
 
	
 #ifdef EN_USART3_RX		  	//如果使能了接收
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据长度
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;///奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式

    USART_Init(USART3, &USART_InitStructure); ; //初始化串口
  
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; //使能串口3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //从优先级2级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道
	NVIC_Init(&NVIC_InitStructure); //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
 
	//USART_ITConfig(USART3,USART_IT_TXE,ENABLE);  //使能发送中断
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启接收中断
   
    USART_Cmd(USART3, ENABLE);                    //使能串口 

 #endif

 RS485A_TX_EN=0;			//默认为接收模式
 
}

//RS485A发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void RS485A_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485A_TX_EN=1;			//设置为发送模式
	delay_ms(5);			//使能延时，防止代码发送错误
  	for(t=0;t<len;t++)		//循环发送数据
	{		   
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART3,buf[t]);
	}	 
 
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);		
	RS485A_RX_CNT=0;	  
	RS485A_TX_EN=0;				//设置为接收模式	
}


//RS485A发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void RS485A_Perfect_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	u8 HEAD;
	RS485A_TX_EN=1;			//设置为发送模式
	delay_ms(5);			//使能延时，防止代码发送错误
	HEAD=0X55;				//包头
	buf[len]=0xaa;			//包尾
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	USART_SendData(USART3,HEAD);
  	for(t=0;t<len+1;t++)	//循环发送数据（
	{		   
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART3,buf[t]);
	}	 
 
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);		
	RS485A_RX_CNT=0;	  
	RS485A_TX_EN=0;			 //设置为接收模式
	delay_ms(5);			 //使能延时，防止代码接收错误	
}

//RS485A查询接收到的数据
//buf:接收缓存首地址
//len:读到的数据长度
void RS485A_Receive_Data(u8 *buf,u8 *len)
{
	u8 rxlen=RS485A_RX_CNT;
	u8 i=0;
	*len=0;				//默认为0
	delay_ms(10);		//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
	if(rxlen==RS485A_RX_CNT&&rxlen)//接收到了数据,且接收完成了
	{
		for(i=0;i<rxlen;i++)
		{
			buf[i]=RS485A_RX_BUF[i];	
		}		
		*len=RS485A_RX_CNT;	//记录本次数据长度
		RS485A_RX_CNT=0;		//清零
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef EN_UART4_RX   	//如果使能了接收

//接收缓存区 	
u8 RS485B_RX_BUF[RS485_REC_LEN];  	//接收缓冲,最大64个字节.
//接收到的数据长度
u8 RS485B_RX_CNT=0;   		  	  
u8 RS485B_begin=0;	                //RS485 2包头标记复位
u8 RS485B_end=0;                    //RS485 2包尾标记复位
u8 RS485B_overflow=0;               //RS4852溢出标记复位 
void UART4_IRQHandler(void)
{
	u8 res;	    
 
 	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) //接收到数据
	{	 
	 			 
		res =USART_ReceiveData(UART4); 	//读取接收到的数据
		if((res==0x55)&&(RS485B_begin==0))             //接收到了包头0x55
				{
				RS485B_RX_CNT=0;
				RS485B_begin=1;
				}

		if((RS485B_RX_CNT<RS485_REC_LEN)&&(RS485B_begin==1))
		{
			RS485B_RX_BUF[RS485B_RX_CNT]=res;		//记录接收到的值
			RS485B_RX_CNT++;						//接收数据增加1 
		}
		if((res==0xaa)&&(RS485B_RX_CNT<RS485_REC_LEN)&&(RS485B_begin==1))  //接收到了包尾0xaa
		{	
		  RS485B_end=1;
		  RS485B_begin=0;
		}
		/*	if(RS485_RX_CNT>(RS485_REC_LEN))
			{
		  RS485_end=1;
		  RS485_begin=0;
		  RS485_overflow=1;
		 }
		  */ 
	}  											 
} 
#endif


void RS485B_Init(u32 bound)
{ 
    GPIO_InitTypeDef GPIO_InitStructure;
  	USART_InitTypeDef USART_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
//使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//PC12端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //PC12端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
//配置接收管脚PC11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
//配置发送管脚PC10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4,ENABLE);//复位串口4
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4,DISABLE);//停止复位


 #ifdef EN_USART3_RX		  	//如果使能了接收
//波特率、字长、停止位、奇偶校验位、硬件流控制、异步串口为默认（被屏蔽字设置）
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStructure);
	//中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器


    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

    USART_Cmd(UART4, ENABLE);

  #endif

 RS485B_TX_EN=0;			//默认为接收模式
}
//RS485B发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void RS485B_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485B_TX_EN=1;			//设置为发送模式
	delay_ms(5);			//使能延时，防止代码发送错误
  	for(t=0;t<len;t++)		//循环发送数据
	{		   
		while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);	  
		USART_SendData(UART4,buf[t]);
	}	 
 
	while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);		
	RS485B_RX_CNT=0;	  
	RS485B_TX_EN=0;				//设置为接收模式	
}


//RS485B发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void RS485B_Perfect_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	u8 HEAD;
	RS485B_TX_EN=1;			//设置为发送模式
	delay_ms(5);			//使能延时，防止代码发送错误
	HEAD=0X55;				//包头
	buf[len]=0xaa;			//包尾
	while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
	USART_SendData(UART4,HEAD);
  	for(t=0;t<len+1;t++)	//循环发送数据（
	{		   
		while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);	  
		USART_SendData(UART4,buf[t]);
	}	 
 
	while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);		
	RS485B_RX_CNT=0;	  
	RS485B_TX_EN=0;			 //设置为接收模式
	delay_ms(5);			 //使能延时，防止代码接收错误	
}








//RS485B查询接收到的数据
//buf:接收缓存首地址
//len:读到的数据长度
void RS485B_Receive_Data(u8 *buf,u8 *len)
{
	u8 rxlen=RS485B_RX_CNT;
	u8 i=0;
	*len=0;				//默认为0
	delay_ms(10);		//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
	if(rxlen==RS485B_RX_CNT&&rxlen)//接收到了数据,且接收完成了
	{
		for(i=0;i<rxlen;i++)
		{
			buf[i]=RS485B_RX_BUF[i];	
		}		
		*len=RS485B_RX_CNT;	//记录本次数据长度
		RS485B_RX_CNT=0;		//清零
	}
}
