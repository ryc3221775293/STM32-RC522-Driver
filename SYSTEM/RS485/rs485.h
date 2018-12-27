#ifndef __RS485_H
#define __RS485_H			 
#include "sys.h"	 								  
//////////////////////////////////////////////////////////////////////////////////	 

//RS485驱动 代码	   
									  
//////////////////////////////////////////////////////////////////////////////////

#define RS485_REC_LEN  			64  	//定义最大接收字节数 64
	  		  	
extern u8 RS485A_RX_BUF[RS485_REC_LEN]; 	//接收缓冲,最大RS485_REC_LEN个字节.
extern u8 RS485A_RX_CNT;   			    //接收到的数据长度

extern u8 RS485B_RX_BUF[RS485_REC_LEN]; 	//接收缓冲,最大RS485_REC_LEN个字节.
extern u8 RS485B_RX_CNT;   			    //接收到的数据长度

//模式控制
#define RS485A_TX_EN		PBout(8)	//485模式控制.0,接收;1,发送.
#define RS485B_TX_EN		PCout(12)	//485模式控制.0,接收;1,发送.
//如果想串口中断接收，请不要注释以下宏定义
#define EN_USART3_RX 	1			//0,不接收;1,接收.
#define EN_UART4_RX 	1			//0,不接收;1,接收.



void RS485A_Init(u32 bound);
void RS485B_Init(u32 bound);
void RS485A_Send_Data(u8 *buf,u8 len);
void RS485A_Receive_Data(u8 *buf,u8 *len);
void RS485A_Perfect_Send_Data(u8 *buf,u8 len);
void RS485B_Send_Data(u8 *buf,u8 len);
void RS485B_Receive_Data(u8 *buf,u8 *len);
void RS485B_Perfect_Send_Data(u8 *buf,u8 len);

#endif	   
















