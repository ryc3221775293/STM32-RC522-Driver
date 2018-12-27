#include "output.h"

//////////////////////////////////////////////////////////////////////////////////	 

//输出输入驱动代码	   
									  
////////////////////////////////////////////////////////////////////////////////// 	   

void OUTPUT_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOF, ENABLE);	 //使能PE、PF端口时钟

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;	 //IO-->PF1、PF2、PF3、PF4 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		                     //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		                     //IO口速度为50MHz
 GPIO_Init(GPIOF, &GPIO_InitStructure);					                     //根据设定参数初始化PF1、PF2、PF3、PF4
 GPIO_SetBits(GPIOF,GPIO_Pin_1);			 //PF1、PF2、PF3、PF4 输出高
 GPIO_SetBits(GPIOF,GPIO_Pin_0);
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_2;	                     //LED端口配置 
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		                     //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		                     // IO口速度为50MHz
 GPIO_Init(GPIOE, &GPIO_InitStructure);	  				                     //根据设定参数初始化PE0，PE2
 GPIO_SetBits(GPIOE,GPIO_Pin_0 |GPIO_Pin_2); 			                     //PE0 PE2 输出低

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;	                     //LED端口配置 
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		                     //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		                     // IO口速度为50MHz
 GPIO_Init(GPIOE, &GPIO_InitStructure);	  				                     //根据设定参数初始化PE0，PE2
 GPIO_SetBits(GPIOE,GPIO_Pin_5); 			                     //PE0 PE2 输出低
 
 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;	                     //LED端口配置 
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		                     //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		                     // IO口速度为50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	  				                     //根据设定参数初始化PE0，PE2
 GPIO_SetBits(GPIOB,GPIO_Pin_8); 

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;	                     //LED端口配置 
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		                     //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		                     // IO口速度为50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	  				                     //根据设定参数初始化PE0，PE2
 GPIO_SetBits(GPIOB,GPIO_Pin_5); 			                     //PE0 PE2 输出低
 
}
 
