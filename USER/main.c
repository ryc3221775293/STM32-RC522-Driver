#include "output.h"
#include "delay.h"
#include "sys.h"
#include "rc522.h"
#include "lcd.h"			       //显示模块
#include "usart.h"
#include "string.h" 


int main(void)
{		

 	delay_init();	    	 //延时函数初始化	  
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
 	OUTPUT_Init();			 //输出模块初始化
	uart_init(115200);				
	InitRc522();				//初始化射频卡模块
	BEEP=1;
  	while(1) 
	{		
			RC522_Handel();
	
	}
	
}
			




