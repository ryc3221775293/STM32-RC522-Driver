#ifndef __OUTPUT_H
#define __OUTPUT_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 

//输出输入驱动代码	   
									  
////////////////////////////////////////////////////////////////////////////////// 
#define LEDA PBout(5)          // PB5
#define LEDB PEout(5)          // PE5
#define BEEP PBout(8)        	 // PB8
/*
#define BZJS PFin(5)           // PF5光电避障减速信号
#define BZTC PFin(6)           // PF6光电避障停车信号	
#define XJMXF PFin(7)          // PF7前循迹门限信号
#define XJMXB PFin(8)          // PF后循迹门限信号
*/
//#define BEEP    PFout(0)       // PF0蜂鸣器
#define qmotor1 PFout(1)       // PF1前左马达启动信号
#define qmotor2 PFout(2)       // PF2前右马达启信号	
#define qmotor3 PFout(3)       // PF3后左马达启动信号
#define qmotor4 PFout(4)       // PF4后右马达启动信号

void OUTPUT_Init(void);           //初始化

		 				    
#endif
