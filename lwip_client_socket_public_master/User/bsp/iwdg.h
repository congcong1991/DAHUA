#ifndef __IWDG_H
#define __IWDG_H
#include "bsp.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32H7开发板
//独立看门狗驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/8/12
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

void IWDG_Init(uint8_t prer,uint16_t rlr);//IWDG初始化
void IWDG_Feed(void);			//IWDG喂狗
#endif
