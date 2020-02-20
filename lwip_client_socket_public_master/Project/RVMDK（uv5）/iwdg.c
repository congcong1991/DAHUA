#include "iwdg.h"
#include "stm32h7xx_hal_iwdg.h"
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



IWDG_HandleTypeDef IWDG_Handler; //独立看门狗句柄

//初始化独立看门狗
//prer:分频数:0~7(只有低3位有效!)
//rlr:自动重装载值,0~0XFFF.
//分频因子=4*2^prer.但最大值只能是256!
//rlr:重装载寄存器值:低11位有效.
//时间计算(大概):Tout=((4*2^prer)*rlr)/32 (ms).
void IWDG_Init(uint8_t prer,uint16_t rlr)
{
    IWDG_Handler.Instance=IWDG1;
    IWDG_Handler.Init.Prescaler=prer;    //设置IWDG分频系数
    IWDG_Handler.Init.Reload=rlr;        //重装载
    IWDG_Handler.Init.Window=IWDG_WINDOW_DISABLE;//关闭窗口功能
    HAL_IWDG_Init(&IWDG_Handler);    
}
    
//喂独立看门狗
void IWDG_Feed(void)
{   
    HAL_IWDG_Refresh(&IWDG_Handler); //重装载
}
