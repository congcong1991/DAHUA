/*
*********************************************************************************************************
*
*	模块名称 : LED指示灯驱动模块
*	文件名称 : bsp_led.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_ESP32_H
#define __BSP_ESP32_H
#include "bsp.h"			/* 底层硬件驱动 */

#include <string.h>  
#include <stdbool.h>
#include <stdarg.h>
#include "stdio.h"
/* 供外部调用的函数声明 */

void bsp_Esp32_Init(void);

#define EnableEsp32Power()  {HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,GPIO_PIN_SET);}
#define DisableEsp32Power() {HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,GPIO_PIN_RESET);}

#define ResetEsp32()  {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_SET);}
#define RunEsp32() 		{HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_RESET);}

bool Esp32_SetIP_1(char *localIP,char *LocalMASK,char *LocalGATEWAY,uint8_t dhcp);  //设置IP地址
bool Esp32_SetAP_1(char *APssid,char *APpassword );  //返回IP地址
bool Esp32_applynetset(void);  //返回IP地址
bool Esp32_SetTCPSERVER_1(char *TcpServer_IP,char *TcpServer_Port);
//uint8_t WriteDataToTXDBUF(uint8_t * source,uint16_t length);
#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
