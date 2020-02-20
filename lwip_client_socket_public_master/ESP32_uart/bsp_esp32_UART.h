#ifndef  __BSP_Esp32_UART_H
#define	 __BSP_Esp32_UART_H

#include <fsl_gpio.h>
#include "stdint.h"
//#include <core_cm4.h>
#include <stdio.h>
#include <app.h>
#include <stdbool.h>



#if defined ( __CC_ARM   )
#pragma anon_unions
#endif



/******************************* Esp32 数据类型定义 ***************************/
typedef enum{
	STA,
  AP,
  STA_AP  
} ENUM_Net_ModeTypeDef;


typedef enum{
	 enumTCP,
	 enumUDP,
} ENUM_NetPro_TypeDef;
	

typedef enum{
	Multiple_ID_0 = 0,
	Multiple_ID_1 = 1,
	Multiple_ID_2 = 2,
	Multiple_ID_3 = 3,
	Multiple_ID_4 = 4,
	Single_ID_0 = 5,
} ENUM_ID_NO_TypeDef;
	

typedef enum{
	OPEN = 0,
	WEP = 1,
	WPA_PSK = 2,
	WPA2_PSK = 3,
	WPA_WPA2_PSK = 4,
} ENUM_AP_PsdMode_TypeDef;



/******************************* ESP32 外部全局变量声明 ***************************/
#define RX_BUF_MAX_LEN     1024                                     //最大接收缓存字节数

extern struct  STRUCT_USARTx_Fram                                  //串口数据帧的处理结构体
{
	char  Data_RX_BUF [ RX_BUF_MAX_LEN ];
	volatile uint16_t FramHeadIndex    ;
  union {
    volatile uint16_t InfAll;
    struct {
		  volatile uint16_t FramLength       :15;                               // 14:0 	  
			volatile uint16_t FramFinishFlag   :1;                                // 15 
	  } InfBit;
	
  }; 
	
} strEsp32_Fram_Record;



/******************************** Esp32 连接引脚定义 ***********************************/


 

#define      macEsp32_USART_BAUD_RATE                       115200

#define      macEsp32_USARTx                                USART1


#define      macEsp32_USART_IRQ                             USART1_IRQn
#define      macEsp32_USART_INT_FUN                         USART1_IRQHandler



/*********************************************** Esp32 函数宏定义 *******************************************/
//#define     macEsp32_Usart( fmt, ... )           USART_printf ( LPUART1, fmt, ##__VA_ARGS__ ) 
#define     macEsp32_Usart( fmt, ... )           USART_CMD_Esp32(LPUART1, fmt, ##__VA_ARGS__  )
#define     macPC_Usart( fmt, ... )                printf ( fmt, ##__VA_ARGS__ )
//#define     macPC_Usart( fmt, ... )                
#ifdef  New_Integrated_Sensor
#define     macEsp32_CH_ENABLE()                 GPIO_PortSet (GPIOE , 1<<5U )
#define     macEsp32_CH_DISABLE()                GPIO_PortClear (GPIOE , 1<<5U )
#else
#define     macEsp32_CH_ENABLE()                 GPIO_PortSet (GPIOE , 1<<4U )
#define     macEsp32_CH_DISABLE()                GPIO_PortClear (GPIOE , 1<<4U )
#endif


#define     macEsp32_RST_HIGH_LEVEL()            GPIO_PortClear (GPIOE , 1<<4U )  //  應該是2
#define     macEsp32_RST_LOW_LEVEL()             GPIO_PortSet (GPIOE , 1<<4U )    //

/****************************************** Esp32 函数声明 ***********************************************/
void                     Esp32_Init                        ( void );
void                     Esp32_Rst                         ( void );
bool                     Esp32_Cmd                         ( char * cmd, char * reply1, char * reply2, uint32_t waittime );
void                     Esp32_AT_Test                     ( void );
bool                     Esp32_Net_Mode_Choose             ( ENUM_Net_ModeTypeDef enumMode );
bool                     Esp32_JoinAP                      ( char * pSSID, char * pPassWord );
bool                     Esp32_BuildAP                     ( char * pSSID, char * pPassWord, ENUM_AP_PsdMode_TypeDef enunPsdMode );
bool                     Esp32_Enable_MultipleId           ( FunctionalState enumEnUnvarnishTx );
bool                     Esp32_Link_Server                 ( ENUM_NetPro_TypeDef enumE, char * ip, char * ComNum, ENUM_ID_NO_TypeDef id);
bool                     Esp32_StartOrShutServer           ( FunctionalState enumMode, char * pPortNum, char * pTimeOver );
uint8_t                  Esp32_Get_LinkStatus              ( void );
uint8_t                  Esp32_Get_IdLinkStatus            ( void );
uint8_t                  Esp32_Inquire_ApIp                ( char * pApIp, uint8_t ucArrayLength );
bool                     Esp32_UnvarnishSend               ( void );
void                     Esp32_ExitUnvarnishSend           ( void );
bool                     Esp32_SendString                  ( FunctionalState enumEnUnvarnishTx, char * pStr, uint32_t ulStrLength, ENUM_ID_NO_TypeDef ucId );
char *                   Esp32_ReceiveString               ( FunctionalState enumEnUnvarnishTx );
//void                     USART_printf                      ( LPUART_Type * USARTx, char * Data, ... )
void                     USART_DATA_Esp32                  ( LPUART_Type * USARTx, uint8_t * Data, uint16_t length );
void                     startTxd                          (LPUART_Type * USARTx);
void                     Set_Esp32_UnvarnishSend           (void);
bool                     Esp32_ReturnSENDCMD               (bool disable);
bool                     Esp32_SETUARTBaudrate             ( uint32_t Baudrate );
void                     MK27_USART_Config                ( uint32_t Baudrate );
uint8_t                  WriteDataToTXDBUF                 (uint8_t * source,uint16_t length);
bool                     Esp32_SetIP                       ( char * IP, char * MASK,char * GATEWAY );
bool                     Esp32_AutoConn                    ( FunctionalState enumEnUnvarnishTx );
bool                     Esp32_DisconnAP                   ( void );
bool                     Esp32_setDHCP                     ( FunctionalState disable);
uint8_t                  Esp32_Get_Sation_IP               ( void );
bool Esp32_SetIP_1(char *localIP,char *LocalMASK,char *LocalGATEWAY,uint8_t dhcp);  //设置IP地址
bool Esp32_SetAP_1(char *APssid,char *APpassword );  //返回IP地址
bool Esp32_applynetset(void);  //返回IP地址
bool Esp32_SetTCPSERVER_1(char *TcpServer_IP,char *TcpServer_Port);
#endif


