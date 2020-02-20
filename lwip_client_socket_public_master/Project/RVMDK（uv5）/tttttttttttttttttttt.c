/*
*********************************************************************************************************
*
*	模块名称 : tcp_client
*	文件名称 : tcp_client.c
*	版    本 : V1.0
*	说    明 : TCP 客户端 用于连接后台服务器
*
*	修改记录 :
*						版本号    日期        作者       说明
*						V1.0  2019年04月04日  suozhang   首次发布
*
*********************************************************************************************************
*/


#include "tcp_client.h"

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
/**
 * Log default configuration for EasyLogger.
 * NOTE: Must defined before including the <elog.h>
 */
#if !defined(LOG_TAG)
#define LOG_TAG                    "tcp_client_tag:"
#endif
#undef LOG_LVL
#if defined(XX_LOG_LVL)
    #define LOG_LVL                    XX_LOG_LVL
#endif

//#include "elog.h"

#if LWIP_NETCONN

#include "lwip/sys.h"
#include "lwip/api.h"

#include "lwip/tcp.h"
#include "lwip/ip.h"
#include "app.h"

extern TaskHandle_t xHandleTaskLED;

#define TCP_SERVER_IP   "192.168.100.88"
#define TCP_SERVER_PORT 9527

#define sw_version "aifude11"
/* 服务器通信使用权信号量 */
SemaphoreHandle_t xServerCommunicationLockSemaphore = NULL;

uint8_t NetReceiveBuf[NetReceiveBufSize];
uint32_t NetReceiveBufHeadIndex=0;
uint32_t NetReceiveBufTailIndex=0;

uint8_t    RxdTelBuf[RX_BUFFER_SIZE];
uint8_t    RXDCmdBuf[RX_BUFFER_SIZE];
uint8_t    CmdBuf[TX_BUFFER_SIZE];

volatile uint16_t RxdBytes=0;
uint16_t CmdBufLength=0;

struct netconn *tcp_client_server_conn=NULL;

void RxdByte(uint8_t c);
	struct JoinNet joinnetstatue;
void vTaskLwip(void *pvParameters)
{
  struct netbuf *buf;
  err_t err;
	struct pbuf *q;
	uint8_t *NetPbuf;
	ip_addr_t server_ip;

	
	u16_t server_port;
	
	TCPIP_Init();
	xServerCommunicationLockSemaphore = xSemaphoreCreateBinary();

	if( NULL == xServerCommunicationLockSemaphore )
	{
//			log_e("err:xServerCommunicationLockSemaphore == NULL,while(1).");
			while(1);
	}
	
	for( ;; )
	{
		server_port = TCP_SERVER_PORT;				     // 服务器端口号初始化

		ip4addr_aton( TCP_SERVER_IP, &server_ip ); 			 // 服务器IP地址初始化		
		
//		log_i("tcp server connecting %s:%d......", ipaddr_ntoa(&server_ip), server_port );
		
		
		xTaskNotify( xHandleTaskLED, 200, eSetValueWithOverwrite );/* 服务器断开连接状态，LED闪烁为200mS一次. */
		
		/* Create a new connection identifier. */
		tcp_client_server_conn = netconn_new( NETCONN_TCP );
				
		if( tcp_client_server_conn != NULL )
		{		
			//打开TCP 的保活功能 （客户端不默认打开），2018年12月6日10:00:41，SuoZhang
//			tcp_client_server_conn->pcb.tcp->so_options |= SOF_KEEPALIVE;

			err = netconn_connect( tcp_client_server_conn, &server_ip, server_port );
				
			if(err == ERR_OK)
			{
//				tcp_client_server_conn->send_timeout = 10;
//				log_i("TCP Server %s:%d connected sucess.", ipaddr_ntoa(&server_ip), server_port );
						
				xSemaphoreGive( xServerCommunicationLockSemaphore ); /* 释放服务器通信使用权 */

				xTaskNotify( xHandleTaskLED, 1000, eSetValueWithOverwrite );/* 服务器连接状态，LED闪烁为1000mS一次. */
				
				for( ;; )
				{
					/* receive data until the other host closes the connection */
					if((err = netconn_recv(tcp_client_server_conn, &buf)) == ERR_OK) 
					{
						CollectServer();	
														 //获取一个指向netbuf 结构中的数据的指针
						taskENTER_CRITICAL();
						for(q=buf->p;q!=NULL;q=q->next)  //遍历完整个pbuf链表
						{
								//判断要拷贝到UDP_DEMO_RX_BUFSIZE中的数据是否大于UDP_DEMO_RX_BUFSIZE的剩余空间，如果大于
								//的话就只拷贝UDP_DEMO_RX_BUFSIZE中剩余长度的数据，否则的话就拷贝所有的数据
							NetPbuf=q->payload;
							for(uint32_t i=0;i<q->len;i++)
							{
								if(!isNetReceiveBufFull())
								{
									NetReceiveBuf[NetReceiveBufHeadIndex]=*NetPbuf;
									NetPbuf++;
									
									IncreaseNetReceiveBuf(NetReceiveBufHeadIndex);
								}
							}	
							 
						}	
						taskEXIT_CRITICAL();
						netbuf_delete(buf);						
					}
					else//if((err = netconn_recv(conn, &buf)) == ERR_OK)
					{
						DisCollectServer();  //链接断开标志
//						log_e("err:netconn_recv(conn, &buf):%d.",err);
						netbuf_delete(buf);	
						break; //连接发生错误，退出死等数据的循环，重新建立链接
					}
				 }
			 }
			 DisCollectServer(); //链接断开标志
//			 log_e("err:TCP Server %s:%d connect fail,err:%d.", ipaddr_ntoa(&server_ip), server_port, err );
			 netconn_close  ( tcp_client_server_conn );
			 netconn_delete ( tcp_client_server_conn );		
		   vTaskDelay(1000);
		}
		else//(conn!=NULL)
		{
			DisCollectServer();  //链接断开标志
//			log_e("err:Can not create tcp_client_server_conn.");
			vTaskDelay(1000);
		}
	}
}

void receive_server_data_task( void )
{
	for(;;)
	{
		if(!isNetReceiveBufEmpty()) //
		{
			RxdByte(NetReceiveBuf[NetReceiveBufTailIndex]);
			IncreaseNetReceiveBuf(NetReceiveBufTailIndex);
		} else
		{
		vTaskDelay(2);

		}
	}
}

extern void AD7606_EnterAutoMode(uint32_t _ulFreq);

uint8_t command_start()
{ 
	//EnableIEPEPower();
//	 for(uint32_t i=0;i<add_data;i++)
//		xx[i]+=i;
	 {
		for(uint8_t i=0;i<CmdBufLength;i++)
		CmdBuf[i]=RXDCmdBuf[i];
	  CmdBufLength=CmdBufLength;
	 }
	 WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 AD7606_StartRecord(config.ADfrequence);
	 return 1;
}

uint8_t command_stop()
{ 

	 {
		for(uint8_t i=0;i<CmdBufLength;i++)
		CmdBuf[i]=RXDCmdBuf[i];
	  CmdBufLength=CmdBufLength;
	 }
	  WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 AD7606_StopRecord();
	 return 1;
}

uint16_t command_id(void)
{ 
	
	 {
		CmdBuf[0]=0x7e;
		CmdBuf[1]=0x02;
		CmdBuf[2]=0x08;
		CmdBuf[3]=0x00;
		CmdBuf[11]=config.SNnumber;
		CmdBuf[10]=config.SNnumber>>8;
		CmdBuf[9]=config.SNnumber>>16;
		CmdBuf[8]=config.SNnumber>>24;
		CmdBuf[7]=config.SNnumber>>32;
		CmdBuf[6]=config.SNnumber>>40;
		CmdBuf[5]=config.SNnumber>>48;
		CmdBuf[4]=config.SNnumber>>56;
		CmdBuf[12]=0;
		for(uint16_t i=1;i<12;i++)
		CmdBuf[12]+=CmdBuf[i];
		CmdBuf[13]=0x7e;
	  CmdBufLength=14;
    WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 }
	 return 1;
}



uint8_t command_receive_beacon()
{ 
	//EnableIEPEPower();
	
	 {
		for(uint8_t i=0;i<CmdBufLength;i++)
		CmdBuf[i]=RXDCmdBuf[i];
	  CmdBufLength=CmdBufLength;
	 }
	 WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 ReceiveBeaconMessage();
	 return 1;
}


uint16_t command_channelkind(void)
{
	
	 {
		CmdBuf[0]=0x7e;
		CmdBuf[1]=0x03;
		CmdBuf[2]=0x06;
		CmdBuf[3]=0x00;
		CmdBuf[4]=Acceleration_ADCHS+Temp_ADCHS; //两个，一个温度一个加速度
		CmdBuf[5]=0x02;  //这一块，每种传感器手工添加
		CmdBuf[6]=0x01;
		CmdBuf[7]=Acceleration_ADCHS;
		CmdBuf[8]=0x02;
		CmdBuf[9]=Temp_ADCHS;
		CmdBuf[10]=0;
		for(uint16_t i=1;i<10;i++)
		CmdBuf[10]+=CmdBuf[i];
		 
    CmdBuf[11]=0x7e;
	  CmdBufLength=12;
    WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 }
	 return 1;
}
uint8_t command_setip()  //设置IP地址  //设置DHCP
{
	{
		for(uint8_t i=0;i<CmdBufLength;i++)
		CmdBuf[i]=RXDCmdBuf[i];
	  CmdBufLength=CmdBufLength;
	 }

	 WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 for(uint8_t i=0;i<CmdBuf[5];i++)
	 config.LocalIP[i]=CmdBuf[6+i];//把AP字符串赋值给config
	 config.LocalIP[CmdBuf[5]]=0;
	 for(uint8_t i=0;i<CmdBuf[7+CmdBuf[5]];i++)
	 config.LocalMASK[i]=CmdBuf[8+CmdBuf[5]+i];//把AP字符串赋值给config
	 config.LocalMASK[CmdBuf[7+CmdBuf[5]]]=0;
	 for(uint8_t i=0;i<CmdBuf[9+CmdBuf[5]+CmdBuf[7+CmdBuf[5]]];i++)
	 config.LocalGATEWAY[i]=CmdBuf[10+CmdBuf[5]+CmdBuf[7+CmdBuf[5]]+i];//把AP字符串赋值给config
	 config.LocalGATEWAY[CmdBuf[9+CmdBuf[5]+CmdBuf[7+CmdBuf[5]]]]=0;
	 //for(uint8_t i=0;i<CmdBuf[11+CmdBuf[5]+CmdBuf[7+CmdBuf[5]]];i++)
	 config.DHCP=CmdBuf[12+CmdBuf[5]+CmdBuf[7+CmdBuf[5]]+CmdBuf[9+CmdBuf[5]+CmdBuf[7+CmdBuf[5]]]];//把AP字符串赋值给config
	 return 1;
}
uint8_t command_replyip()  //设置IP地址
{
	 uint8_t localIPlength=strlen(config.LocalIP);
	 uint8_t LocalMASKlength=strlen(config.LocalMASK);
	 uint8_t LocalGATEWAYlength=strlen(config.LocalGATEWAY);
	 CmdBufLength=15+localIPlength+LocalMASKlength+LocalGATEWAYlength;
	 CmdBuf[0]=0X7E;
	 CmdBuf[1]=0X04;   //T
	 CmdBuf[2]=CmdBufLength-6;
	 CmdBuf[3]=(CmdBufLength-6)>>8;   //L
	 CmdBuf[4]=0X00;   //V_IP
	 CmdBuf[5]=localIPlength;
	{
		for(uint8_t i=0;i<localIPlength;i++)
		CmdBuf[6+i]=config.LocalIP[i];
	 
	 }
   CmdBuf[6+localIPlength]=0X01;   //V_MASK
	 CmdBuf[7+localIPlength]=LocalMASKlength;
	{
		for(uint8_t i=0;i<LocalMASKlength;i++)
		CmdBuf[8+localIPlength+i]=config.LocalMASK[i];
	 
	 }
	 CmdBuf[8+localIPlength+LocalMASKlength]=0X02;   //V_GATEWAY
	 CmdBuf[9+localIPlength+LocalMASKlength]=LocalGATEWAYlength;
	{
		for(uint8_t i=0;i<LocalGATEWAYlength;i++)
		CmdBuf[10+localIPlength+LocalMASKlength+i]=config.LocalGATEWAY[i];
	 
	 }
	 CmdBuf[10+localIPlength+LocalMASKlength+LocalGATEWAYlength]=0X03;
	 CmdBuf[11+localIPlength+LocalMASKlength+LocalGATEWAYlength]=0X01;
	 CmdBuf[12+localIPlength+LocalMASKlength+LocalGATEWAYlength]=config.DHCP;
	 CmdBuf[13+localIPlength+LocalMASKlength+LocalGATEWAYlength]=0;
	 for(uint8_t i=1;i<(13+localIPlength+LocalMASKlength+LocalGATEWAYlength);i++)
	 CmdBuf[13+localIPlength+LocalMASKlength+LocalGATEWAYlength]+=CmdBuf[i];//把AP字符串赋值给config
	 CmdBuf[14+localIPlength+LocalMASKlength+LocalGATEWAYlength]=0x7e;
	 CmdBufLength=15+localIPlength+LocalMASKlength+LocalGATEWAYlength;
	 WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 return 1;
}

uint8_t command_replyap()  //返回IP地址
{  
	//这边要判断是否工厂设置
	 uint8_t APSSIDlength=strlen(config.APssid);
	 uint8_t APPASSWORDlength=strlen(config.APpassword);
	 CmdBufLength=10+APSSIDlength+APPASSWORDlength;
	 CmdBuf[0]=0X7E;
	 CmdBuf[1]=0X53;   //T
	 CmdBuf[2]=CmdBufLength-6;
	 CmdBuf[3]=(CmdBufLength-6)>>8;   //L
	 CmdBuf[4]=0X00;   //V_IP
	 CmdBuf[5]=APSSIDlength;
	{
		for(uint8_t i=0;i<APSSIDlength;i++)
		CmdBuf[6+i]=config.APssid[i];
	 
	 }
   CmdBuf[6+APSSIDlength]=0X01;   //V_MASK
	 CmdBuf[7+APSSIDlength]=APPASSWORDlength;
	{
		for(uint8_t i=0;i<APPASSWORDlength;i++)
		CmdBuf[8+APSSIDlength+i]=config.APpassword[i];
	 
	 }
	 CmdBuf[8+APSSIDlength+APPASSWORDlength]=0X00;   //校验和
	 
	 for(uint8_t i=1;i<(8+APSSIDlength+APPASSWORDlength);i++)
	 CmdBuf[8+APSSIDlength+APPASSWORDlength]+=CmdBuf[i];//把AP字符串赋值给config
	 CmdBuf[9+APSSIDlength+APPASSWORDlength]=0x7e;
	 CmdBufLength=10+APSSIDlength+APPASSWORDlength;
	 WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 return 1;
}

uint8_t command_replytcpserve()  //返回IP地址
{
	 uint8_t TcpServer_IPlength=strlen(config.TcpServer_IP);
	 uint8_t TcpServer_Portlength=strlen(config.TcpServer_Port);
	 CmdBufLength=10+TcpServer_Portlength+TcpServer_IPlength;
	 CmdBuf[0]=0X7E;
	 CmdBuf[1]=0X54;   //T
	 CmdBuf[2]=CmdBufLength-6;
	 CmdBuf[3]=(CmdBufLength-6)>>8;   //L
	 CmdBuf[4]=0X00;   //V_IP
	 CmdBuf[5]=TcpServer_IPlength;
	{
		for(uint8_t i=0;i<TcpServer_IPlength;i++)
		CmdBuf[6+i]=config.TcpServer_IP[i];
	 
	 }
   CmdBuf[6+TcpServer_IPlength]=0X01;   //V_MASK
	 CmdBuf[7+TcpServer_IPlength]=TcpServer_Portlength;
	{
		for(uint8_t i=0;i<TcpServer_Portlength;i++)
		CmdBuf[8+TcpServer_IPlength+i]=config.TcpServer_Port[i];
	 
	 }
	 CmdBuf[8+TcpServer_IPlength+TcpServer_Portlength]=0X00;   //校验和
	 
	 for(uint8_t i=1;i<(8+TcpServer_IPlength+TcpServer_Portlength);i++)
	 CmdBuf[8+TcpServer_IPlength+TcpServer_Portlength]+=CmdBuf[i];//把AP字符串赋值给config
	 CmdBuf[9+TcpServer_IPlength+TcpServer_Portlength]=0x7e;
	 CmdBufLength=10+TcpServer_IPlength+TcpServer_Portlength;
	 WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 return 1;
}


float interscale=0;
uint8_t command_reply_scale()
{  
	 unsigned char *floatCdata;
	 uint16_t channelnum=Acceleration_ADCHS;  //这边限制通道个数
	 CmdBuf[0]=0X7E;
	 CmdBuf[1]=0X08;   
	 CmdBuf[2]=channelnum*17;
	 CmdBuf[3]=(channelnum*17)>>8;   //L
	 for(uint8_t i=0;i<channelnum;i++)
	 {
	 CmdBuf[4+17*i]=i;   //V_IP
	
	 CmdBuf[5+17*i]=config.channel_freq[i];
	 CmdBuf[6+17*i]=config.channel_freq[i]>>8; 
	 CmdBuf[7+17*i]=config.channel_freq[i]>>16; 
	 CmdBuf[8+17*i]=config.channel_freq[i]>>24; 		 
   interscale=1.0f/config.floatscale[i];  

	 floatCdata=(unsigned char *)&interscale;
	 CmdBuf[9+17*i]=*floatCdata;
	 CmdBuf[10+17*i]=*(floatCdata+1);;
	 CmdBuf[11+17*i]=*(floatCdata+2);
	 CmdBuf[12+17*i]=*(floatCdata+3);
	 floatCdata=(unsigned char *)&config.alarmgate[i];
	 CmdBuf[13+17*i]=*floatCdata;
	 CmdBuf[14+17*i]=*(floatCdata+1);;
	 CmdBuf[15+17*i]=*(floatCdata+2);
	 CmdBuf[16+17*i]=*(floatCdata+3);
	 floatCdata=(unsigned char *)&config.floatrange[i];
	 CmdBuf[17+17*i]=*floatCdata;
	 CmdBuf[18+17*i]=*(floatCdata+1);;
	 CmdBuf[19+17*i]=*(floatCdata+2);
	 CmdBuf[20+17*i]=*(floatCdata+3);
		 

	
	 }

	 CmdBuf[channelnum*17+4]=0;
	 for(uint16_t j=1;j<(channelnum*17+4);j++)
	 {
		CmdBuf[channelnum*17+4]+=CmdBuf[j];
	 }
	 CmdBuf[channelnum*17+5]=0x7e;
	 CmdBufLength=channelnum*17+6;
	 WriteDataToTXDBUF(CmdBuf,CmdBufLength);

   return 1;
}



uint8_t command_adjust_adc()
{ 
	 {
		for(uint8_t i=0;i<CmdBufLength;i++)
		CmdBuf[i]=RXDCmdBuf[i];
		CmdBufLength=CmdBufLength;
	 }
	 WriteDataToTXDBUF(CmdBuf,CmdBufLength);


	 if(CmdBuf[4]==1){
	 config.floatscale[0]=1;
   config.floatscale[1]=1;
	 config.floatscale[2]=1;
	 config.floatadc[0]=1;
   config.floatadc[1]=1;
	 config.floatadc[2]=1;
	 return 1;
	 }
	 if(CmdBuf[5]==1){
	 config.floatadc[0]=100.0f/Parameter.EffectiveValue[0];	 		 
	 return 1;
	 }
	 if(CmdBuf[6]==1){
	 config.floatadc[1]=100.0f/Parameter.EffectiveValue[1];	 		 
	 return 1;
	 }
	 if(CmdBuf[7]==1){
	 config.floatadc[2]=100.0f/Parameter.EffectiveValue[2];	 		 

	 saveConfig();
   return 1;

	 }
 
	return 1;

}
	 


typedef union  
{  
    float fdata;  
    unsigned long ldata;  
}FloatLongType; 


uint8_t command_set_scale()
{ 
	 {
		for(uint8_t i=0;i<CmdBufLength;i++)
		CmdBuf[i]=RXDCmdBuf[i];
		CmdBufLength=CmdBufLength;

	 }
	 WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 uint8_t channelnum=(((uint32_t)CmdBuf[3]<<8)+CmdBuf[2])/17;
	 for(uint16_t j=0;j<Acceleration_ADCHS;j++)//for(uint16_t j=0;j<channelnum;j++) //只对加速度进行设置
	 {
		FloatLongType fl;  
		fl.ldata=0;  
    config.channel_freq[j]=(((uint32_t)CmdBuf[8+17*j]<<24)+((uint32_t)CmdBuf[7+17*j]<<16)
															+((uint32_t)CmdBuf[6+17*j]<<8)+CmdBuf[5+17*j]); 
   
		fl.ldata=0;  
    fl.ldata=CmdBuf[12+17*j];  
    fl.ldata=(fl.ldata<<8)|CmdBuf[11+17*j];  
    fl.ldata=(fl.ldata<<8)|CmdBuf[10+17*j];  
    fl.ldata=(fl.ldata<<8)|CmdBuf[9+17*j];  
		config.floatscale[j]=1.0f/fl.fdata;  
    
		fl.ldata=0;  
    fl.ldata=CmdBuf[16+17*j];  
    fl.ldata=(fl.ldata<<8)|CmdBuf[15+17*j];  
    fl.ldata=(fl.ldata<<8)|CmdBuf[14+17*j];  
    fl.ldata=(fl.ldata<<8)|CmdBuf[13+17*j];  
		
		config.alarmgate[j]=fl.fdata; 
		fl.ldata=0; 
		fl.ldata=CmdBuf[20+17*j];  
    fl.ldata=(fl.ldata<<8)|CmdBuf[19+17*j];  
    fl.ldata=(fl.ldata<<8)|CmdBuf[18+17*j];  
    fl.ldata=(fl.ldata<<8)|CmdBuf[17+17*j];  
		config.floatrange[j]=fl.fdata; 
   
		Parameter.ReciprocalofRange[j]=32768/config.floatrange[j];
	 }
	 saveConfig();
   
   return 1;
}

uint8_t command_set_ap(void)   //设置TCP_SERVER的目标上位机地址
{
	{
		for(uint8_t i=0;i<CmdBufLength;i++)
		CmdBuf[i]=RXDCmdBuf[i];
	  CmdBufLength=CmdBufLength;
    WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 }
#ifdef ProgramESP32ByMyself
	 CmdBuf[1]=0xe5; //
	 for(uint32_t i=1;i<(CmdBufLength-2);i++)
	 CmdBuf[CmdBufLength-2]+=CmdBuf[i];
	 WriteDataToTXDBUF(CmdBuf,CmdBufLength);
#endif
	 for(uint8_t i=0;i<CmdBuf[5];i++)
	 config.APssid[i]=CmdBuf[6+i];//把AP字符串赋值给config
	 config.APssid[CmdBuf[5]]=0;
	 for(uint8_t i=0;i<CmdBuf[7+CmdBuf[5]];i++)
	 config.APpassword[i]=CmdBuf[8+CmdBuf[5]+i];//把AP字符串赋值给config
	 config.APpassword[CmdBuf[7+CmdBuf[5]]]=0;
	 return (1);
}

uint8_t command_reply_runmode(void)   //查询工作模式
{
	{
		CmdBuf[0]=0x7e;
		CmdBuf[1]=0x56;
		CmdBuf[2]=0x01;
		CmdBuf[3]=0x00;
		CmdBuf[4]=config.DataToBoardMode; //两个，一个温度一个加速度
		
		CmdBuf[5]=0;
		for(uint16_t i=1;i<5;i++)
		CmdBuf[5]+=CmdBuf[i];		 
    CmdBuf[6]=0x7e;
	  CmdBufLength=7;
    WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 }
	 return 1;
}

uint16_t command_receive_active_beacon(void)
{
	ReceiveActiveBeaconMessage();
	return 1;
}


uint8_t command_setrunmode(void)   //设置工作模式
{
	{
		for(uint8_t i=0;i<CmdBufLength;i++)
		CmdBuf[i]=RXDCmdBuf[i];
	  CmdBufLength=CmdBufLength;
    WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 }
	 switch(CmdBuf[4]){
		 case 1:
			
		   //vTaskDelay (pdMS_TO_TICKS(10));
		   
			 config.DataToBoardMode=PARAMETERMODE;
		   EnableTempTransmission();
		  // TPM_StartSHAOTimer(TPM1, kTPM_SystemClock); //去低通效果，后期不能这么采，要直接减去一个值
		 break;
		 case 2:
		
			 config.DataToBoardMode=WAVEMODE;
		   EnableTempTransmission();
	
		 break;
		 case 3:
			
			 config.DataToBoardMode=FFTWAVEMODE;
		   EnableTempTransmission();
		 break;
		 case 4:
			
			 config.DataToBoardMode=FFTPARAMETERMODE;
		   EnableTempTransmission();
		 break;
		 case 5:
			 config.DataToBoardMode=IDLEMODE;
		   DisableTempTransmission();
		  
		 break;
		 case 6:
			 config.DataToBoardMode=LITEWAVEMODE;
		   DisableTempTransmission();
		
		 break;
		 default:
		 break;
	 } 

 return 1;	
}  
uint8_t command_set_snnumber(void)   //设置TCP_SERVER的目标上位机地址
{
	{
		for(uint8_t i=0;i<CmdBufLength;i++)
		CmdBuf[i]=RXDCmdBuf[i];
	  CmdBufLength=CmdBufLength;
    WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 }

	 config.SNnumber=(uint64_t)CmdBuf[4]+((uint64_t)CmdBuf[5]<<8)+((uint64_t)CmdBuf[6]<<16)+((uint64_t)CmdBuf[7]<<24)+((uint64_t)CmdBuf[8]<<32)+((uint64_t)CmdBuf[9]<<40)+((uint64_t)CmdBuf[10]<<48)+((uint64_t)CmdBuf[11]<<56);//把AP字符串赋值给config
	 saveConfig();
	 return (1);
}
uint8_t command_set_channel_condition(void)   //设置TCP_SERVER的目标上位机地址
{
	{
		for(uint8_t i=0;i<CmdBufLength;i++)
		CmdBuf[i]=RXDCmdBuf[i];
	  CmdBufLength=CmdBufLength;
    WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 }

	 config.DataToSendChannel=((uint16_t)RXDCmdBuf[5]<<8)+RXDCmdBuf[4];;//把AP字符串赋值给config
	 //saveConfig();
	 return (1);
}

extern void EarseFlash(uint32_t address,uint32_t length);

volatile uint32_t receive_iap_data_length=0;
volatile uint8_t receive_iap_pack_index=0;
uint8_t reply_shut_iap(void)
{
	CmdBuf[0]=0x7e;	
	CmdBuf[1]=COMMAND_IAP_STATUE;	
	CmdBuf[2]=0x05;	
	CmdBuf[3]=0x00;	
	CmdBuf[4]=0x04;	
	CmdBuf[5]=0x00;	
	CmdBuf[6]=0x00;	
	CmdBuf[7]=0x00;	
	CmdBuf[8]=0x00;	
	CmdBuf[9]=0x00;
	CmdBuf[9]=COMMAND_IAP_STATUE+0X09;	
	CmdBuf[10]=0x7e;
	CmdBufLength=11;
	WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	return 1;
}

uint8_t reply_complete_iap(void)
{
	CmdBuf[0]=0x7e;	
	CmdBuf[1]=COMMAND_IAP_STATUE;	
	CmdBuf[2]=0x05;	
	CmdBuf[3]=0x00;	
	CmdBuf[4]=0x03;	
	CmdBuf[5]=0x00;	
	CmdBuf[6]=0x00;	
	CmdBuf[7]=0x00;	
	CmdBuf[8]=0x00;	
	CmdBuf[9]=0x00;	
	CmdBuf[9]=COMMAND_IAP_STATUE+0X08;	
	CmdBuf[10]=0x7e;
	CmdBufLength=11;
	WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	return 1;
}

uint8_t reply_requie_contioue_iap(void)
{
	CmdBuf[0]=0x7e;	
	CmdBuf[1]=COMMAND_IAP_STATUE;	
	CmdBuf[2]=0x05;	
	CmdBuf[3]=0x00;	
	CmdBuf[4]=0x02;	
	CmdBuf[5]=0x00;	
	CmdBuf[6]=0x00;	
	CmdBuf[7]=0x00;	
	CmdBuf[8]=0x00;	
	CmdBuf[9]=0x00;	
	CmdBuf[9]=COMMAND_IAP_STATUE+0X07;	
	CmdBuf[10]=0x7e;
	CmdBufLength=11;
	WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	return 1;
}

uint8_t reply_requie_contioue_iap_data(void)
{
	CmdBuf[0]=0x7e;	
	CmdBuf[1]=COMMAND_IAP_DATA;	
	CmdBuf[2]=0x01;	
	CmdBuf[3]=0x00;	
	CmdBuf[4]=0x01;	
	CmdBuf[5]=0x00;	
	CmdBuf[5]=COMMAND_IAP_DATA+0X02;	
	CmdBuf[6]=0x7e;
	CmdBufLength=7;
	WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	return 1;
}

uint8_t reply_stop_iap_data(void)
{
	CmdBuf[0]=0x7e;	
	CmdBuf[1]=COMMAND_IAP_DATA;	
	CmdBuf[2]=0x01;	
	CmdBuf[3]=0x00;	
	CmdBuf[4]=0x00;	
	CmdBuf[5]=0x00;	
	CmdBuf[5]=COMMAND_IAP_DATA+0X01;	
	CmdBuf[6]=0x7e;
	CmdBufLength=7;
	WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	return 1;
}

uint8_t command_reply_sw_version(void)
{
	CmdBuf[0]=0x7e;	
	CmdBuf[1]=COMMAND_REPLY_SW_VERSION;	
	CmdBuf[2]=0x10;
	CmdBuf[3]=0x00;
	for(uint32_t i=0;i<16;i++) //strcpy
	{
		CmdBuf[4+i]=sw_version[i];
	}
	CmdBuf[20]=0;
	for(uint32_t i=1;i<20;i++) //strcpy
	{
		CmdBuf[20]+=CmdBuf[i];
	}
	CmdBuf[21]=0x7e;
	CmdBufLength=22;
	WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	return 1;
}

volatile uint32_t erase_data_length=0;
uint8_t command_iap_statue(void)   //设置TCP_SERVER的目标上位机地址
{
	if(RXDCmdBuf[4]==0x01)
	{
	 
//	 stop_ad7606_samp();
		Parameter.IapDataLength=(uint32_t)RXDCmdBuf[5]+((uint32_t)RXDCmdBuf[6]<<8)+((uint32_t)RXDCmdBuf[7]<<16)+((uint32_t)RXDCmdBuf[8]<<24);
		if((Parameter.IapDataLength%64)!=0)
			erase_data_length=(Parameter.IapDataLength/64+1)*64;
		else 
		erase_data_length=Parameter.IapDataLength;
	EarseFlash(IAP_ADDRESS,erase_data_length);
	receive_iap_data_length=0;	
	receive_iap_pack_index=0;
	reply_requie_contioue_iap();
	}else if(RXDCmdBuf[4]==0x03)
	{
	receive_iap_data_length=0;	
	receive_iap_pack_index=0;
	
	reply_shut_iap();
	}
	
	 return (1);
}
extern void WRBYTES(uint32_t address,uint32_t num, uint8_t *Data);
extern uint8_t WRBYTES_iap_data(uint32_t address,uint32_t num, uint8_t *Data);
uint8_t rece_iap_data[128];
uint8_t error_iap;
uint8_t command_iap_data(void)   //设置TCP_SERVER的目标上位机地址
{
	uint8_t write_32_Byte[64]={0};
	if(receive_iap_pack_index==RXDCmdBuf[7])
	{
	receive_iap_pack_index++;
	uint32_t current_iap_pack_length=RXDCmdBuf[2]+((uint16_t)RXDCmdBuf[3]<<8)-4;
		if(Parameter.IapDataLength>=(receive_iap_data_length+current_iap_pack_length))
		{
		
		if(Parameter.IapDataLength==(receive_iap_data_length+current_iap_pack_length))
		{
			for(uint32_t i=0;i<64;i++ )
			write_32_Byte[i]=0;
			for(uint32_t i=0;i<current_iap_pack_length;i++ )
			write_32_Byte[i]=RXDCmdBuf[8+i];
//			WRBYTES_iap_data(IAP_ADDRESS+receive_iap_data_length,64,write_32_Byte);
		}
		else{
//			WRBYTES_iap_data(IAP_ADDRESS+receive_iap_data_length,current_iap_pack_length,(uint8_t *)RXDCmdBuf+8);
		}
		receive_iap_data_length+=current_iap_pack_length;
			if(Parameter.IapDataLength==receive_iap_data_length)
			{
			
					reply_complete_iap();
				  config.Iap_datalength=erase_data_length;
				  config.Iap_flag=0x01;
				  saveConfig();
//				  software_reset();
			}
			else 
				reply_requie_contioue_iap_data();
		}
		else
		{
		receive_iap_data_length=0;	
		receive_iap_pack_index=0;	
		reply_stop_iap_data();
		}
	}else
	{
	reply_stop_iap_data();	
	}
	 //saveConfig();
	 return (1);
}



uint16_t command_set_factory_parameter(void)
{
	return 1;
}

extern void BoardPeroidWave(void); 

RTC_T Requirdperiodwave_tRTC;
uint8_t command_require_periodwave(void)
{ 
	{
		Requirdperiodwave_tRTC.Year=g_tRTC.Year;
		Requirdperiodwave_tRTC.Mon=g_tRTC.Mon;
		Requirdperiodwave_tRTC.Day=g_tRTC.Day; //时间用32位表示
		Requirdperiodwave_tRTC.Hour=g_tRTC.Hour;
		Requirdperiodwave_tRTC.Min=g_tRTC.Min;
		Requirdperiodwave_tRTC.Sec=g_tRTC.Sec; //时间用32位表示		
		
//		for(uint8_t i=0;i<CmdBufLength;i++)
//		CmdBuf[i]=RXDCmdBuf[i];
//	  CmdBufLength=CmdBufLength;
//    WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	}
	config.RequirePeriodChannel=((uint16_t)RXDCmdBuf[5]<<8)+RXDCmdBuf[4];

//	if(config.DataToBoardMode==PARAMETERMODE)
	{
		EnablePeroidWaveTransmission();
		Parameter.PeroidWaveTransmissionCounter=((uint16_t)RXDCmdBuf[7]<<8)+RXDCmdBuf[6];
	}
	return 1;
}
uint8_t command_reply_channel_condition(void)   //返回通道字
{
	{
		CmdBuf[0]=0x7e;
		CmdBuf[1]=0x57;
		CmdBuf[2]=0x02;
		CmdBuf[3]=0x00;
		CmdBuf[4]=config.DataToSendChannel; //两个，一个温度一个加速度
		CmdBuf[5]=config.DataToSendChannel>>8; //两个，一个温度一个加速度
		CmdBuf[6]=0;
		for(uint16_t i=1;i<6;i++)
		CmdBuf[6]+=CmdBuf[i];
		 
    CmdBuf[7]=0x7e;
	  CmdBufLength=8;
    WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 }
	 return 1;
}

uint8_t command_reply_SampleParameter(void)   //设置采样参数
{


	{
		CmdBuf[0]=0x7e;
		CmdBuf[1]=0x06;
		CmdBuf[2]=0x0B;
		CmdBuf[3]=0x00;
		CmdBuf[4]=config.ADfrequence; //两个，一个温度一个加速度
		
		CmdBuf[5]=config.ADfrequence>>8;
		CmdBuf[6]=config.ADfrequence>>16; //两个，一个温度一个加速度
		
		CmdBuf[7]=config.ADfrequence>>24;
    CmdBuf[8]=config.ADtime;
		CmdBuf[9]=config.workcycleseconds;
		CmdBuf[10]=config.workcycleseconds>>8;
		CmdBuf[11]=(uint8_t)config.PeriodTransimissonStatus;
		CmdBuf[12]=(config.PeriodTransimissonCounter*config.workcycleseconds);
		CmdBuf[13]=(config.PeriodTransimissonCounter*config.workcycleseconds)>>8;
		CmdBuf[14]=(uint8_t)config.ParameterTransimissonStatus;
		CmdBuf[15]=0;
		for(uint16_t i=1;i<15;i++)
		CmdBuf[15]+=CmdBuf[i];
		 
    CmdBuf[16]=0x7e;
	  CmdBufLength=17;
    WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 }

	 return 1;
}


uint8_t command_set_SampleParameter(void)   //设置采样参数
{
	{
		for(uint8_t i=0;i<CmdBufLength;i++)
		CmdBuf[i]=RXDCmdBuf[i];
	  CmdBufLength=CmdBufLength;
    WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 }
 
	 
	 config.ADfrequence=(uint32_t)CmdBuf[4]+((uint32_t)CmdBuf[5]<<8)+((uint32_t)CmdBuf[6]<<16)+((uint32_t)CmdBuf[7]<<24);//设置采样速率
	 config.ADtime=CmdBuf[8]; //AD采样时间
	 if(config.ADtime>2) config.ADtime=2;   //限制AD采样时间为2S
	 config.workcycleseconds =(uint16_t)CmdBuf[9]+((uint16_t)CmdBuf[10]<<8);

	 config.PeriodTransimissonStatus=(bool)CmdBuf[11];
	 uint32_t counter=(uint16_t)CmdBuf[12]+((uint16_t)CmdBuf[13]<<8);	  //是否使能自动发送
	 config.PeriodTransimissonCounter=(uint32_t)(counter/config.workcycleseconds); //自动发送周期
	 config.ParameterTransimissonStatus=(bool)CmdBuf[14];

	 
	// TPM_SetTimerPeriod(TPM1, ((CLOCK_GetFreq(kCLOCK_PllFllSelClk)/1)/(config.ADfrequence*ADCHS)-1));
//	 Parameter.ReciprocalofADfrequence=1.0f/((float)config.ADfrequence); //采样频率倒数;
//	 Parameter.ReciprocalofEMUnumber=1.0f/((float)config.ADfrequence*config.ADtime); //跟采样时间有关	 
	 saveConfig();
	 return (1);
}


uint8_t command_set_time(void)   //设置时间
{ 
	
	{
		for(uint8_t i=0;i<CmdBufLength;i++)
		CmdBuf[i]=RXDCmdBuf[i];
	  CmdBufLength=CmdBufLength;
    WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 }
	 
	 return (1);
}


uint8_t command_set_tcpserve(void)   //设置TCP_SERVER的目标上位机地址
{
	{
		for(uint8_t i=0;i<CmdBufLength;i++)
		CmdBuf[i]=RXDCmdBuf[i];
	  CmdBufLength=CmdBufLength;
    WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	 }
	 for(uint8_t i=0;i<CmdBuf[5];i++)
	 config.TcpServer_IP[i]=CmdBuf[6+i];//把AP字符串赋值给config
	 config.TcpServer_IP[CmdBuf[5]]=0;
	 for(uint8_t i=0;i<CmdBuf[7+CmdBuf[5]];i++)
	 config.TcpServer_Port[i]=CmdBuf[8+CmdBuf[5]+i];//把AP字符串赋值给config
	 config.TcpServer_Port[CmdBuf[7+CmdBuf[5]]]=0;
	 return (1);
}

uint8_t command_applynetset()
{ 
	{
	for(uint8_t i=0;i<CmdBufLength;i++)
	CmdBuf[i]=RXDCmdBuf[i];
	CmdBufLength=CmdBufLength;
	
 }

	CmdBuf[0]=0x7e;
	CmdBuf[1]=0xff;
	CmdBuf[2]=0x00;
	CmdBuf[3]=0x00;
	CmdBuf[4]=0xff;
	CmdBuf[5]=0x7e;
  CmdBufLength=6;
	WriteDataToTXDBUF(CmdBuf,CmdBufLength);
  saveConfig();

	return 1;
	
}

uint8_t command_reset_system(void)
{
//	 software_reset();
	return 1;
}
uint8_t AnalyCmd(uint16_t length)
{ 

	switch( RXDCmdBuf[1] ){     //看命令行
		case COMMAND_ID:     //0x02
			   command_id();
			break;
		case COMMAND_CHANNELKIND:     //0x03 通道类型设置
			   command_channelkind();
			break;
//		case COMMAND_REPLYIP:     //0x03 通道类型设置
//			   command_replyip();
//			break;
		case COMMAND_STOP:
			   command_stop();
			break;
//		case COMMAND_RECEIVE_BEACON:
//				command_receive_beacon();
//			break;
//		case COMMAND_SETIP:   //设置IP地址
//			   command_setip();
//			break;
//		case COMMAND_SET_RUNMODE:   //设置IP地址
//			   command_setrunmode();
//			break;
		case COMMAND_START:
			   command_start();
			break;
		case COMMAND_REQUIRE_PERIODWAVE:
			   command_require_periodwave();
			break;
//		case COMMAND_SET_AP:
//			   command_set_ap();
//			break;
//		case COMMAND_RECEIVE_ACTIVE_BEACON:
//					command_receive_active_beacon();
//			break;
//		case COMMAND_SET_FACTORY_PARAMETER:
//					command_set_factory_parameter();
//			break;
		case COMMAND_SET_CHANNEL_CONDITION:
			   command_set_channel_condition();
			break;
		case COMMAND_REPLY_CHANNEL_CONDITION:
			   command_reply_channel_condition();
			break;
		case COMMAND_REPLY_RATE:    //设置采样参数
			   command_reply_SampleParameter();
		  break;
		case COMMAND_SAMPLE_RATE:    //设置采样参数
			   command_set_SampleParameter();
			break;
//		case COMMAND_SET_TCPSERVE:   //设置tcp server的目标地址类似
//			   command_set_tcpserve();
//		  break;
//		case COMMAND_REPLYTCPSERVE:   //返回tcp server的目标地址类似
//			   command_replytcpserve();
//			break; 
//		case COMMAND_SET_TIME:           //返回ap值
//			   command_set_time();
//      break;		
//		case COMMAND_REPLYAP:
//			   command_replyap();  //返回ap值
//			break;
//    case COMMAND_APPLYNETSET:
//			   command_applynetset();  //应用网络设置
//			break;
//		 case 0x40:
//			   command_counter();  //应用网络设置
//			break;
//		case COMMAND_REPLY_RUNMODE:
//			   command_reply_runmode();
//		  break;
//		case COMMAND_ADJUST_TEMP:
//			   command_adjust_temp();
//		  break;
//		case COMMAND_ADJUST_ADC:
//			   command_adjust_adc();
//		  break;
		case COMMAND_SET_SCALE:
			   command_set_scale();
			break;
		case COMMAND_REPLY_SCALE:
			   command_reply_scale();
			break;
    case COMMAND_SET_SNNUMBER:
			   command_set_snnumber();
			break;
//		case COMMAND_IAP_STATUE:
//			   command_iap_statue();
//			break;
//		case COMMAND_REPLY_SW_VERSION:
//			   command_reply_sw_version();
//			break;
//		case COMMAND_IAP_DATA:
//			   command_iap_data();
//			break;
//		case COMMAND_RESET_SYSTEM:
//			   command_reset_system();
			break;
		default:
			return 1;
	}
 return 1;
	
}


uint16_t getTelLength(void)  //求的是TLV中V的长度
{
					
	return(((uint16_t)RxdTelBuf[3]<<8)+RxdTelBuf[2]);
  
}
uint8_t  isVaildTel(void)
{
	if(RxdBytes>=1)if(RxdTelBuf[0]!=0x7e)return(0);
	if(RxdBytes>=2)if((RxdTelBuf[1]!=COMMAND_START)&&(RxdTelBuf[1]!=COMMAND_STOP)&&(RxdTelBuf[1]!=COMMAND_ID)
		                  &&(RxdTelBuf[1]!=COMMAND_CHANNELKIND)&&(RxdTelBuf[1]!=COMMAND_REPLYIP)&&(RxdTelBuf[1]!=COMMAND_SETIP)
						  &&(RxdTelBuf[1]!=COMMAND_REPLY_RATE)&&(RxdTelBuf[1]!=COMMAND_SAMPLE_RATE)&&(RxdTelBuf[1]!=COMMAND_ADJUST_TEMP)
						  &&(RxdTelBuf[1]!=COMMAND_REPLY_SCALE)&&(RxdTelBuf[1]!=COMMAND_SET_SCALE)&&(RxdTelBuf[1]!=COMMAND_ADJUST_ADC)
						  &&(RxdTelBuf[1]!=COMMAND_SET_SNNUMBER)&&(RxdTelBuf[1]!=COMMAND_REQUIRE_PERIODWAVE)&&(RxdTelBuf[1]!=COMMAND_SET_CHANNEL_CONDITION)
							&&(RxdTelBuf[1]!=COMMAND_SET_RUNMODE)&&(RxdTelBuf[1]!=COMMAND_SET_TIME)&&(RxdTelBuf[1]!=COMMAND_SET_AP)
						  &&(RxdTelBuf[1]!=COMMAND_RECEIVE_BEACON)&&(RxdTelBuf[1]!=COMMAND_SET_TCPSERVE)&&(RxdTelBuf[1]!=COMMAND_REPLYAP)
						  &&(RxdTelBuf[1]!=COMMAND_REPLYTCPSERVE)&&(RxdTelBuf[1]!=COMMAND_APPLYNETSET)&&(RxdTelBuf[1]!=COMMAND_REPLY_RUNMODE)
							&&(RxdTelBuf[1]!=COMMAND_REPLY_CHANNEL_CONDITION)&&(RxdTelBuf[1]!=COMMAND_RECEIVE_ACTIVE_BEACON)&&(RxdTelBuf[1]!=COMMAND_SET_FACTORY_PARAMETER)
							&&(RxdTelBuf[1]!=COMMAND_IAP_DATA)&&(RxdTelBuf[1]!=COMMAND_IAP_STATUE)&&(RxdTelBuf[1]!=COMMAND_REPLY_SW_VERSION)&&(RxdTelBuf[1]!=COMMAND_RESET_SYSTEM))
				return(0);  //命令行最大ID
	
  if(RxdBytes>=4) { 
		uint16_t length=getTelLength();
		if((length>200)) return(0);  //限制最大的为1000
	}
	return(1);				 // 合法的
}
uint8_t sumofRxdBuf(uint16_t l)  //求和的报文，不包含起始标识,	l	的长度包括起始标识
{ 
	uint8_t sum=0;
	if(l<2) return (0);
	for(uint16_t i=1;i<l-2;i++)
	 sum=sum+RxdTelBuf[i];
	return (sum);
}
uint8_t isTelComplete(void)	   // =0 不完整  =1 sum Error =2 正确
{
	uint32_t  temp8;
	uint32_t   dat_len;

	if(RxdBytes<4)return(0);
  ////////////////
	dat_len=getTelLength()+6;	//
	if(dat_len==0)return(0);
	if(RxdBytes<(dat_len))return(0);

	temp8=sumofRxdBuf(dat_len);
 
  if (RxdTelBuf[dat_len-1]==0x7e)
		return(2); 
	if (RxdTelBuf[dat_len-2]==temp8)
		return(2); 
	else{
		return(1);
	}	
}						 

uint8_t leftRxdTel(void)		//数组左移一位
{
	uint32_t i;
	if(RxdBytes<1)return(0);     // 无法左移
	for	(i=1;i<RxdBytes;i++)
	{
		RxdTelBuf[i-1]=RxdTelBuf[i];		
	}
	RxdBytes--;
	return(1);					 // 丢弃一个字节成功

}

 void RxdByte(uint8_t c)
{	
	uint32_t 	i;
	RxdTelBuf[RxdBytes]=c;
	RxdBytes++;

	switch(RxdBytes)
	{
		case 0:	break;
		case 3:	break;
		case 1:
		case 2:
		case 4:while(!isVaildTel())	//如果不合法			 
				{
					if(!leftRxdTel())break;	  // 丢弃首字节
				}
				break;
			
		
		default:		
				i=isTelComplete();
				if(i==2)
				{
					//do some thing
					for(uint16_t j=0;j<RxdBytes;j++)
					RXDCmdBuf[j]=RxdTelBuf[j];
					CmdBufLength=RxdBytes;
					AnalyCmd(CmdBufLength);
					
					RxdBytes=0;
					
				}
				else if(i==1)	 // CRC error
				{
					leftRxdTel();
					while(!isVaildTel())	//如果不合法			 
					{
						if(!leftRxdTel())break;
					}	
				}
				else if(i==0) //没收完继续收
				{
				
				}
				else
				{
				}
				break;
			
		}
	
}
int send_server_data( uint8_t *data, uint16_t len )
{
												 
	if( tcp_client_server_conn )
	{
		return netconn_write( tcp_client_server_conn, data, len, NETCONN_COPY);
	}
	else
		
	
		return ERR_CONN; 

}

#endif /* LWIP_NETCONN */
















