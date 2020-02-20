    /*
    *********************************************************************************************************
    *        函 数 名: MDMA_SpeedTest
    *        功能说明: MDMA性能测试
    *        形    参: 无
    *        返 回 值: 无
    *********************************************************************************************************
    */
		
#include "bsp.h"			/* 底层硬件驱动 */
#include "app.h"			/* 底层硬件驱动 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"
#include <lwip/sockets.h>

#include "lwip/sys.h"
#include "lwip/api.h"

#include "lwip/tcp.h"
#include "lwip/ip.h"

MDMA_HandleTypeDef MDMA_Handle;

//float emu_data[2][8][16384] __attribute__((at(0xC0000000)));  //双缓存，8通道51200个数据 0x19 0000
//uint8_t TXD_BUFFER_NET[5000][1600] __attribute__((at(0xC0400000))); //0xa0 0000
//
//uint32_t TXD_BUFFER_NET_Length[5000];
uint16_t volatile TransferCompleteDetected=0;
//uint16_t testtt[1024]={0x1111,0x2222,0x3333,0x5445,0x6666,0x1111};
extern volatile uint32_t TxdBufHeadIndex;
extern volatile uint32_t TxdBufTailIndex;
extern SemaphoreHandle_t WRITE_ready;
uint8_t WriteDataToTXDBUF(uint8_t * source,uint16_t length);


uint8_t WriteDataToTXDBUF(uint8_t * source,uint16_t length)
{
	uint32_t i = 0;
	xSemaphoreTake(WRITE_ready, portMAX_DELAY);//等信号量
	
		if(isTxdBufFull()) 
		{		
			taskEXIT_CRITICAL();
			xSemaphoreGive(WRITE_ready);//释放信号量这块
		return 0;
		} //队列满了就不完
		
		
		for(i=0;i<length;i++)
			TXDBUF[TxdBufHeadIndex][i]=source[i];

		TXDBUFLength[TxdBufHeadIndex]=length;
		Increase(TxdBufHeadIndex);  
		taskEXIT_CRITICAL();
		xSemaphoreGive(WRITE_ready);//释放信号量这块

		return 1;

}

void MDMA_IRQHandler(void)
{
	HAL_MDMA_IRQHandler(&MDMA_Handle);
}
static void MDMA_TransferCompleteCallback(MDMA_HandleTypeDef *hmdma)
{
	TransferCompleteDetected = 1;
}
void MDMA_init(void)
{
				/* MDMA配置 **********************************************************************/
	__HAL_RCC_MDMA_CLK_ENABLE();  

	MDMA_Handle.Instance = MDMA_Channel0;  

	MDMA_Handle.Init.Request              = MDMA_REQUEST_SW;         /* 软件触发 */
	MDMA_Handle.Init.TransferTriggerMode  = MDMA_BLOCK_TRANSFER;     /* 块传输 */
	MDMA_Handle.Init.Priority             = MDMA_PRIORITY_HIGH;      /* 优先级高*/
	MDMA_Handle.Init.Endianness           = MDMA_LITTLE_ENDIANNESS_PRESERVE; /* 小端 */
	MDMA_Handle.Init.SourceInc            = MDMA_SRC_INC_BYTE;         /* 源地址自增，双字，即8字节 */
	MDMA_Handle.Init.DestinationInc       = MDMA_DEST_INC_BYTE;        /* 目的地址自增，双字，即8字节 */
	MDMA_Handle.Init.SourceDataSize       = MDMA_SRC_DATASIZE_BYTE;    /* 源地址数据宽度双字，即8字节 */
	MDMA_Handle.Init.DestDataSize         = MDMA_DEST_DATASIZE_BYTE;   /* 目的地址数据宽度双字，即8字节 */
	MDMA_Handle.Init.DataAlignment        = MDMA_DATAALIGN_PACKENABLE;       /* 小端，右对齐 */                    
	MDMA_Handle.Init.SourceBurst          = MDMA_SOURCE_BURST_128BEATS;      /* 源数据突发传输，128次 */
	MDMA_Handle.Init.DestBurst            = MDMA_DEST_BURST_128BEATS;        /* 源数据突发传输，128次 */
	
	MDMA_Handle.Init.BufferTransferLength = 128;    /* 每次传输128个字节 */

	MDMA_Handle.Init.SourceBlockAddressOffset  = 0; /* 用于block传输，地址偏移0 */
	MDMA_Handle.Init.DestBlockAddressOffset    = 0; /* 用于block传输，地址偏移0 */

	/* 初始化MDMA */
	if(HAL_MDMA_Init(&MDMA_Handle) != HAL_OK)
	{
					 Error_Handler(__FILE__, __LINE__);
	}
	
	/* 设置传输完成回调和中断及其优先级配置 */
	HAL_MDMA_RegisterCallback(&MDMA_Handle, HAL_MDMA_XFER_CPLT_CB_ID, MDMA_TransferCompleteCallback);
	HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MDMA_IRQn);  

	HAL_MDMA_Start_IT(&MDMA_Handle,
																(uint32_t)TXDBUF[1],
																(uint32_t)TXDBUF[0],
																10,
																1);  //要先空传一次，不然第一次总是0，奇怪		 

	
}

extern SemaphoreHandle_t WRITE_ready;

//uint8_t WriteDataToTXDBUF(uint8_t * source,uint32_t length)
//{ 
//	uint32_t i=0;
//	
//	if(xSemaphoreTake(WRITE_ready, portMAX_DELAY) != pdTRUE);//  等信号量，没毛病
//	taskENTER_CRITICAL();   //不应该屏蔽所有中断，先用着
//	if(isTxdBufFull()) 
//	{
//	
//	taskEXIT_CRITICAL();
//	xSemaphoreGive(WRITE_ready);//释放信号量这块

//	return 0;
//	} //队列满了就不完
//	
//	
//	for(i=0;i<length;i++)
//		TXDBUF[TxdBufHeadIndex][i]=source[i];
//		/* AXI SRAM向SDRAM的64KB数据传输测试 ***********************************************/
////	TransferCompleteDetected = 0;
////	HAL_MDMA_Start_IT(&MDMA_Handle,
////																(uint32_t)source,
////																(uint32_t)TXD_BUFFER_NET[TxdBufHeadIndex],
////																length,
////																1);
////	
////	while (TransferCompleteDetected == 0) 
////  {
////    /* wait until MDMA transfer complete or transfer error */
////  }

//	TXDBUFLength[TxdBufHeadIndex]=length;
//	Increase(TxdBufHeadIndex);  
//	taskEXIT_CRITICAL();
//	xSemaphoreGive(WRITE_ready);//释放信号量这块
////	bsp_LedStatue(0,1);
//	return 1;
//}

extern struct netconn *tcp_client_server_conn;
volatile int32_t err_flag=0;
volatile int32_t mem_remain=0;
volatile uint32_t tcp_delay_counter=0;
extern uint8_t REGISTER_FLAG;
extern ip_addr_t dest_ipaddr;
extern uint16_t dest_port;
extern struct netconn *tcp_client_server_conn;
extern  SemaphoreHandle_t SAMPLEDATA_ready;
extern void BoardAutoPeroidWave(void);
void send_buffer_task( void )
{
	MDMA_init();

	while(1)
	
{
	while(!(xSemaphoreTake(SAMPLEDATA_ready, portMAX_DELAY) == pdTRUE))
	{};
	BoardAutoPeroidWave();

}
//	for(;;)
//	{
//		if(isCollectServer()) //tcp_client_server_conn)//
//		{
//			if( !isTxdBufEmpty() )//!isTxdBufEmpty()
//			{
//				mem_remain=(int32_t)TxdBufHeadIndex-(int32_t)TxdBufTailIndex;
////				SCB_InvalidateDCache_by_Addr ((uint32_t *)TXD_BUFFER_NET[TxdBufTailIndex], TXD_BUFFER_NET_Length[TxdBufTailIndex]*0.25f);
//				netbuf_alloc(sendbuf,TXDBUFLength[TxdBufTailIndex]);
//				memcpy(sendbuf->p->payload,TXDBUF[TxdBufTailIndex],TXDBUFLength[TxdBufTailIndex]);
//				err = netconn_sendto(tcp_client_server_conn,sendbuf,&dest_ipaddr,dest_port);
//				if(err != ERR_OK) {
//				printf("============================");
//				}
//				netbuf_delete(sendbuf);
//				if( err==ERR_OK)
//				{
//					Increase(TxdBufTailIndex);

//				}
//				else
//				{
//					DisCollectServer();  //链接断开标志
//					REGISTER_FLAG = 0x00;
//					vTaskDelay(1);
////					AD7606_StopRecord();
//				}

//			}else
//			{
//				vTaskDelay(2);
//			}
		
//		} else
//		{
//		AD7606_StopRecord();
//		TxdBufHeadIndex=TxdBufTailIndex;	
//		vTaskDelay(2);

//		}
//	}
}
	static	struct netbuf* udpsendbuf;
void Udp_To_Pc(uint8_t *send_buf,uint32_t size)
{

  err_t err;
	udpsendbuf = netbuf_new();
	netbuf_alloc(udpsendbuf,size);
	memcpy(udpsendbuf->p->payload,send_buf,size);

	err = netconn_sendto(tcp_client_server_conn,udpsendbuf,&dest_ipaddr,dest_port);//dest_port);
//		err = netconn_send(tcp_client_server_conn, udpsendbuf);
				if(err != ERR_OK) {
				printf("============================");
				}
				netbuf_delete(udpsendbuf);
}
extern uint8_t Sencond_Cnt;
extern uint16_t BoardPeroidWave_packege_flag[3];  //请求波形包号
extern uint8_t write_data_canbuf(uint8_t * source,uint16_t length);
void Analytic_Function(uint8_t *rec_buf)
{
	uint8_t can_data_send_buf[8];
	uint16_t camm_type = 0;
	uint16_t crc_check=0;
	camm_type = (rec_buf[1]<<8)+rec_buf[0];
	uint32_t udp_send_length = 0;
	switch(camm_type)
	{
		case 0x00://停止
	
		BoardPeroidWave_packege_flag[0]=0;
		BoardPeroidWave_packege_flag[1]=0;
		BoardPeroidWave_packege_flag[2]=0;
//		{
//			for(uint8_t i=0;i<8;i++)
//			CmdBuf[i]=rec_buf[i];			
//		}
			CmdBuf[0] = 0x00;
			CmdBuf[1] = 0x00;
			CmdBuf[2] = 0x04;
			CmdBuf[3] = 0x00;
			CmdBuf[4] = 0x00;
			CmdBuf[5] = 0x00;
			CmdBuf[6] = 0x01;
			CmdBuf[7] = 0x00;
			CmdBuf[8] = 0x01;
			CmdBuf[9] = 0x00;
			for(uint8_t j=0;j<10;j++)
			{
				crc_check+=CmdBuf[j];			
			}
			CmdBuf[10] = crc_check;
			CmdBuf[11] = crc_check<<8;
			CmdBufLength = 12;//(rec_buf[2]<<24)+(rec_buf[3]<<16)+(rec_buf[4]<<8)+rec_buf[5];
			WriteDataToTXDBUF(CmdBuf,CmdBufLength);
//		AD7606_StopRecord();
			can_data_send_buf[0] = 0x00;
			can_data_send_buf[1] = 0x00;
			write_data_canbuf(can_data_send_buf,8);
//			can1_SendPacket(can_data_send_buf,2);
			break;
		case 0x01://启动

		BoardPeroidWave_packege_flag[0]=0;
		BoardPeroidWave_packege_flag[1]=0;
		BoardPeroidWave_packege_flag[2]=0;
			CmdBuf[0] = 0x01;
			CmdBuf[1] = 0x00;
		
			CmdBuf[2] = 0x04;
			CmdBuf[3] = 0x00;
			CmdBuf[4] = 0x00;
			CmdBuf[5] = 0x00;
		
			CmdBuf[6] = 0x01;
			CmdBuf[7] = 0x00;
			CmdBuf[8] = 0x01;
			CmdBuf[9] = 0x00;
			for(uint8_t j=0;j<10;j++)
			{
				crc_check+=CmdBuf[j];			
			}
			CmdBuf[10] = crc_check;
			CmdBuf[11] = crc_check<<8;
			CmdBufLength = 12;//(rec_buf[2]<<24)+(rec_buf[3]<<16)+(rec_buf[4]<<8)+rec_buf[5];
			WriteDataToTXDBUF(CmdBuf,CmdBufLength);
//		 AD7606_StartRecord(config.ADfrequence);
			can_data_send_buf[0] = 0x00;
			can_data_send_buf[1] = 0x01;
			write_data_canbuf(can_data_send_buf,8);
			break;
		case 0x02://设置采样率
		for(uint8_t i=0;i<14;i++)
		{
			CmdBuf[i]=rec_buf[i];
		}
//			// 类型 2
//			CmdBuf[0] = 0x02;
//			CmdBuf[1] = 0x00;
//			//L 4
//			CmdBuf[2] = 0x06;
//			CmdBuf[3] = 0x00;
//			CmdBuf[4] = 0x00;
//			CmdBuf[5] = 0x00;
//			//板卡号 2
//			CmdBuf[6] = 0x01;
//			CmdBuf[7] = 0x00;
//			//采样率 4
//			CmdBuf[8] = 0x00;
//			CmdBuf[9] = 0x40;
//			CmdBuf[10] = 0x00;
//			CmdBuf[11] = 0x00;
//			//校验 2
//			for(uint8_t j=0;j<12;j++)
//			{
//				crc_check+=CmdBuf[j];			
//			}
//			CmdBuf[12] = crc_check;
//			CmdBuf[13] = crc_check<<8;
			CmdBufLength = 14;
//			WriteDataToTXDBUF(CmdBuf,CmdBufLength);
	

			for(uint8_t i=0;i<12;i++)
			{
				config.channel_freq[i]=(((uint32_t)rec_buf[11]<<24)+((uint32_t)rec_buf[10]<<16)
											+((uint32_t)rec_buf[9]<<8)+rec_buf[8]); 
			}
			can_data_send_buf[0]=rec_buf[11];
			can_data_send_buf[1]=rec_buf[10];
			can_data_send_buf[2]=rec_buf[9];
			can_data_send_buf[3]=rec_buf[8];
			WriteDataToTXDBUF(CmdBuf,CmdBufLength);
			write_data_canbuf(can_data_send_buf,8);
//						saveConfig();
			break;
		case 0x03://设置网络参数
						
		for(uint8_t i=0;i<20;i++)
		{
			CmdBuf[i]=rec_buf[i];
		}
		CmdBufLength = 20;
//	WriteDataToTXDBUF(CmdBuf,CmdBufLength);
    WriteDataToTXDBUF(CmdBuf,20);
//保存板卡IP
			//IP
			config.LocalIP[0]=rec_buf[6];//把AP字符串赋值给config
			config.LocalIP[1]=rec_buf[7];//把AP字符串赋值给config
			config.LocalIP[2]=rec_buf[8];//把AP字符串赋值给config
			config.LocalIP[3]=rec_buf[9];//把AP字符串赋值给config
		//mask
			config.LocalMASK[0]=rec_buf[10];//把AP字符串赋值给config
			config.LocalMASK[1]=rec_buf[11];//把AP字符串赋值给config
			config.LocalMASK[2]=rec_buf[12];//把AP字符串赋值给config
			config.LocalMASK[3]=rec_buf[13];//把AP字符串赋值给config
		//gateway
			config.LocalGATEWAY[0]=rec_buf[14];
			config.LocalGATEWAY[1]=rec_buf[15];
			config.LocalGATEWAY[2]=rec_buf[16];
			config.LocalGATEWAY[3]=rec_buf[17];


			saveConfig();
			break;
		case 0x04://读取采集器型号 这个不知道返回什么数据
						
			for(uint8_t i=0;i<10;i++)
			{
				CmdBuf[i]=rec_buf[i];
			}
			CmdBufLength = 10;
//			WriteDataToTXDBUF(CmdBuf,CmdBufLength);
			WriteDataToTXDBUF(CmdBuf,10);
		break;
	}
}