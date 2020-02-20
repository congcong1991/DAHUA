/*
*********************************************************************************************************
*
*	模块名称 : 主程序模块
*	文件名称 : main.c
*	版    本 : V1.1
*	说    明 : 
*
*	修改记录 :
*		版本号   日期         作者        说明
*		V1.0    2018-12-12   Eric2013     1. CMSIS软包版本 V5.4.0
*                                     2. HAL库版本 V1.3.0
*
*   V1.1    2019-04-01   suozhang     1. add FreeRTOS V10.20
*
*	Copyright (C), 2018-2030, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/	
#include "bsp.h"			/* 底层硬件驱动 */
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"  
#include "bsp_spi_txrx.h"

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/

float PT_Temp[AD7606_ADCHS];  //温度值
void UpdateTempParameter(float yy,uint8_t i);

uint8_t TXDBUF[512][2000] __attribute__((at(0xC0400000)));
float emu_data[2][SLAVE_MACHINE_NUM][AD7606_ADCHS][51200] __attribute__((at(0xC0000000)));  //双缓存，8通道16384个数据 0x19 0000

volatile uint32_t wp[16];
volatile uint8_t jiaoyansum[16];
volatile uint16_t wave_packege_flag=0;
//uint8_t WaveToSend[AD7606_ADCHS][1024];
volatile uint32_t currentSAMPLEblock=0;
void UpdateWave(float yy,uint8_t i) //????,??yy??????i???
{   
//	 int16_t senddata=(int16_t)(yy*Parameter.ReciprocalofRange[i]);
//		if(((config.DataToSendChannel>>i)&0x01)) 
//		{
//			WaveToSend[i][wp[i]+14]=senddata;//*floatdata;  //修改sendkk;//
//			WaveToSend[i][wp[i]+15]=senddata>>8;
//			jiaoyansum[i]+=(WaveToSend[i][wp[i]+14]+WaveToSend[i][wp[i]+15]);
//			wp[i]=wp[i]+2;			
//		}
//	 if(wp[i]>(BOARDPOINTS-1)) {	

//		  
//		  uint16_t buflength=BOARDPOINTS+7+2+1;
//			WaveToSend[i][0]=0x7e;//TELid;
//			WaveToSend[i][1]=0x40;//TELid>>8;	
//			WaveToSend[i][2]=(uint8_t)buflength;//TELid>>16;
//			WaveToSend[i][3]=buflength>>8;// TELid>>24; //2,3????????482??
//		  WaveToSend[i][4]=g_tRTC.Year;
//		  WaveToSend[i][5]=g_tRTC.Year>>8;
//		  WaveToSend[i][6]=g_tRTC.Mon;
//		  WaveToSend[i][7]=g_tRTC.Day; //时间用32位表示
//		  WaveToSend[i][8]=g_tRTC.Hour;
//		  WaveToSend[i][9]=g_tRTC.Min;
//		  WaveToSend[i][10]=g_tRTC.Sec; //时间用32位表示
//		  WaveToSend[i][11]=(uint8_t)wave_packege_flag; //时间用32位表示
//		  WaveToSend[i][12]=wave_packege_flag>>8; //时间用32位表示
//			WaveToSend[i][13]=i;
//		  for(uint32_t ii=1;ii<14;ii++)
//		  jiaoyansum[i]+=WaveToSend[i][ii];  //adch是从0开始的
//			WaveToSend[i][14+BOARDPOINTS]=jiaoyansum[i];  //2,3????????482??
//		 	WaveToSend[i][15+BOARDPOINTS]=0x7e; 
//		  WriteDataToTXDBUF(WaveToSend[i],BOARDPOINTS+16);
//			
//		
//		  wave_packege_flag++;
//		  jiaoyansum[i]=0;			
//			wp[i]=0;
//		}
	
}


void UpdateAccelerationData(float yyy,uint8_t ii,uint8_t i,uint32_t DataIndex)
{
	if(config.DataToBoardMode==PARAMETERMODE)  //在特征值模式下，且正在采集
		{
		 emu_data[currentSAMPLEblock][ii][i][DataIndex]=yyy;//yyy;
		}else if(config.DataToBoardMode==WAVEMODE)
		{  
		emu_data[currentSAMPLEblock][ii][i][DataIndex]=yyy;
//		 UpdateWave(yyy,i); 
		}	
	
}

int32_t halfref=0;
uint32_t ActualIndex=0;
int32_t AD_ZERO[AD7606_ADCHS],AD_ZEROlowpass[AD7606_ADCHS],AD_INTER[AD7606_ADCHS],lastdata[AD7606_ADCHS],filtercounter[AD7606_ADCHS];
extern  SemaphoreHandle_t SAMPLEDATA_ready;
extern __attribute__((section (".RAM_D1"))) int16_t Ad7606_Data[SLAVE_MACHINE_NUM][AD7606SAMPLEPOINTS*AD7606_ADCHS*2];
extern volatile uint32_t CurrentAD7606DataCounter;
extern  SemaphoreHandle_t AD7606_ready;
uint32_t sprase_counter[SLAVE_MACHINE_NUM][12],StoreDateIndex[SLAVE_MACHINE_NUM][12];
void emu_sprase_index()
{
	uint32_t i=0;
	for(i=0;i<AD7606_ADCHS;i++){  //某方面限制了基准采样率为51200，因为8通道必须同步采样，所以8个通道只能一个基准采样率
			Parameter.sparse_index[i]=config.ADfrequence/config.channel_freq[i];
		}
}
float test_buf_yy[8];
extern int current_rx_machine;
extern uint8_t spi_data_come[SLAVE_MACHINE_NUM];
extern SemaphoreHandle_t sem_spi_idle;
extern uint8_t board_all_read;
void AD7606_TASK(void)
{
    uint32_t i,j,ii;
		int16_t *p;  //????????16???
		int32_t y;
	  float volatile vy=0;//速度中间变量
	//	uint32_t si=0;
	  uint32_t FreFactor=1;
		float yy=0;
		float yy_temp=0;
	  halfref=32768;
		
  	for(i=0;i<AD7606_ADCHS;i++){
		  AD_ZERO[i]=(uint64_t)32768*16384;  // tao 10s 
			AD_ZEROlowpass[i]=0u;
			AD_INTER[i]=0;
			Parameter.vs[i]=0;
			lastdata[i]=0;
			wp[i]=0;
			jiaoyansum[i]=0;		
			filtercounter[i]=0;
			sprase_counter[0][i]=0;
			sprase_counter[1][i]=0;
			sprase_counter[2][i]=0;
			StoreDateIndex[0][i]=0;
			StoreDateIndex[1][i]=0;
			StoreDateIndex[2][i]=0;
			Parameter.ReciprocalofRange[i]=32768/config.floatrange[i];
			Parameter.sparse_index[i]=config.ADfrequence/config.channel_freq[i];
		}

		while(1)
		{		
			while(!(xSemaphoreTake(AD7606_ready, portMAX_DELAY) == pdTRUE))
			{};
			for(ii=0;ii<SLAVE_MACHINE_NUM;ii++){
				
				if(((board_all_read>>ii)&0x01)==0)  //board_all_read总共几块板子，没有的不用处理
				{
				 continue;
				}
					
				SCB_InvalidateDCache_by_Addr ((uint32_t *)&Ad7606_Data[ii][CurrentAD7606DataCounter], AD7606SAMPLEPOINTS*AD7606_ADCHS);

				p=(int16_t *)&Ad7606_Data[ii][CurrentAD7606DataCounter];

				for(j=0;j<AD7606SAMPLEPOINTS;j++){
					for(i=0;i<AD7606_ADCHS;i++){
						y=(int32_t)((*(int16_t*)p));//
		
						if(config.interface_type[i]==TYPE_IEPE){	
								 //直接使用上次的低通值既可以					

							 AD_ZERO[i]=y+(((int64_t)AD_ZERO[i]*32767)>>15);
							 y=y-(int32_t)(AD_ZERO[i]>>15); //AD_ZEROlowpass[i]
							
							 if(config.channel_freq[i]==25600)
							 {
								FreFactor = 0;
							 }else if(config.channel_freq[i]==12800)
							 {
								FreFactor = 1;
							 }else if(config.channel_freq[i]==6400)
							 {
								FreFactor = 2;
							 }else if(config.channel_freq[i]==3200)
							 {
								FreFactor = 3;
							 }									 						
							 /**************************抽样部分************************/		
							sprase_counter[ii][i]++;
							if(sprase_counter[ii][i]>=Parameter.sparse_index[i]) 
							{
								sprase_counter[ii][i]=0;
									//用来记录储存数据的下标
								yy=((float)y*0.15258789f*config.floatscale[i]-config.floatadjust[i])*config.floatadc[i];//*config.floatscale[i];//*0.1f;//-config.floatscale[i];//-500.0f; // mv???0.038146f						
								StoreDateIndex[ii][i] = ActualIndex>>FreFactor;
								UpdateAccelerationData(yy,ii,i,StoreDateIndex[ii][i]);//ii代表板卡  i代表通道
								StoreDateIndex[ii][i]++;							
							}
						}
						else if(config.interface_type[i]==TYPE_4_20MA)//温度
						{ 						
							 if(config.channel_freq[i]==25600)
								{
										FreFactor=0;	
								}
								else if(config.channel_freq[i]==12800)
								{
										FreFactor=1;	
								}
								else if(config.channel_freq[i]==6400)
								{
										FreFactor=2;	
								}
								else if(config.channel_freq[i]==3200)
								{
										FreFactor=3;	
								}
							sprase_counter[ii][i]++;
							if(sprase_counter[ii][i]>=Parameter.sparse_index[i]) 
							{
								sprase_counter[ii][i]=0;
								yy=(float)y*0.15258789f; //+-5V量程  range拉低   16位采样    /32786*5000 
								StoreDateIndex[ii][i] = ActualIndex>>FreFactor;
								UpdateAccelerationData(yy,ii,i,StoreDateIndex[ii][i]);
	//							yy_temp  = yy;
								StoreDateIndex[ii][i]++;						
	//							UpdateTempParameter(yy_temp,i);
							}

						}
						else if(config.interface_type[i]==TYPE_NONE)
						{
							halfref=y;
						}				 			
					p++;       
					}
					ActualIndex++;   //原则上应该加个中断锁的，特别是当特征值模式下，写这个下标参数为0时，后来改了一个方案				
				
//						if(ActualIndex>=config.ADfrequence){ //config.ADfrequence,多采一秒数据，进行滤波分析
					if(ActualIndex>=BUFFERSIZE){
//							RTC_ReadClock();	/* 读时钟，结果在 g_tRTC */
							for(i=0;i<AD7606_ADCHS;i++){
								StoreDateIndex[ii][i]=0;
								sprase_counter[ii][i]=0; //全部归0
							}					
							 ActualIndex=0;
							 currentSAMPLEblock=(currentSAMPLEblock+1)%2;

//							 xSemaphoreGive(SAMPLEDATA_ready);
						}
					}

	
			}
			xSemaphoreGive(SAMPLEDATA_ready);
	}
}

uint32_t TEMPwp[AD7606_ADCHS];
float Temp_sum[AD7606_ADCHS];
void UpdateTempParameter(float yy,uint8_t i)
{   
		if(config.interface_type[i]==TYPE_PT){
		if(TEMPwp[i]<config.channel_freq[i]){
		Temp_sum[i]+=yy;
		TEMPwp[i]++;
		}else	  
		{
		 Parameter.average[i]=Temp_sum[i]/config.channel_freq[i];//???127??,?????????????0.0078125f;//(yy+(Parameter.Abs_average[i]*255/256))/256; 	
		 Temp_sum[i]=0;
		 TEMPwp[i]=0;
		}
	 }
		else if(config.interface_type[i]==TYPE_4_20MA){
		if(TEMPwp[i]<config.channel_freq[i]){
		Temp_sum[i]+=yy;
		TEMPwp[i]++;
		}else	  
		{
		 Parameter.average[i]=Temp_sum[i]/config.channel_freq[i]*0.004-4;//*config.floatscale[i];//???127??,?????????????0.0078125f;//(yy+(Parameter.Abs_average[i]*255/256))/256; 	
		 Temp_sum[i]=0;
		 TEMPwp[i]=0;
		}
	 }
}


	//A＝0.0038623139728
		//R(t)＝R0(1＋At)
//		(1+At) = R(t) / R0;
//		(R(t) / R0) - 1 = At;
//		A＝0.0038623139728
//		((R(t) / R0) - 1) / A = t;

void Send_Temp(void)
{
}
