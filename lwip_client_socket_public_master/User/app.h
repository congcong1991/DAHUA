#ifndef _APP_H_
#define _APP_H_
/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "stm32h7_flash.h"

#include <stdbool.h>
#define SLAVE_MACHINE_NUM 3
#define MAIN_BOARD  1




//#define MAIN_BOARD  0
#define UDP_BUF_SIZE 50
extern uint8_t udp_server_recvbuf_index_tail;
extern uint8_t udp_server_recvbuf_index_head;
#define isReceiveUdpBufEmpty() (udp_server_recvbuf_index_tail==udp_server_recvbuf_index_head)

#define      RX_BUFFER_SIZE                              1024
#define      TX_BUFFER_SIZE                              1024

//extern uint8_t TXDBUF[TxdBufLine][BOARDPOINTS+20];
extern volatile uint32_t TxdBufHeadIndex;
extern volatile uint32_t TxdBufTailIndex;

extern void DelayMS ( uint16_t t);
extern volatile uint16_t RxdBufHeadIndex;
extern volatile uint16_t RxdBufTailIndex;


extern uint8_t CmdBuf[TX_BUFFER_SIZE] __attribute__((at(0xC03E0000)));
extern uint16_t CmdBufLength;


#define IdleRxdBufSize 1024  //串口空闲中断用来接受数据缓存区长度
extern uint8_t iap_data_in_sdram[800000] __attribute__((at(0xC1000000)));

#define AD7606_ADCHS 8
#define Acceleration_ADCHS 8
#define Temp_ADCHS 0
#define AD7606SAMPLEPOINTS 128
#define AD7606_SAMPLE_COUNT AD7606_ADCHS*2*AD7606SAMPLEPOINTS
#define BUFFERSIZE AD7606_ADCHS*AD7606SAMPLEPOINTS

#define BOARDPOINTS AD7606SAMPLEPOINTS*Acceleration_ADCHS*sizeof(int16_t) //(4*ADCHS*SAMPLEPOINTS)
#define PERIODBOARDPOINTS AD7606SAMPLEPOINTS*sizeof(int16_t) //(4*ADCHS*SAMPLEPOINTS)
#define ADC16_ByDMA_SAMPLE_COUNT (2*ADCHS*SAMPLEPOINTS) /* The ADC16 sample count. */


/*用于发送网络的数据*/
#define TxdBufLine 512
#define Increase(x)   {x=(x+1)%TxdBufLine;}
#define isTxdBufFull() ((TxdBufHeadIndex+1)%TxdBufLine==TxdBufTailIndex)
#define isTxdBufEmpty() (TxdBufHeadIndex==TxdBufTailIndex)

extern uint16_t TXDBUFLength[TxdBufLine];

/*用于接收网络的数据*/
extern uint32_t NetReceiveBufHeadIndex;
extern uint32_t NetReceiveBufTailIndex;
#define NetReceiveBufSize  1024
#define IncreaseNetReceiveBuf(x)   {x=(x+1)%NetReceiveBufSize;}
#define isNetReceiveBufFull() ((NetReceiveBufHeadIndex+1)%NetReceiveBufSize==NetReceiveBufTailIndex)
#define isNetReceiveBufEmpty() (NetReceiveBufHeadIndex==NetReceiveBufTailIndex)



extern float emu_data[2][SLAVE_MACHINE_NUM][AD7606_ADCHS][51200] __attribute__((at(0xC0000000)));  //双缓存，8通道51200个数据 0x19 0000
//extern uint8_t TXD_BUFFER_NET[5000][1600] __attribute__((at(0xC0400000))); //0xa0 0000
extern uint8_t TXDBUF[512][2000] __attribute__((at(0xC0400000)));
extern uint8_t WriteDataToTXDBUF(uint8_t * source,uint16_t length);
extern void AD7606_StartRecord(uint32_t _ulFreq);
extern void AD7606_StopRecord(void);

#define RxdBufLine  1024
//#define EMUPONITS  16390*2  //最大取2S的数据

#define IncreaseRxdBufNum(x)   {x=(x+1)%RxdBufLine;}
#define isRxdBufFull() ((RxdBufHeadIndex+1)%RxdBufLine==RxdBufTailIndex)
#define isRxdBufEmpty() (RxdBufHeadIndex==RxdBufTailIndex)
//命令行
#define COMMAND_STOP 0x00
#define COMMAND_START 0x01
#define COMMAND_ID 0x02
#define COMMAND_CHANNELKIND 0x03  //读取通道类型
#define COMMAND_REPLYIP 0x04
#define COMMAND_SETIP 0x05  //设置IP地址
#define COMMAND_REPLY_RATE 0x06   //设置采样参数
#define COMMAND_SAMPLE_RATE 0x07   //设置采样参数
#define COMMAND_REPLY_SCALE 0x08
#define COMMAND_SET_SCALE 0x09
#define COMMAND_ADJUST_TEMP 0x69
#define COMMAND_ADJUST_ADC 0x68
#define COMMAND_SET_SNNUMBER 0x88

#define COMMAND_DEVICE_KEY 0x10

#define COMMAND_RECEIVE_BEACON 0x11
#define COMMAND_RECEIVE_ACTIVE_BEACON 0x12
#define COMMAND_SET_FACTORY_PARAMETER 0x13
#define COMMAND_SET_BEACON_PARAMETER 0x20
#define COMMAND_REPLY_BEACON_PARAMETER 0x21


#define COMMAND_REQUIRE_PERIODWAVE 0x30 //请求一次单次波形
#define COMMAND_SET_CHANNEL_CONDITION 0x0a  //设置通道字
#define COMMAND_SET_RUNMODE 0x0B  //工作模式，特征值模式，还是波形模式，或者报警模式啥的
#define COMMAND_SET_TIME 0x50  //设置时间
#define COMMAND_SET_AP 0x51  //设置AP地址
#define COMMAND_SET_TCPSERVE 0x52  //设置server端的地址
#define COMMAND_REPLYAP 0x53
#define COMMAND_REPLYTCPSERVE 0x54
#define COMMAND_APPLYNETSET 0x55
#define COMMAND_REPLY_RUNMODE 0x56  //工作模式，特征值模式，还是波形模式，或者报警模式啥的
#define COMMAND_REPLY_CHANNEL_CONDITION 0x57  //设置通道字
#define COMMAND_SET_DNS 0x58  //工作模式，特征值模式，还是波形模式，或者报警模式啥的
#define COMMAND_REPLY_DNS 0x59  //设置通道字



#define COMMAND_REPLY_SW_VERSION 0x80  //版本号
#define COMMAND_SET_SW_VERSION 0x81  //版本号
#define COMMAND_IAP_DATA 0x82  //IAP数据包
#define COMMAND_IAP_STATUE 0x83  //IAP状态包
#define COMMAND_RESET_SYSTEM 0x84  //重启软件

#define COMMAND_SET_CHANNELKIND   0x94 //设置采集信号类别  
#define COMMAND_REPLY_CHANNELKIND   0x93 //读取采集信号类别  

#define COMMAND_REPLY_Connection_Method 0x95 //读取连接方式  wifi还是网线
#define COMMAND_SET_Connection_Method   0x96  //设置连接方式

#define COMMAND_ESP32_STATUE 0xEA  //esp32连接状态
#define COMMAND_ADJUST_4_20MA 0x67  //校准4-20ma，类似温度校准

#define IAP_ADDRESS 0x08100000

#define EEPROM_START  0X081C0000 


typedef struct  CONFIG				 // 配置信息
{
	uint16_t vaildsign;

	uint8_t baundrate;    /* =0:600    =1:1200 		=2:2400 		=3:4800 		=4:9600 */
	uint8_t addr; 
	uint64_t SNnumber; 
	uint8_t parity;		// =0 : n,8,1   =1: o,8,1  =2: e,8,1	 数据格式
	float floatscale[12];
	//float floatrange[12];
	uint8_t DisplayMode;  // 显示模式　=0　固定　=1 循环
	uint8_t interface_type[12]; // 输入类型
	uint8_t unit[12];  // 单位
	float floatrange[12]; // 转换系数
	float floatadjust[12]; // 修正值
	//uint16_t interface_addr[12]; // modbus 地址 上传
	float alarmgate[12]; // 绝对值
	float floatadc[12]; // 绝对值
	uint8_t means	;// 均值类型
	uint16_t means_times; // 均值积算周期
	uint16_t freq;  // 采样频率 Hz
	uint16_t avr_count;
	uint8_t reflash; // 刷新时间 
	uint16_t din_addr;  //  开关量输入寄存器地址
	uint16_t dout_addr; //  开关量输出寄存器地址
	uint32_t force_gate,force_backlash;  // 应变启动阈值， 回差。
	uint16_t max_addr0,max_addr1;        //
	uint16_t vlpsseconds;            //  
	uint16_t vlprseconds;           //
	uint16_t runseconds;           //
	uint16_t pga;                 //
	uint16_t workcycleseconds;   //  工作周期
	uint16_t fangda;            //  放大倍数
	uint16_t boardset;         // 发射功率设置
	uint16_t ADtime;          //AD单次采样时间
	uint16_t ADfrequence;    //AD采样频率
	  
	uint64_t alarmminutetime;  //开始报警时间
	uint32_t FLASH_WRADDR;    //flash起始地址
	uint8_t  DataToBoardMode; 
	uint16_t  DataToSendChannel;
	uint8_t  DHCP;
	char APssid[20];
	char APpassword[20];
	char TcpServer_IP[20];
	char TcpServer_Port[10];
	char LocalIP[20];
	char LocalGATEWAY[20];
	char LocalMASK[20];
	uint32_t PeriodTransimissonCounter;
	uint8_t PeriodTransimissonStatus;
	uint8_t ParameterTransimissonStatus;
	uint8_t RequirePeriodChannel; //请求哪些单次波形的通道
	uint8_t RESETTIME;
	uint8_t Enable_active_beacon;
	uint8_t Iap_flag;
	uint32_t Iap_datalength;
	uint32_t channel_freq[12];
	uint32_t BeaconInterval;
	char server_address[20];
	char DNS_SERVERIP[20];
	uint8_t Connection_Method;
}CONFIG;
//	uint8_t Vibrate_Cnt;//振动通道个数
//	uint8_t Temp_Cnt;//温度通道个数
//	uint8_t Rotate_Speed_Cnt;//转速通道个数
extern  struct CONFIG  config;  //配置信息



typedef enum {No_OS_Smapling = 0x000, OS_2_Smapling, OS_4_Smapling, OS_8_Smapling, OS_16_Smapling, OS_32_Smapling, OS_64_Smapling, }  Average_level;

//typedef enum {Wifi_Connection=0,Wire_Connection=1,} Connection_Method;
typedef enum {NoBrainTransmission=0,BrainTransmission=1,} Esp32TransmissionMode;
typedef enum {EnableDHCP=1,DisableDHCP=0,} DHCPmode;
typedef enum {PARAMETERMODE=1,WAVEMODE=2,FFTWAVEMODE=3,FFTPARAMETERMODE=4,IDLEMODE=5,LITEWAVEMODE=6} DataToBoard_TYPE;
/* TYPE_HBR 全桥 */
//typedef enum {TYPE_NONE=0,TYPE_MA=1,TYPE_V=2, TYPE_IEPE=3,TYPE_PT=4,TYPE_HBR=5,TYPE_AC_V=6,TYPE_SPEED=7,} INTERFACE_TYPE;
typedef enum {TYPE_NONE=0,TYPE_IEPE=1,TYPE_SPEED=2, TYPE_PT=3, TYPE_4_20MA=4} INTERFACE_TYPE;
typedef enum {UNIT_NONE=0,UNIT_V=1,UNIT_A=2,UNIT_KV=3,UNIT_TEMP=4,UNIT_M=5,UNIT_M_S=6,UNIT_M_S2=7,UNIT_G=8,UNIT_MA=9,UNIT_DB=10,UNIT_MM_S=11,} INTERFACE_UNIT;
typedef enum {MEANS_NONE=0,
							MEANS_MEANS, // 均值
							MEANS_RMS, // 均方根值
							MEANS_P,  // 峰值
							MEANS_PP, // 峰峰值
} MEANS_TYPE;  
typedef enum {BAUND2400=0,BAUND4800,BAUND9600,BAUND19200,BAUND38400,} BAUND_RATE;
//typedef enum {BAM4E33=0,BAM4E31=1, BAM4I33=2,BAM4I31=3,BAM4U33=4,BAM4U31=5,BAM4H33=6,BAM4H31=7,BAM4P33=8,BAM4P31=9,BAM4Q33=10,BAM4Q31=11}MODEL ;
extern uint8_t 	brightness; 
extern	uint8_t FirstBlood; //解决温度上传冲突
typedef  struct PARAMETER				 // 所有参数
{
		/*******新增的参数**************/
	uint32_t can_addr;
	uint32_t can_target_addr;
	uint8_t selftest_statue;
	uint8_t board_kind;
	uint16_t sensor_satue;
	float InterAD_avg[16];
	/*******新增的参数**************/
	uint16_t vaildsign;
	int32_t int_v[12];
	int32_t int_av[12];
	int64_t s[12]; // 累积
	int64_t as[12]; // 累积
	uint32_t sparse_index[12];
	float v[12];
	float fv[12];
	float vs[12];  //速度累加和，浮点型
	float av[12];
	float adate;                //加速度
	float vdate;                 //速度
	float xdate;                  //位移
	float pdate;                   //温度
	float scale_v[12];
	int32_t gate[12];
	int32_t backlash[12];
	uint8_t alarm[12]; // 报警标志
	uint16_t din;  // 低8位对应 8个通道输入
	uint16_t dout; // 低4位对应 4个输出通道 
	uint32_t status;
	int32_t ch1_int_max,ch2_int_max;
	int32_t ch1_int_max1,ch2_int_max2;
	float ch1_max,ch2_max;
	int32_t vibrate_gate1,vibrate_gate2,vibrate_backlash1,vibrate_backlash2;
	int32_t force_gate,force_backlash;
	int16_t selfpgaallow;
	uint16_t daytime;
	uint16_t minutetime;
	uint16_t selffangda;
  uint16_t alarmminutetime;
	uint8_t Esp32TransmissionMode;
	uint32_t sleeptime;
	uint8_t wakeupsourec;
	float Abs_average[12];
	float ReciprocalofADfrequence[16];
	float Inter_Skew[12]; //歪度
	float InterIIMarginIndex[12];//裕度指标中间变量
	float InterMAX[12];
	float InterMIN[12];
	float S_sum[12];
	float SS_sum[12];
	float SSS_sum[12];
	float SSSS_sum[12];
	float Abs_S_average[12];
	float PeakValue[12];  //峰峰值
	float EffectiveValue[12]; //有效值
	float Skew[12]; //歪度
	float MaxValue[12]; //峰值
	float Kurtosis[12]; //峭度
	float Mean[12]; //均值
	float WaveformIndex[12]; //波形指标
	float PeakIndex[12];//峰值指标
	float PulseIndex[12];//脉冲指标
	float MarginIndex[12];//裕度指标
	float KurtosisIndex[12];//峭度指标
	float Inter_MarginIndex[12]; //峭度
	float S_average[12]; //平均值总值
	float average[12]; //平均值
	float ReciprocalofEMUnumber[16]; //采样频率的倒数，采样周期
	float ReciprocalofRange[16];//量程的倒数
	//
	float F_sum[12];     //s(k)之和
	float FS_sum[12];   //f(k)s(k)的和
	float FFS_sum[12];  //F(K)F(K)S(K)
	float FFFFS_sum[12];  //F(K)F(K)S(K)
	float F2_sum[12];     //feature2的中间和
	float F3_sum[12];     //feature3的中间和
	float F4_sum[12];     //feature4的中间和
	float F6_sum[12];     //feature6的中间和
	float F11_sum[12];     //feature11的中间和
	float F12_sum[12];     //feature12的中间和
	float F13_sum[12];     //feature12的中间和
	float F_feature1[12];
	float F_feature2[12];
	float F_feature3[12];
	float F_feature4[12];
	float F_feature5[12];
	float F_feature6[12];
	float F_feature7[12];
	float F_feature8[12];
	float F_feature9[12];
	float F_feature10[12];
	float F_feature11[12];
	float F_feature12[12];
	float F_feature13[12];
	uint32_t IapDataLength;
	uint32_t PeroidWaveTransmissionCounter;
	uint32_t AutoPeriodTransmissonCounter;
	uint64_t DeviceKey;
	uint8_t LINK_STATUS;//连接状态
	uint8_t LINK_SELECT;//连接哪个上位机IP
	float SPEED_ROTATE[8];
}PARAMETER;

extern struct PARAMETER Parameter;
typedef enum {LINK_STATUS_SUCCESSFUL=0,LINK_STATUS_FAILURE=1,} LINKcondition;
typedef enum {LINK_SELECT_1=0,LINK_SELECT_2=1,} LINKSELECTcondition;
typedef enum {PRE_TX=0,IN_TX=1,} TXcondition;
//typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
//typedef enum {VLLS = 0, MANUALRESET = !DISABLE} WakeupSourec;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

				
struct ADJUSTDATA					 // 校准数据
{	uint16_t vaildsign;
	uint32_t  UaSub,UbSub,UcSub;
	uint32_t IaSub,IbSub,IcSub;
	int32_t PhaseA,PhaseB,PhaseC;
	uint32_t  CVSub,OVSub;
};

struct JoinNet					 // 校准数据
{	uint32_t JoinCount;
	uint32_t CurrentLink;

};
extern struct JoinNet joinnetstatue;
extern  struct PARAMETER Parameter;
#define LPTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_LpoClk)

#define isReceiveActiveBeaconMessage() ((Parameter.status&0x01)!=0)
#define ReceiveActiveBeaconMessage() {Parameter.status|=0x01;}
#define DeleteActiveBeaconMessage() {Parameter.status&=~0x01;}

#define isFactoryLink() ((Parameter.status&0x02)!=0)
#define setFactoryLink() {Parameter.status|=0x02;}
#define clrFactoryLink() {Parameter.status&=~0x02;}

#define isAbleTempTransmission() ((Parameter.status&0x04)!=0)
#define EnableTempTransmission() {Parameter.status|=0x04;}
#define DisableTempTransmission() {Parameter.status&=~0x04;}

#define isAblePeroidWaveTransmission() ((Parameter.status&0x08)!=0)
#define EnablePeroidWaveTransmission() {Parameter.status|=0x08;}
#define DisablePeroidWaveTransmission() {Parameter.status&=~0x08;}


#define isAbleLiteWaveTransmission() ((Parameter.status&0x10)!=0)
#define EnableLiteWaveTransmission() {Parameter.status|=0x10;}
#define DisableLiteWaveTransmission() {Parameter.status&=~0x10;}

#define isAblePeroidWaveAutoTransmission() ((Parameter.status&0x20)!=0)
#define EnablePeroidWaveAutoTransmission() {Parameter.status|=0x20;}
#define DisablePeroidWaveAutoTransmission() {Parameter.status&=~0x20;}

#define isCollectingOverInParameterMode() ((Parameter.status&0x40)!=0)  //正在采集，不上传数据
#define overCollectingInParameterMode() {Parameter.status|=0x40;}
#define BeginCollectingInParameterMode() {Parameter.status&=~0x40;}

#define isReceiveBeaconMessage() ((Parameter.status&0x80)!=0)  //正在采集，不上传数据
#define ReceiveBeaconMessage() {Parameter.status|=0x80;}
#define DeleteBeaconMessage() {Parameter.status&=~0x80;}

#define IsableProcessInParameterMode() ((Parameter.status&0x0100)!=0)  //使能特征值下的数据处理
#define EnableProcessInParameterMode() {Parameter.status|=0x0100;}
#define DisableProcessInParameterMode() {Parameter.status&=~0x0100;}

#define IsNeedRestartCollect() ((Parameter.status&0x0200)!=0)  //使能特征值下的数据处理
#define NeedRestartCollect() {Parameter.status|=0x0200;}
#define NotNeedRestartCollect() {Parameter.status&=~0x0200;}

#define isCollectServer() ((Parameter.status&0x0400)!=0)
#define CollectServer() {Parameter.status|=0x0400;}
#define DisCollectServer() {Parameter.status&=~0x0400;}

//#define EnableEsp32Power()  {GPIO_PortSet(GPIOC, 1u << 0);}  //是否给esp32供电
//#define DisableEsp32Power() {GPIO_PortClear(GPIOC, 1u << 0);}

#define EnableAV3V3Power()  {GPIO_PortSet(GPIOC, 1u << 2);}  //是否供电模拟部分
#define DisableAV3V3Power() {GPIO_PortClear(GPIOC, 1u << 2);}

#define RUN1_SET() {GPIO_PortClear(GPIOA, 1u << 10);}  //呼吸等，GPIOA 10
#define RUN1_CLR() {GPIO_PortSet(GPIOA, 1u << 10);}

#define PSRAM_CS_HIGH() {GPIO_PortSet(GPIOD, 1u << 15);}  //呼吸等，GPIOA 10
#define PSRAM_CS_LOW() {GPIO_PortClear(GPIOD, 1u << 15);}

#define DisableIEPEPower() {GPIO_PortClear(GPIOC, 1u << 4);}  //IEPE,PE4使能
#define EnableIEPEPower() {GPIO_PortSet(GPIOC, 1u << 4);}

#define DisableBatteryChargePower() {GPIO_PortClear(GPIOD, 1u << 1);}  //IEPE,PC3使能
#define EnableBatteryChargePower() {GPIO_PortSet(GPIOD, 1u << 1);}

#endif
