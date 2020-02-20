/*
*********************************************************************************************************
*
*	模块名称 : AD7606数据采集模块
*	文件名称 : bsp_ad7606.c
*	版    本 : V1.0
*	说    明 : AD7606挂在STM32的FMC总线上。
*
*			本例子使用了 TIM3 作为硬件定时器，定时启动ADC转换
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2015-10-11 armfly  正式发布
*
*	Copyright (C), 2015-2020, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

/*
	STM32-V7开发板 + AD7606模块， 控制采集的IO:
	
	PC6/TIM3_CH1/TIM8_CH1     ----> AD7606_CONVST  (和摄像头复用),  输出PWM方波，作为ADC启动信号
	PE5/DCMI_D6/AD7606_BUSY   <---- AD7606_BUSY    , CPU在BUSY中断服务程序中读取采集结果
*/

#include "bsp.h"
#include "app.h"
#include "stm32h743xx.h"


//#define MASTER_BOARD
#define BUFFERSIZE 4096
/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;
static DMA_HandleTypeDef hdma_tx;
static DMA_HandleTypeDef hdma_rx;

volatile uint8_t aTxBuffer[BUFFERSIZE] __attribute__((at(0x30002000))); //放在SRAM1
volatile uint8_t aRxBuffer[BUFFERSIZE] __attribute__((at(0x30000000))); //放在SRAM1


#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA2_CLK_ENABLE()



#define SPIx_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

/* Definition for SPIx Pins */


/* Definition for SPIx's DMA */
#define SPIx_TX_DMA_STREAM               DMA2_Stream3
#define SPIx_RX_DMA_STREAM               DMA2_Stream2

#define SPIx_TX_DMA_REQUEST              DMA_REQUEST_SPI1_TX
#define SPIx_RX_DMA_REQUEST              DMA_REQUEST_SPI1_RX

/* Definition for SPIx's NVIC */
#define SPIx_DMA_TX_IRQn                 DMA2_Stream3_IRQn
#define SPIx_DMA_RX_IRQn                 DMA2_Stream2_IRQn

#define SPIx_DMA_TX_IRQHandler           DMA2_Stream3_IRQHandler
#define SPIx_DMA_RX_IRQHandler           DMA2_Stream2_IRQHandler

#define SPIx_IRQn                        SPI1_IRQn
#define SPIx_IRQHandler                  SPI1_IRQHandler


#ifdef SLAVE_BOARD
/* 采集板采集的数据准备好后拉这个脚给主机，准备接受 */

#define SPIx_CS_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define SPIx_CS_PIN                     GPIO_PIN_4
#define SPIx_CS_GPIO_PORT               GPIOA
#define SPIx_CS_AF                      GPIO_AF5_SPI1

#define SPIx_SCK_PIN                     GPIO_PIN_5
#define SPIx_SCK_GPIO_PORT               GPIOA
#define SPIx_SCK_AF                      GPIO_AF5_SPI1

#define SPIx_MISO_PIN                    GPIO_PIN_6
#define SPIx_MISO_GPIO_PORT              GPIOA
#define SPIx_MISO_AF                     GPIO_AF5_SPI1

#define SPIx_MOSI_PIN                    GPIO_PIN_7   
#define SPIx_MOSI_GPIO_PORT              GPIOA
#define SPIx_MOSI_AF                     GPIO_AF5_SPI1

#define DATA_READY_GPIO_CLK_ENABLE	__HAL_RCC_GPIOC_CLK_ENABLE
#define DATA_READY_GPIO		GPIOC
#define DATA_READY_PIN		GPIO_PIN_11

#define DATA_READY_1()	DATA_READY_GPIO->BSRR=DATA_READY_PIN
#define DATA_READY_0()	DATA_READY_GPIO->BSRR=((uint32_t)DATA_READY_PIN<<16)

static void BTB_SPIConfig(void);
static void BTB_CtrlLinesConfig(void);
enum {
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR
};

enum {
  board1_curt_read,
  board2_curt_read,
  idle_curt_read
};

__IO uint32_t wTransferState=TRANSFER_COMPLETE;
/*
*********************************************************************************************************
*	函 数 名: bsp_InitAD7606
*	功能说明: 配置连接外部SRAM的GPIO和FSMC
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitBTB_SPI(void)
{
	for(uint32_t i=0;i<4096;i++)
	aTxBuffer[i]=i;
	BTB_CtrlLinesConfig();
	BTB_SPIConfig();
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_SPIConfig
*	功能说明: 配置FSMC并口访问时序
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/

static void BTB_CtrlLinesConfig(void)
{
	

	GPIO_InitTypeDef gpio_init_structure;

	DATA_READY_GPIO_CLK_ENABLE();
	/* 设置 GPIOD 相关的IO为复用推挽输出 */
	gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	
	gpio_init_structure.Pin = DATA_READY_PIN;
	HAL_GPIO_Init(DATA_READY_GPIO, &gpio_init_structure);
	DATA_READY_1();

	
}


void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  if (hspi->Instance == SPIx)
  {
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
		SPIx_CS_GPIO_CLK_ENABLE();
    SPIx_SCK_GPIO_CLK_ENABLE();
    SPIx_MISO_GPIO_CLK_ENABLE();
    SPIx_MOSI_GPIO_CLK_ENABLE();
    /* Enable SPI1 clock */
    SPIx_CLK_ENABLE();
    /* Enable DMA clock */
    DMAx_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* SPI SCK GPIO pin configuration  */
    GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = SPIx_SCK_AF;
    HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

		/* SPI CS GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPIx_CS_PIN;
    GPIO_InitStruct.Alternate = SPIx_CS_AF;
    HAL_GPIO_Init(SPIx_CS_GPIO_PORT, &GPIO_InitStruct);
		
    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPIx_MISO_PIN;
    GPIO_InitStruct.Alternate = SPIx_MISO_AF;
    HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
    GPIO_InitStruct.Alternate = SPIx_MOSI_AF;
    HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);

    /*##-3- Configure the DMA ##################################################*/
    /*##-3- Configure the DMA ##################################################*/
    /* Configure the DMA handler for Transmission process */
    hdma_tx.Instance                 = SPIx_TX_DMA_STREAM;
    hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;
    hdma_tx.Init.Request             = SPIx_TX_DMA_REQUEST;
    hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode                = DMA_NORMAL;
    hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;

    HAL_DMA_Init(&hdma_tx);

    /* Associate the initialized DMA handle to the the SPI handle */
    __HAL_LINKDMA(hspi, hdmatx, hdma_tx);

    /* Configure the DMA handler for Transmission process */
    hdma_rx.Instance                 = SPIx_RX_DMA_STREAM;

    hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;
    hdma_rx.Init.Request             = SPIx_RX_DMA_REQUEST;
    hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode                = DMA_NORMAL;
    hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&hdma_rx);

    /* Associate the initialized DMA handle to the the SPI handle */
    __HAL_LINKDMA(hspi, hdmarx, hdma_rx);
    
    /*##-4- Configure the NVIC for DMA #########################################*/ 
    /* NVIC configuration for DMA transfer complete interrupt (SPI1_TX) */
    HAL_NVIC_SetPriority(SPIx_DMA_TX_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(SPIx_DMA_TX_IRQn);
    
    /* NVIC configuration for DMA transfer complete interrupt (SPI1_RX) */
    HAL_NVIC_SetPriority(SPIx_DMA_RX_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(SPIx_DMA_RX_IRQn);

    /*##-5- Configure the NVIC for SPI #########################################*/ 
    /* NVIC configuration for SPI transfer complete interrupt (SPI1) */
    HAL_NVIC_SetPriority(SPIx_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(SPIx_IRQn);
  }
}

static void BTB_SPIConfig(void)
{
 
  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
	SpiHandle.Instance               = SPIx;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  SpiHandle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE; 

	SpiHandle.Init.Mode = SPI_MODE_SLAVE;

  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
//    Error_Handler();
  }	
}


/*
*********************************************************************************************************
*	函 数 名: AD7606_ReadNowAdc
*	功能说明: 读取8路采样结果。结果存储在全局变量 g_tAD7606
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint32_t test1=0;
uint32_t test2=0;
void SLAVE_DATA_Ready_RX(void)
{
	
	if(wTransferState==TRANSFER_ERROR)  //如果传输错误
	{
//		Error_Handler();
	}
	SCB_InvalidateDCache_by_Addr ((uint32_t *)aTxBuffer, BUFFERSIZE);
	SCB_CleanDCache_by_Addr ((uint32_t *)aRxBuffer, BUFFERSIZE);
	if(HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE) != HAL_OK)
  {
    /* Transfer error in transmission process */
//    Error_Handler();
  }
	DATA_READY_0();
}

void SLAVE_DATA_Ready_RX_ADDR(int16_t * data_add)
{
	
	if(wTransferState==TRANSFER_ERROR)  //如果传输错误
	{
//		Error_Handler();
	}
	SCB_CleanDCache_by_Addr ((uint32_t *)aRxBuffer, BUFFERSIZE);
	if(HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*)data_add, (uint8_t *)aRxBuffer, BUFFERSIZE) != HAL_OK)
  {
    /* Transfer error in transmission process */
//    Error_Handler();
  }
	DATA_READY_0();
}

void SLAVE_DATA_Ready_RX_test(void)
{
	DATA_READY_1();
	
}

void SPIx_DMA_RX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(SpiHandle.hdmarx);
}

/**
  * @brief  This function handles DMA Tx interrupt request.
  * @param  None
  * @retval None
  */
void SPIx_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(SpiHandle.hdmatx);
}

/**
  * @brief  This function handles SPIx interrupt request.
  * @param  None
  * @retval None
  */
void SPIx_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&SpiHandle);
}
//__attribute__((section (".RAM_D1"))) int16_t Ad7606_Data[AD7606SAMPLEPOINTS*AD7606_ADCHS*2];
uint8_t board_all_read=1; //这个应该由前期can总线问询有几块板子决定，比如有两块则是B 11=3 只有第一块则是B 01=1 只有第二块则是B10=2
uint8_t rxxx_bf[12];
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	int16_t y;
	test2++;
	DATA_READY_1();
	SCB_InvalidateDCache_by_Addr ((uint32_t *)aRxBuffer, BUFFERSIZE);

	for(uint16_t i=0;i<12;i++)
	{
		rxxx_bf[i]=aRxBuffer[i];
	}
  wTransferState = TRANSFER_COMPLETE;
}


/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  wTransferState = TRANSFER_ERROR;
}

#endif

#ifdef MASTER_BOARD

#define SPIx_CS_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()


#define SPIx_SCK_PIN                     GPIO_PIN_3
#define SPIx_SCK_GPIO_PORT               GPIOB
#define SPIx_SCK_AF                      GPIO_AF5_SPI1

#define SPIx_MISO_PIN                    GPIO_PIN_4
#define SPIx_MISO_GPIO_PORT              GPIOB
#define SPIx_MISO_AF                     GPIO_AF5_SPI1

#define SPIx_MOSI_PIN                    GPIO_PIN_5   
#define SPIx_MOSI_GPIO_PORT              GPIOB
#define SPIx_MOSI_AF                     GPIO_AF5_SPI1

/* 采集板采集的数据准备好后拉这个脚给主机，准备接受 */
#define BOARD1_DATA_READY_GPIO_CLK_ENABLE	__HAL_RCC_GPIOJ_CLK_ENABLE
#define BOARD1_DATA_READY_GPIO		GPIOJ
#define BOARD1_DATA_READY_PIN		GPIO_PIN_7
#define BOARD1_DATA_READY_IRQn		EXTI9_5_IRQn
#define BOARD1_DATA_READY_IRQHandler	EXTI9_5_IRQHandler

#define BOARD1_CS_GPIO_CLK_ENABLE	__HAL_RCC_GPIOA_CLK_ENABLE
#define BOARD1_CS_GPIO		GPIOA
#define BOARD1_CS_PIN		GPIO_PIN_15

#define BOARD1_CS_1()	BOARD1_CS_GPIO->BSRR=BOARD1_CS_PIN
#define BOARD1_CS_0()	BOARD1_CS_GPIO->BSRR=((uint32_t)BOARD1_CS_PIN<<16)

#define BOARD2_DATA_READY_GPIO_CLK_ENABLE	__HAL_RCC_GPIOB_CLK_ENABLE
#define BOARD2_DATA_READY_GPIO		GPIOB
#define BOARD2_DATA_READY_PIN		GPIO_PIN_10
#define BOARD2_DATA_READY_IRQn		EXTI15_10_IRQn
#define BOARD2_DATA_READY_IRQHandler	EXTI15_10_IRQHandler

#define BOARD2_CS_GPIO_CLK_ENABLE	__HAL_RCC_GPIOB_CLK_ENABLE
#define BOARD2_CS_GPIO		GPIOB
#define BOARD2_CS_PIN		GPIO_PIN_14
#define BOARD2_CS_1()	BOARD2_CS_GPIO->BSRR=BOARD2_CS_PIN
#define BOARD2_CS_0()	BOARD2_CS_GPIO->BSRR=((uint32_t)BOARD2_CS_PIN<<16)

static void BTB_SPIConfig(void);
static void BTB_CtrlLinesConfig(void);
enum {
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR
};

enum {
  board1_curt_read,
  board2_curt_read,
  idle_curt_read
};

__IO uint32_t wTransferState=TRANSFER_COMPLETE;
/*
*********************************************************************************************************
*	函 数 名: bsp_InitAD7606
*	功能说明: 配置连接外部SRAM的GPIO和FSMC
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitBTB_SPI(void)
{
	for(uint32_t i=0;i<4096;i++)
	aTxBuffer[i]=i;
	
	BTB_CtrlLinesConfig();
	BTB_SPIConfig();
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_SPIConfig
*	功能说明: 配置FSMC并口访问时序
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/

static void BTB_CtrlLinesConfig(void)
{
	

	GPIO_InitTypeDef gpio_init_structure;

	BOARD1_DATA_READY_GPIO_CLK_ENABLE();
	BOARD2_DATA_READY_GPIO_CLK_ENABLE();
	
	
	/* 设置 GPIOD 相关的IO为复用推挽输出 */
	gpio_init_structure.Mode = GPIO_MODE_IT_FALLING;	/* 中断下降沿触发 */
	gpio_init_structure.Pull = GPIO_NOPULL;
	gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;		
	
	gpio_init_structure.Pin = BOARD1_DATA_READY_PIN;
	HAL_GPIO_Init(BOARD1_DATA_READY_GPIO, &gpio_init_structure);
	
	gpio_init_structure.Pin = BOARD2_DATA_READY_PIN;
	HAL_GPIO_Init(BOARD2_DATA_READY_GPIO, &gpio_init_structure);
	
	HAL_NVIC_SetPriority(BOARD1_DATA_READY_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(BOARD1_DATA_READY_IRQn);		
	
	HAL_NVIC_SetPriority(BOARD2_DATA_READY_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(BOARD2_DATA_READY_IRQn);	
	
	BOARD1_CS_GPIO_CLK_ENABLE();
	BOARD2_CS_GPIO_CLK_ENABLE();
	
	gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	
	gpio_init_structure.Pin = BOARD1_CS_PIN;
	HAL_GPIO_Init(BOARD1_CS_GPIO, &gpio_init_structure);
	
	gpio_init_structure.Pin = BOARD2_CS_PIN;
	HAL_GPIO_Init(BOARD2_CS_GPIO, &gpio_init_structure);
	
	BOARD1_CS_1();
	BOARD2_CS_1();
	

	
	
}


void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  if (hspi->Instance == SPIx)
  {
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    SPIx_SCK_GPIO_CLK_ENABLE();
    SPIx_MISO_GPIO_CLK_ENABLE();
    SPIx_MOSI_GPIO_CLK_ENABLE();
    /* Enable SPI1 clock */
    SPIx_CLK_ENABLE();
    /* Enable DMA clock */
    DMAx_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* SPI SCK GPIO pin configuration  */
    GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = SPIx_SCK_AF;
    HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

		
    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPIx_MISO_PIN;
    GPIO_InitStruct.Alternate = SPIx_MISO_AF;
    HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
    GPIO_InitStruct.Alternate = SPIx_MOSI_AF;
    HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);

    /*##-3- Configure the DMA ##################################################*/
    /*##-3- Configure the DMA ##################################################*/
    /* Configure the DMA handler for Transmission process */
    hdma_tx.Instance                 = SPIx_TX_DMA_STREAM;
    hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;
    hdma_tx.Init.Request             = SPIx_TX_DMA_REQUEST;
    hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode                = DMA_NORMAL;
    hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;

    HAL_DMA_Init(&hdma_tx);

    /* Associate the initialized DMA handle to the the SPI handle */
    __HAL_LINKDMA(hspi, hdmatx, hdma_tx);

    /* Configure the DMA handler for Transmission process */
    hdma_rx.Instance                 = SPIx_RX_DMA_STREAM;

    hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;
    hdma_rx.Init.Request             = SPIx_RX_DMA_REQUEST;
    hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode                = DMA_NORMAL;
    hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&hdma_rx);

    /* Associate the initialized DMA handle to the the SPI handle */
    __HAL_LINKDMA(hspi, hdmarx, hdma_rx);
    
    /*##-4- Configure the NVIC for DMA #########################################*/ 
    /* NVIC configuration for DMA transfer complete interrupt (SPI1_TX) */
    HAL_NVIC_SetPriority(SPIx_DMA_TX_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(SPIx_DMA_TX_IRQn);
    
    /* NVIC configuration for DMA transfer complete interrupt (SPI1_RX) */
    HAL_NVIC_SetPriority(SPIx_DMA_RX_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(SPIx_DMA_RX_IRQn);

    /*##-5- Configure the NVIC for SPI #########################################*/ 
    /* NVIC configuration for SPI transfer complete interrupt (SPI1) */
    HAL_NVIC_SetPriority(SPIx_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(SPIx_IRQn);
  }
}

static void BTB_SPIConfig(void)
{
 
  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
	SpiHandle.Instance               = SPIx;
  SpiHandle.Init.BaudRatePrescaler =SPI_BAUDRATEPRESCALER_4;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  SpiHandle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE; 
  SpiHandle.Init.IOSwap=SPI_IO_SWAP_ENABLE;  //引脚画错了，交换下
  SpiHandle.Init.Mode = SPI_MODE_MASTER;
//  SpiHandle.Init.MasterInterDataIdleness=SPI_MASTER_INTERDATA_IDLENESS_15CYCLE;

  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
//    Error_Handler();
  }	
}


/*
*********************************************************************************************************
*	函 数 名: AD7606_ReadNowAdc
*	功能说明: 读取8路采样结果。结果存储在全局变量 g_tAD7606
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*	函 数 名: EXTI9_5_IRQHandler
*	功能说明: 外部中断服务程序入口。PE5 / AD7606_BUSY 下降沿中断触发
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/

void BOARD1_DATA_READY_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(BOARD1_DATA_READY_PIN);
}

void BOARD2_DATA_READY_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(BOARD2_DATA_READY_PIN);
}

/*
*********************************************************************************************************
*	函 数 名: EXTI15_10_IRQHandler
*	功能说明: 外部中断服务程序入口。 AD7606_BUSY 下降沿中断触发
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
volatile uint8_t current_read_statue=0;
uint32_t test1=0;
uint32_t test2=0;
uint32_t test3=0;
uint32_t test4=0;
void master_test(void)
{
	if(HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE) != HAL_OK)
		{
			/* Transfer error in transmission process */
		//    Error_Handler();
		}
	
}

extern SemaphoreHandle_t SPI_sem_req;
extern uint8_t spi_data_come[SLAVE_MACHINE_NUM];
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (GPIO_Pin == BOARD1_DATA_READY_PIN)
	{
		test1++;
		BOARD1_CS_0(); //拉低片选信号
		spi_data_come[0] = 1;
		xSemaphoreGiveFromISR( SPI_sem_req, &xHigherPriorityTaskWoken );  
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		current_read_statue|=1; //第一个板子就置第一位就可以
	}else if (GPIO_Pin == BOARD2_DATA_READY_PIN)
	{
		test2++;
		BOARD2_CS_0(); //拉低片选信号
		spi_data_come[1] = 1;
		xSemaphoreGiveFromISR( SPI_sem_req, &xHigherPriorityTaskWoken );  
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		current_read_statue|=2; //第二个板子就置第二位就可以
	}
}

void SPIx_DMA_RX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(SpiHandle.hdmarx);
}

/**
  * @brief  This function handles DMA Tx interrupt request.
  * @param  None
  * @retval None
  */
void SPIx_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(SpiHandle.hdmatx);
}

/**
  * @brief  This function handles SPIx interrupt request.
  * @param  None
  * @retval None
  */
void SPIx_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&SpiHandle);
}
uint8_t rxxx_bf[12];

__attribute__((section (".RAM_D1"))) int16_t Ad7606_Data[SLAVE_MACHINE_NUM][AD7606SAMPLEPOINTS*AD7606_ADCHS*2];
uint8_t board_all_read=1; //这个应该由前期can总线问询有几块板子决定，比如有两块则是B 11=3 只有第一块则是B 01=1 只有第二块则是B10=2

extern SemaphoreHandle_t sem_spi_idle;
uint32_t AD7606DataCounter=0;
extern  SemaphoreHandle_t AD7606_ready;
uint32_t CurrentAD7606DataCounter=0;
extern int current_rx_machine;
volatile bool flag_board_all_read=false;
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	int16_t y;
	uint32_t j = 0;
	uint32_t i = 0;
	short aa_0,aa_1,aa_0_1;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	SCB_InvalidateDCache_by_Addr ((uint32_t *)aRxBuffer,2*BUFFERSIZE);
	memcpy(&Ad7606_Data[current_rx_machine][AD7606DataCounter],(uint32_t *)aRxBuffer,2*BUFFERSIZE);
	if(current_rx_machine==0)//当前是第一块
	{
		BOARD1_CS_1();		
	}
	else if(current_rx_machine==1)//当前是第二块
	{
		BOARD2_CS_1();
	}
	//判定几块板子都上传了，并且达到一定数据量//总共几块板子current_read_statue，从can问询得到
	if(current_read_statue==board_all_read )
	{
		flag_board_all_read=true; //这个标志位用来释放另一个数据号的信号量
		current_read_statue = 0;		
		if(AD7606DataCounter==0)
		{
			CurrentAD7606DataCounter=BUFFERSIZE;
			AD7606DataCounter=0;
		}else if(AD7606DataCounter==BUFFERSIZE)
		{
			CurrentAD7606DataCounter=0;
			AD7606DataCounter=BUFFERSIZE;
		}
	}
	xSemaphoreGiveFromISR( sem_spi_idle, &xHigherPriorityTaskWoken );  
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	current_rx_machine = -1;
  wTransferState = TRANSFER_COMPLETE;
}



/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  wTransferState = TRANSFER_ERROR;
}

#endif
/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
