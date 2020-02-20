/*
*********************************************************************************************************
*
*	模块名称 : AD7606数据采集模块
*	文件名称 : bsp_ad7606.h
*	版    本 : V1.0
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_SPI_TXRX_H
#define _BSP_SPI_TXRX_H


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

void bsp_InitBTB_SPI(void);
void SLAVE_DATA_Ready_RX_ADDR(int16_t * data_add);
void SLAVE_DATA_Ready_RX(void);
void SLAVE_DATA_Ready_RX_test(void);
void master_test(void);
#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
