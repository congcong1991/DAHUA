#include "stm32h7_flash.h"
#include "app.h"
//////////////////////////////////////////////////////////////////////////////////	 


//#define EEPROM_START  0X08020000 

/*************************************************************************/
/*************************本代码仅仅实现了BANK1****************************/
/*************************************************************************/


//读取指定地址的字(32位数据) 
//faddr:读地址 
//返回值:对应数据.
uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
	return *(__IO uint32_t *)faddr; 
}

//获取某个地址所在的flash扇区,仅用于BANK1！！
//addr:flash地址
//返回值:0~11,即addr所在的扇区
uint16_t STMFLASH_GetFlashSector(uint32_t addr)
{
		if(addr<ADDR_FLASH_SECTOR_1_BANK1)return 0;
	else if(addr<ADDR_FLASH_SECTOR_2_BANK1)return 1;
	else if(addr<ADDR_FLASH_SECTOR_3_BANK1)return 2;
	else if(addr<ADDR_FLASH_SECTOR_4_BANK1)return 3;
	else if(addr<ADDR_FLASH_SECTOR_5_BANK1)return 4;
	else if(addr<ADDR_FLASH_SECTOR_6_BANK1)return 5;
	else if(addr<ADDR_FLASH_SECTOR_7_BANK1)return 6; 
	else if(addr<ADDR_FLASH_SECTOR_0_BANK2)return 7;
	else if(addr<ADDR_FLASH_SECTOR_1_BANK2)return 8;
	else if(addr<ADDR_FLASH_SECTOR_2_BANK2)return 9;
	else if(addr<ADDR_FLASH_SECTOR_3_BANK2)return 10; 
	else if(addr<ADDR_FLASH_SECTOR_4_BANK2)return 11;
	else if(addr<ADDR_FLASH_SECTOR_5_BANK2)return 12;
	else if(addr<ADDR_FLASH_SECTOR_6_BANK2)return 13; 
	else if(addr<ADDR_FLASH_SECTOR_7_BANK2)return 14;    
	return 15;	
//	if(addr<ADDR_FLASH_SECTOR_1_BANK1)return FLASH_SECTOR_0;
//	else if(addr<ADDR_FLASH_SECTOR_2_BANK1)return FLASH_SECTOR_1;
//	else if(addr<ADDR_FLASH_SECTOR_3_BANK1)return FLASH_SECTOR_2;
//	else if(addr<ADDR_FLASH_SECTOR_4_BANK1)return FLASH_SECTOR_3;
//	else if(addr<ADDR_FLASH_SECTOR_5_BANK1)return FLASH_SECTOR_4;
//	else if(addr<ADDR_FLASH_SECTOR_6_BANK1)return FLASH_SECTOR_5;
//	else if(addr<ADDR_FLASH_SECTOR_7_BANK1)return FLASH_SECTOR_6;
//	return FLASH_SECTOR_7;	
}

//从指定地址开始写入指定长度的数据
//特别注意:因为STM32H7的扇区实在太大,没办法本地保存扇区数据,所以本函数
//         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
//         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
//         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
//该函数对OTP区域也有效!可以用来写OTP区!
//OTP区域地址范围:0X1FF0F000~0X1FF0F41F
//WriteAddr:起始地址(此地址必须为4的倍数!!)
//pBuffer:数据指针
//NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)	
{ 
    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus=HAL_OK;
    uint32_t SectorError=0;
	uint32_t addrx=0;
	uint32_t endaddr=0;	
    if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
    
 	HAL_FLASH_Unlock();             //解锁	
	addrx=WriteAddr;				//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
    
    if(addrx<0X1FF00000)
    {
        while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS;       //擦除类型，扇区擦除 
				FlashEraseInit.Sector=STMFLASH_GetFlashSector(addrx);   //要擦除的扇区
				FlashEraseInit.Banks=FLASH_BANK_2;						//操作BANK1
                FlashEraseInit.NbSectors=1;                             //一次只擦除一个扇区
                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;      //电压范围，VCC=2.7~3.6V之间!!
                if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) 
                {
                    break;//发生错误了	
                }
                SCB_CleanInvalidateDCache();                            //清除无效的D-Cache
			}else addrx+=4;
            FLASH_WaitForLastOperation(FLASH_WAITETIME,FLASH_BANK_2);    //等待上次操作完成
        }
    }
    FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME,FLASH_BANK_2);       //等待上次操作完成
	if(FlashStatus==HAL_OK)
	{
		while(WriteAddr<endaddr)//写数据
		{
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,WriteAddr,(uint64_t)pBuffer)!=HAL_OK)//写入数据
			{ 
				break;	//写入异常
			}
			WriteAddr+=32;
			pBuffer+=8;
		} 
	}
	HAL_FLASH_Lock();           //上锁
} 
//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToRead:字(32位)数
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)   	
{
	uint32_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
	}
}



void EarsePage(uint32_t address)  //
{

}


void EarseFlash(uint32_t address,uint32_t length)  //
{

}

void RDBYTES(uint32_t address,uint32_t num, uint8_t *Data)
{
	uint32_t count;
	for(count=0;count<num/2;count++)
	{
		*((uint16_t *)Data+count)=*((uint16_t *)address+count);
  }
}

uint32_t temp_num = 0;
void WRBYTES(uint32_t address,uint32_t num, uint8_t *Data)
{
	num=(num+3)&0xfffffffc;
//	temp_num = num;
//		__disable_irq();
	STMFLASH_Write(address,(uint32_t*)Data,num);
//		__enable_irq();
}		

void readProtect(void){
	uint32_t t=0xfffffBf4;
	uint32_t rt=*((uint32_t*)(unsigned char*)(0x040c));
	if((rt &0x0f )!=4)	WRBYTES(0x040C,4,(unsigned char*)&t);
}

uint32_t llength;
void saveConfig()
{
	llength=sizeof(struct CONFIG);
	EarsePage(EEPROM_START);
//		__disable_irq();
//	STMFLASH_Write(EEPROM_START,(uint32_t*)&config,sizeof(struct CONFIG));
//		__enable_irq();
	WRBYTES(EEPROM_START,sizeof(struct CONFIG),(uint8_t*)&config);
}


void loadConfig(void)
{
//	STMFLASH_Read(EEPROM_START,(uint32_t*)&config,sizeof(config.vaildsign));
	RDBYTES(EEPROM_START,sizeof(config.vaildsign),(unsigned char*)&config);
	if(config.vaildsign!=0x45aa)
	{	
		config.vaildsign=0x45aa;
		saveConfig();
		return;
	}
//	STMFLASH_Read(EEPROM_START,(uint32_t*)&config,sizeof(config.vaildsign));
	RDBYTES(EEPROM_START,sizeof(struct CONFIG),((unsigned char*)&config));
}

void initEEPROM()
{

	loadConfig();

}
