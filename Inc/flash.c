#include "flash.h"
#include "main.h"

typedef volatile uint8_t  vu8;
uint16_t BSP_FLASH_GetFlashSector(uint32_t addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_SECTOR_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_SECTOR_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_SECTOR_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_SECTOR_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_SECTOR_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_SECTOR_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_SECTOR_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_SECTOR_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_SECTOR_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_SECTOR_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_SECTOR_10; 
	return FLASH_SECTOR_11;	
}





uint8_t BSP_FLASH_Write(uint32_t WriteAddr, uint8_t *pBuffer, uint32_t ByteToWrite)	
{ 
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t res=1;
    uint32_t addrx=0;
    uint32_t endaddr=0;	
		int i = 0;
    
    uint32_t start_sector = 0;
    uint32_t end_sector = 0;
    
    if(WriteAddr<STM32_FLASH_BASE)return 0;	//非法地址
		HAL_FLASH_Unlock();									//解锁 
    __HAL_FLASH_DATA_CACHE_DISABLE();//FLASH擦除期间,必须禁止数据缓存
 		
		addrx=WriteAddr;				//写入的起始地址
		endaddr=WriteAddr+ByteToWrite;	//写入的结束地址
    
    start_sector = BSP_FLASH_GetFlashSector(addrx);
    end_sector = BSP_FLASH_GetFlashSector(endaddr);
    
		if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
		{             
				 for(i = start_sector; i <= end_sector; i += 8)
				 {
						FLASH_Erase_Sector(i, FLASH_VOLTAGE_RANGE_3);
						 if(status!=HAL_OK)
						 {
								 res = 0;	//发生错误了
								 break;
						 }           
				 }           
		}
			
		if(status == HAL_OK)
		{
			while(WriteAddr < endaddr)//写数据
			{
				if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, WriteAddr,*pBuffer) != HAL_OK)//写入数据
				{ 
					res = 0;	//写入异常
									break;
				}
				WriteAddr+=1;
				pBuffer = (uint8_t*)pBuffer+1;
			} 
		}
   
    __HAL_FLASH_DATA_CACHE_ENABLE();	//FLASH擦除结束,开启数据缓存
    HAL_FLASH_Lock();//上锁
    return res;
} 






static uint8_t BSP_FLASH_ReadByte(uint32_t faddr)
{
	return *(vu8*)faddr; 
} 



void BSP_FLASH_Read(uint32_t ReadAddr, uint8_t *pBuffer, uint32_t ByteToRead)   	
{
	uint32_t i;
//    uint32_t NumToRead = ((ByteToRead+3u)&(~3u))/4u;
	for(i=0;i<ByteToRead;i++)
	{
		pBuffer[i]=BSP_FLASH_ReadByte(ReadAddr);//读取1个字节.
		ReadAddr+=1;//偏移1个字节.	
	}
}




