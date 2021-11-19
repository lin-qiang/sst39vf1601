#ifndef _SST39VF1601_H_
#define _SST39VF1601_H_

#include "stm32f4xx.h"

#define u8 unsigned char
#define u16 unsigned short int  
#define u32 unsigned int
	
/* Exported types ------------------------------------------------------------*/
typedef struct
{
  u16 Manufacturer_Code;
  u16 Device_Code1;
  u16 Device_Code2;
  u16 Device_Code3;
}NOR_IDTypeDef;
extern NOR_IDTypeDef NOR_ID;

/* NOR Status */
typedef enum
{
  NOR_SUCCESS = 0,
  NOR_ONGOING,
  NOR_ERROR,
  NOR_TIMEOUT
}NOR_Status;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void sst39vf1601_nor_Init(void);
void FSMC_NOR_ReadID(NOR_IDTypeDef* NOR_ID);
NOR_Status FSMC_NOR_EraseBlock(u32 BlockAddr);
NOR_Status FSMC_NOR_EraseChip(void);
NOR_Status FSMC_NOR_WriteHalfWord(u32 WriteAddr, u16 Data);
NOR_Status FSMC_NOR_WriteBuffer(u16* pBuffer, u32 WriteAddr, u32 NumHalfwordToWrite);
NOR_Status FSMC_NOR_ProgramBuffer(u16* pBuffer, u32 WriteAddr, u32 NumHalfwordToWrite);
u16 FSMC_NOR_ReadHalfWord(u32 ReadAddr);
void FSMC_NOR_ReadBuffer(u16* pBuffer, u32 ReadAddr, u32 NumHalfwordToRead);
NOR_Status FSMC_NOR_ReturnToReadMode(void);
NOR_Status FSMC_NOR_Reset(void);
NOR_Status FSMC_NOR_GetStatus(u32 Timeout);





#endif

