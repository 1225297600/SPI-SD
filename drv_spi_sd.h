#ifndef __DRV_W25QXX_H
#define __DRV_W25QXX_H
#include <stdint.h>
#include <stdbool.h>

#define 	SD_TRY_TIMES 		100

typedef enum{
	SD_CMD0=0,			//restart
	SD_CMD1=1,			//mmc init
	SD_CMD8=8,			//status
	SD_CMD9=9,			//read data reg
	SD_CMD12=12,		//read more stop
	SD_CMD10=10,		//read sign reg
	SD_CMD16=16,		//set block Byte size
	SD_CMD17=17,		//read a block
	SD_CMD18=18,		//read more block
	SD_CMD23=23,		//erase a block
	SD_CMD24=24,		//write a block
	SD_CMD25=25,		//write more block
	SD_CMD41=41,		//messig & init
	SD_CMD55=55,		//next cmd
	SD_CMD58=58,		//read OCR reg
}SD_CMD;

typedef enum{
	SD_TYPE_INIT_FAIL=0,
	SD_TYPE_V1_SD,
	SD_TYPE_V1_MMC,
	SD_TYPE_V2_SD,
	SD_TYPE_V2_SDHC,
}SD_TYPE;

typedef struct{
	struct{
		SD_TYPE type;
		uint32_t sector_num;
		uint32_t sector_byte;
		bool ready;
	}status;
	
	struct{
		void (*SPI_TransmitReceive)(uint8_t* tx, uint8_t* rx, uint16_t len);
		void (*SPI_Speed)(uint32_t kbps);
		void (*SPI_Nss)(bool high);
	}hal;
}SPI_SD;


/**************** PORT ****************/

void DRV_SPI_SD_Init(SPI_SD *spi_sd);	

uint8_t DRV_SPI_SD_WriteMultiBlock(SPI_SD *spi_sd, uint32_t sector,uint8_t*buf,uint8_t cnt);
uint8_t DRV_SPI_SD_ReadMultiBlock(SPI_SD *spi_sd, uint32_t sector,uint8_t*buf,uint8_t cnt);

#endif
