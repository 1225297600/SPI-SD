#ifndef __DRV_SPI_SD_H
#define __DRV_SPI_SD_H
#include <stdint.h>
#include <stdbool.h>

#define 	SD_TRY_TIMES 		100

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
