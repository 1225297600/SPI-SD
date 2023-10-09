#include "drv_spi_sd.h"

SPI_SD sd;

void SD_Init(void){
	file_obj.sd.hal.SPI_Nss = BOARD_SPI2_Nss;
	file_obj.sd.hal.SPI_Speed = BOARD_SPI2_Speed;
	file_obj.sd.hal.SPI_TransmitReceive = BOARD_SPI2_TransmitRecieve;
	DRV_SPI_SD_Init(&file_obj.sd);
}
