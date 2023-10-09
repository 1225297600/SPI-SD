#include "drv_spi_sd.h"
#include <math.h>
#include <stdio.h>

/**********************************************************  **********************************************************/
static void DRV_SPI_SD_TransmitReceive(SPI_SD *spi_sd, uint8_t* tx, uint8_t* rx, uint16_t len);
static void DRV_SPI_SD_Speed(SPI_SD *spi_sd, uint32_t kbps);
static void DRV_SPI_SD_Cs(SPI_SD *spi_sd, bool high);
		
static uint8_t DRV_SPI_SD_Select(SPI_SD *spi_sd, bool select);
static uint8_t DRV_SPI_SD_SendByte(SPI_SD *spi_sd, uint8_t data);
static uint8_t DRV_SPI_SD_WaitReady(SPI_SD *spi_sd);
static uint8_t DRV_SPI_SD_SendCmd(SPI_SD *spi_sd, uint8_t cmd, uint32_t arg, uint8_t crc);

static uint8_t DRV_SPI_SD_SendBlock(SPI_SD *spi_sd, uint8_t*buf,uint8_t cmd);
static uint8_t DRV_SPI_SD_GetResponse(SPI_SD *spi_sd, uint8_t Response);
static uint8_t DRV_SPI_SD_RecvData(SPI_SD *spi_sd, uint8_t*buf,uint16_t len);
static uint8_t DRV_SPI_SD_GetCSD(SPI_SD *spi_sd, uint8_t *csd_data);
static uint32_t DRV_SPI_SD_GetSectorCount(SPI_SD *spi_sd);
static uint8_t DRV_SPI_SD_GetCID(SPI_SD *spi_sd, uint8_t *cid_data);

/**********************************************************  **********************************************************/
void DRV_SPI_SD_Init(SPI_SD *spi_sd){
	uint8_t r1=0;      	// 存放SD卡的返回值
  uint16_t retry;	// 用来进行超时计数
  uint8_t buf[4];
	uint16_t i;
	
	spi_sd->status.type = SD_TYPE_INIT_FAIL;
	
	DRV_SPI_SD_Select(spi_sd, true);
	DRV_SPI_SD_Speed(spi_sd, 250);	//配置为低速度模式
	
	for(i=0;i<10;i++)	//发送至少74个脉冲
	{
		DRV_SPI_SD_SendByte(spi_sd, 0xff);
	}
	retry=20;
	do
	{
		//进入IDLE状态
		r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD0,0,0x95);
	}	
	while((r1!=0X01) && (retry--));
	//默认无卡
	spi_sd->status.type=0;
	//识别卡类型
	if(r1==0X01)
	{
		//SD V2.0
		if(DRV_SPI_SD_SendCmd(spi_sd, SD_CMD8,0x1AA,0x87)==1)
		{
			//Get trailing return value of R7 resp
			for(i=0;i<4;i++)buf[i]=DRV_SPI_SD_SendByte(spi_sd, 0XFF);	
			//卡是否支持2.7~3.6V
			if(buf[2]==0X01&&buf[3]==0XAA)
			{
				retry=0XFFFE;
				do
				{
					//发送SD_CMD55
					DRV_SPI_SD_SendCmd(spi_sd, SD_CMD55,0,0X01);	
					//发送SD_CMD41
					r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD41,0x40000000,0X01);
				}
				while(r1&&retry--);
				//鉴别SD2.0卡版本开始
				if(retry&&DRV_SPI_SD_SendCmd(spi_sd, SD_CMD58,0,0X01)==0)
				{
					//得到OCR值
					for(i=0;i<4;i++)buf[i]=DRV_SPI_SD_SendByte(spi_sd, 0XFF);
					//检查CCS
					if(buf[0]&0x40)
					{
						spi_sd->status.type=SD_TYPE_V2_SDHC;   
					}						
					else
					{
						spi_sd->status.type=SD_TYPE_V2_SD; 
					}						
				}
			}
		}
	}
	//SD V1.x/ MMC	V3
	else
	{
		//发送SD_CMD55
		DRV_SPI_SD_SendCmd(spi_sd, SD_CMD55,0,0X01);		
		//发送SD_CMD41
		r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD41,0,0X01);	
		if(r1<=1)
		{		
			spi_sd->status.type=SD_TYPE_V1_SD;
			retry=0XFFFE;
			//等待退出IDLE模式
			do 
			{
				//发送SD_CMD55
				DRV_SPI_SD_SendCmd(spi_sd, SD_CMD55,0,0X01);	
				//发送SD_CMD41
				r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD41,0,0X01);
			}while(r1&&retry--);
		}
		//MMC卡不支持SD_CMD55+SD_CMD41识别
		else
		{
			//MMC V3
			spi_sd->status.type=SD_TYPE_V1_MMC;
			retry=0XFFFE;
			//等待退出IDLE模式
			do 
			{											    
				//发送SD_CMD1
				r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD1,0,0X01);
			}while(r1&&retry--);  
		}
		//错误的卡
		if(retry==0||DRV_SPI_SD_SendCmd(spi_sd, SD_CMD16,512,0X01)!=0)
		{
			spi_sd->status.type=SD_TYPE_INIT_FAIL;
		}
	}
	//取消片选
	DRV_SPI_SD_Select(spi_sd, false);
	//配置为高速度模式
	DRV_SPI_SD_Speed(spi_sd, 16*1024);	//配置为低速度模式
	if(spi_sd->status.type)
	{
		spi_sd->status.sector_num = DRV_SPI_SD_GetSectorCount(spi_sd);
		spi_sd->status.sector_byte = 512;
		spi_sd->status.ready = true;
	}
	else if(r1)
	{
		spi_sd->status.ready = false;
	}
	
}

//写SD卡
//buf:数据缓存区
//sector:起始扇区
//cnt:扇区数
//返回值:0,ok;其他,失败.
uint8_t DRV_SPI_SD_WriteMultiBlock(SPI_SD *spi_sd, uint32_t sector,uint8_t*buf,uint8_t cnt)
{
	uint8_t r1;
	//转换为字节地址
	if(spi_sd->status.type!=SD_TYPE_V2_SDHC)
	{
		sector *= 512;
	}
	if(cnt==1)
	{
		//读命令
		r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD24,sector,0X01);
		//指令发送成功
		if(r1==0)
		{
			//写512个字节	   
			r1=DRV_SPI_SD_SendBlock(spi_sd, buf,0xFE);
		}
	}
	else
	{
		if(spi_sd->status.type!=SD_TYPE_V1_MMC)
		{
			DRV_SPI_SD_SendCmd(spi_sd, SD_CMD55,0,0X01);	
			//发送指令	
			DRV_SPI_SD_SendCmd(spi_sd, SD_CMD23,cnt,0X01);
		}
		//连续读命令
 		r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD25,sector,0X01);
		if(r1==0)
		{
			do
			{
				//接收512个字节	 
				r1=DRV_SPI_SD_SendBlock(spi_sd, buf,0xFC);
				buf+=512;  
			}
			while(--cnt && r1==0);
			//接收512个字节 
			r1=DRV_SPI_SD_SendBlock(spi_sd, 0,0xFD);
		}
	}   
	//取消片选
	DRV_SPI_SD_Select(spi_sd, false);
	return r1;
}	


uint8_t DRV_SPI_SD_ReadMultiBlock(SPI_SD *spi_sd, uint32_t sector,uint8_t*buf,uint8_t cnt)
{
	uint8_t r1;
	//转换为字节地址
	if(spi_sd->status.type!=SD_TYPE_V2_SDHC)
	{
		sector <<= 9;
	}
	if(cnt==1)
	{
		//读命令
		r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD17,sector,0X01);
		//指令发送成功
		if(r1==0)
		{
			//接收512个字节	 
			r1=DRV_SPI_SD_RecvData(spi_sd, buf,512);  
		}
	}
	else
	{
		//连续读命令
		r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD18,sector,0X01);
		do
		{
			//接收512个字节	 
			r1=DRV_SPI_SD_RecvData(spi_sd, buf,512);
			buf+=512;  
		}
		while(--cnt && r1==0); 	
		//发送停止命令
		DRV_SPI_SD_SendCmd(spi_sd, SD_CMD12,0,0X01);	
	}   
	//取消片选
	DRV_SPI_SD_Select(spi_sd, false);
	return r1;
}


/**********************************************************  **********************************************************/
static void DRV_SPI_SD_TransmitReceive(SPI_SD *spi_sd, uint8_t* tx, uint8_t* rx, uint16_t len){
	if(spi_sd->hal.SPI_TransmitReceive != NULL){
		spi_sd->hal.SPI_TransmitReceive(tx, rx, len);
	}else{
		
	}
}
static void DRV_SPI_SD_Speed(SPI_SD *spi_sd, uint32_t kbps){
	if(spi_sd->hal.SPI_Speed != NULL){
		spi_sd->hal.SPI_Speed(kbps);
	}else{
		
	}
}
static void DRV_SPI_SD_Cs(SPI_SD *spi_sd, bool high){
	if(spi_sd->hal.SPI_Nss != NULL){
		spi_sd->hal.SPI_Nss(high);
	}else{
		
	}
}

static uint8_t DRV_SPI_SD_SendByte(SPI_SD *spi_sd, uint8_t data)
{
	uint8_t rx;
	DRV_SPI_SD_TransmitReceive(spi_sd, &data,&rx,1);
	return rx;
}

//等待卡准备好
static uint8_t DRV_SPI_SD_WaitReady(SPI_SD *spi_sd)
{
	uint32_t t=0;
	uint8_t reg;
	for(t=0;t<0xffff;t++)
	{
		reg=DRV_SPI_SD_SendByte(spi_sd, 0XFF);//获取返回值
		if(reg==0XFF)
			break; 	
	}
	if(t<0xffffff)
		return 0;
	else
		return 1;
}


//1.选择sd卡,并且等待卡准备OK
//2.取消选择,释放SPI总线
//返回值:0,成功;1,失败;
static uint8_t DRV_SPI_SD_Select(SPI_SD *spi_sd, bool select)
{
	if(select){
		DRV_SPI_SD_Cs(spi_sd, false);	//等待失败
		if(DRV_SPI_SD_WaitReady(spi_sd)==0){
			//等待成功
			return 0;
		}else{
			DRV_SPI_SD_Cs(spi_sd, true);	//等待失败
			return 1;
		}
	}else{
		DRV_SPI_SD_Cs(spi_sd, true);
		DRV_SPI_SD_SendByte(spi_sd, 0xff);//提供额外的8个时钟
		return true;
	}
}

//向SD卡发送一个命令
//输入: uint8_t cmd   命令 
//      uint32_t arg  命令参数
//      uint8_t crc   crc校验值	   
//返回值:SD卡返回的响应															  
static uint8_t DRV_SPI_SD_SendCmd(SPI_SD *spi_sd, uint8_t cmd, uint32_t arg, uint8_t crc)
{
  uint8_t r1=0;	
	uint8_t Retry=0; 
	DRV_SPI_SD_Select(spi_sd, false);	//取消上次片选
	
	if(DRV_SPI_SD_Select(spi_sd, true))	//片选失效 
	{
		return 0XFF;
	}
	//发送
	//分别写入命令
  DRV_SPI_SD_SendByte(spi_sd, cmd | 0x40);
  DRV_SPI_SD_SendByte(spi_sd, arg >> 24);
  DRV_SPI_SD_SendByte(spi_sd, arg >> 16);
  DRV_SPI_SD_SendByte(spi_sd, arg >> 8);
  DRV_SPI_SD_SendByte(spi_sd, arg);	  
  DRV_SPI_SD_SendByte(spi_sd, crc); 
	if(cmd==SD_CMD12)DRV_SPI_SD_SendByte(spi_sd, 0xff);
  //等待响应，或超时退出
	Retry=0X1F;
	do
	{
		r1=DRV_SPI_SD_SendByte(spi_sd, 0xFF);
	}
	while((r1&0X80) && Retry--);	 
	//返回状态值
  return r1;
}


//等待SD卡回应
//Response:要得到的回应值
//返回值:0,成功得到了该回应值
//    其他,得到回应值失败
static uint8_t DRV_SPI_SD_GetResponse(SPI_SD *spi_sd, uint8_t Response)
{
	//等待次数	 
	uint16_t Count=0xFFFF;  						  
	//等待得到准确的回应  	
	while ((DRV_SPI_SD_SendByte(spi_sd, 0XFF)!=Response)&&Count)
	{
		Count--;  
	}
	if (Count==0)
	{
		//得到回应失败 
		return 1;  
	}
	else
	{
		//正确回应
		return 0;
	}
}




//从sd卡读取一个数据包的内容
//buf:数据缓存区
//len:要读取的数据长度.
//返回值:0,成功;其他,失败;	
static uint8_t DRV_SPI_SD_RecvData(SPI_SD *spi_sd, uint8_t*buf,uint16_t len)
{			  	  
	//等待SD卡发回数据起始令牌0xFE
	if(DRV_SPI_SD_GetResponse(spi_sd, 0xFE))
	{
		return 1;
	}
	//开始接收数据
  while(len--)
  {
    *buf=DRV_SPI_SD_SendByte(spi_sd, 0xFF);
    buf++;
  }
  //下面是2个伪CRC（dummy CRC）
  DRV_SPI_SD_SendByte(spi_sd, 0xFF);
  DRV_SPI_SD_SendByte(spi_sd, 0xFF);		
  //读取成功							  					    
  return 0;
}




//获取SD卡的CSD信息，包括容量和速度信息
//输入:uint8_t *cid_data(存放CID的内存，至少16Byte）	    
//返回值:0：NO_ERR
//		 1：错误														   
static uint8_t DRV_SPI_SD_GetCSD(SPI_SD *spi_sd, uint8_t *csd_data)
{
  uint8_t r1;	 
	//发SD_CMD9命令，读CSD
  r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD9,0,0x01);
  if(r1==0)
	{
		//接收16个字节的数据 
    r1=DRV_SPI_SD_RecvData(spi_sd, csd_data, 16);
  }
	//取消片选
	DRV_SPI_SD_Select(spi_sd, false);
	if(r1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}  




//获取SD卡的总扇区数（扇区数）   
//返回值:0： 取容量出错 
//       其他:SD卡的容量(扇区数/512字节)
//每扇区的字节数必为512，因为如果不是512，则初始化不能通过.														  
static uint32_t DRV_SPI_SD_GetSectorCount(SPI_SD *spi_sd)
{
    uint8_t csd[16];
    uint32_t Capacity;  
    uint8_t n;
		uint16_t csize;  					    
		//取CSD信息，如果期间出错，返回0
    if(DRV_SPI_SD_GetCSD(spi_sd, csd)!=0)
		{
			return 0;	    
		}
    //如果为SDHC卡，按照下面方式计算
		//V2.00的卡
    if((csd[0]&0xC0)==0x40)	 
    {	
			csize = csd[9] + ((uint16_t)csd[8] << 8) + 1;
			//得到扇区数	
			Capacity = (uint32_t)csize << 10; 		   
    }
		//V1.XX的卡
		else
    {	
			n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
			csize = (csd[8] >> 6) + ((uint16_t)csd[7] << 2) + ((uint16_t)(csd[6] & 3) << 10) + 1;
			//得到扇区数 
			Capacity= (uint32_t)csize << (n - 9);  
    }
    return Capacity;
}





//获取SD卡的CID信息，包括制造商信息
//输入: uint8_t *cid_data(存放CID的内存，至少16Byte）	  
//返回值:0：NO_ERR
//		 1：错误														   
static uint8_t DRV_SPI_SD_GetCID(SPI_SD *spi_sd, uint8_t *cid_data)
{
  uint8_t r1;	   
  //发SD_CMD10命令，读CID
  r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD10,0,0x01);
  if(r1==0x00)
	{
		//接收16个字节的数据	 
		r1=DRV_SPI_SD_RecvData(spi_sd, cid_data,16);
  }
	//取消片选
	DRV_SPI_SD_Select(spi_sd, false);
	if(r1)
		return 1;
	else
		return 0;
}




//向sd卡写入一个数据包的内容 512字节
//buf:数据缓存区
//cmd:指令
//返回值:0,成功;其他,失败;	
static uint8_t DRV_SPI_SD_SendBlock(SPI_SD *spi_sd, uint8_t*buf,uint8_t cmd)
{	
	uint16_t t;		  	  
	//等待准备失效
	if(DRV_SPI_SD_WaitReady(spi_sd))
	{
		return 1;
	}
	DRV_SPI_SD_SendByte(spi_sd, cmd);
	//不是结束指令
	if(cmd!=0XFD)
	{
		//提高速度,减少函数传参时间
		for(t=0;t<512;t++)
		{
			DRV_SPI_SD_SendByte(spi_sd, buf[t]);
		}
		//忽略crc
	  DRV_SPI_SD_SendByte(spi_sd, 0xFF);
	  DRV_SPI_SD_SendByte(spi_sd, 0xFF);
		//接收响应
		t=DRV_SPI_SD_SendByte(spi_sd, 0xFF);
		if((t&0x1F)!=0x05)
		{
			//响应错误		
			return 2;		
		}			
	}						 		
	//写入成功							  					    
  return 0;
}


