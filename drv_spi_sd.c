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
	uint8_t r1=0;      	// ���SD���ķ���ֵ
  uint16_t retry;	// �������г�ʱ����
  uint8_t buf[4];
	uint16_t i;
	
	spi_sd->status.type = SD_TYPE_INIT_FAIL;
	
	DRV_SPI_SD_Select(spi_sd, true);
	DRV_SPI_SD_Speed(spi_sd, 250);	//����Ϊ���ٶ�ģʽ
	
	for(i=0;i<10;i++)	//��������74������
	{
		DRV_SPI_SD_SendByte(spi_sd, 0xff);
	}
	retry=20;
	do
	{
		//����IDLE״̬
		r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD0,0,0x95);
	}	
	while((r1!=0X01) && (retry--));
	//Ĭ���޿�
	spi_sd->status.type=0;
	//ʶ������
	if(r1==0X01)
	{
		//SD V2.0
		if(DRV_SPI_SD_SendCmd(spi_sd, SD_CMD8,0x1AA,0x87)==1)
		{
			//Get trailing return value of R7 resp
			for(i=0;i<4;i++)buf[i]=DRV_SPI_SD_SendByte(spi_sd, 0XFF);	
			//���Ƿ�֧��2.7~3.6V
			if(buf[2]==0X01&&buf[3]==0XAA)
			{
				retry=0XFFFE;
				do
				{
					//����SD_CMD55
					DRV_SPI_SD_SendCmd(spi_sd, SD_CMD55,0,0X01);	
					//����SD_CMD41
					r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD41,0x40000000,0X01);
				}
				while(r1&&retry--);
				//����SD2.0���汾��ʼ
				if(retry&&DRV_SPI_SD_SendCmd(spi_sd, SD_CMD58,0,0X01)==0)
				{
					//�õ�OCRֵ
					for(i=0;i<4;i++)buf[i]=DRV_SPI_SD_SendByte(spi_sd, 0XFF);
					//���CCS
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
		//����SD_CMD55
		DRV_SPI_SD_SendCmd(spi_sd, SD_CMD55,0,0X01);		
		//����SD_CMD41
		r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD41,0,0X01);	
		if(r1<=1)
		{		
			spi_sd->status.type=SD_TYPE_V1_SD;
			retry=0XFFFE;
			//�ȴ��˳�IDLEģʽ
			do 
			{
				//����SD_CMD55
				DRV_SPI_SD_SendCmd(spi_sd, SD_CMD55,0,0X01);	
				//����SD_CMD41
				r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD41,0,0X01);
			}while(r1&&retry--);
		}
		//MMC����֧��SD_CMD55+SD_CMD41ʶ��
		else
		{
			//MMC V3
			spi_sd->status.type=SD_TYPE_V1_MMC;
			retry=0XFFFE;
			//�ȴ��˳�IDLEģʽ
			do 
			{											    
				//����SD_CMD1
				r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD1,0,0X01);
			}while(r1&&retry--);  
		}
		//����Ŀ�
		if(retry==0||DRV_SPI_SD_SendCmd(spi_sd, SD_CMD16,512,0X01)!=0)
		{
			spi_sd->status.type=SD_TYPE_INIT_FAIL;
		}
	}
	//ȡ��Ƭѡ
	DRV_SPI_SD_Select(spi_sd, false);
	//����Ϊ���ٶ�ģʽ
	DRV_SPI_SD_Speed(spi_sd, 16*1024);	//����Ϊ���ٶ�ģʽ
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

//дSD��
//buf:���ݻ�����
//sector:��ʼ����
//cnt:������
//����ֵ:0,ok;����,ʧ��.
uint8_t DRV_SPI_SD_WriteMultiBlock(SPI_SD *spi_sd, uint32_t sector,uint8_t*buf,uint8_t cnt)
{
	uint8_t r1;
	//ת��Ϊ�ֽڵ�ַ
	if(spi_sd->status.type!=SD_TYPE_V2_SDHC)
	{
		sector *= 512;
	}
	if(cnt==1)
	{
		//������
		r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD24,sector,0X01);
		//ָ��ͳɹ�
		if(r1==0)
		{
			//д512���ֽ�	   
			r1=DRV_SPI_SD_SendBlock(spi_sd, buf,0xFE);
		}
	}
	else
	{
		if(spi_sd->status.type!=SD_TYPE_V1_MMC)
		{
			DRV_SPI_SD_SendCmd(spi_sd, SD_CMD55,0,0X01);	
			//����ָ��	
			DRV_SPI_SD_SendCmd(spi_sd, SD_CMD23,cnt,0X01);
		}
		//����������
 		r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD25,sector,0X01);
		if(r1==0)
		{
			do
			{
				//����512���ֽ�	 
				r1=DRV_SPI_SD_SendBlock(spi_sd, buf,0xFC);
				buf+=512;  
			}
			while(--cnt && r1==0);
			//����512���ֽ� 
			r1=DRV_SPI_SD_SendBlock(spi_sd, 0,0xFD);
		}
	}   
	//ȡ��Ƭѡ
	DRV_SPI_SD_Select(spi_sd, false);
	return r1;
}	


uint8_t DRV_SPI_SD_ReadMultiBlock(SPI_SD *spi_sd, uint32_t sector,uint8_t*buf,uint8_t cnt)
{
	uint8_t r1;
	//ת��Ϊ�ֽڵ�ַ
	if(spi_sd->status.type!=SD_TYPE_V2_SDHC)
	{
		sector <<= 9;
	}
	if(cnt==1)
	{
		//������
		r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD17,sector,0X01);
		//ָ��ͳɹ�
		if(r1==0)
		{
			//����512���ֽ�	 
			r1=DRV_SPI_SD_RecvData(spi_sd, buf,512);  
		}
	}
	else
	{
		//����������
		r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD18,sector,0X01);
		do
		{
			//����512���ֽ�	 
			r1=DRV_SPI_SD_RecvData(spi_sd, buf,512);
			buf+=512;  
		}
		while(--cnt && r1==0); 	
		//����ֹͣ����
		DRV_SPI_SD_SendCmd(spi_sd, SD_CMD12,0,0X01);	
	}   
	//ȡ��Ƭѡ
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

//�ȴ���׼����
static uint8_t DRV_SPI_SD_WaitReady(SPI_SD *spi_sd)
{
	uint32_t t=0;
	uint8_t reg;
	for(t=0;t<0xffff;t++)
	{
		reg=DRV_SPI_SD_SendByte(spi_sd, 0XFF);//��ȡ����ֵ
		if(reg==0XFF)
			break; 	
	}
	if(t<0xffffff)
		return 0;
	else
		return 1;
}


//1.ѡ��sd��,���ҵȴ���׼��OK
//2.ȡ��ѡ��,�ͷ�SPI����
//����ֵ:0,�ɹ�;1,ʧ��;
static uint8_t DRV_SPI_SD_Select(SPI_SD *spi_sd, bool select)
{
	if(select){
		DRV_SPI_SD_Cs(spi_sd, false);	//�ȴ�ʧ��
		if(DRV_SPI_SD_WaitReady(spi_sd)==0){
			//�ȴ��ɹ�
			return 0;
		}else{
			DRV_SPI_SD_Cs(spi_sd, true);	//�ȴ�ʧ��
			return 1;
		}
	}else{
		DRV_SPI_SD_Cs(spi_sd, true);
		DRV_SPI_SD_SendByte(spi_sd, 0xff);//�ṩ�����8��ʱ��
		return true;
	}
}

//��SD������һ������
//����: uint8_t cmd   ���� 
//      uint32_t arg  �������
//      uint8_t crc   crcУ��ֵ	   
//����ֵ:SD�����ص���Ӧ															  
static uint8_t DRV_SPI_SD_SendCmd(SPI_SD *spi_sd, uint8_t cmd, uint32_t arg, uint8_t crc)
{
  uint8_t r1=0;	
	uint8_t Retry=0; 
	DRV_SPI_SD_Select(spi_sd, false);	//ȡ���ϴ�Ƭѡ
	
	if(DRV_SPI_SD_Select(spi_sd, true))	//ƬѡʧЧ 
	{
		return 0XFF;
	}
	//����
	//�ֱ�д������
  DRV_SPI_SD_SendByte(spi_sd, cmd | 0x40);
  DRV_SPI_SD_SendByte(spi_sd, arg >> 24);
  DRV_SPI_SD_SendByte(spi_sd, arg >> 16);
  DRV_SPI_SD_SendByte(spi_sd, arg >> 8);
  DRV_SPI_SD_SendByte(spi_sd, arg);	  
  DRV_SPI_SD_SendByte(spi_sd, crc); 
	if(cmd==SD_CMD12)DRV_SPI_SD_SendByte(spi_sd, 0xff);
  //�ȴ���Ӧ����ʱ�˳�
	Retry=0X1F;
	do
	{
		r1=DRV_SPI_SD_SendByte(spi_sd, 0xFF);
	}
	while((r1&0X80) && Retry--);	 
	//����״ֵ̬
  return r1;
}


//�ȴ�SD����Ӧ
//Response:Ҫ�õ��Ļ�Ӧֵ
//����ֵ:0,�ɹ��õ��˸û�Ӧֵ
//    ����,�õ���Ӧֵʧ��
static uint8_t DRV_SPI_SD_GetResponse(SPI_SD *spi_sd, uint8_t Response)
{
	//�ȴ�����	 
	uint16_t Count=0xFFFF;  						  
	//�ȴ��õ�׼ȷ�Ļ�Ӧ  	
	while ((DRV_SPI_SD_SendByte(spi_sd, 0XFF)!=Response)&&Count)
	{
		Count--;  
	}
	if (Count==0)
	{
		//�õ���Ӧʧ�� 
		return 1;  
	}
	else
	{
		//��ȷ��Ӧ
		return 0;
	}
}




//��sd����ȡһ�����ݰ�������
//buf:���ݻ�����
//len:Ҫ��ȡ�����ݳ���.
//����ֵ:0,�ɹ�;����,ʧ��;	
static uint8_t DRV_SPI_SD_RecvData(SPI_SD *spi_sd, uint8_t*buf,uint16_t len)
{			  	  
	//�ȴ�SD������������ʼ����0xFE
	if(DRV_SPI_SD_GetResponse(spi_sd, 0xFE))
	{
		return 1;
	}
	//��ʼ��������
  while(len--)
  {
    *buf=DRV_SPI_SD_SendByte(spi_sd, 0xFF);
    buf++;
  }
  //������2��αCRC��dummy CRC��
  DRV_SPI_SD_SendByte(spi_sd, 0xFF);
  DRV_SPI_SD_SendByte(spi_sd, 0xFF);		
  //��ȡ�ɹ�							  					    
  return 0;
}




//��ȡSD����CSD��Ϣ�������������ٶ���Ϣ
//����:uint8_t *cid_data(���CID���ڴ棬����16Byte��	    
//����ֵ:0��NO_ERR
//		 1������														   
static uint8_t DRV_SPI_SD_GetCSD(SPI_SD *spi_sd, uint8_t *csd_data)
{
  uint8_t r1;	 
	//��SD_CMD9�����CSD
  r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD9,0,0x01);
  if(r1==0)
	{
		//����16���ֽڵ����� 
    r1=DRV_SPI_SD_RecvData(spi_sd, csd_data, 16);
  }
	//ȡ��Ƭѡ
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




//��ȡSD����������������������   
//����ֵ:0�� ȡ�������� 
//       ����:SD��������(������/512�ֽ�)
//ÿ�������ֽ�����Ϊ512����Ϊ�������512�����ʼ������ͨ��.														  
static uint32_t DRV_SPI_SD_GetSectorCount(SPI_SD *spi_sd)
{
    uint8_t csd[16];
    uint32_t Capacity;  
    uint8_t n;
		uint16_t csize;  					    
		//ȡCSD��Ϣ������ڼ��������0
    if(DRV_SPI_SD_GetCSD(spi_sd, csd)!=0)
		{
			return 0;	    
		}
    //���ΪSDHC�����������淽ʽ����
		//V2.00�Ŀ�
    if((csd[0]&0xC0)==0x40)	 
    {	
			csize = csd[9] + ((uint16_t)csd[8] << 8) + 1;
			//�õ�������	
			Capacity = (uint32_t)csize << 10; 		   
    }
		//V1.XX�Ŀ�
		else
    {	
			n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
			csize = (csd[8] >> 6) + ((uint16_t)csd[7] << 2) + ((uint16_t)(csd[6] & 3) << 10) + 1;
			//�õ������� 
			Capacity= (uint32_t)csize << (n - 9);  
    }
    return Capacity;
}





//��ȡSD����CID��Ϣ��������������Ϣ
//����: uint8_t *cid_data(���CID���ڴ棬����16Byte��	  
//����ֵ:0��NO_ERR
//		 1������														   
static uint8_t DRV_SPI_SD_GetCID(SPI_SD *spi_sd, uint8_t *cid_data)
{
  uint8_t r1;	   
  //��SD_CMD10�����CID
  r1=DRV_SPI_SD_SendCmd(spi_sd, SD_CMD10,0,0x01);
  if(r1==0x00)
	{
		//����16���ֽڵ�����	 
		r1=DRV_SPI_SD_RecvData(spi_sd, cid_data,16);
  }
	//ȡ��Ƭѡ
	DRV_SPI_SD_Select(spi_sd, false);
	if(r1)
		return 1;
	else
		return 0;
}




//��sd��д��һ�����ݰ������� 512�ֽ�
//buf:���ݻ�����
//cmd:ָ��
//����ֵ:0,�ɹ�;����,ʧ��;	
static uint8_t DRV_SPI_SD_SendBlock(SPI_SD *spi_sd, uint8_t*buf,uint8_t cmd)
{	
	uint16_t t;		  	  
	//�ȴ�׼��ʧЧ
	if(DRV_SPI_SD_WaitReady(spi_sd))
	{
		return 1;
	}
	DRV_SPI_SD_SendByte(spi_sd, cmd);
	//���ǽ���ָ��
	if(cmd!=0XFD)
	{
		//����ٶ�,���ٺ�������ʱ��
		for(t=0;t<512;t++)
		{
			DRV_SPI_SD_SendByte(spi_sd, buf[t]);
		}
		//����crc
	  DRV_SPI_SD_SendByte(spi_sd, 0xFF);
	  DRV_SPI_SD_SendByte(spi_sd, 0xFF);
		//������Ӧ
		t=DRV_SPI_SD_SendByte(spi_sd, 0xFF);
		if((t&0x1F)!=0x05)
		{
			//��Ӧ����		
			return 2;		
		}			
	}						 		
	//д��ɹ�							  					    
  return 0;
}


