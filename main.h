
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ---------------------------------------------------------------*/
#include "main.h"

#define I2C_PHER_SEL_TX                0x01
#define I2C_PHER_SEL_RX                0x02


#define I2C_IO_PORT                    GPIO5
#define I2C_SCL_PIN                    GPIO_Pin_27
#define I2C_SDA_PIN                    GPIO_Pin_26
#define CODEC_I2S_SCL_PINSRC           GPIO_Pin_27
#define CODEC_I2S_SDA_PINSRC           GPIO_Pin_26



#define DMA_Tx_req 			0x00080000   //  DMA Tx buffer enable ( and TXDMAE (Bit 20) =1)
#define DMA_Rx_req 			0x00200000   //DMA Rx buffer enable  (RDMAE (Bit 22) =1)

#define ENABLE   1
#define DISABLE  0

#define CHIP_ADDRESS 0x2F      //0b0010 110 1  //Chip adrress| AD0| Read
#define GPIO5  
#define GPI0_Pin_26 
#define GPIO_Pin_27

volatile bool RxStatus, TxStatus;

volatile uint32_t  DataTemp;
bool CHIP_SEL_0;
bool CHIP_SEL_1;

void I2C_DATA_START();
void I2C_SEND_ADDR(uint8_t Address, uint8_t I2C_Direction);
void I2C_SEND_DATA( uint8_t Data);
void uint8_t I2C_Data_RECV();
void I2C_STOP();
uint32_t Write_Codec(uint8_t REG_ADDR, uint8_t REG_VALUE);
uint32_t Read_Codec(uint8_t REG_ADDR);
void I2S_Send(unit32_t DMA_Tx_req);
void I2S_RCV(unit32_t DMA_Tx_req);
void I2C_CONFIG_ACK(unit8_t CLR_BIT);
void  Config_I2S(bool STAT );
void Init_I2S_DMA(unit32_t  DMA_req, bool STAT);
void Init_CODEC();
void AUDIO_RCV_TRANS();
