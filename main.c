/********************************** Interfacing codec Audio driver with ARM processor*********/




// Necessary Headers must be initialised here

/**
  */
  
  
  
static void I2C_DATA_START()
{
  /* Generate a START condition to transfer the control word to CODEC */
        I2C1_I2CR |= IE;     //I2C Enable 
	I2C1_I2CR |= IIE;    //I2C interrupt Enable
	I2C1_I2CR |= TXAK;   //Transmit ack Enable
	I2C1_I2CR |= MTX;   //Master config
}
static void I2C_SEND_ADDR(uint8_t Address, uint8_t I2C_Direction)
{

  /* Configure I2C to Transmit or Receive*/
  if (I2C_Direction == I2C_PHER_SEL_TX)
  {
    /* Reset the address bit 0 for write */
    Address &= ~0x01;

  }
  else
  {
    /* Set the address bit0 for read */
    Address |= 0x01
  }
  
  I2C1_I2DR = Address;   // send the address
}

static void I2C_SEND_DATA( uint8_t Data)
{
  /* Update the register with the data to be sent */
  I2C1_I2DR = Data;
}

static uint8_t I2C_Data_RECV()
{
  /* Return the data in the DR register */
  return (uint8_t)I2C1_IDR;
}


static void I2C_STOP()
{
	I2C1_I2CR & = ~ 0x0080;  // Clear IE bit ; Bit 7 of Control Register
}

uint32_t Write_Codec(uint8_t REG_ADDR, uint8_t REG_VALUE)
{
  uint32_t result = 0;

  /*!Check for Busy Chip Status*/
  while((TXAK== 1b0)    //Checkin the TXAK bit of Status Register (i.e 0 bit of SR) //received if the bit is '0'
	{
	//wait until the chip is ready
	}
	
  /* Start the config sequence */
	I2C_START(START);  //configure I2C channel

  while((TXAK== 1b0)    //Checkin the TXAK bit of Status Register (i.e 0 bit of SR) //received if the bit is '0'
	{
	//wait until the chip is ready
	}

  /* Transmit the slave address and enable writing operation */
	I2C_SEND_ADDR(CHIP_ADDRESS, I2C_SEND);

  while((TXAK == 0b0)    //Checkin the TXAK bit of Status Register (i.e 0 bit of SR) //received if the bit is '0'
	{
	//wait until the chip is ready
	}
  /* Transmit the first address for write operation */
  I2C_SEND_DATA(REG_ADDR);

 
  while((TXAK == 0b0)    //Checkin the TXAK bit of Status Register (i.e 0 bit of SR) //received if the bit is '0'
	{
	//wait until the chip is ready
	}

  /* Prepare the register value to be sent */
  I2C_SEND_DATA(REG_VALUE);

  /*!< Wait till all data have been physically transferred on the bus */

  while((TXAK == 0b0)    //Checkin the TXAK bit of Status Register (i.e 0 bit of SR) //received if the bit is '0'
	{
	//wait until the chip is ready
	}

  /* End the  sequence */
  I2C_SEND_STOP();

	#ifdef CHECK_DATA
	  /* Check for successful data write*/  
	  PARITY_BIT = (Read_Codec(REG_ADDR) == REG_VALUE)? 0:1;
	#endif 

  /* Return the verifying value: 0 (Passed) or 1 (Failed) */
  if(!PARITY_BIT) //unsuccessful break;
}

/***********************************
 * Read Codec Chip Register contents
 * *********************************/

static uint32_t Read_Codec(uint8_t REG_ADDR)
{
  uint32_t RECV_BYTE = 0;
  
   while((TXAK == 0b0)    //Checkin the TXAK bit of Status Register (i.e 0 bit of SR) //received if the bit is '0'
	{
	//wait until the chip is ready
	}
  
  /* Start the config sequence */
  I2C_START(START);
  
  /* Transmit the slave address and enable read operation */
  I2C_SEND_ADDR(CHIP_ADDRESS, I2C_SEND);

	while((TXAK == 0b0)    //Checkin the TXAK bit of Status Register (i.e 0 bit of SR) //received if the bit is '0'
	{
	//wait until the chip is ready
	}

  /* Transmit the register address to be read */
  I2C_SEND_DATA(REG_ADDR);

  while((TXAK == 0b0)    //Checkin the TXAK bit of Status Register (i.e 0 bit of SR) //received if the bit is '0'
	{
	//wait until the chip is ready
	}

  /*!< Disable Acknowledgment */
  I2C_CONFIG_ACK(ENABLE);

  /*!< Send STOP Condition */
  I2C_STOP(1);

  while((TXAK == 0b0)    //Checkin the TXAK bit of Status Register (i.e 0 bit of SR) //received if the bit is '0'
	{
	//wait until the chip is ready
	}
  /*Read the byte received from the Codec */
  RCV_BYTE = I2C_DATA_RECV();

  /-Enable Ack  for another reception */
  I2C_CONFIG_ACK(ENABLE);

  /* Return the read config byte of the RCV BYTE */
  return RCV_BYTE;
}

void I2C_CONFIG_ACK(unit8_t CLR_BIT)
{
  if(!CLR_BIT)
	I2C1_I2CR & = ~0x03; /Clear TXAK bit of control register 
  else 
	I2C1_I2CR | =  0x03; /Set TXAK bit of control register
}

static  void Config_I2S(bool STAT )
{
  if(STAT)
  {
	SSI1_SCR   |=  0x00000030  //I2S mode (01) = Master ;SYN bit=1
	SSI1_STCR  |=  0x0000000D //  TX clocked ar falling | Tx frame sync active low (SSI_STCR[2]=1)|Tx frame sync initiated one bit before data is transmitted (SSI_STCR[0]=1)
	SSI1_STCR  &= ~0x00000010 //MSB TX 
	SSI1_SRCR  |=  0x0000000D //  RX latched at rising | Rx frame sync active low (SSI_SRCR[2]=1)|Rx frame sync initiated one bit before data is received (SSI_SRCR[0]=1)
	SSI1_SRCR  &= ~0x00000010 //MSB RX 
	SSI1_STCCR |=  0x00001F00 //TX Frame Rate should be 2 (SSI_STCCR[12:8] = 1)
	SSI1_SRCCR |=  0x00001F00 //RX Frame Rate should be 2 (SSI_SRCCR[12:8] = 1)
  }
  else
  {
	  //Disable the SSI_ I2S and switch to normal mode
	SSI1_SCR  =0;
	SSI1_STCR =0;
	SSI1_SRCR =0;
	SSI1_SRCCR=0;
  }	
//I2S in Master mode has a fixed WL = 32 bits.
		
}
static void Init_I2S_DMA(unit32_t  DMA_req, bool STAT)
{
	if (STAT)
	SSI1_IER  |= DMA_req  //  DMA of Rx or Tx FIFO is enabled
	else 
	SSI1_IER  &=~DMA_req; //  DMA of Rx or Tx FIFO is disabled
}

// Sends the data available in DMA FIFO buffer

static void I2S_Send(unit32_t DMA_Tx_req)
{
  /* Enable the I2S TX DMA request */
  Init_I2S_DMA(DMA_Tx_req, ENABLE);
  
  /* Enable the SPI1 Master peripheral */
  Config_I2S(ENABLE);

  /* Wait until Trasnsfer is complete*/
  while(TxStatus == 0);
  
  /* Disable the I2S TX DMA request */
  Init_I2S_DMA(DMA_Tx_req, DISABLE);
  
  /* Disable the SPI1 Master peripheral */
  Config_I2S(DISABLE);
}

//Receives the data into DMA Rx FIFO buffer
static void I2S_RCV(unit32_t DMA_Tx_req)
{
  /* Enable the I2S TX DMA request */
  Init_I2S_DMA(DMA_Rx_req, ENABLE);
  
  /* Enable the SPI1 Master peripheral */
  Config_I2S(ENABLE);

  /* Wait until Trasnsfer is complete*/
  while(RxStatus == 0);
  
  /* Disable the I2S RX DMA request */
  Init_I2S_DMA(DMA_Rx_req, DISABLE);
  
  /* Disable the SPI1 Master peripheral */
  Config_I2S(DISABLE);
  
}


 
 
static  void Init_CODEC()
{
		/****************************************************************************
	 * Using a 32bit MUX/DEMUX with 2 bit select signal it is possible to interface
     * 4 Codec Chips on the same bus.
	 * Chip_Sel0 =0 chip_Sel1 =0   =>Codec A can be used
	 * Chip_Sel0=0  chip_Sel1 =1   =>Codec B can be used
	 * Chip_Sel0=1  chip_Sel1 =0   =>Codec C can be used
	 * **********************************************************************/
	CHIP_SEL_0 = 0;
	CHIP_SEL_1 = 0;
	
	Config_GPIO(CHIP_SEL_0, CHIP_SEL_1);  //Any two IO pins of the processor can be used to act as select signals
	
	
	//configure  ADC channel of CODECA to receive input audio from AIN1+/- channel
	
	// Since The below functions are the core of the given assignment I have refered to the datasheet of IMX6 and properly developed the code 
	Write_Codec(0x06, 0x00);  // clock and speed mode config 
	
	Write_Codec(0x07,0xF0);   // Width at SD0 and SD1 is set to 24 bits. 
	
	Write_Codec(0x08,0x04);  // Config Codec to I2S mode 
	
	Write_Codec(0x10,0x01);  //ADC1 Power Down
	
	
	
	//CODEC B is selected
	 CHIP_SEL_0 = 0;
	 CHIP_SEL_1 = 1;
	
	Config_GPIO(CHIP_SEL_0, CHIP_SEL_1);  //Any two IO pins of the processor can be used to act as select signals
	
	Write_Codec(0x06, 0x00);  // clock and speed mode config 
	
	Write_Codec(0x07,0xF0);   // Width at SD0 and SD1 is set to 24 bits. 
	
	Write_Codec(0x08,0x04);  // Config Codec to I2S mode 
	
	Write_Codec(0x15,0x01);  //DAC1 Power Down
}



/*******************************************************
 * Function that alternates the selection between CODECA 
 * and CODECB to receive and send audo */
 
static void AUDIO_RCV_TRANS()
{
	CHIP_SEL_0 = 0;
	CHIP_SEL_1 = 0;
	Config_GPIO(CHIP_SEL_0, CHIP_SEL_1);  //Any two IO pins of the processor can be used to act as select signals
	I2S_RCV();            	 //The data receive register is updated in the DMA handler (Audio from CODECA)
   	
	CHIP_SEL_0 = 0;
	CHIP_SEL_1 = 1;
	Config_GPIO(CHIP_SEL_0, CHIP_SEL_1);  //Any two IO pins of the processor can be used to act as select signals
	SSI1_SRX1
	while(RxStatus);
	SSI1_STX1=DataTemp;    //wait until receive is complete and update the Tx buffer to be trasmitted
	I2S_SEND();           //The data send to CODECB  (Audio to CODECB)
}
/*Pragma vector for DMA interrupt handler... ( I have given some meaningful name for the service routine as I donot 
 * know the exact handler. ISR is triggered when DMA operation is complete**/
#pragma ___ISR_DMA_IRQHandler(void)     
{ 
  if (RRDY)    //DMA status flag;  SET when RX complte
  {
    RxStatus = 0;  //indicate the DMA RX is compelete
    DataTemp = (uint16_t)SSI1_SRX1 ;   //Data received from the codec A
  } 
  if (TRDY)    //DMA status flag; SET when TX complete
  {
    TxStatus = 0;  //indicate the DMA TX is compelete
  }
}


void main()
{
	/*	 IO configure the I2C  to the audio codec.
	/*	 Reset the codec registers.
	/*	 Configure the I2C to initialize the audio codec control interface.*/
	
	/* There are few configuration steps which I choose to ignore for simplicity. Inorder to express my understanding and importance of 
	* those configuration steps, I will just describe in simple terms
	* Clock config  // Scale the clocks MCLK (Main Clock ) and (SCLK-Sub Main Clk) ; freq of MCLK > freq of SCLK 
	* GPIO config   // config General Prpose IO pins to SSI functions
	* SPI config    // Config frequency
	* DMA config   //Word Size config  etc */
	
	
       Initi_CODEC();

	Config_I2S();          // config SSI to operate in I2S  mode
	CHIP_SEL_0 = 0;
	CHIP_SEL_1 = 0;
	Config_GPIO(CHIP_SEL_0, CHIP_SEL_1);  //Any two IO pins of the processor can be used to act as select signals
	Write_Codec(0x10,0x00);  //ADC1 of CodecA Power Up
	
	
	CHIP_SEL_0 = 0;
	CHIP_SEL_1 = 1;
	Config_GPIO(CHIP_SEL_0, CHIP_SEL_1);  //Any two IO pins of the processor can be used to act as select signals
	Write_Codec(0x15,0x00);  //DAC1 Power of CodecB Power UP
	
	
	while(1)    //Infinite loop to keep the processor active and prevent from exiting
	{
		AUDIO_RCV_TRANS();  /// receivs data from codec A and sends it to the codec B
	}

}




