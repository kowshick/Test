# Test
This repo consists of main.c and main.h

The code contains important functions which are required to interface Imx6 processor with multiple audio Codec.



CS4244 codec:
I have developed the code based on the below algorithm

Interfacing the audio codec with processor rquires 2 steps 
   1. Configure the codec using I2C
   2. Send/Receive audio data via I2S 
Configuring via I2C requires the following sequence
        Send start condition.
        Send 0010xxx0 (chip address & write operation).
        Receive acknowledge bit.
        Send MAP byte, auto increment off.
        Receive acknowledge bit.
        Send stop condition, aborting write.
        Send start condition.
        Send 0010xxx1 (chip address & read operation).
        Receive acknowledge bit.
        Receive byte, contents of selected register.
        Send acknowledge bit.
        Send stop condition.
Then configure SSI to function in I2S mode 
Then enable the ADC/DAC channel using the above seq of the codec to rceive or send the data via I2S


For Multiple Codec:

An MUX/DEMUX is used for high frequencing switching betwen COdec A and COdec B so that data can be read from A and send to B for Playback
******************
(Another Solution I could think of which can be implemented is :
1.Differentiate the codecA and Codec B using 'Device ID' and Read Digital data from CodecA on Read Cycle (Rising edge of MCLK)and Send DAta to CodecB on Write Cycle Falling edge of MCLK)
If I work little more this I can try implement it. But to explain you that I have understood the functioning of protocols I am just proposing the solution
**************
Note:
This code ignore many basic GPIO and NIVC external Interrupt config and focuses mainly on the functions required to implement the above algorithm using IMX6 Dual Lite Processor

The repo includes the datasheet to which I have reffered to develop this application.
###################################################################################################

