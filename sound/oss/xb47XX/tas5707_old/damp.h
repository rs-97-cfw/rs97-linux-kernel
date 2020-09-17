/*======================================================
;	Digital Amplifier Driver (TAS5707)
;
;	Proprietary and Confidential Infomation
;
;	Do Not Copy or Distribute
;
;	Copyright, GGEC Audio Team
;	Author: SQ Huang
;	Created:  Jul 29, 2010
;=======================================================*/
#ifndef _DAMP_H
#define	_DAMP_H


#define	TAS5707_I2C_ADDR			0x36
#define	TAS5707A_I2C_ADDR			0x3A

#define	REG_CTRL_1				0x03			//system control 1
#define	REG_I2S_FMT				0x04			//serial data mode
#define	REG_CTRL_2				0x05			//system control 2
#define	REG_SOFT_MUTE			0x06

#define	REG_M_VOL				0x07
#define	REG_CH1_VOL			0x08
#define	REG_CH2_VOL			0x09

#define	REG_VOL_CFG			0x0E			//volume config
#define	REG_OSC_TRIM			0x1B
#define	REG_IN_MUX				0x20			//input mux, 4-byte length
#define	REG_OUT_MUX			0x25			//output mux, 4-byte length
#define	REG_CH1_BQ0			0X29			//CH1 BQ0 addr, 20-byte length
#define	REG_CH2_BQ0			0X30			//CH2 BQ0 addr, 20-byte length

#define	REG_DRC_CTRL			0x46			

// for serial data format
#define	FMT_RIGHT_JUSTIFIED_16		0x00
#define	FMT_RIGHT_JUSTIFIED_20		0x01
#define	FMT_RIGHT_JUSTIFIED_24		0x02
#define	FMT_I2S_16					0x03
#define	FMT_I2S_20					0x04
#define	FMT_I2S_24					0x05
#define	FMT_LEFT_JUSTIFIED_16		0x06
#define	FMT_LEFT_JUSTIFIED_20		0x07
#define	FMT_LEFT_JUSTIFIED_24		0x08


#define	VOL_GAIN_24_DB			0x00
#define	VOL_GAIN_0_DB			0x30
#define	DAMP_GAIN(d)			(0x30-d*2)		
#define	VOL_GAIN_INC(d)			(-(d)*2)
#define	VOL_GAIN_OFFSET			VOL_GAIN_0_DB 
#define	MIN_VOL_GAIN			0xCE			

#define	DEFAULT_DAMP_GAIN		DAMP_GAIN(11.5)		 

#define	SOFT_MUTE_ON			0x03
#define	SOFT_MUTE_OFF			0x00

#define TRUE  1
#define FAULSE 0

unsigned char DAMP_I2C_Write(unsigned char addr, unsigned char value);
unsigned char DAMP_I2C_Read(unsigned char addr, unsigned char * value);
unsigned char Damp_Write_Coefficent(unsigned char addr, unsigned char *buffer, unsigned char len);

void DAMP_ChannelGain(unsigned char gain);
void DAMP_MuteOn(void);
void DAMP_MuteOff(void);
void DAMP_Init(void);
void DAMP_PowerDown(unsigned char pwr_down);
void DAMP_set_volume(unsigned char vol);
void DAMP_EnterShutdown(void);
void DAMP_ExitShutdown(void);

#endif

