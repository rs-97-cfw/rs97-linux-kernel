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
#include	"damp.h"
#include	"damp_filter.h"
#include    	"../jz47XX_codec.h"

//#define	DAMP_DBG

 unsigned char DAMP_I2C_Write(unsigned char addr, unsigned char value)
{
	unsigned char res;

#ifdef DAMP_DBG
	printk("DAMP write reg 0x%02bx:0x%02bx\n", addr, value);
#endif
	res = sky_i2c_write_reg(addr, &value, 1);

#ifdef DAMP_DBG
	if( res )
		printk("write reg fail!\n");    
#endif
	return res;
}

 unsigned char DAMP_I2C_Read(unsigned char addr, unsigned char * value)
{	
	return sky_i2c_read_reg( addr, value, 1);
}

#if 0
unsigned char Damp_Write_Coefficent(unsigned char addr, unsigned char *buffer, unsigned char len)
{	
#ifdef DAMP_DBG
	printk("DAMP write coef 0x%02bx:", addr);
	printk(" 0x%02bx", buffer[0]);
	printk(" 0x%02bx", buffer[1]);
	printk(" 0x%02bx", buffer[2]);
	printk(" 0x%02bx\n", buffer[3]);
#endif
	return sky_i2c_write_reg(addr, buffer, len);
}

#else
unsigned char Damp_Write_Coefficent(unsigned char addr, unsigned char *buffer, unsigned char len)
{
	int i;
	unsigned char buf[30] = {0};

	if(len / 4){
		for(i = 0; i < len/4; i++)
		{
			buf[i*4]   = *(buffer + 3);
			buf[i*4+1] = *(buffer + 2);
			buf[i*4+2] = *(buffer + 1);
			buf[i*4+3] = *buffer;
			buffer = buffer + 4;
		}
	}

	return sky_i2c_write_reg(addr, buf, len);
}
#endif

void DAMP_ConfigInit(void)
{
#ifdef DAMP_DBG
	printk("DAMP Config Init...\n");
#endif

	DAMP_I2C_Write(REG_OSC_TRIM, 0x00);			
	mdelay(50);	
	
	DAMP_I2C_Write(REG_I2S_FMT, FMT_I2S_24);	
}

 void DAMP_ChannelGain(unsigned char gain) 
{
	DAMP_I2C_Write(REG_CH1_VOL, gain);			
	DAMP_I2C_Write(REG_CH2_VOL, gain);			
}

void DAMP_MuteOn(void)
{
	DAMP_I2C_Write(REG_SOFT_MUTE, SOFT_MUTE_ON); //by software
}

void DAMP_MuteOff(void)
{
	DAMP_I2C_Write(REG_SOFT_MUTE, SOFT_MUTE_OFF);
}

void	DAMP_EnterShutdown(void)
{
	mdelay(50);
	printk("%s\n",__FUNCTION__);
	DAMP_I2C_Write(REG_CTRL_2, 0x40);
	mdelay(150);
}

void	DAMP_ExitShutdown(void)
{
	mdelay(50);
	printk("%s\n",__FUNCTION__);
	DAMP_I2C_Write(REG_CTRL_2, 0x00); 
	mdelay(150);
}

#define MAX_MAIN_VOLUME 30
 unsigned char damp_volume_table[MAX_MAIN_VOLUME+1] =
{
	MIN_VOL_GAIN,							// 0        --> -79db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-65),		// 1        --> -60db  
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-58),		// 2        --> -52db    
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-52),		// 3        --> -45db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-47),		// 4        --> -39db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-42),		// 5        --> -34db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-38),		// 6        --> -30db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-34),		// 7        --> -26db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-31),		// 8        --> -23db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-28),		// 9        --> -21db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-25),		// 10       --> -20db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-23),		// 11       --> -19db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-21),		// 12       --> -18db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-19),		// 13       --> -17db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-17),		// 14       --> -16db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-15),		// 15       --> -15db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-14),		// 16       --> -14db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-13),		// 17       --> -13db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-12),		// 18       --> -12db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-11),		// 19       --> -11db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-10),		// 20       --> -10db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-9),		// 21       --> -9db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-8),		// 22       --> -8db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-7),		// 23       --> -7db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-6),		// 24       --> -6db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-5),		// 25       --> -5db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-4),		// 26       --> -4db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-3),		// 27       --> -3db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-2),		// 28       --> -2db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(-1),		// 29       --> -1db
	VOL_GAIN_OFFSET+VOL_GAIN_INC(0)		// 30       --> 0db   tas5707,master V reg 0x30->0db(default)
};

void	DAMP_set_volume(unsigned char vol)
{
	unsigned char gain;

	gain = damp_volume_table[vol];
	DAMP_I2C_Write(REG_M_VOL, gain);	 // set the master volume
	//DAMP_Dynamic_EQ(vol);                  //It is the old guoguang set.          
}

static void EQ_OFF(void)
{
	unsigned char reg[4]={0};
	unsigned char reg_addr = 0x50;

	sky_i2c_read_reg(reg_addr, reg, 4);
	reg[3] = 0x80;
	sky_i2c_write_reg(reg_addr, reg, 4);
}

/* change volume speed */
static void change_volume_step(void)
{
	unsigned char reg;
	unsigned char reg_addr = 0x0E;
	reg = 0x92;
	sky_i2c_write_reg(reg_addr, &reg, 1);
}

 void DAMP_Init(void)
{
	DAMP_ConfigInit();							
	DAMP_Filter_Init();  
	DAMP_DRC_Init();						
	DAMP_DRC_Control(TRUE);				 
	DAMP_ChannelGain(DEFAULT_DAMP_GAIN);	
	change_volume_step();
	//DAMP_set_volume(30 * INIT_VOLUME /100);	   // init master replay volume
	mdelay(300);
	//DAMP_MuteOn();         
	DAMP_ExitShutdown();					
}

