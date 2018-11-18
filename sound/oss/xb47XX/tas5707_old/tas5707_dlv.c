/*
 * Linux/sound/oss/jz_dlv.c
 *
 * DLV CODEC driver for Ingenic Jz4750 MIPS processor
 *
 * 2009-12-xx	Steven <dsqiu@ingenic.cn>
 * 2010-01-xx	Jason <xwang@ingenic.cn>
 * 2010-11-xx   jbbi <jbbi@ingenic.cn>
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/sound.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/soundcard.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>

#include <asm/hardirq.h>
#include <asm/jzsoc.h>
//#include <mach/chip-aic.h>
//#include <linux/jz_audio.h>
#include <linux/i2c.h>

#include "../jz47XX_codec.h"

static int sky_i2c_read_reg (unsigned char reg, unsigned char* data, unsigned int len);
static int sky_i2c_write_reg(unsigned char reg, unsigned char* data, unsigned int len);

extern struct __dlv_route_info tas5707_dlv_route_info[];

#include "damp.c"
#include "damp_filter.c"

#define USE_TAS5713_OLD    0

#define DUMP_FUNC() printk("TAS5713_DLV:%s\tline:%d\n", __func__, __LINE__)

#define GPIO_TAS5713_RESET  		(32*1 + 16)
#define GPIO_TAS5713_PDN		(32*4 + 4)
//#define GPIO_TAS5713_INPUT_SELECT	(32*5 + 3)

#define REG_ERROR_STATUS	0x02
#define REG_DATA_INTERFACE	0x04

#if USE_TAS5713_OLD
static int tas5713_dac_flag = 0;

//static unsigned char tas5713_reg_defs[128];
#define TAS5713_CHIPADDR	0x36

#define REG_CLOCK_CONTROL	0x00
#define REG_DEVICE_ID		0x01
#define REG_SYSTEM_CONTROL1	0x03
#define REG_I2S_MODE	0x04
#define REG_SYSTEM_CONTROL2	0x05
#define REG_SOFT_MUTE	0x06
#define REG_VOLUME_MASTER	0x07
#define REG_VOLUME_CHAN1	0x08
#define REG_VOLUME_CHAN2	0x09
#define REG_VOLUME_HP	0x0A
#define REG_VOLUME_CONFIG	0x0E
#define REG_MODULATION_LIMIT	0x10
#define REG_INTERCHANNEL1_DELAY	0x011
#define REG_INTERCHANNEL2_DELAY	0x012
#define REG_INTERCHANNEL3_DELAY	0x013
#define REG_INTERCHANNEL4_DELAY	0x014
#define REG_PWM_SHUTDOWN	0x19
#define REG_START_PERIOD	0x1A
#define REG_OSCILLATOR_TRIM	0X1B
#define REG_BKND_ERR	0x1C
#define REG_INPUT_MULTIPLEXER	0x20
#define REG_CHANNEL_SELECT		0x21
#define REG_PWM_MUX	0x25
#define REG_DRC_CONTROL	0x46
#define REG_BANK_AND_EQ	0x50

#endif

/****************** i2c gpio ****************************************/
#define GPIO_I2C_SDA 32*4+30
#define GPIO_I2C_SCL 32*4+31


/****  I2C bus GPIO interface function ****/
extern void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk);
static struct i2c_client *tas5713_client = NULL;

static int sky_i2c_write_reg(unsigned char reg, unsigned char* data, unsigned int len)
{
	int i,ret =-1;
	unsigned char buf[30] = {0};
        struct i2c_client *client = tas5713_client;

	if(!data || len <= 0){
		printk("err argument: sky_i2c_write_reg\n");
		return ret;
	}

	buf[0] = reg;
	for(i=0; i < len;i++){
		buf[i+1] = *data;
		data++;
	}

//	printk("enter sky_i2c_write_reg reg 0x%02x, len %d\n", reg, len);
	ret = i2c_master_send(client, buf, len+1);
        if (ret < 0) 
	        printk("sky_i2c_write_reg 0x%02x err\n", reg);
	return ret;
}

static int sky_i2c_read_reg(unsigned char reg, unsigned char* data, unsigned int len)
{
	int ret = -1;
        struct i2c_client *client = tas5713_client;
	
	if(!data || len <= 0){
		printk("err argument: sky_i2c_read_reg\n");
		return ret;
	}
//	printk("enter sky_i2c_read_reg reg 0x%02x, len %d\n", reg, len);
	ret = i2c_master_send(client, &reg, 1);
        if (ret < 0) 
	{
		printk("sky_i2c_read_reg send reg addr 0x%02x err\n", reg);
		return ret;
	}

	ret = i2c_master_recv(client, data, len);
        if (ret < 0) 
	{
		printk("sky_i2c_read_reg 0x%02x err\n", reg);
		return ret;
	}
	return ret;
}

/**** 
 *  
 * Tas5707 I2C GPIO controller init. if successful, you can use the above function to access the Tas5707 internel registers.
 *
 * ***/
static int __devinit tas5707_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		return err;
	}
	
	tas5713_client = client;
	i2c_jz_setclk(client, 100*1000);

	return 0;
}
static int __devexit tas5707_i2c_remove(struct i2c_client *client)
{
		tas5713_client = NULL;
        return 0;
}

static const struct i2c_device_id tas5707_id[] = {
        { "tas5707", 0 },
        { }
};      
MODULE_DEVICE_TABLE(i2c, tas5707_id);

static struct i2c_driver tas5707_i2c_driver = {
        .driver.name = "tas5707",
        .probe       = tas5707_i2c_probe,
        .remove      = __devexit_p(tas5707_i2c_remove),
        .id_table    = tas5707_id,
};

/************ end I2C controller ***********/


/****************** tas5713 api ****************************************/

void tas5713_read_reg(unsigned char reg, unsigned char len)
{
	unsigned char buf[32] = {0};
	int i = 0;
	sky_i2c_read_reg(reg, buf, len);
	
	printk("{0x%02x, ", reg);
	for (i = 0; i< len; i++) printk("0x%02x, ", buf[i]);
	printk("},\n\n");
}

/****
 *
 *  tas5713_reset is a important step,you can read Tas5707 spec to understand it.
 *  i2s_controller_init() function is just to support the I2S clks for tas5707.
 *
 * ***/
extern void i2s_controller_init(void);      
void tas5713_reset(void)
{
	__gpio_as_output(GPIO_TAS5713_RESET);
	__gpio_as_output(GPIO_TAS5713_PDN);

//       __gpio_as_output(GPIO_TAS5713_INPUT_SELECT);
//       __gpio_clear_pin(GPIO_TAS5713_INPUT_SELECT);	 //select cpu's i2s output
	
	__gpio_clear_pin(GPIO_TAS5713_PDN);
	mdelay(10);
	__gpio_clear_pin(GPIO_TAS5713_RESET);
	mdelay(300);
	printk("\nrestart TAS5707\n");
	__gpio_set_pin(GPIO_TAS5713_PDN);
	mdelay(10);
	
    //i2s_controller_init();
	//REG_AIC_I2SCR |= AIC_I2SCR_ESCLK;   // start MCLK
	mdelay(10);

	__gpio_set_pin(GPIO_TAS5713_RESET);
	mdelay(20);
	return ;
}

int tas5713_check_err_status(void)
{
	unsigned char value = 0;

	sky_i2c_read_reg(REG_ERROR_STATUS, &value, 1);

	if (value) {
		printk("tas5713_check_err_status 0x%02x\n", value);
		value = 0;
		sky_i2c_write_reg(REG_ERROR_STATUS, &value, 1); // clear the status
		return -1;
	}
	return 0;
}

static void tas5713_shutdown(void)
{
//	unsigned char value = 0x40;
	printk("tas5713_shutdown\n");
//	sky_i2c_write_reg(REG_SYSTEM_CONTROL2, &value,1);

	return ;
}

#if USE_TAS5713_OLD
static void tas5713_wakeup(void)
{
//	unsigned char value = 0x00;
	printk("tas5713_wakeup\n");
//	sky_i2c_write_reg(REG_SYSTEM_CONTROL2, &value,1);

	return ;
}

static int tas5713_set_volume(unsigned char vol)
{
	int ret;
	printk("\n tas5713_set_volume vol=%d\n", vol);
	sky_i2c_write_reg(REG_VOLUME_MASTER, &vol, 1);
	tas5713_check_err_status();

	return 0;
}

static int tas5713_set_mute(int mute)
{
	unsigned char value = 0x00;
	
	if (mute) 
		value = 0x07;
	else  
		value = 0x00;

	sky_i2c_write_reg(REG_SOFT_MUTE, &value,1);
	
	return 0;
}
#endif

/*
int tas5713_input_select(int input)
{
	printk(KERN_DEBUG "tas5713_input_select %s !\n", input?"adc line in":"local play");
	__gpio_as_output(GPIO_TAS5713_INPUT_SELECT);
	
	if (!input) {
		tas5713_dac_flag = 0;
		__gpio_clear_pin(GPIO_TAS5713_INPUT_SELECT);  //select cpu  i2s output
		tas5713_set_mute(1); // mute the tas5713 output if tvout
		
	} else {
		tas5713_dac_flag = 1;
		__gpio_set_pin(GPIO_TAS5713_INPUT_SELECT);  //select pcm1808 adc  i2s output
		tas5713_set_mute(0); // unmute
	}
	return 0;
}
*/

void tas5713_dump_all(void)
{
#if 1
	tas5713_read_reg(0x00, 1);
	tas5713_read_reg(0x01, 1);
	tas5713_read_reg(0x02, 1);
	tas5713_read_reg(0x03, 1);
	tas5713_read_reg(0x04, 1);
	tas5713_read_reg(0x05, 1);
	tas5713_read_reg(0x06, 1);
	tas5713_read_reg(0x07, 1);
	tas5713_read_reg(0x08, 1);
	tas5713_read_reg(0x09, 1);
	tas5713_read_reg(0x0A, 1);
	tas5713_read_reg(0x0E, 1);
	tas5713_read_reg(0x10, 1);
	tas5713_read_reg(0x11, 1);
	tas5713_read_reg(0x12, 1);
	tas5713_read_reg(0x13, 1);
	tas5713_read_reg(0x14, 1);
	tas5713_read_reg(0x1A, 1);
	tas5713_read_reg(0x1B, 1);
	tas5713_read_reg(0x1C, 1);
	tas5713_read_reg(0x20, 4);
	tas5713_read_reg(0x25, 4);
	tas5713_read_reg(0x29, 20);
	tas5713_read_reg(0x2A, 20);
	tas5713_read_reg(0x2B, 20);
	tas5713_read_reg(0x2C, 20);
	tas5713_read_reg(0x2D, 20);
	tas5713_read_reg(0x2E, 20);
	tas5713_read_reg(0x2F, 20);
	tas5713_read_reg(0x30, 20);
	tas5713_read_reg(0x31, 20);
	tas5713_read_reg(0x32, 20);
	tas5713_read_reg(0x33, 20);
	tas5713_read_reg(0x34, 20);
	tas5713_read_reg(0x35, 20);
	tas5713_read_reg(0x36, 20);
	tas5713_read_reg(0x3A, 8);
	tas5713_read_reg(0x3B, 8);
	tas5713_read_reg(0x3C, 8);
	tas5713_read_reg(0x40, 4);
	tas5713_read_reg(0x41, 4);
	tas5713_read_reg(0x42, 4);
	tas5713_read_reg(0x46, 4);
	tas5713_read_reg(0x50, 4);
	tas5713_read_reg(0x53, 8);
	tas5713_read_reg(0x54, 8);
	tas5713_read_reg(0xF9, 4);

#endif
}

/***************************************************************************************\
 *global variable and structure interface                                              *
\***************************************************************************************/

static unsigned int cur_route = -1;
unsigned int keep_old_route = -1;

unsigned int g_current_out_dev;

/*=================== lock ============================*/
static struct semaphore *g_dlv_sem = 0;

#define DLV_DEBUG_SEM(x,y...) //printk(x,##y);

#define DLV_LOCK()							\
	do{								\
		if(g_dlv_sem)						\
			down(g_dlv_sem);				\
		DLV_DEBUG_SEM("dlvsemlock lock\n");			\
	}while(0)

#define DLV_UNLOCK()							\
	do{								\
		if(g_dlv_sem)						\
			up(g_dlv_sem);					\
		DLV_DEBUG_SEM("dlvsemlock unlock\n");			\
	}while(0)

#define DLV_LOCKINIT()							\
	do{								\
		if(g_dlv_sem == NULL)					\
			g_dlv_sem = (struct semaphore *)vmalloc(sizeof(struct semaphore)); \
		if(g_dlv_sem)						\
			init_MUTEX_LOCKED(g_dlv_sem);			\
		DLV_DEBUG_SEM("dlvsemlock init\n");			\
	}while(0)

#define DLV_LOCKDEINIT()						\
	do{								\
		if(g_dlv_sem)						\
			vfree(g_dlv_sem);				\
		g_dlv_sem = NULL;					\
		DLV_DEBUG_SEM("dlvsemlock deinit\n");			\
	}while(0)

/***************************************************************************************\
 *debug part                                                                           *
\***************************************************************************************/
/*###############################################*/

#define DLV_DUMP_IOC_CMD		0
#define DLV_DUMP_ROUTE_REGS		0
#define DLV_DUMP_ROUTE_PART_REGS	0
#define DLV_DUMP_GAIN_PART_REGS		0
#define DLV_DUMP_ROUTE_NAME		1

/*##############################################*/
/***
 *
 *This is jzdlv_ioctl() command list.jzdlv_ioctl() will be called in jz47xx_i2s.c to configure the codec.
 *
 * ***/
#if DLV_DUMP_IOC_CMD 
static void dlv_print_ioc_cmd(int cmd)
{
	int i;

	int cmd_arr[] = {
		CODEC_INIT,			CODEC_TURN_OFF,			
		CODEC_SHUTDOWN,			CODEC_RESET,
		CODEC_SUSPEND,			CODEC_RESUME,
		CODEC_ANTI_POP, 		CODEC_SET_ROUTE,
 		CODEC_SET_DEVICE,		CODEC_SET_RECORD_RATE,
 		CODEC_SET_RECORD_DATA_WIDTH, 	CODEC_SET_MIC_VOLUME,
		CODEC_SET_RECORD_CHANNEL, 	CODEC_SET_REPLAY_RATE,
 		CODEC_SET_REPLAY_DATA_WIDTH,   	CODEC_SET_REPLAY_VOLUME,
		CODEC_SET_REPLAY_CHANNEL, 	CODEC_DAC_MUTE,
		CODEC_DEBUG_ROUTINE,		CODEC_SET_STANDBY
	};

	char *cmd_str[] = {
		"CODEC_INIT", 			"CODEC_TURN_OFF",
		"CODEC_SHUTDOWN", 		"CODEC_RESET",
		"CODEC_SUSPEND",		"CODEC_RESUME",
		"CODEC_ANTI_POP", 		"CODEC_SET_ROUTE",
		"CODEC_SET_DEVICE",		"CODEC_SET_RECORD_RATE",
		"CODEC_SET_RECORD_DATA_WIDTH", 	"CODEC_SET_MIC_VOLUME",
		"CODEC_SET_RECORD_CHANNEL", 	"CODEC_SET_REPLAY_RATE",
		"CODEC_SET_REPLAY_DATA_WIDTH", 	"CODEC_SET_REPLAY_VOLUME",
		"CODEC_SET_REPLAY_CHANNEL", 	"CODEC_DAC_MUTE",
		"CODEC_DEBUG_ROUTINE",		"CODEC_SET_STANDBY"
	};

	for ( i = 0; i < sizeof(cmd_arr) / sizeof(int); i++) {
		if (cmd_arr[i] == cmd) {
			printk("CODEC IOC: Command name : %s\n", cmd_str[i]);
			return;
		}
	}

	if (i == sizeof(cmd_arr) / sizeof(int)) {
		printk("CODEC IOC: command is not under control\n");
	}
}
#endif //DLV_DUMP_IOC_CMD 

/***
 *
 *  Below is some routes for internal codec. Here is no use.
 *
 * ***/
#if DLV_DUMP_ROUTE_NAME
static void dlv_print_route_name(int route)
{
	int i;

	int route_arr[] = {
		ROUTE_ALL_CLEAR,
		ROUTE_REPLAY_CLEAR,
		ROUTE_RECORD_CLEAR,
		RECORD_MIC1_MONO_DIFF_WITH_BIAS,
		RECORD_MIC1_MONO_DIFF_WITHOUT_BIAS,
		RECORD_MIC2_MONO_DIFF_WITH_BIAS,
		RECORD_MIC2_MONO_DIFF_WITHOUT_BIAS,
		REPLAY_OUT,
		REPLAY_HP_STEREO,
		REPLAY_LINEOUT_MONO,
		REPLAY_BTL,
		BYPASS_MIC1_DIFF_WITH_BIAS_TO_OUT,
		BYPASS_MIC1_DIFF_WITH_BIAS_TO_HP,
		BYPASS_MIC1_DIFF_WITH_BIAS_TO_LINEOUT_MONO,
		BYPASS_MIC1_DIFF_WITH_BIAS_TO_BTL,
		BYPASS_MIC2_DIFF_WITH_BIAS_TO_OUT,
		BYPASS_MIC2_DIFF_WITH_BIAS_TO_HP,
		BYPASS_MIC2_DIFF_WITH_BIAS_TO_LINEOUT_MONO,
		BYPASS_MIC2_DIFF_WITH_BIAS_TO_BTL,
		BYPASS_LINEIN_TO_OUT,
		BYPASS_LINEIN_TO_HP,
		BYPASS_LINEIN_TO_LINEOUT_MONO,
		BYPASS_LINEIN_TO_BTL,
		RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_OUT,
		RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_HP,
		RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_LINEOUT_MONO,
		RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_BTL,
		RECORD_MIC2_MONO_DIFF_WITHOUT_BIAS_REPLAY_LINEOUT_MONO
	};

	char *route_str[] = {
		"ROUTE_ALL_CLEAR",
		"ROUTE_REPLAY_CLEAR",
		"ROUTE_RECORD_CLEAR",
		"RECORD_MIC1_MONO_DIFF_WITH_BIAS",
		"RECORD_MIC1_MONO_DIFF_WITHOUT_BIAS",
		"RECORD_MIC2_MONO_DIFF_WITH_BIAS",
		"RECORD_MIC2_MONO_DIFF_WITHOUT_BIAS",
		"REPLAY_OUT",
		"REPLAY_HP_STEREO",
		"REPLAY_LINEOUT_MONO",
		"REPLAY_BTL",
		"BYPASS_MIC1_DIFF_WITH_BIAS_TO_OUT",
		"BYPASS_MIC1_DIFF_WITH_BIAS_TO_HP",
		"BYPASS_MIC1_DIFF_WITH_BIAS_TO_LINEOUT_MONO",
		"BYPASS_MIC1_DIFF_WITH_BIAS_TO_BTL",
		"BYPASS_MIC2_DIFF_WITH_BIAS_TO_OUT",
		"BYPASS_MIC2_DIFF_WITH_BIAS_TO_HP",
		"BYPASS_MIC2_DIFF_WITH_BIAS_TO_LINEOUT_MONO",
		"BYPASS_MIC2_DIFF_WITH_BIAS_TO_BTL",
		"BYPASS_LINEIN_TO_OUT",
		"BYPASS_LINEIN_TO_HP",
		"BYPASS_LINEIN_TO_LINEOUT_MONO",
		"BYPASS_LINEIN_TO_BTL",
		"RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_OUT",
		"RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_HP",
		"RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_LINEOUT_MONO",
		"RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_BTL",
		"RECORD_MIC2_MONO_DIFF_WITHOUT_BIAS_REPLAY_LINEOUT_MONO",
	};

	for ( i = 0; i < sizeof(route_arr) / sizeof(unsigned int); i++) {
		if (route_arr[i] == route) {
			printk("\nCODEC SET ROUTE: Route name : %s\n", route_str[i]);
			return;
		}
	}

	if (i == sizeof(route_arr) / sizeof(unsigned int)) {
		printk("\nCODEC SET ROUTE: Route is not configed yet!\n");
	}
}
#endif //DLV_DUMP_ROUTE_NAME

/*=========================================================*/

#if DLV_DUMP_ROUTE_NAME
#define DUMP_ROUTE_NAME(route) dlv_print_route_name(route)
#else //DLV_DUMP_ROUTE_NAME
#define DUMP_ROUTE_NAME(route)
#endif //DLV_DUMP_ROUTE_NAME

/*-------------------*/
#if DLV_DUMP_IOC_CMD 
#define DUMP_IOC_CMD()								\
	do {									\
		printk("[dlv IOCTL]++++++++++++++++++++++++++++\n");		\
		printk("%s  cmd = %d, arg = %lu\n", __func__, cmd, arg); 	\
		dlv_print_ioc_cmd(cmd);						\
		printk("[dlv IOCTL]----------------------------\n");		\
										\
	} while (0)
#else //DLV_DUMP_IOC_CMD
#define DUMP_IOC_CMD()	
#endif //DLV_DUMP_IOC_CMD

#if DLV_DUMP_ROUTE_REGS
#define DUMP_ROUTE_REGS(value)							\
	do {									\
		printk("codec register dump,%s\tline:%d-----%s:\n",		\
		       __func__, __LINE__, value);				\
		dump_dlv_regs();						\
										\
	} while (0)
#else //DLV_DUMP_ROUTE_REGS
#define DUMP_ROUTE_REGS(value)
#endif //DLV_DUMP_ROUTE_REGS

#if DLV_DUMP_ROUTE_PART_REGS
#define DUMP_ROUTE_PART_REGS(value)						\
	do {									\
		if (mode != DISABLE) {						\
			printk("codec register dump,%s\tline:%d-----%s:\n", 	\
			       __func__, __LINE__, value);			\
			dump_dlv_route_regs();					\
		}								\
										\
	} while (0)
#else //DLV_DUMP_ROUTE_PART_REGS
#define DUMP_ROUTE_PART_REGS(value)
#endif //DLV_DUMP_ROUTE_PART_REGS

#if DLV_DUMP_GAIN_PART_REGS
#define DUMP_GAIN_PART_REGS(value)						\
	do {									\
		printk("codec register dump,%s\tline:%d-----%s:\n", 		\
		       __func__, __LINE__, value);				\
		dump_dlv_gain_regs();						\
										\
	} while (0)
#else //DLV_DUMP_GAIN_PART_REGS
#define DUMP_GAIN_PART_REGS(value)
#endif //DLV_DUMP_GAIN_PART_REGS

/***************************************************************************************\
 *                                                                                     *
 *route part and attibute                                                              *
 *                                                                                     *
\***************************************************************************************/
/*=========================power on==========================*/
static void dlv_set_route_ready(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");


	DUMP_ROUTE_PART_REGS("leave");
}

/*=================route part functions======================*/

static void dlv_set_mic1(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case MIC1_DIFF_WITH_MICBIAS:

		break;

	case MIC1_DIFF_WITHOUT_MICBIAS:

		break;

	case MIC1_SING_WITH_MICBIAS:

		break;

	case MIC1_SING_WITHOUT_MICBIAS:

		break;

	case MIC1_DISABLE:
	
		break;

	default:
		printk("TAS5713_DLV: line: %d, mic1 mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_mic2(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case MIC2_DIFF_WITH_MICBIAS:
		
		break;

	case MIC2_DIFF_WITHOUT_MICBIAS:
	
		break;

	case MIC2_SING_WITH_MICBIAS:
	
		break;

	case MIC2_SING_WITHOUT_MICBIAS:
	
		break;

	case MIC2_DISABLE:
	
		break;

	default:
		printk("TAS5713_DLV: line: %d, mic2 mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_linein(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){
		
	case LINEIN_WITHOUT_BYPASS:

		break;

	case LINEIN_WITH_BYPASS:
		
		break;

	case LINEIN_DISABLE:
		
		break;
		
	default:
		printk("TAS5713_DLV: line: %d, linein mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_agc(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case AGC_ENABLE:

		break;
		
	case AGC_DISABLE:

		break;

	default:
		printk("TAS5713_DLV: line: %d, agc mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_record_mux(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case RECORD_MUX_MIC1_TO_LR:

		break;

	case RECORD_MUX_MIC2_TO_LR:
		break;

	case RECORD_MUX_MIC1_TO_R_MIC2_TO_L:

		break;

	case RECORD_MUX_MIC2_TO_R_MIC1_TO_L:

		break;

	case RECORD_MUX_LINE_IN:
		break;

	default:
		printk("TAS5713_DLV: line: %d, record mux mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_adc(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case ADC_STEREO:

		break;

	case ADC_MONO:
		break;

	case ADC_DISABLE:

		break;

	default:
		printk("TAS5713_DLV: line: %d, adc mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_record_mixer(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case RECORD_MIXER_MIX1_INPUT_ONLY:
		break;

	case RECORD_MIXER_MIX1_INPUT_AND_DAC:
		break;

	default:
		printk("TAS5713_DLV: line: %d, record mixer mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_replay_mixer(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case REPLAY_MIXER_PLAYBACK_DAC_ONLY:
		break;

	case REPLAY_MIXER_PLAYBACK_DAC_AND_ADC:
		break;

	default:
		printk("TAS5713_DLV: line: %d, replay mixer mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_dac(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case DAC_STEREO:
		break;

	case DAC_MONO:
		break;

	case DAC_DISABLE:
		break;

	default:
		printk("TAS5713_DLV: line: %d, dac mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_replay_filter(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case REPLAY_FILTER_STEREO:
		break;

	case REPLAY_FILTER_MONO:
		break;

	default:
		printk("TAS5713_DLV: line: %d, replay filter mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_replay_mux(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case REPLAY_MUX_MIC1_TO_LR:
		break;

	case REPLAY_MUX_MIC2_TO_LR:
		break;

	case REPLAY_MUX_MIC1_TO_R_MIC2_TO_L:
		break;

	case REPLAY_MUX_MIC2_TO_R_MIC1_TO_L:
		break;

	case REPLAY_MUX_BYPASS_PATH:
		break;

	case REPLAY_MUX_DAC_OUTPUT:
		break;

	default:
		printk("TAS5713_DLV: line: %d, replay mux mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_hp(int mode)
{
//	int load_flag = 0;

	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case HP_ENABLE:

		break;

	case HP_DISABLE:

		break;

	default:
		printk("TAS5713_DLV: line: %d, hp mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_lineout(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case LINEOUT_STEREO:
		break;

	case LINEOUT_MONO:
		break;

	case LINEOUT_DISABLE:
		break;

	default:
		printk("TAS5713_DLV: line: %d, lineout mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_btl(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case BTL_ENABLE:
		break;

	case BTL_DISABLE:
		break;

	default:
		printk("TAS5713_DLV: line: %d, btl mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

/*=================route attibute(gain) functions======================*/

static void dlv_set_gain_mic1(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_mic2(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_linein_left(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_linein_right(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");


	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_adc_left(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_adc_right(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");


	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_record_mixer(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");


	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_replay_mixer(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_dac_left(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_dac_right(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_hp_left(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_hp_right(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");


	DUMP_GAIN_PART_REGS("leave");
}
/***************************************************************************************\
 *                                                                                     *
 *dlv route        This is first designed for jz47xx internel codec route, other outside codecs can use it freely as you need. Here retains the route interface for future.
 *                                                                                     *
\***************************************************************************************/

static void dlv_set_route_base(const void *arg)
{
	route_conf_base *conf = (route_conf_base *)arg;

	/*codec turn on sb and sb_sleep*/
	if (conf->route_ready_mode)
		dlv_set_route_ready(conf->route_ready_mode);

	/*--------------route---------------*/
	/* record path */
	if (conf->route_mic1_mode)
		dlv_set_mic1(conf->route_mic1_mode);

	if (conf->route_mic2_mode)
		dlv_set_mic2(conf->route_mic2_mode);

	if (conf->route_linein_mode)
		dlv_set_linein(conf->route_linein_mode);

	if (conf->route_record_mux_mode)
		dlv_set_record_mux(conf->route_record_mux_mode);

	if (conf->route_adc_mode)
		dlv_set_adc(conf->route_adc_mode);

	if (conf->route_record_mixer_mode)
		dlv_set_record_mixer(conf->route_record_mixer_mode);

	/* replay path */
	if (conf->route_replay_mixer_mode)
		dlv_set_replay_mixer(conf->route_replay_mixer_mode);

	if (conf->route_replay_filter_mode)
		dlv_set_replay_filter(conf->route_replay_filter_mode);

	if (conf->route_dac_mode)
		dlv_set_dac(conf->route_dac_mode);

	if (conf->route_replay_mux_mode)
		dlv_set_replay_mux(conf->route_replay_mux_mode);

	if (conf->route_hp_mode)
		dlv_set_hp(conf->route_hp_mode);

	if (conf->route_lineout_mode)
		dlv_set_lineout(conf->route_lineout_mode);

	if (conf->route_btl_mode)
		dlv_set_btl(conf->route_btl_mode);

	/*----------------attibute-------------*/
	/* auto gain */
	if (conf->attibute_agc_mode)
		dlv_set_agc(conf->attibute_agc_mode);

	/* gain , use 32 instead of 0 */
	if (conf->attibute_mic1_gain) {
		if (conf->attibute_mic1_gain == 32)
			dlv_set_gain_mic1(0);
		else 
			dlv_set_gain_mic1(conf->attibute_mic1_gain);
	}

	if (conf->attibute_mic2_gain) {
		if (conf->attibute_mic2_gain == 32)
			dlv_set_gain_mic2(0);
		else
			dlv_set_gain_mic2(conf->attibute_mic2_gain);
	}

	if (conf->attibute_linein_l_gain) {
		if (conf->attibute_linein_l_gain == 32)
			dlv_set_gain_linein_left(0);
		else
			dlv_set_gain_linein_left(conf->attibute_linein_l_gain);
	}

	if (conf->attibute_linein_r_gain) {
		if (conf->attibute_linein_r_gain == 32)
			dlv_set_gain_linein_right(0);
		else 
			dlv_set_gain_linein_right(conf->attibute_linein_r_gain);
	}

	if (conf->attibute_adc_l_gain) {
		if (conf->attibute_adc_l_gain == 32)
			dlv_set_gain_adc_left(0);
		else
			dlv_set_gain_adc_left(conf->attibute_adc_l_gain);
	}

	if (conf->attibute_adc_r_gain) {
		if (conf->attibute_adc_r_gain == 32)
			dlv_set_gain_adc_right(0);
		else 
			dlv_set_gain_adc_right(conf->attibute_adc_r_gain);
	}

	if (conf->attibute_record_mixer_gain) {
		if (conf->attibute_record_mixer_gain == 32)
			dlv_set_gain_record_mixer(0);
		else 
			dlv_set_gain_record_mixer(conf->attibute_record_mixer_gain);
	}

	if (conf->attibute_replay_mixer_gain) {
		if (conf->attibute_replay_mixer_gain == 32)
			dlv_set_gain_replay_mixer(0);
		else 
			dlv_set_gain_replay_mixer(conf->attibute_replay_mixer_gain);
	}

	if (conf->attibute_dac_l_gain) {
		if (conf->attibute_dac_l_gain == 32)
			dlv_set_gain_dac_left(0);
		else
			dlv_set_gain_dac_left(conf->attibute_dac_l_gain);
	}
	
	if (conf->attibute_dac_r_gain) {
		if (conf->attibute_dac_r_gain == 32)
			dlv_set_gain_dac_right(0);
		else
			dlv_set_gain_dac_right(conf->attibute_dac_r_gain);
	}

	if (conf->attibute_hp_l_gain) {
		if (conf->attibute_hp_l_gain == 32)
			dlv_set_gain_hp_left(0);
		else
			dlv_set_gain_hp_left(conf->attibute_hp_l_gain);
	}

	if (conf->attibute_hp_r_gain) {
		if (conf->attibute_hp_r_gain == 32)
			dlv_set_gain_hp_right(0);
		else
			dlv_set_gain_hp_right(conf->attibute_hp_r_gain);
	}
}

/***************************************************************************************\
 *ioctl support function                                                               *
\***************************************************************************************/

/*------------------sub fun-------------------*/

/**
 * CODEC set gpio before set route and dlv set gpio after set route
 *
 * these two function below is just demo frames, they should be realized 
 * depend on difficent boards, they should not be modifiy here
 *
 **/

static int dlv_set_gpio_before_set_route(int route)
{
	switch(route){

	case ROUTE_ALL_CLEAR:
		break;

	case RECORD_MIC1_MONO_DIFF_WITH_BIAS:
		break;

	case REPLAY_HP_STEREO:
		break;

	default:
		printk("%s: dlv set route gpio error!, undecleard route\n", __func__);
	}
	
	return 0;
}

static int dlv_set_gpio_after_set_route(int route)
{
	switch(route){

	case ROUTE_ALL_CLEAR:
		break;

	case RECORD_MIC1_MONO_DIFF_WITH_BIAS:
		break;

	case REPLAY_HP_STEREO:
		break;
		
	default:
		printk("%s: dlv set route gpio error!, undecleard route\n", __func__);
	}

	return 0;
}

/*-----------------main fun-------------------*/

int dlv_set_route(int route)
{
	int i = 0;

	DUMP_ROUTE_REGS("enter");

	/* set gpio befor set route */
	dlv_set_gpio_before_set_route(route);

	/* set route */
	DUMP_ROUTE_NAME(route);

	if(cur_route != route)
	{
		for (i = 0; i < ROUTE_COUNT; i ++)
		{
			if (route == tas5707_dlv_route_info[i].route_name)
			{
				/* set route */
				dlv_set_route_base(tas5707_dlv_route_info[i].route_conf);
				/* keep_old_route is used in resume part */
				keep_old_route = cur_route;
				/* change cur_route */
				cur_route = route;
				break;
			}
		}
		if (i == ROUTE_COUNT)
			printk("SET_ROUTE: dlv set route error!, undecleard route, route = %d\n", route);
	} else 
		printk("SET_ROUTE: need not to set!, current route is route now!\n");
	
	/* set gpio after set route */
	dlv_set_gpio_after_set_route(route);

	DUMP_ROUTE_REGS("leave");

	return cur_route;
}

/*----------------------------------------*/
/****** dlv_init ********/
/**
 * CODEC dlv init part
 *
 * it will do the initialization as default, it can be recode 
 * depend on difficent boards if necessary
 *
 **/

static int dlv_init_part(void)
{
	return 0;
}

static int dlv_init(void)
{
	int ret;

	DLV_LOCKINIT();

	/* set default route */
	g_current_out_dev = DEFAULT_REPLAY_ROUTE;

	/* dlv init */
	ret = dlv_init_part();

	return ret;
}

/****** dlv_turn_off ********/
/**
 * CODEC dlv turn off part
 *
 * it will turn off the codec by modes as default, it can be recode 
 * depend on difficent boards if necessary
 *
 **/
static int dlv_turn_off_part(int mode)
{
	int ret;
	int route = keep_old_route;

	printk("TAS5713_DLV: waring, %s() is a default function\n", __func__);

	if ((mode & REPLAY) && (mode & RECORD)) {
		printk("TAS5713 DLV: Close REPLAY & RECORD\n");
		tas5713_shutdown();

		ret = dlv_set_route(ROUTE_ALL_CLEAR);
		if(ret != ROUTE_ALL_CLEAR)
		{
			printk("TAS5713 CODEC: dlv_turn_off_part replay & record mode error!\n");
			return -1;
		}
	} else if (mode & REPLAY) {
		printk("TAS5713 DLV: Close REPLAY\n");
		tas5713_shutdown();

		ret = dlv_set_route(ROUTE_REPLAY_CLEAR);
		if(ret != ROUTE_REPLAY_CLEAR)
		{
			printk("TAS5713 CODEC: dlv_turn_off_part replay mode error!\n");
			return -1;
		}
	} else if (mode & RECORD) {
		printk("TAS5713 DLV: Close RECORD\n");
		tas5713_shutdown();
		ret = dlv_set_route(ROUTE_RECORD_CLEAR);
		if(ret != ROUTE_RECORD_CLEAR)
		{
			printk("TAS5713 CODEC: dlv_turn_off_part record mode error!\n");
			return -1;
		}
		
		ret = dlv_set_route(route);
		if(ret != route)
		{
			printk("TAS5713 CODEC: %s record mode error!\n", __func__);
			return -1;
		}
	}

	return 0;
}

static int dlv_turn_off(int mode)
{
	int ret;

	ret = dlv_turn_off_part(mode);

	DLV_LOCKDEINIT();  

	return ret;
}

/****** dlv_shutdown *******/
/**
 * CODEC dlv shutdown part
 *
 * it will shutdown the gpio when system shutdown,
 * it can be recode depend on difficent boards if necessary
 *
 **/

static int dlv_shutdown_part(void)
{
	return 0;
}

static int dlv_shutdown(void)
{
	int ret;
		
	ret = dlv_shutdown_part();

	return ret;
}

/****** dlv_reset **********/
/**
 * CODEC dlv reset part
 *
 * it will run to set the codec when codec power on as default,
 * it can be recode depend on difficent boards if necessary
 *
 **/
static int dlv_reset_part(void)
{
	return 0;
}

static int dlv_reset(void)
{
	int ret;

	ret = dlv_reset_part();

	return ret;
}

/******** dlv_anti_pop ********/
/**
 * CODEC dlv anti pop part
 *
 * it will be used as default, it can be recode 
 * depend on difficent boards if necessary
 *
 **/
static int dlv_anti_pop_part(void)
{
	return 0;
}

static int dlv_anti_pop(int mode)
{
	int ret = 0;

	switch(mode) {
	case CODEC_WRMODE:
		break;
	case CODEC_RMODE:
		break;
	case CODEC_WMODE:
		ret = dlv_anti_pop_part();
		break;
	}

	return ret;
}

/******** dlv_suspend ************/
/**
 * CODEC dlv suspend part
 *
 * it will do the suspend as default, it can be recode 
 * depend on difficent boards if necessary
 *
 **/
static int dlv_suspend_part(void)
{
	int ret;

	ret = dlv_set_route(ROUTE_ALL_CLEAR);
	if(ret != ROUTE_ALL_CLEAR)
	{
		printk("TAS5713 CODEC: dlv_suspend_part error!\n");
		return -1;
	}

	return 0;
}

static int dlv_suspend(void)
{
	int ret;
	
	ret = dlv_suspend_part();

	return ret;
}

/********* dlv_resume ***********/
/**
 * CODEC dlv resume part
 *
 * it will do the resume as default, it can be recode 
 * depend on difficent boards if necessary
 *
 **/

static int dlv_resume_part(void)
{
	int ret;
	int route = keep_old_route;

	/*default, the resume will restore the route before suspend*/
	ret = dlv_set_route(route);

	if(ret != route)
	{
		printk("TAS5713 CODEC: dlv_resume_part error!\n");
		return -1;
	}

	return 0;
}

static int dlv_resume(void)
{
	int ret;
	ret = dlv_resume_part();

	return ret;
}

/*---------------------------------------*/

/**
 * CODEC set device
 *
 * this is just a demo function, and it will be use as default 
 * if it is not realized depend on difficent boards.
 * Here just a unuseful interface for tas5707. 
 *
 */
static int dlv_set_device(struct snd_device_config *snd_dev_cfg)
{
	int ret;
	int iserror = 0;

	printk("TAS5713_DLV: waring, %s() is a default function\n", __func__);

	switch (snd_dev_cfg->device) {

	case SND_DEVICE_HEADSET:
		ret = dlv_set_route(REPLAY_HP_STEREO);
		if(ret != REPLAY_HP_STEREO)
		{
			printk("TAS5713 CODEC: set device SND_DEVICE_HEADSET error!\n");
			return -1;
		}
		break;

	case SND_DEVICE_HANDSET:
		ret = dlv_set_route(REPLAY_LINEOUT_MONO);
		if(ret != REPLAY_LINEOUT_MONO)
		{
			printk("TAS5713 CODEC: set device SND_DEVICE_HANDSET error!\n");
			return -1;
		}
		break;

	case SND_DEVICE_SPEAKER:
		ret = dlv_set_route(REPLAY_BTL);
		if(ret != REPLAY_BTL)
		{
			printk("TAS5713 CODEC: set device SND_DEVICE_SPEAKER error!\n");
			return -1;
		}
		break;

	case SND_DEVICE_HEADSET_AND_SPEAKER:
		ret = dlv_set_route(REPLAY_BTL);
		if(ret != REPLAY_BTL)
		{
			printk("TAS5713 CODEC: set device SND_DEVICE_HEADSET_AND_SPEAKER error!\n");
			return -1;
		}			
		break;

	default:
		iserror = 1;
		printk("TAS5713 DLV: Unkown ioctl argument in SND_SET_DEVICE\n");
	};

	if (!iserror)
		g_current_out_dev = snd_dev_cfg->device;

	return 0;
}

/*---------------------------------------*/

/**
 * CODEC set standby
 *
 * this is just a demo function, and it will be use as default 
 * if it is not realized depend on difficent boards 
 *
 */

static int dlv_set_standby(unsigned int sw)
{
	switch(g_current_out_dev) {

	case SND_DEVICE_HEADSET:
		break;

	case SND_DEVICE_HANDSET:
		break;

	case SND_DEVICE_SPEAKER:
		break;

	case SND_DEVICE_HEADSET_AND_SPEAKER:
		break;

	default:
		printk("TAS5713 DLV: Unkown ioctl argument in SND_SET_STANDBY\n");

	}

	return 0;
}

/*---------------------------------------*/
/**
 * CODEC set record rate & data width & volume & channel, TAS5707 can't support it.It is just interface. You can use it for other outside codecs in the future.  
 *
 */

static int dlv_set_record_rate(int rate)
{
	printk("TAS5707 can't support record\n");
	return rate;
}

static int dlv_set_record_data_width(int width)
{
	printk("TAS5707 can't support record\n");
	return width;
}

static int dlv_set_record_volume(int val)
{
	printk("TAS5707 can't support record\n");
	return val;
}

static int dlv_set_record_channel(int channel)
{
	printk("TAS5707 can't support record\n");
	return channel;
}

/*---------------------------------------*/
/**
 * CODEC set replay rate & data width & volume & channel  
 *
 */

static int dlv_set_replay_rate(int rate)
{
	
	int speed = 0, val;
	int mrate[MAX_RATE_COUNT] = { 
		96000, 48000, 44100, 32000, 24000,
		22050, 16000, 12000, 11025, 8000
	};
	
	for (val = 0; val < MAX_RATE_COUNT; val++) {
		if (rate >= mrate[val]) {
			speed = val;
			break;
		}
	}
	if (rate < mrate[MAX_RATE_COUNT - 1]) {
		speed = MAX_RATE_COUNT - 1;
	}
#if 0
	if( mrate[speed] == 48000 ){
		cpm_set_clock(CGU_I2SCLK, JZ_EXTAL);
	} else if( mrate[speed] == 44100 ){
		cpm_set_clock(CGU_I2SCLK, 11289600);
	}
#else
	if( mrate[speed] == 48000 ){
		cpm_set_clock(CGU_I2SCLK, 12288000);
	} else if( mrate[speed] == 44100 ){
		cpm_set_clock(CGU_I2SCLK, 11289600);
	}
#endif
	
//	REG_AIC_I2SDIV = 0x03;            //add by tjin
	tas5713_check_err_status();
	printk("%s = %d\n",__FUNCTION__, mrate[speed]);

	return mrate[speed];
}

static int dlv_set_replay_data_width(int width)
{
	int supported_width[3] = {16, 20, 24};
	unsigned char reg[3] = {0x03, 0x04, 0x05};
	int fix_width;

	for(fix_width = 0; fix_width < 3; fix_width ++)
	{
		if (width <= supported_width[fix_width])
			break;
	}

	sky_i2c_write_reg(REG_DATA_INTERFACE, &reg[fix_width], 1);	

	return width;
}

static int dlv_set_replay_volume(int vol)
{
#if USE_TAS5713_OLD
	vol = 255 * (100 - vol) / 100;
	tas5713_set_volume(vol & 0xff);
#else
	if(vol < 0)
		vol = 0;
	else if(vol > 30)
		vol = 30;
	
	if(vol == 0){
		DAMP_MuteOn();
	}else{
		DAMP_MuteOff();
	}
	
	mdelay(10);
	DAMP_set_volume(vol);
#endif
	return vol;
}

/* The function is not be use, just a interface for the future. */
static int dlv_set_replay_channel(int channel)
{
	channel = (channel >= 2) + 1;

	switch (channel) {
	case 1:
		// MONO->1 for Mono
		dlv_set_replay_filter(REPLAY_FILTER_MONO);
		break;
	case 2:
		// MONO->0 for Stereo
		dlv_set_replay_filter(REPLAY_FILTER_STEREO);
		break;
	}

	return channel;
}

/*---------------------------------------*/
/**
 * CODEC set mute
 *  
 * set dac mute used for anti pop
 *
 */

#if USE_TAS5713_OLD
static int dlv_mute(int val)
{
	tas5713_set_mute(val);

	return 0;
}
#endif
/*---------------------------------------*/

static int dlv_debug_routine(void *arg)
{
	return 0;
}

/**
 * CODEC ioctl (simulated) routine. It will be called by jz47xx_i2s.c 
 * to configure the codec.
 *
 * Provide control interface for i2s driver
 */
static int jzdlv_ioctl(void *context, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	DUMP_IOC_CMD();

	DLV_LOCK();
	{
		switch (cmd) {

		case CODEC_INIT:
			ret = dlv_init();
			break;

		case CODEC_TURN_OFF:
			ret = dlv_turn_off(arg);
			break;

		case CODEC_SHUTDOWN:
			ret = dlv_shutdown();
			break;

		case CODEC_RESET:
			ret = dlv_reset();
			break;

		case CODEC_SUSPEND:
			ret = dlv_suspend();
			break;

		case CODEC_RESUME:
			ret = dlv_resume();
			break;

		case CODEC_ANTI_POP:
			ret = dlv_anti_pop((int)arg);
			break;

		case CODEC_SET_ROUTE:
			ret = dlv_set_route((int)arg);
			break;

		case CODEC_SET_DEVICE:
			ret = dlv_set_device((struct snd_device_config *)arg);
			break;

		case CODEC_SET_STANDBY:
			ret = dlv_set_standby((unsigned int)arg);
			break;

		case CODEC_SET_RECORD_RATE:
			ret = dlv_set_record_rate((int)arg);
			break;

		case CODEC_SET_RECORD_DATA_WIDTH:
			ret = dlv_set_record_data_width((int)arg);
			break;

		case CODEC_SET_MIC_VOLUME:
			ret = dlv_set_record_volume((int)arg);
			break;

		case CODEC_SET_RECORD_CHANNEL:
			ret = dlv_set_record_channel((int)arg);
			break;

		case CODEC_SET_REPLAY_RATE:
			ret = dlv_set_replay_rate((int)arg);
			break;

		case CODEC_SET_REPLAY_DATA_WIDTH:
			ret = dlv_set_replay_data_width((int)arg);
			break;

		case CODEC_SET_REPLAY_VOLUME:
			ret = dlv_set_replay_volume((int)arg);
	
			break;

		case CODEC_SET_REPLAY_CHANNEL:
			ret = dlv_set_replay_channel((int)arg);
			break;

		case CODEC_DAC_MUTE:
#if USE_TAS5713_OLD
			ret = dlv_mute((int)arg);
#else
			if(arg == 1)
				DAMP_MuteOn();
			else
				DAMP_MuteOff();	
#endif
			break;

		case CODEC_DEBUG_ROUTINE:
			ret = dlv_debug_routine((void *)arg);
			break;

		default:
			printk("TAS5713 DLV:%s:%d: Unkown IOC commond\n", __FUNCTION__, __LINE__);
			ret = -1;
		}
	}
	DLV_UNLOCK();

	return ret;
}

/**
 * Module init
 */
static int __init init_dlv(void)
{
	cpm_start_clock(CGM_AIC);

	printk("=============================================\n");
	
	/* just notice jz47xx_i2s.c the codec's ioctl function */
	//register_jz_codecs((void *)jzdlv_ioctl);    
	register_jz_codecs_ex((void *)jzdlv_ioctl, NULL);  

	/* PND and RESET pin power on */
	tas5713_reset();       

	/* register I2C GPIO driver */
	i2c_add_driver(&tas5707_i2c_driver);   

	/* This is main steps to init the codec */
#if USE_TAS5713_OLD
	tas5713_init_reg();
	tas5713_dump_all();
#else
	DAMP_Init();             
#endif
	return 0;
}

/**
 * Module exit
 */
static void __exit cleanup_dlv(void)
{
	/* unregister I2C GPIO driver */
	i2c_del_driver(&tas5707_i2c_driver);
	return ;
}

module_init(init_dlv);
module_exit(cleanup_dlv);
