/*
 *
 * DLV CODEC driver for Ingenic Jz4760B MIPS processor
 *
 * 2009-12-xx	Steven <dsqiu@ingenic.cn>
 * 2010-01-xx	Jason <xwang@ingenic.cn>
 * 2010-11-xx   jbbi <jbbi@ingenic.cn>
 * 2014-06-01   tjin <tjin@ingenic.cn>
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

#include "damp.c"
#include "damp_filter.c"

#define DUMP_FUNC() printk("TAS5707_DLV:%s\tline:%d\n", __func__, __LINE__)

#define GPIO_TAS5707_RESET  		(32*1 + 16)
#define GPIO_TAS5707_PDN		(32*1 + 8)

//#define GPIO_TAS5707_INPUT_SELECT	(32*5 + 3)
//static int tas5707_dac_flag = 0;

/****************** i2c gpio ****************************************/
#define GPIO_I2C_SDA 32*4+30
#define GPIO_I2C_SCL 32*4+31

/****  I2C bus GPIO interface function ****/
extern void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk);
static struct i2c_client *tas5707_client = NULL;

static int sky_i2c_write_reg(unsigned char reg, unsigned char* data, unsigned int len)
{
	int i,ret =-1;
	unsigned char buf[30] = {0};
        struct i2c_client *client = tas5707_client;

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
        struct i2c_client *client = tas5707_client;
	
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
	
	tas5707_client = client;
	i2c_jz_setclk(client, 100*1000);

	return 0;
}
static int __devexit tas5707_i2c_remove(struct i2c_client *client)
{
	tas5707_client = NULL;
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


/****************** tas5707 api ****************************************/

void tas5707_read_reg(unsigned char reg, unsigned char len)
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
 *  tas5707_reset is a important step,you can read Tas5707 spec to understand it.
 *  i2s_controller_init() function is just to support the I2S clks for tas5707.
 *
 * ***/
//extern void i2s_controller_init(void);      
void tas5707_reset(void)
{
	__gpio_as_output(GPIO_TAS5707_RESET);
	__gpio_as_output(GPIO_TAS5707_PDN);

//       __gpio_as_output(GPIO_TAS5707_INPUT_SELECT);
//       __gpio_clear_pin(GPIO_TAS5707_INPUT_SELECT);	 //select cpu's i2s output
	
	__gpio_clear_pin(GPIO_TAS5707_PDN);
	mdelay(10);
	__gpio_clear_pin(GPIO_TAS5707_RESET);
	mdelay(300);
	printk("\nRestart TAS5707\n");
	__gpio_set_pin(GPIO_TAS5707_PDN);
	mdelay(10);
	
    	//i2s_controller_init();
	//REG_AIC_I2SCR |= AIC_I2SCR_ESCLK;   // start MCLK, this is only a test. 

	__gpio_set_pin(GPIO_TAS5707_RESET);
	mdelay(20);
	return ;
}

int tas5707_check_err_status(void)
{
	unsigned char value = 0;

	sky_i2c_read_reg(REG_ERROR_STATUS, &value, 1);

	if (value) {
		printk("tas5707_check_err_status 0x%02x\n", value);
		value = 0;
		sky_i2c_write_reg(REG_ERROR_STATUS, &value, 1); // clear the status
		return -1;
	}
	return 0;
}

static void tas5707_shutdown(void)
{
	unsigned char value = 0x40;
	printk("tas5707_shutdown\n");
#if 0
	__gpio_clear_pin(GPIO_TAS5707_PDN);   //GPIO shutdown the codec test not good
	mdelay(10);
#else
	sky_i2c_write_reg(REG_CTRL_2, &value,1);
#endif
	mdelay(150);
	return ;
}

static void tas5707_wakeup(void)
{
	unsigned char value = 0x00;
	printk("tas5707_wakeup\n");
#if 0
	__gpio_set_pin(GPIO_TAS5707_PDN);      //GPIO shutdown the codec test not good
	mdelay(10);
#else
	sky_i2c_write_reg(REG_CTRL_2, &value,1);
#endif
	mdelay(150);
	return ;
}

/*
int tas5707_input_select(int input)
{
	printk(KERN_DEBUG "tas5707_input_select %s !\n", input?"adc line in":"local play");
	__gpio_as_output(GPIO_TAS5707_INPUT_SELECT);
	
	if (!input) {
		tas5707_dac_flag = 0;
		__gpio_clear_pin(GPIO_TAS5707_INPUT_SELECT);  //select cpu  i2s output
		tas5713_set_mute(1); // mute the tas5713 output if tvout
		
	} else {
		tas5707_dac_flag = 1;
		__gpio_set_pin(GPIO_TAS5707_INPUT_SELECT);  //select pcm1808 adc  i2s output
		tas5713_set_mute(0); // unmute
	}
	return 0;
}
*/

void tas5707_dump_all(void)
{
	tas5707_read_reg(0x00, 1);
	tas5707_read_reg(0x01, 1);
	tas5707_read_reg(0x02, 1);
	tas5707_read_reg(0x03, 1);
	tas5707_read_reg(0x04, 1);
	tas5707_read_reg(0x05, 1);
	tas5707_read_reg(0x06, 1);
	tas5707_read_reg(0x07, 1);
	tas5707_read_reg(0x08, 1);
	tas5707_read_reg(0x09, 1);
	tas5707_read_reg(0x0A, 1);
	tas5707_read_reg(0x0E, 1);
	tas5707_read_reg(0x10, 1);
	tas5707_read_reg(0x11, 1);
	tas5707_read_reg(0x12, 1);
	tas5707_read_reg(0x13, 1);
	tas5707_read_reg(0x14, 1);
	tas5707_read_reg(0x1A, 1);
	tas5707_read_reg(0x1B, 1);
	tas5707_read_reg(0x1C, 1);
	tas5707_read_reg(0x20, 4);
	tas5707_read_reg(0x25, 4);
	tas5707_read_reg(0x29, 20);
	tas5707_read_reg(0x2A, 20);
	tas5707_read_reg(0x2B, 20);
	tas5707_read_reg(0x2C, 20);
	tas5707_read_reg(0x2D, 20);
	tas5707_read_reg(0x2E, 20);
	tas5707_read_reg(0x2F, 20);
	tas5707_read_reg(0x30, 20);
	tas5707_read_reg(0x31, 20);
	tas5707_read_reg(0x32, 20);
	tas5707_read_reg(0x33, 20);
	tas5707_read_reg(0x34, 20);
	tas5707_read_reg(0x35, 20);
	tas5707_read_reg(0x36, 20);
	tas5707_read_reg(0x3A, 8);
	tas5707_read_reg(0x3B, 8);
	tas5707_read_reg(0x3C, 8);
	tas5707_read_reg(0x40, 4);
	tas5707_read_reg(0x41, 4);
	tas5707_read_reg(0x42, 4);
	tas5707_read_reg(0x46, 4);
	tas5707_read_reg(0x50, 4);
	tas5707_read_reg(0xF9, 4);
}

/***************************************************************************************\
 *global variable and structure interface                                              *
\***************************************************************************************/

static unsigned int g_current_out_dev;

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
			init_MUTEX(g_dlv_sem);				\
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
/*=========================================================*/

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

/***************************************************************************************\
 *                                                                                     *
 *dlv route        This is first designed for jz47xx internel codec route, other outside codecs can use it freely as you need. Here retains the route interface for future.
 *                                                                                     *
\***************************************************************************************/

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

	/* set default route */
	g_current_out_dev = REPLAY_HP_STEREO;

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
	printk("TAS5707_DLV: waring, %s() is a default function\n", __func__);

	if ((mode & REPLAY) && (mode & RECORD)) {
		printk("TAS5707 DLV: Close REPLAY & RECORD\n");
		//tas5707_shutdown();

	} else if (mode & REPLAY) {
		printk("TAS5707 DLV: Close REPLAY\n");
		//tas5707_shutdown();

	} else if (mode & RECORD) {
		printk("TAS5707 DLV: Close RECORD\n");
		//tas5707_shutdown();
	}

	return 0;
}

static int dlv_turn_off(int mode)
{
	int ret;

	ret = dlv_turn_off_part(mode);

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
	//tas5707_wakeup();
	return 0;
}

static int dlv_anti_pop(int mode)
{
	int ret = 0;

	switch(mode) {
	case CODEC_WRMODE:
	case CODEC_RMODE:
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
	tas5707_shutdown();	
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
	tas5707_wakeup();
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
 * This is just a demo function, and it will be use as default. 
 * Wether it need to be realized, depends on difficent boards.
 * Here just a unuseful interface for tas5707. 
 *
 */
static int dlv_set_device(struct snd_device_config *snd_dev_cfg)
{
	int iserror = 0;

	printk("TAS5707_DLV: waring, %s() is a default function\n", __func__);

	switch (snd_dev_cfg->device) {

	case SND_DEVICE_HEADSET:
		break;

	case SND_DEVICE_HANDSET:
		break;

	case SND_DEVICE_SPEAKER:
		break;

	case SND_DEVICE_HEADSET_AND_SPEAKER:
		break;

	default:
		iserror = 1;
		printk("TAS5707 DLV: Unkown ioctl argument in SND_SET_DEVICE\n");
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
		printk("TAS5707 DLV: Unkown ioctl argument in SND_SET_STANDBY\n");

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

	if( mrate[speed] == 48000 ){
		cpm_set_clock(CGU_I2SCLK, 12288000);
	} else if( mrate[speed] == 44100 ){
		cpm_set_clock(CGU_I2SCLK, 11289600);
	}else{
		printk("The sample rate can't support\n");
		return -1;
	}
	tas5707_check_err_status();
	printk("%s  %d\n",__FUNCTION__, mrate[speed]);

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

	sky_i2c_write_reg(REG_I2S_FMT, &reg[fix_width], 1);	

	return width;
}

static int dlv_set_replay_volume(int vol)
{
	if(vol < 0)
		vol = 0;
	else if(vol > 100)
		vol = 100;
	
	if(vol == 0){
		DAMP_MuteOn();
	}else{
		DAMP_MuteOff();
	}
	
	mdelay(10);
	DAMP_set_volume(30 * vol / 100);
	return vol;
}

/* The function is not be use, just a interface for the future. */
static int dlv_set_replay_channel(int channel)
{
	channel = (channel >= 2) + 1;

	switch (channel) {
	case 1:
		// MONO->1 for Mono
		break;
	case 2:
		// MONO->0 for Stereo
		break;
	}

	return channel;
}

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
			ret = (int)arg;
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
/*  
			if(arg == 1)
				DAMP_MuteOn();
			else
				DAMP_MuteOff();
*/
			break;

		case CODEC_DEBUG_ROUTINE:
			ret = dlv_debug_routine((void *)arg);
			break;

		default:
			printk("TAS5707 DLV:%s:%d: Unkown IOC commond\n", __FUNCTION__, __LINE__);
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
	
	DLV_LOCKINIT();
	/* just notice jz47xx_i2s.c the codec's ioctl function */
	register_jz_codecs_ex((void *)jzdlv_ioctl, NULL);  

	/* PND and RESET pin power on */
	tas5707_reset();       

	/* register I2C GPIO driver */
	i2c_add_driver(&tas5707_i2c_driver);   

	/* This is main steps to init the codec */
	DAMP_Init();   
         
	/* Printk the codec's registers */ 
	//tas5707_dump_all();
	return 0;
}

/**
 * Module exit
 */
static void __exit cleanup_dlv(void)
{
	/* unregister I2C GPIO driver */
	i2c_del_driver(&tas5707_i2c_driver);
	
	DLV_LOCKDEINIT();
	return ;
}

module_init(init_dlv);
module_exit(cleanup_dlv);
