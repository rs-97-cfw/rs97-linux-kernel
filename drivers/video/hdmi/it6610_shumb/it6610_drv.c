 /*
 * linux/drivers/video/hdmi/it6610/it6610_drv.c
 *
 * HDMI code for Ingenic JZ SOC.
 *
 *
 * Copyright (C) 2008 Marek Vasut <marek.vasut@gmail.com>
 * Copyright (C) 2011.08   Shumb <sbhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/interrupt.h>
//#include <linux/earlysuspend.h>
//wjx #include <linux/jz_battery.h>
#include <asm/jzsoc.h>
//wjx #include <asm/jzpm/jz_act8930.h>
#include <linux/jz_hdmi.h>
#include "hdmitx.h"


//#define CONFIG_HDMI_DIRECT_MODE		 1


#define DEBUG 1
#if DEBUG
#define dprintk(format, arg...) printk(format , ## arg)
#else
#define dprintk(format, arg...) do { } while(0) 
#endif


#define INIT_OK					1
#define INIT_NO  				0

#define DRIVER_NAME						"hdmi"
#define HDMI_DEVLOOP_TIMES				10
#define HDMI_DEVLOOP_DELAY				20
#define HDMI_INIT_MS					50
#define TIMER_CNT_MAX 					50
#define HDMI_HOTPLUG_ENABLE				1
#define HDMI_HOTPLUG_DISABLE			0	

#define RETURN_POWEROFF_CTRL			1
#define RETURN_POWERON_CTRL				2



static unsigned int init_state=INIT_NO;
static unsigned int ite6610_rate = 200; /* ite6610 scan rate(ms)*/
static int g_timer_cnt = 0;
static int g_delay_mode = 0;
struct timer_list LoopProc_timer;
static HDMI_Video_Type VideoMode = HDMI_480p60;
static HDMI_OutputColorMode OutputColorMode = HDMI_YUV444 ;//HDMI_RGB444;
/*
#ifdef CONFIG_JZ4760B_LYNX	
#define LCD_SIGNAL_DRIVING_STRENGTHEN
#endif

#ifdef LCD_SIGNAL_DRIVING_STRENGTHEN
#define     EP932M_5VEN_PIN (32*5+6)
int ep932m_5ven	= 1;
#endif
*/

//#define HDMI_RST_N_PIN  (32*4 + 6)
static unsigned int HDMI_RST_N_PIN;

//struct hdmi_it6610_board_info *hdmi_board_info;
extern void IT6610I2cPinInit(struct hdmi_it6610_pin_info *pin_info);


char *hdmi_it6610_pin_des[]={
	"PIN_SYSRSTN",
	"PIN_INT",
	"PIN_PCSCL",
	"PIN_PCSDA"
	"PIN_MCLK",
	"PIN_SCK",
	"PIN_WS",
	"PIN_I2S0",
	"PIN_SPDIF",
	"PIN_PCLK",
	"PIN_VSYNC",
	"PIN_HSYNC",
	"PIN_DE",
};

/************************************************/
typedef enum HDMI_IOCTL_CMD{
	HDMI_POWER_ON	= 3,
	HDMI_POWER_OFF	,
	HDMI_INFO_GET	,
	HDMI_HOTPLUG_CTRL,
	HDMI_MODE_GET,
	HDMI_MODE_SET,	
	HDMI_STANDBY,	
	HDMI_HOTPLUGABLE,
	HDMI_HOTPLUG_TRIGGER,
	HDMI_AUDIO_SAMPLE_ADJUST,
} HDMI_IOCTL_CMD_T;
typedef enum HDMI_SCAN_TYPE{
	HDMI_POLLING = 1,
	HDMI_HOTPLUG = 2,
	HDMI_DIRECT  = 3,
} HDMI_SCAN_TYPE_T;
typedef enum HMDI_STATUS{
	HDMI_STAT_STANDBY = 0,
	HDMI_STAT_WORKING = 1,
	HDMI_STAT_DISABLED = 2,
} HMDI_STATUS_T;
typedef enum HMDI_HDP_STATUS{
	HDMI_DISCONNECT = 0,
	HDMI_CONNECT = 1,
} HMDI_HDP_STATUS_T;

typedef enum HMDI_POWER{
	POWER_OFF = 0,				
	POWER_ON,	
	POWER_INVAILD,
} HMDI_POWER_T;

static struct workqueue_struct *workqueue;

struct hdmi_mode{
HDMI_SCAN_TYPE_T 	scan_type;
unsigned int 		output_type;
HMDI_POWER_T 	 	power;
};
struct hdmi_info{
HMDI_STATUS_T 		hdmi_status;
HMDI_HDP_STATUS_T  	HPD_status;
struct hdmi_mode mode;
};

struct hdmi_host{
struct device	dev;
struct class 	*class;
struct delayed_work	detect;
struct workqueue_struct *detect_workqueue;
struct hdmi_it6610_board_info *board_info;
struct hdmi_info hdmi_info;
#ifdef CONFIG_HAS_EARLYSUSPEND
struct early_suspend early_suspend;
#endif
unsigned int irq;
unsigned int hotplug_enable;
unsigned int hotplug_support;
unsigned int is_probe;
unsigned int flag;
unsigned int is_suspended;
};

struct hdmi_host * phdmi_host = NULL;

static struct miscdevice it6610_miscdev;
static int is_hdmi_dev_reg;
static int is_polling_always;
static int hdmi_earlysuspended_check = 0;
/************************************************/
int change_delay_mode(int mode)
{
	g_delay_mode = mode;
	return 0;
}
void sDelayMS(unsigned int ms)
{
	if(g_delay_mode) DelayMS(ms);
	else	msleep(ms);
}
int hdmi_check_invalid_pins(struct hdmi_it6610_pin_info *pboard_pin)
{
	unsigned int *ppin;
	int i,all_pins;

#ifndef  INVALID_PIN
//#error "Please defined INVALID_PIN in Board Head file first!!!"
#define INVALID_PIN   0
#endif
	dprintk("Check invaild pin !\n");
	ppin = (unsigned int *)pboard_pin;
	all_pins = sizeof(struct hdmi_it6610_pin_info)/sizeof(unsigned int);
	for(i=0;i< all_pins;i++){
		if(*ppin == INVALID_PIN){
			printk("WARNNING:%s is NULL!\n",hdmi_it6610_pin_des[i]);
		}
	}
	
	if(!pboard_pin->PIN_PCSCL)
		goto chkfail;
	if(!pboard_pin->PIN_PCSDA)
		goto chkfail;

	return 0;
	
chkfail:
	printk("HDMI Pins ERROR!(%s)\n",__FUNCTION__);
	return -1;
	
}
void hdmi_pin_init(struct hdmi_it6610_board_info *pboard_info)
{
	unsigned int pin;
	IT6610I2cPinInit(pboard_info->pin_info);
	HDMI_RST_N_PIN = pboard_info->pin_info->PIN_SYSRSTN;
	printk("HDMI reset pin :(32 * %d + %d)\n",HDMI_RST_N_PIN/32,HDMI_RST_N_PIN%32);
	
	pin = pboard_info->pin_info->PIN_HPD;
	printk("HDMI hotplug pin :(32 * %d + %d)\n",pin/32,pin%32);
	if(!pin) return ;
	if(pboard_info->hpd_connect_active)
		__gpio_disable_pull(pin);
	else
		__gpio_enable_pull(pin);
	gpio_as_input(pin);
	printk("HDMI hotplug Level: %d ,connect_active(%d)\n",gpio_get_pin(pin),pboard_info->hpd_connect_active);
}
void hdmi_reset(void)
{
    gpio_set_pin(HDMI_RST_N_PIN);
    gpio_clear_pin(HDMI_RST_N_PIN);
    sDelayMS(30);
                   
    printk("GPIO LVL:%d \n",gpio_get_pin(HDMI_RST_N_PIN));                    
}
static void HdmiStandy(void)
{
	gpio_set_pin(HDMI_RST_N_PIN);
	HDMITX_Standby(2);
	gpio_clear_pin(HDMI_RST_N_PIN);
}

static void hdmi_real_poweroff(void)
{
	if(phdmi_host->board_info->hdmi_power_off)
		phdmi_host->board_info->hdmi_power_off();
}

static void hdmi_power_enable(void)
{

	if(phdmi_host->board_info->lcd_signal_driving_strengthen_up)
		phdmi_host->board_info->lcd_signal_driving_strengthen_up();

	if(phdmi_host->board_info->hdmi_power_on)
		phdmi_host->board_info->hdmi_power_on();

    sDelayMS(30);
}
static void hdmi_power_disable(void)
{

	if(phdmi_host->board_info->lcd_signal_driving_strengthen_down)
		phdmi_host->board_info->lcd_signal_driving_strengthen_down();

//	if(phdmi_host->hdmi_info.mode.scan_type != HDMI_POLLING)
	HdmiStandy();
#ifndef CONFIG_HDMI_IT6610
	hdmi_real_poweroff();
#endif
	
}
#if 0
static int hdmi_audio_sync(void)
{
	int i,err=-1;


	IT6610I2cInit();
	for(i = 0; i< HDMI_DEVLOOP_TIMES; i++){
		err= HDMITX_DevLoopProc();
		printk("----hdmi_audio_sync  err=%d\n",err);
		if( !err ) break;
		sDelayMS(ite6610_rate);
	}
	IT6610I2cDeinit();
	if(i >= HDMI_DEVLOOP_TIMES)
		printk("HDMI Audio sync timeout!!!\n");


	return err;
}
#endif
static void LoopProc_timer_finish(void)
{
	dprintk("---%d:%s, timer_cnt=%d \n",__LINE__,__func__,g_timer_cnt);
	change_delay_mode(0);
	del_timer(&LoopProc_timer);
	g_timer_cnt = 0;
	printk(KERN_INFO "HDMI Loop Finish!\n");
}
static void LoopProc_do_timer(unsigned long arg)
{
	int err;

	g_timer_cnt++;
	if(phdmi_host->is_suspended){
		printk("Earlysuspend,hdmi check over!");	
		LoopProc_timer_finish();	
		return ;
	}
	IT6610I2cInit();	
	err = HDMITX_DevLoopProc();
	IT6610I2cDeinit();

	dprintk("---%d:%s, timer_cnt=%d, err=%d\n",__LINE__,__func__,g_timer_cnt,err);

	if(is_polling_always) return;

	if( !err || g_timer_cnt >= TIMER_CNT_MAX){
		LoopProc_timer_finish();	
		if(g_timer_cnt >= TIMER_CNT_MAX)
			printk(" Hdmi check timeout(%d ms)!!!\n",g_timer_cnt*ite6610_rate);
	}else{
		mod_timer(&LoopProc_timer,jiffies+ (ite6610_rate* HZ) / 1000);
	}
}
static void LoopProc_timer_init(void)
{
	if(timer_pending(&LoopProc_timer)){
		printk("Hdmi timer is pending !!!\n");
		return ;
	}
	g_delay_mode = 1;
	init_timer(&LoopProc_timer);
	LoopProc_timer.function=&LoopProc_do_timer;
	LoopProc_timer.expires=jiffies + (ite6610_rate* HZ) / 1000; 
	add_timer(&LoopProc_timer);

}

static void it6610_enter_hdmi(unsigned int mode)
{
	HDMITX_ChangeDisplayOption(mode,OutputColorMode);
	init_state=INIT_OK;
	gpio_set_pin(HDMI_RST_N_PIN);
	hdmi_power_enable();
	VideoMode = mode;
	IT6610I2cInit();
	InitCAT6611();
	IT6610I2cDeinit();
	is_polling_always = 1;
	LoopProc_timer_init();
}
static void it6610_exit_hdmi(void)
{
	if(INIT_OK==init_state)
	{
		LoopProc_timer_finish();
		hdmi_power_disable();
		init_state=INIT_NO;
	}
}

static int hdmi_devproc_handler(struct hdmi_mode *pmode)
{
	int try_cnt,i,err,mode = pmode->output_type;
	
	dprintk("---%d:%s\n",__LINE__,__func__);

	HDMITX_ChangeDisplayOption(mode,OutputColorMode);
	VideoMode = mode;

	for(try_cnt=0; try_cnt< 3; try_cnt++){
		gpio_set_pin(HDMI_RST_N_PIN);
		IT6610I2cInit();
		InitCAT6611();
		IT6610I2cDeinit();
		sDelayMS(50);

		IT6610I2cInit();
		for(i = 0; i< HDMI_DEVLOOP_TIMES; i++){
			err= HDMITX_DevLoopProc();
			printk("-----HDMITX_DevLoopProc err=%d\n",err);
			if(err >= 0) break;
			sDelayMS(HDMI_INIT_MS);
		}
		IT6610I2cDeinit();
		if(err < 0){
			printk("Try again (cnt=%d) ...\n",try_cnt+1);
			hdmi_power_disable();
			hdmi_power_enable();
			sDelayMS(20);
		}else
			break;
	}
	

	if( i == HDMI_DEVLOOP_TIMES && err){
		printk("HDMI Output ...done ? \n");
		return err;
	}
	printk("HDMI Output done!(%d)\n",i);
	return 0;
}
static void hdmi_devclose_handler(struct hdmi_mode *pmode)
{
	dprintk("---%d:%s\n",__LINE__,__func__);
	gpio_set_pin(HDMI_RST_N_PIN);
	HDMITX_DisableOutput();
//	HdmiStandy();
}


static void hdmi_power_on(struct hdmi_host *host)
{
	dprintk("---HDMI %s\n",__func__);
	hdmi_power_enable();
//	host->hdmi_info.mode.power = POWER_ON;
	host->hdmi_info.hdmi_status  = HDMI_STAT_WORKING;
	
}
static void hdmi_power_off(struct hdmi_host *host)
{
	dprintk("---HDMI %s\n",__func__);
	hdmi_power_disable();
//	host->hdmi_info.mode.power = POWER_OFF;
	host->hdmi_info.hdmi_status  = HDMI_STAT_DISABLED;
	
}
static int hdmi_standy(struct hdmi_host *host)
{
	dprintk("---HDMI %s\n",__func__);
	if(host->hdmi_info.hdmi_status != HDMI_STAT_WORKING)
		return -1;
	
	HdmiStandy();
//	host->hdmi_info.mode.power = POWER_ON;
	host->hdmi_info.hdmi_status  = HDMI_STAT_STANDBY;
	return 0;
}
	

/*
 * Internal function. Schedule delayed work in the HDMI work queue.
 */
static int hdmi_schedule_delayed_work(struct hdmi_host *host,
				     unsigned long delay)
{
	return queue_delayed_work(host->detect_workqueue, &(host->detect), delay);
}

/*
 * Internal function. Flush all scheduled work from the HDMI work queue.
 */
static void hdmi_flush_scheduled_work(struct hdmi_host *host)
{
	flush_workqueue(host->detect_workqueue);
}
int  hdmi_detect_status(struct hdmi_host *host)
{
#define HDMI_DETECT_TRY_CNT  10
#define HDMI_DETECT_FIX_CNT  3

	int ret,i,cur_s,pre_s=0,cnt=0;
	unsigned int hotplug = host->board_info->pin_info->PIN_HPD;
	
	for(i=0; i<HDMI_DETECT_TRY_CNT; i++){
		cur_s = gpio_get_pin(hotplug);

		if(cur_s == pre_s)	cnt++;
		else				cnt = 0;
		
		if(cnt >= HDMI_DETECT_FIX_CNT)	break;
		pre_s = cur_s;
		
		sDelayMS(10);
	}
	if(i == HDMI_DETECT_TRY_CNT ) printk("HDMI detect timeout!!!\n");
	if(host->board_info->hpd_connect_active)
		ret = cur_s ? HDMI_CONNECT : HDMI_DISCONNECT ;
	else
		ret = cur_s ? HDMI_DISCONNECT : HDMI_CONNECT ;
			
#if 0
	if(gpio_get_pin(hotplug))
		ret = HDMI_CONNECT;
	else
		ret = HDMI_DISCONNECT;
#endif
	return ret;
}

void hdmi_detect_change(struct hdmi_host *host, unsigned long delay)
{
	hdmi_schedule_delayed_work(host, delay);
}

int hdmi_re_init_host(struct hdmi_host *host)
{
	host->detect_workqueue = workqueue;
	return 0;
}
int is_hang_pin(unsigned int pin)
{
	if(pin == 0)
		return 1;

	gpio_as_output(pin);
	gpio_clear_pin(pin);
	gpio_as_input(pin);
	if(0 != gpio_get_pin(pin))
		return 0;

	gpio_as_output(pin);
	gpio_set_pin(pin);
	gpio_as_input(pin);
	if(1 != gpio_get_pin(pin))
		return 0;

	gpio_as_output(pin);
	gpio_clear_pin(pin);
	gpio_as_input(pin);
	if(0 != gpio_get_pin(pin))
		return 0;

	return 1;
}

void hdmi_hotplug_base_enable(void)
{

	unsigned int hotplug = phdmi_host->board_info->pin_info->PIN_HPD;
	unsigned int status = phdmi_host->board_info->hpd_connect_active ? HDMI_CONNECT : HDMI_DISCONNECT ;
	
	if(phdmi_host->hdmi_info.HPD_status == status)
		__gpio_as_irq_fall_edge(hotplug);
	else
		__gpio_as_irq_rise_edge(hotplug);			

//	__gpio_unmask_irq(hotplug);
	enable_irq(phdmi_host->irq);

}

int hdmi_hotplug_enable(struct hdmi_host *host)
{
	if(!(host->hotplug_support))
		return -1;


		
	hdmi_hotplug_base_enable();


	host->hotplug_enable = HDMI_HOTPLUG_ENABLE;

	return 0;
};

void hdmi_hotplug_base_disable(void)
{
#if 0
	unsigned int hotplug = phdmi_host->board_info->pin_info->PIN_HPD;
	
	__gpio_mask_irq(hotplug);
	__gpio_as_input(hotplug);
#endif
	disable_irq(phdmi_host->irq);
}

int hdmi_hotplug_disable(struct hdmi_host *host)
{
	if(!(host->hotplug_support))
		return -1;
	
	hdmi_hotplug_base_disable();
//	disable_irq(host->irq);

	host->hotplug_enable = HDMI_HOTPLUG_DISABLE;

	return 0;
};

static irqreturn_t hdmi_detect_irq(int irq, void *devid)
{
	struct hdmi_host *host = (struct hdmi_host *) devid;

	dprintk("---%d:%s\n",__LINE__,__func__);

	hdmi_detect_change(host,10);

	return IRQ_HANDLED;
}

static int hdmi_detect_init(struct hdmi_host *host)
{
	int ret = 0;

	if (!host->board_info->pin_info->PIN_HPD){
		host->irq = 0;
		printk("no hotplug irq for hdmi\n");
		return -1;
	}
	
	host->irq = IRQ_GPIO_0 + host->board_info->pin_info->PIN_HPD;
	
	ret = request_irq(host->irq,
					  hdmi_detect_irq,
					  0,
					  "hdmi(it6610 int_gpio)",
					  host);
	if (ret) {
		printk(KERN_ERR "Unable to get slot IRQ %d (%d)\n",host->irq, ret);
		return ret;
	}
	//disable_irq(host->irq);
	hdmi_hotplug_base_disable();

	return 0;
}

int hdmi_dev_register(void)
{
	int ret;
		
	ret = misc_register(&it6610_miscdev);
	if(!ret) is_hdmi_dev_reg = 1; 
	
	return ret;
}
int hdmi_dev_deregister(void)
{	
	int ret = 0;
	
	if(is_hdmi_dev_reg){
		ret = misc_deregister(&it6610_miscdev);
		is_hdmi_dev_reg = 0;
	}
	
	return ret;
}

int hdmi_connect(struct hdmi_host *host)
{
	int ret;

	printk("HDMI Connect!\n");
	
//	printk("HDMI_RST_N_PIN GPIO LVL:%d \n",gpio_get_pin(HDMI_RST_N_PIN));
	ret = hdmi_dev_register();	

	if (ret) {
//		printk( "cannot register miscdev on minor=%d (%d)\n",-1, ret);
		return -1;
	}
#if 0
	if(host->is_probe){
		hdmi_standy(host);
		host->is_probe = 0;
		return 0;
	}

	if(host->hdmi_info.hdmi_status != HDMI_STAT_WORKING){
		hdmi_power_on(host);
		hdmi_devproc_handler(&host->hdmi_info.mode);
	}
#endif

	return 0;
}

void hdmi_disconnect(struct hdmi_host *host)
{
	printk("HDMI Disconnect!\n");


	if(host->hdmi_info.hdmi_status == HDMI_STAT_WORKING){
		hdmi_devclose_handler(&host->hdmi_info.mode);
		hdmi_power_off(host);
	}
	
	cancel_delayed_work(&host->detect);

	
	hdmi_dev_deregister();


//	printk("HDMI_RST_N_PIN GPIO LVL:%d \n",gpio_get_pin(HDMI_RST_N_PIN));
}
static int  suspended_check(struct hdmi_host *host,int hdmi_status)
{
	if(! hdmi_earlysuspended_check) return 0;
			
	hdmi_earlysuspended_check = 0;

	if(hdmi_status == HDMI_CONNECT){
		host->hdmi_info.HPD_status = HDMI_CONNECT;
		if( host->hdmi_info.hdmi_status == HDMI_STAT_WORKING 
			&& host->hotplug_enable == HDMI_HOTPLUG_ENABLE ){
			hdmi_devproc_handler(&(host->hdmi_info.mode));
		}
	}else{
		host->hdmi_info.HPD_status = HDMI_DISCONNECT;
		hdmi_devclose_handler(&(host->hdmi_info.mode));
	}
		
	return 1;
}

void hdmi_detect_handler_base(struct hdmi_host *host)
{
	int status,enable;
	
	status = hdmi_detect_status(host);
	enable = (host->flag == 1)?1:(status ^ host->hdmi_info.HPD_status);
	if(enable)
	{
		if(status == HDMI_CONNECT){
			host->hdmi_info.HPD_status = HDMI_CONNECT;
			hdmi_connect(host);
		}
		else{
			host->hdmi_info.HPD_status = HDMI_DISCONNECT;
			hdmi_disconnect(host);
		}
	}else{
		dprintk("HPD_status same (%d)!\n",status);		
		suspended_check(host,status);
	}

//	host->flag = 0;
}
void hdmi_detect_handler(struct work_struct *work)
{
	struct hdmi_host *host = container_of(work, struct hdmi_host, detect.work);
	
	dprintk("---%d:%s\n",__LINE__,__func__);

	if(host->hdmi_info.mode.scan_type == HDMI_DIRECT){
		return ;
	}
	
	hdmi_hotplug_base_disable();
		
	hdmi_detect_handler_base(host);
	
	if(host->hotplug_enable == HDMI_HOTPLUG_ENABLE)
		hdmi_hotplug_base_enable();
	
}
static int hdmi_detect_early(struct hdmi_host *host)
{
	int status,ret;
	
	status = hdmi_detect_status(host);
	if(status == HDMI_CONNECT){
		dprintk("HDMI connected !\n");
		host->hdmi_info.HPD_status = HDMI_CONNECT;
		ret = hdmi_dev_register();
		if (ret) {
			printk( "cannot register miscdev on minor=%d (%d)\n",-1, ret);
			return -1;
		}
	}
	else{
		dprintk("HDMI disconnected !\n");
		host->hdmi_info.HPD_status = HDMI_DISCONNECT;
	}
		
	hdmi_power_on(host);
	hdmi_standy(host);
	
	return 0;
}
static int it6610_open(struct inode *inode, struct file *file)
{
	struct hdmi_mode *phdmi_mode;
	dprintk("enter it6610_open ... \n");
	file->private_data = phdmi_host;
	phdmi_mode = &phdmi_host->hdmi_info.mode;
	#if 0
	if(phdmi_mode->scan_type  == HDMI_HOTPLUG){
		mode = phdmi_mode->output_type;
		HDMITX_ChangeDisplayOption(mode,OutputColorMode);
		init_state=INIT_OK;
		hdmi_power_enable();
		VideoMode = mode;
		IT6610I2cInit();
		InitCAT6611();
		IT6610I2cDeinit();
	}
	#endif
	return 0;
}
static int it6610_release(struct inode *inode, struct file *filp)
{
	struct hdmi_mode *phdmi_mode;
	struct hdmi_host *host =(struct hdmi_host *) filp->private_data;
	filp->private_data = NULL;
	phdmi_mode = &host->hdmi_info.mode;
	dprintk("enter it6610_release ... \n");
	#if 0
	if(phdmi_mode->scan_type  == HDMI_HOTPLUG){
		hdmi_power_disable();
		init_state=INIT_NO;
	}
	#endif
	return 0;
}
static int it6610_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct hdmi_host *host =(struct hdmi_host *) file->private_data;
	char	buf_byte = 0 ;
	unsigned int pin;
	
	if (copy_from_user((void *)&buf_byte, buffer, 1)) {
		printk("HDMI: copy_from_user failed !\n");
		return -EFAULT;
	}
	printk("buf_byte = %d \n",buf_byte);
	switch (buf_byte) {
	case '1':
		printk("HMDI try  Standby Mode 0!\n");
		HDMITX_Standby(1);
		printk("HMDI in Standby Mode 0!\n");
		break;
	case '2':
		printk("HMDI try  Standby Mode 1!\n");
		gpio_set_pin(HDMI_RST_N_PIN);
		HDMITX_Standby(2);
		printk("HMDI in Standby Mode 1!\n");
		break;
	case '3':
		printk("HMDI try  Standby Mode 2!\n");
		gpio_clear_pin(HDMI_RST_N_PIN);
		HDMITX_Standby(2);
		printk("HMDI in Standby Mode 2!\n");
		break;
	case '4':
		printk("HMDI SYSRSTN High:\n");
		gpio_set_pin(HDMI_RST_N_PIN);
		printk("GPIO LVL:%d \n",gpio_get_pin(HDMI_RST_N_PIN));   
		break;
	case '5':
		printk("HMDI SYSRSTN Low:\n");
		gpio_clear_pin(HDMI_RST_N_PIN);
		printk("GPIO LVL:%d \n",gpio_get_pin(HDMI_RST_N_PIN));   
		break;

	case '6':
		printk("HDMI_RST_N_PIN GPIO LVL:%d \n",gpio_get_pin(HDMI_RST_N_PIN));  
		break;

	case '7':
		printk("HMDI try  Standby Mode!\n");
		HDMITX_Standby(2);
		printk("HMDI in Standby Mode!\n");
		break;
	case '8':
        pin = host->board_info->pin_info->PIN_HPD;
        printk("HDMI hotplug pin :(32 * %d + %d)\n",pin/32,pin%32);
        gpio_as_output(pin);
        printk("gpio set!\n");
        gpio_set_pin(pin);
        break;
    case '9':
        pin = host->board_info->pin_info->PIN_HPD;
        printk("HDMI hotplug pin :(32 * %d + %d)\n",pin/32,pin%32);
        gpio_as_output(pin);
        printk("gpio clear!\n");
        gpio_clear_pin(pin);
        break;

	}

	return count;
}

static long it6610_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	unsigned int hdmi_enable;
	struct hdmi_mode *phdmi_mode,try_mode;
	struct hdmi_host *host =(struct hdmi_host *) file->private_data;
	int __user *argp = (int __user *)arg;

	phdmi_mode = &host->hdmi_info.mode;
	dprintk("it6610_ioctl cmd %d ...\n",cmd);
	
	if(host->hdmi_info.mode.scan_type == HDMI_DIRECT){
		printk("HDMI DIRECT mode do NOT need ioctl !!!\n");
		return 0;
	}

	switch(cmd)
	{
		case HDMI_POWER_ON:
		{
			unsigned int SetVal=0;
			if(0!=get_user(SetVal,argp))
			{
				printk("IT6610_SET_VIDEO_MODE get_user error\n");
				return -EFAULT;
			}
			printk("HDMI_POWER_ON VideoMode=%d\n",SetVal);
			
			if(phdmi_mode->scan_type  == HDMI_HOTPLUG){
				phdmi_mode->output_type = SetVal;
				hdmi_power_on(host);
				ret = hdmi_devproc_handler(phdmi_mode);
			}else
				it6610_enter_hdmi(SetVal);
			break;
		}
		case HDMI_POWER_OFF:
			if(phdmi_mode->scan_type  == HDMI_HOTPLUG){
//			 	hdmi_devclose_handler(phdmi_mode);
				hdmi_power_off(host);
			}else
				it6610_exit_hdmi();
		     break;
		case HDMI_INFO_GET:
			if (copy_to_user(argp, &host->hdmi_info, sizeof(struct hdmi_info)))
				return -EFAULT;
			break;
		case HDMI_HOTPLUG_CTRL:
			if(host->hdmi_info.mode.scan_type != HDMI_HOTPLUG){
				printk("hdmi is not in HDMI_HOTPLUG mode !!!\n");
				return -EFAULT; 
			}
			
			hdmi_enable = arg;
			if(hdmi_enable == HDMI_HOTPLUG_ENABLE){
				hdmi_hotplug_enable(host);
				printk("hdmi hotplug enable!\n");
			}else if(hdmi_enable == HDMI_HOTPLUG_DISABLE){
				hdmi_hotplug_disable(host);
				printk("hdmi hotplug disable!\n");
				hdmi_standy(host);
			}
			break;
		case HDMI_MODE_GET:
			if (copy_to_user(argp, phdmi_mode, sizeof(struct hdmi_mode)))
				return -EFAULT;
			break;
		case HDMI_MODE_SET:
			if (copy_from_user(&try_mode, argp, sizeof(struct hdmi_info))){
            	printk("copy_from_user error \n");
				return -EFAULT;       
			}
			dprintk("hdmi_status = %d\n",host->hdmi_info.hdmi_status);
			dprintk("Old Mode:power=%d,scan_type=%d,output_type=%d\n",
			  phdmi_mode->power,phdmi_mode->scan_type,phdmi_mode->output_type);
			dprintk("Try Mode:power=%d,scan_type=%d,output_type=%d\n",
			  try_mode.power,try_mode.scan_type,try_mode.output_type);
			/* scan_type ctrl */
			if(try_mode.scan_type ^ phdmi_mode->scan_type)
			{
				if(try_mode.scan_type == HDMI_HOTPLUG){
					hdmi_hotplug_enable(host);
					phdmi_mode->scan_type = HDMI_HOTPLUG;
				}else{
					hdmi_hotplug_disable(host);
					phdmi_mode->scan_type = HDMI_POLLING;
				}
			}
			/* power ctrl */
			if(try_mode.power != POWER_INVAILD){
				if(try_mode.power == POWER_OFF){
					hdmi_power_off(host);
					phdmi_mode->power = POWER_INVAILD;
					printk("HDMI Power OFF!!!\n");
					return RETURN_POWEROFF_CTRL;
				}
				hdmi_power_on(host);
				phdmi_mode->power = POWER_INVAILD;	
			}
			/* output ctrl */
			if(host->hdmi_info.hdmi_status != HDMI_STAT_WORKING){
				hdmi_power_on(host);
				ret = hdmi_devproc_handler(&try_mode);
				if(!ret)
					phdmi_mode->output_type = try_mode.output_type;
			}else{
				if(try_mode.output_type ^ phdmi_mode->output_type)
				{
					ret = hdmi_devproc_handler(&try_mode);
					if(!ret)
						phdmi_mode->output_type = try_mode.output_type;
				}
			}
			break;
		case HDMI_STANDBY:
			printk("HMDI try  Standby Mode !\n");
			hdmi_standy(host);
			printk("HMDI in Standby Mode !\n");
			break;
		case HDMI_HOTPLUGABLE:
			hdmi_enable = host->hotplug_enable;
			if (copy_to_user(argp,&hdmi_enable, sizeof(unsigned int))){
				printk("copy_to_user error \n");
				return -EFAULT;       
			}
			break;
		case HDMI_HOTPLUG_TRIGGER:
			if(host->hotplug_enable == HDMI_HOTPLUG_DISABLE)
				return -EFAULT;  
			hdmi_hotplug_enable(host);
			hdmi_detect_change(host,0);
			break;
		case HDMI_AUDIO_SAMPLE_ADJUST:
			if(host->is_suspended){
				printk("Hdmi don't audio sync when suspend!!!\n");	
				return -EFAULT;  
			}
			printk("HDMI Audio Sample Sync ...\n");
			LoopProc_timer_init();
			//hdmi_audio_sync();
			break;
		default:
		     printk("it6610_ioctl Not supported command: 0x%x\n", cmd);
		     break;

	}
	dprintk("it6610_ioctl cmd %d done!\n",cmd);
	return ret;
}
static ssize_t hdmi_hotplug_support_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct hdmi_host *host = dev_get_drvdata(dev);
//	ssize_t count = 0;
	if(host->hotplug_support == 1){
		printk("HDMI support hotplug!\n");
		*buf = 1;
	}else{
		printk("HDMI don't support hotplug!\n");
		*buf = 0;
	}
	
	return 1;
}
static ssize_t hdmi_hotplug_enable_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct hdmi_host *host = dev_get_drvdata(dev);
//	ssize_t count = 0;
	
	if(host->hotplug_enable == HDMI_HOTPLUG_ENABLE){
		*buf = HDMI_HOTPLUG_ENABLE;
		printk("HDMI hotplug enable!\n(code = %d)\n",buf[0]);
	}else{
		*buf = HDMI_HOTPLUG_DISABLE;
		printk("HDMI hotplug disable!\n(code = %d)\n",buf[0]);
	}
	
	return 1;
}
#define STR_HDMI_HOTPLUG_ENABLE 	"HOTPLUG_ENABLE"
#define STR_HDMI_HOTPLUG_DISABLE 	"HOTPLUG_DISABLE"
#define STR_HDMI_POLLING_MODE 		"HDMI_POLLING"
#define STR_HDMI_HOTPLUG_MODE 		"HDMI_HOTPLUG"



static ssize_t hdmi_hotplug_enable_set(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct hdmi_host *host = dev_get_drvdata(dev);
	
	if (buf == NULL)
		return count;
	
	if (strncmp(buf, STR_HDMI_HOTPLUG_ENABLE ,strlen(STR_HDMI_HOTPLUG_ENABLE)) == 0){
		printk("HDMI hotplug enable!\n");
		hdmi_hotplug_enable(host);
	}else if (strncmp(buf, STR_HDMI_HOTPLUG_DISABLE ,strlen(STR_HDMI_HOTPLUG_DISABLE)) == 0){
		printk("HDMI hotplug disable!\n");
		hdmi_hotplug_disable(host);
	}else
		printk("invalid operation!\n");

	return count;
}
int hdmi2hotplug_mode(struct hdmi_host *host)
{
	if(!(host->hotplug_support))
		return -1;
	hdmi_flush_scheduled_work(host);
	hdmi_hotplug_enable(host);
	host->hdmi_info.mode.scan_type = HDMI_HOTPLUG;
	hdmi_detect_change(host,0);
	return 0;
}
int hdmi2polling_mode(struct hdmi_host *host)
{
	hdmi_hotplug_disable(host);
	host->hdmi_info.mode.scan_type = HDMI_POLLING;
	hdmi_dev_register();
	return 0;
}

static ssize_t hdmi_drvier_mode_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct hdmi_host *host = dev_get_drvdata(dev);
	ssize_t count = 0;

	if(host->hdmi_info.mode.scan_type == HDMI_HOTPLUG){
		*buf = HDMI_HOTPLUG;
		printk("HDMI in HDMI_HOTPLUG mode!(code = %d)\n",buf[0]);
		
	}else if(host->hdmi_info.mode.scan_type == HDMI_POLLING){
		*buf = HDMI_POLLING;
		printk("HDMI in HDMI_POLLING mode!(code = %d)\n",buf[0]);
		
	}else
		printk("HDMI in unkown mode!\n");
	
	return count;
}
static ssize_t hdmi_drvier_mode_set(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct hdmi_host *host = dev_get_drvdata(dev);
	
	if (buf == NULL)
		return count;
	
	if (strncmp(buf, STR_HDMI_HOTPLUG_MODE ,strlen(STR_HDMI_HOTPLUG_MODE)) == 0){
		if(! hdmi2hotplug_mode(host))
			printk("HDMI in HDMI_HOTPLUG mode!\n");
		else
			printk("operation fail!\n");
	}else if (strncmp(buf, STR_HDMI_POLLING_MODE ,strlen(STR_HDMI_POLLING_MODE)) == 0){
		if(! hdmi2polling_mode(host))
			printk("HDMI in HDMI_POLLING mode!\n");
		else
			printk("operation fail!\n");
	}else
		printk("invalid operation!\n");

	return count;
}

static DEVICE_ATTR(hotplug_support, S_IRUSR | S_IRGRP | S_IROTH , hdmi_hotplug_support_show, NULL);
static DEVICE_ATTR(hotplug_enable, S_IRUSR | S_IWUSR, hdmi_hotplug_enable_show, hdmi_hotplug_enable_set);
static DEVICE_ATTR(driver_mode, S_IRUSR | S_IWUSR, hdmi_drvier_mode_show,hdmi_drvier_mode_set);

static struct attribute *hdmi_attributes[] = {
	&dev_attr_hotplug_support.attr,
	&dev_attr_hotplug_enable.attr,
	&dev_attr_driver_mode.attr,
	NULL
};

static const struct attribute_group hdmi_attr_group = {
	.attrs = hdmi_attributes,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hdmi_early_suspend(struct early_suspend *h)
{
	dprintk("----%s\n",__func__);
	phdmi_host->is_suspended = 1;
	hdmi_earlysuspended_check = 1;
	hdmi_hotplug_base_disable();
	hdmi_flush_scheduled_work(phdmi_host);
	cancel_delayed_work(&phdmi_host->detect);
	
	hdmi_power_disable();
	hdmi_real_poweroff();
}
static void hdmi_late_resume(struct early_suspend *h)
{
	dprintk("----%s\n",__func__);
	phdmi_host->is_suspended = 0;
	hdmi_power_enable();
	if( phdmi_host->hdmi_info.hdmi_status != HDMI_STAT_WORKING )
		HdmiStandy();

#ifdef CONFIG_HDMI_DIRECT_MODE
	if(phdmi_host->hdmi_info.mode.scan_type == HDMI_DIRECT){
		it6610_exit_hdmi();
		printk("HDMI_DIRECT reset ...\n");
		phdmi_host->hdmi_info.mode.output_type = HDMI_720p60;
		hdmi_power_on(phdmi_host);
		//hdmi_devproc_handler(&(host->hdmi_info.mode));
		it6610_enter_hdmi(phdmi_host->hdmi_info.mode.output_type);
		return;
	}
#endif
	if(phdmi_host->hotplug_support){
		hdmi_detect_change(phdmi_host,2*HZ);
		if(phdmi_host->hotplug_enable == HDMI_HOTPLUG_ENABLE ){
			hdmi_hotplug_base_enable();
		}
	}
}
#endif

static const struct file_operations it6610_hdmi_fops = {
	.open       = it6610_open,
	.release    = it6610_release,
	.unlocked_ioctl   = it6610_ioctl,
	.write	    = it6610_write,
};

static struct miscdevice it6610_miscdev = {
	.minor          = MISC_DYNAMIC_MINOR,
	.name           = DRIVER_NAME,
	.fops           = &it6610_hdmi_fops,
};
static int __devinit ite6610_drv_probe(struct platform_device *dev)
{
	int ret;
	struct hdmi_host *host = NULL;
	struct hdmi_it6610_pin_info *pboard_pin;
	struct hdmi_it6610_board_info *board_info;
	
	if (!dev)
		goto pb_fail;
	board_info = (struct hdmi_it6610_board_info *)dev->dev.platform_data;
	
	if(!board_info) {
		printk("platform_data is null!\n");
		goto pb_fail;
	}
	pboard_pin = board_info->pin_info;
	if(!pboard_pin){
		printk("pin_info is null!\n");
		goto pb_fail;
	}

	ret = hdmi_check_invalid_pins(pboard_pin);
	if(ret)
		goto pb_fail;

	host = kzalloc(sizeof(struct hdmi_host), GFP_KERNEL);
	if (!host)
		goto pb_fail;
		
	host->board_info = board_info;

	if(phdmi_host){
		printk("phdmi_host exist!!!\n");
		goto pb_fail;
	}
	else
		phdmi_host = host;
	
	platform_set_drvdata(dev, host);
	
	is_hdmi_dev_reg = 0;
	is_polling_always = 0;
	host->irq = 0;
	host->is_suspended = 0;
	host->is_probe = 1;
	host->hdmi_info.mode.output_type = HDMI_Unkown;
	host->hdmi_info.mode.scan_type = 0;

	host->hdmi_info.mode.power = POWER_INVAILD;
	host->hdmi_info.hdmi_status = HDMI_STAT_DISABLED;
	host->hotplug_support = 1;
	host->hotplug_enable = HDMI_HOTPLUG_DISABLE;
	host->flag = 0;

#ifdef CONFIG_HDMI_HOTPLUG_MODE	
	host->hdmi_info.mode.scan_type = HDMI_HOTPLUG;
#else
	host->hdmi_info.mode.scan_type = HDMI_POLLING;
#endif

#ifdef CONFIG_HDMI_DIRECT_MODE
	host->hdmi_info.mode.scan_type = HDMI_DIRECT;
	host->hotplug_support = 0;
#endif
		
	ret = hdmi_detect_init(host);
	if(ret){
		if(!host->irq)
			host->hotplug_support = 0;
	}
	
	hdmi_pin_init(board_info);

	if (ret < 0)
		goto pb_fail;

	if(host->board_info->hdmi_board_init)
		host->board_info->hdmi_board_init();
		
	if(host->board_info->hdmi_power_on)
		host->board_info->hdmi_power_on();
	
/*******************************************/
#if	0
	if(is_hang_pin(hotplug)){
		printk("HPD pin is hanging !!!\n");
		host->hdmi_info.mode.scan_type = HDMI_POLLING;
	}
#endif	
	hdmi_re_init_host(host);
	
	INIT_DELAYED_WORK(&host->detect, hdmi_detect_handler);
	hdmi_flush_scheduled_work(host);

#ifdef CONFIG_HAS_EARLYSUSPEND
	host->early_suspend.suspend = hdmi_early_suspend;
	host->early_suspend.resume =  hdmi_late_resume;
	host->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 15;
	register_early_suspend(&host->early_suspend);
#endif

//	hdmi_detect_change(host,0);

	if(!(host->hotplug_support)){
		if(!host->hdmi_info.mode.scan_type){
			printk("\nHDMI change to HDMI_POLLING mode !\n");		
			host->hdmi_info.mode.scan_type = HDMI_POLLING;
		}
	}

	if(host->hdmi_info.mode.scan_type == HDMI_HOTPLUG){
		hdmi_detect_early(host);
		host->hotplug_enable = HDMI_HOTPLUG_ENABLE;
	}else{
		host->hotplug_enable = HDMI_HOTPLUG_DISABLE;
		ret = hdmi_dev_register();
		if (ret) {
			printk( "cannot register miscdev on minor=%d (%d)\n",-1, ret);
			
		}	
	}
	
	if(host->hotplug_enable == HDMI_HOTPLUG_ENABLE)
			hdmi_hotplug_enable(host);
	else
			hdmi_hotplug_disable(host);
	
	if(host->hdmi_info.mode.scan_type == HDMI_HOTPLUG)
		printk("HDMI driver in HDMI_HOTPLUG mode !\n");
	else if(host->hdmi_info.mode.scan_type == HDMI_POLLING)
		printk("HDMI driver in HDMI_POLLING mode !\n");
	else if(host->hdmi_info.mode.scan_type == HDMI_DIRECT)
		printk("HDMI driver in HDMI_DIRECT mode !\n");
	else
		printk("HDMI driver in unknown mode !!!\n");


	ret = sysfs_create_group(&dev->dev.kobj, &hdmi_attr_group);
	if(ret){
		printk("sysfs_create_group Error!(ret=%d)\n",ret);
	}
	
	printk("HDMI ite6610_drv_probe done!\n");

#ifdef CONFIG_HDMI_DIRECT_MODE
	if(host->hdmi_info.mode.scan_type == HDMI_DIRECT){
		printk("HDMI_DIRECT enable ...\n");
		host->hdmi_info.mode.output_type = HDMI_720p60;
		hdmi_power_on(host);
		//hdmi_devproc_handler(&(host->hdmi_info.mode));
		it6610_enter_hdmi(host->hdmi_info.mode.output_type);
	}
#endif
	
	return 0;
pb_fail:
	if(host)
		kfree(host);
	printk("HDMI ite6610_drv_probe fail!!!\n");
	return -EINVAL;
}

static int __devexit ite6610_drv_remove(struct platform_device *dev)
{
	struct hdmi_host *host = platform_get_drvdata(dev);
	unsigned int hotplug = host->board_info->pin_info->PIN_HPD;
	dprintk("---%d:%s\n",__LINE__,__func__);
	hdmi_disconnect(host);
	sysfs_remove_group(&dev->dev.kobj, &hdmi_attr_group);
	
	free_irq(hotplug + IRQ_GPIO_0 , host);


	kfree(host);
	phdmi_host = NULL;
	
	return 0;
}
#if 0
static int ite6610_drv_suspend(struct platform_device *dev,pm_message_t state)
{

	struct hdmi_host *host = platform_get_drvdata(dev);
	dprintk("---%d:%s\n",__LINE__,__func__);

	host->is_suspended = 1;
	cancel_delayed_work(&host->detect);
	hdmi_power_disable();
	hdmi_real_poweroff();

	return 0;
}
static int ite6610_drv_resume(struct platform_device *dev)
{

	struct hdmi_host *host = platform_get_drvdata(dev);
	dprintk("---%d:%s\n",__LINE__,__func__);


	hdmi_flush_scheduled_work(host);

	if( host->hdmi_info.hdmi_status == HDMI_STAT_WORKING ){
		hdmi_power_enable();
	else
		HdmiStandy();

	return 0;
}
#endif
void ite6610_drv_shutdown(struct platform_device *dev)
{
	struct hdmi_host *host = platform_get_drvdata(dev);
	dprintk("---%d:%s\n",__LINE__,__func__);
	hdmi_power_off(host);
	hdmi_real_poweroff();
}

static struct platform_driver ite6610_drv_driver = {
	.driver.name	= "ite6610-drv",
	.driver.owner	= THIS_MODULE,
	.probe		= ite6610_drv_probe,
	.remove		= ite6610_drv_remove,
//	.suspend	= ite6610_drv_suspend,
//	.resume		= ite6610_drv_resume,
	.shutdown	= ite6610_drv_shutdown,
	
};

static int __init ite6610_drv_init(void)
{
	int ret;
	

	workqueue = create_singlethread_workqueue("khdmid");
	if (!workqueue){
		ret = -ENOMEM;
		goto fail;
	}
	
	ret = platform_driver_register(&ite6610_drv_driver);
	if(ret)
		goto fail;
		

	printk("HDMI ite6610 driver register done!\n");	
	return 0;
fail:
	printk("HDMI ite6610 driver register fail !!!\n");	
	return ret;
}

static void __exit ite6610_drv_exit(void)
{
	destroy_workqueue(workqueue);
	
	platform_driver_unregister(&ite6610_drv_driver);
}

late_initcall(ite6610_drv_init);
//module_init(ite6610_drv_init);
module_exit(ite6610_drv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ssg <estt501>");
MODULE_DESCRIPTION("hdmi ite6610 driver");
