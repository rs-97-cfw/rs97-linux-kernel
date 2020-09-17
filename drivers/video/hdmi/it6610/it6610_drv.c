 /*
 * linux/drivers/power/jz_battery
 *
 * Battery measurement code for Ingenic JZ SOC.
 *
 * based on tosa_battery.c
 *
 * Copyright (C) 2008 Marek Vasut <marek.vasut@gmail.com>
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
//wjx #include <linux/jz_battery.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <asm/jzsoc.h>
//wjx #include <asm/jzpm/jz_act8930.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include "hdmitx.h"

#define INITOK	1
#define INITNO  0
#define DRIVER_NAME	"hdmi"
static unsigned int init_state=INITNO;
static unsigned int ite6610_rate = 200; /* ite6610 scan rate(ms)*/
struct timer_list LoopProc_timer;
static HDMI_Video_Type VideoMode = HDMI_480p60;
static HDMI_OutputColorMode OutputColorMode = HDMI_YUV444 ;//HDMI_RGB444;
//static HDMI_OutputColorMode OutputColorMode = HDMI_RGB444 ;//HDMI_RGB444;
#ifdef CONFIG_JZ4760B_LYNX	
#define LCD_SIGNAL_DRIVING_STRENGTHEN
#endif

#ifdef LCD_SIGNAL_DRIVING_STRENGTHEN
#define     EP932M_5VEN_PIN (32*5+6)
int ep932m_5ven	= 1;
#endif

//#define HDMI_RST_N_PIN  (32*4 + 6)
void hdmi_reset()
{
	__gpio_as_output1(HDMI_RST_N_PIN);
	__gpio_as_output0(HDMI_RST_N_PIN);
	mdelay(30);
	//medive changed
	__gpio_as_output1(HDMI_RST_N_PIN);


	//__gpio_as_output1(32*2+9);

	printk("GPIO LVL:%d \n",__gpio_get_pin(HDMI_RST_N_PIN));    
}

static void hdmi_power_enable(void)
{
//	__lcd_close_backlight();
    printk("---it6610 retset ... \n");
#ifdef CONFIG_SOC_JZ4770
    hdmi_reset();
#endif
//	__lcd_power_off();
//wjx	 __hdmi_power_off();
	mdelay(10);
//wjx	__hdmi_power_on();
#ifdef LCD_SIGNAL_DRIVING_STRENGTHEN
  __gpio_set_driving_strength(GPIO_LCD_PCLK_DS,3);//GPIO_LCD_DS_HDMI	);
  __gpio_set_driving_strength(GPIO_LCD_HSYN_DS,3);//GPIO_LCD_DS_HDMI	);
  __gpio_set_driving_strength(GPIO_LCD_VSYN_DS,3);//GPIO_LCD_DS_HDMI	);
	ep932m_5ven = __gpio_get_pin(EP932M_5VEN_PIN);	
	if(!ep932m_5ven){
		printk("open ep932m_5ven \n");
		__gpio_as_output(EP932M_5VEN_PIN);
		__gpio_set_pin(EP932M_5VEN_PIN);
	}
#endif
    mdelay(10);
}
static void hdmi_power_disable(void)
{
//wjx	__hdmi_power_off();
//	__lcd_power_on();
#ifdef LCD_SIGNAL_DRIVING_STRENGTHEN
	__gpio_set_driving_strength(GPIO_LCD_PCLK_DS,GPIO_LCD_DS_DEFAULT);
	__gpio_set_driving_strength(GPIO_LCD_HSYN_DS,GPIO_LCD_DS_DEFAULT);
	__gpio_set_driving_strength(GPIO_LCD_VSYN_DS,GPIO_LCD_DS_DEFAULT);
	if(!ep932m_5ven){
		printk("close ep932m_5ven \n");
		__gpio_as_output(EP932M_5VEN_PIN);
		__gpio_clear_pin(EP932M_5VEN_PIN);
	}
#endif
//	__lcd_init_backlight(100);
}

static void LoopProc_do_timer(unsigned long arg)
{
	mod_timer(&LoopProc_timer,jiffies+ (ite6610_rate* HZ) / 1000);
	IT6610I2cInit();	
	HDMITX_DevLoopProc();
	IT6610I2cDeinit();
	
	return;
}
static void LoopProc_timer_init(void)
{
	init_timer(&LoopProc_timer);
	LoopProc_timer.function=&LoopProc_do_timer;
	LoopProc_timer.expires=jiffies + (ite6610_rate* HZ) / 1000; 
	add_timer(&LoopProc_timer);

}

static void LoopProc_timer_finish(void)
{
	del_timer(&LoopProc_timer);
}



static void it6610_enter_hdmi(unsigned int mode)
{
	HDMITX_ChangeDisplayOption(mode,OutputColorMode);
	init_state=INITOK;
	hdmi_power_enable();
	VideoMode = mode;
	IT6610I2cInit();
	InitCAT6611();
	IT6610I2cDeinit();
#if 0
	/* Medive  add */
	__gpio_as_output(32*2+9);
	__gpio_set_pin(32*2+9);
#endif
	LoopProc_timer_init();
}
static void it6610_exit_hdmi(void)
{
	if(INITOK==init_state)
	{
		LoopProc_timer_finish();
		hdmi_power_disable();
		init_state=INITNO;

	}
}


static int it6610_open(struct inode *inode, struct file *file)
{
	printk("enter it6610_open ... \n");
	return 0;
}
static int it6610_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int it6610_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
#define HDMI_POWER_ON		3
#define HDMI_POWER_OFF		4	
	int __user *argp = (int __user *)arg;
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
			it6610_enter_hdmi(SetVal);
			break;
		}
		case HDMI_POWER_OFF:
		     it6610_exit_hdmi();
		     break;
		default:
		     printk("it6610_ioctl Not supported command: 0x%x\n", cmd);
		     break;

	}
	printk("it5510_ioctl cmd %d ...\n",cmd); //wjx
	return 0;
}

static int it6610_mode = 0;
extern void jz4770_set_is_external_codec(int i);
static int proc_it6610_read_proc(char *page,char **start,off_t off,int count,int *eof,void *data)
{
	printk("Medive printk: read it6610 cur mode is %d\n",it6610_mode);
	return sprintf(page,"%1u\n",it6610_mode);
}

static int proc_it6610_write_proc(struct file *file,const char *buffer,unsigned long count,void *data)
{
    static int last_value = 0;
	it6610_mode = simple_strtoul(buffer,0,10);
	printk("Medive printk: write it610 mode is %d\n",it6610_mode);
        if(last_value != it6610_mode)
        {

            if (it6610_mode){
                //it6610_mode = 2;//FIXME the app enable 480P as 649*480 but in snk product we only need 480P. When use for other project you need comment this and wirte the proc as a correct argv in app.
				jz4770_set_is_external_codec(1);
                it6610_enter_hdmi(it6610_mode);
            }else{
                it6610_exit_hdmi();
				jz4770_set_is_external_codec(0);
            }
            last_value = it6610_mode;
        }
	return count;
}



static const struct file_operations it6610_hdmi_fops = {
	.open       = it6610_open,
	.release    = it6610_release,
	.ioctl      = it6610_ioctl,
};

static struct miscdevice it6610_miscdev = {
	.minor          = MISC_DYNAMIC_MINOR,
	.name           = DRIVER_NAME,
//wjx	.name           = "hdmi_ite6610",
	.fops           = &it6610_hdmi_fops,
};
static int __devinit ite6610_drv_probe(struct platform_device *dev)
{
	int ret = misc_register(&it6610_miscdev);
//wjx	__hdmi_power_off();
	printk("ite6610_drv_probe \n");
	if (ret) {
		printk( "cannot register miscdev on minor=%d (%d)\n",-1, ret);
	}



	return 0;
}

static int __devexit ite6610_drv_remove(struct platform_device *dev)
{
	misc_deregister(&it6610_miscdev);
	return 0;
}

static struct platform_driver ite6610_drv_driver = {
	.driver.name	= "ite6610-drv",
	.driver.owner	= THIS_MODULE,
	.probe		= ite6610_drv_probe,
	.remove		= ite6610_drv_remove,
};


static int __init ite6610_drv_init(void)
{
	/* Medive add it6610 proc*/
	struct proc_dir_entry *res;

#if 0
	__gpio_as_output(32*4+6);
	__gpio_clear_pin(32*4+6);
	mdelay(10);
	printk("Medive printk: 1222222222222222222222222212121\n");
#endif

	res = create_proc_entry("jz/it6610_me",0,NULL);
	if (res)
	{
		printk("Medive printk:  create proc :  it6610_me!\n");
		//res_tvout->owner = THIS_MODULE;
		res->read_proc = proc_it6610_read_proc;
		res->write_proc = proc_it6610_write_proc;
		res->data = NULL;
	}else{
		printk("Medive printk: can't create proc :  it6610_me!\n");
	}

	return platform_driver_register(&ite6610_drv_driver);
}

static void __exit ite6610_drv_exit(void)
{
	platform_driver_unregister(&ite6610_drv_driver);
}

module_init(ite6610_drv_init);
module_exit(ite6610_drv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ssg <estt501>");
MODULE_DESCRIPTION("hdmi ite6610 driver");
