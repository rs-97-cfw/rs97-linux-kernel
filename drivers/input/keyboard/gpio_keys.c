/*
 *  Keyboard driver for the IngeniC JZ SoC
 *
 *  Copyright (c) 2009 Ignacio Garcia Perez <iggarpe@gmail.com>
 *
 *  Mod: <maddrone@gmail.com> 
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  TODO(IGP):
 *  - On power slider long press, use 'o' instead of 'b' to power off instead of reboot.
 *  - Button wake up (when idle and low power stuff works).
 *
 */

#include <linux/init.h>
#include <linux/input-polldev.h>
#include <linux/module.h>
#include <linux/sysrq.h>
#include <linux/sched.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>		
#include <asm/processor.h>
#include <asm/jzsoc.h>
#include <linux/proc_fs.h>

#define SCAN_INTERVAL		(20)	/* (ms) */
#define POWER_INTERVAL		(2000)	/* (ms) */
#define POFF_INTERVAL		(5000)	/* (ms) */
#define POWER_COUNT		(POWER_INTERVAL / SCAN_INTERVAL)
#define POFF_COUNT		(POFF_INTERVAL / SCAN_INTERVAL)
#define CONFIG_JZ4750D_L009


/*
 * NOTE: maximum 32 GPIO, since we use bits in an unsigned long to store states
 */

static const struct {
	unsigned int gpio;
	unsigned int actlow;	/* Active low */
	unsigned int ncode;	/* Normal keycode */
	unsigned int scode;	/* Special keycode */
	unsigned int sysrq;	/* SYSRQ code */
	unsigned int wakeup;
} jz_button[] = {


#ifdef CONFIG_JZ4750D_L009
	{ .gpio = UMIDO_KEY_UP,	.actlow = 1,	.ncode = KEY_UP,	.scode = KEY_VOLUMEUP,		.sysrq = 's'	}, /* D-pad up */
	{ .gpio = UMIDO_KEY_DOWN,	.actlow = 1, 	.ncode = KEY_DOWN,	.scode = KEY_VOLUMEDOWN,	.sysrq = 'u'	}, /* D-pad down */
	{ .gpio = UMIDO_KEY_LEFT,	.actlow = 1,	.ncode = KEY_LEFT,	.scode = KEY_BRIGHTNESSDOWN,	.sysrq = 'e'	}, /* D-pad left */
	{ .gpio = UMIDO_KEY_RIGHT,	.actlow = 1,	.ncode = KEY_RIGHT,	.scode = KEY_BRIGHTNESSUP,	.sysrq = 'i'	}, /* D-pad right */
	{ .gpio = UMIDO_KEY_A,	.actlow = 1,	.ncode = KEY_LEFTCTRL,							}, /* A button */
	{ .gpio = UMIDO_KEY_B,	.actlow = 1,	.ncode = KEY_LEFTALT,							}, /* B button */
	{ .gpio = UMIDO_KEY_X,	.actlow = 1,	.ncode = KEY_SPACE,							}, /* X button */
	{ .gpio = UMIDO_KEY_Y,	.actlow = 1,	.ncode = KEY_LEFTSHIFT,							}, /* Y button */
	{ .gpio = UMIDO_KEY_START,	.actlow = 0,	.ncode = KEY_ENTER,							}, /* START button(SYSRQ) */
	{ .gpio = UMIDO_KEY_SELECT,	.actlow = 0,	.ncode = KEY_ESC,	.scode = KEY_MENU,		.sysrq = 'b'	}, /* SELECT button */
	{ .gpio = UMIDO_KEYL,	.actlow = 1,	.ncode = KEY_TAB,	.scode = KEY_END				}, /* Left shoulder button */
	{ .gpio = UMIDO_KEYR,	.actlow = 1,	.ncode = KEY_BACKSPACE,							}, /* Right shoulder button */
	{ .gpio = UMIDO_KEY_LED, .actlow = 1,	.ncode = KEY_3, 						},

//	{ .gpio = UMIDO_KEY_VOL_UP,	.actlow = 1,	.ncode = KEY_1,							}, /* START button(SYSRQ) */
//	{ .gpio = UMIDO_KEY_VOL_DOWN,	.actlow = 1,	.ncode = KEY_2,						}, /* START button(SYSRQ) */

//#define GPIO_POWER		(125)	/* Power slider */
#define GPIO_POWER_ACTLOW	1
//#define SYSRQ_ALT		10	/* Alternate sysrq button (index in jz_button table) */
#endif
};


struct jz_kbd {
	unsigned int			keycode [2 * ARRAY_SIZE(jz_button)];
	struct input_polled_dev *	poll_dev;
	unsigned long			normal_state;	/* Normal key state */
	unsigned long			special_state;	/* Special key state */
	unsigned long			sysrq_state;	/* SYSRQ key state */
	unsigned long			actlow;		/* Active low mask */
	unsigned int			power_state;	/* Power slider state */
	unsigned int			power_count;	/* Power slider active count */
};

static unsigned int sdl_key_enable = 1;
struct jz_kbd jz_gpio_kbd;

static int proc_alt_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%u\n", sdl_key_enable);
}

static int proc_alt_write_proc(
			struct file *file, const char *buffer,
			unsigned long count, void *data)
{
	sdl_key_enable =  simple_strtoul(buffer, 0, 10);
	return count;
}



//------------------------------------
extern void fb_resize_start();

static void jz_kbd_poll (struct input_polled_dev *dev)
{	
	struct jz_kbd *kbd = dev->private;
	struct input_dev *input = kbd->poll_dev->input;
	unsigned int i, sync = 0;
	unsigned long s, m, x;
		
	if(!sdl_key_enable)
		return;
	
	for (s = 0, m = 1, i = 0; i < ARRAY_SIZE(jz_button); i++, m <<= 1)
	{
		__gpio_enable_pull(jz_button[i].gpio);
		
		if (__gpio_get_pin(jz_button[i].gpio))
			s |= m;
	}

	//tvout  flush!
	fb_resize_start();  //medive change
	

#if 0 //adc key
	val = jz_read_key_handle();
	if (val){
		s &= (~val);
	}
#endif

	/* Invert active low buttons */
	s ^= kbd->actlow;
	
	/* Calculate changed button state for normal keycodes */
	x = s ^ kbd->normal_state;

	/* Generate normal keycodes for changed keys */
	for (i = 0, m = 1; i < ARRAY_SIZE(jz_button); i++, m <<= 1) {
		if ((x & m) && jz_button[i].ncode) {
			input_report_key(input, jz_button[i].ncode, ((s & m)?1:0) );
			sync++;
		}
	}
		
#if 1 //allen add power
	if (0 == __gpio_get_pin(GPIO_POWER_ON)) 
	{
		if(0 == kbd->special_state)
		{
			kbd->special_state = 1;
			input_report_key(input, KEY_END, 1);
			sync++;
		}
	}
	else 
	{
		if(1 == kbd->special_state)
		{
			kbd->special_state = 0;
			input_report_key(input, KEY_END, 0);
			sync++;
		}
	}
#endif
			
	kbd->power_count = 0;	/* Stop power slider pressed counter */
	kbd->power_state = 0;	/* Update power slider state */
	kbd->normal_state = s;	/* Update current normal button state */

	
	/* Synchronize input if any keycodes sent */
	if (sync) 
		input_sync(input);

}

static int __init jz_kbd_init(void)
{
	struct input_polled_dev *poll_dev;
	struct input_dev *input_dev;
	int i, j, error;
	struct proc_dir_entry *res;

	printk("jz-gpio-keys: scan interval %ums\n", SCAN_INTERVAL); 
	
	for(i=0; i<ARRAY_SIZE(jz_button); i++)
	{

#if 1
		if(i==8 && i ==9){   //select start
			__gpio_as_func0(jz_button[i].gpio);
			__gpio_as_input(jz_button[i].gpio);
			__gpio_disable_pull(jz_button[i].gpio);
		}else{
			__gpio_as_func0(jz_button[i].gpio);
			__gpio_as_input(jz_button[i].gpio);
			__gpio_enable_pull(jz_button[i].gpio);
		}
#else
		__gpio_as_func0(jz_button[i].gpio);
		__gpio_as_input(jz_button[i].gpio);
		__gpio_enable_pull(jz_button[i].gpio);
#endif
	}
	
	poll_dev = input_allocate_polled_device();
	if (!poll_dev) {
		error = -ENOMEM;
		goto fail;
	}

	jz_gpio_kbd.poll_dev = poll_dev;

	poll_dev->private = &jz_gpio_kbd;
	poll_dev->poll = jz_kbd_poll;
	poll_dev->poll_interval = SCAN_INTERVAL;

	input_dev = poll_dev->input;
	input_dev->name = "JZ GPIO keys";
	input_dev->phys = "jz-gpio-keys/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;

	/* Prepare active low mask and keycode array/bits */
	for (i = j = 0; i < ARRAY_SIZE(jz_button); i++) {
		if (jz_button[i].actlow) 
			jz_gpio_kbd.actlow |= 1 << i;
		if (jz_button[i].ncode)
		{
			jz_gpio_kbd.keycode[j++] = jz_button[i].ncode;
			__set_bit(jz_button[i].ncode, input_dev->keybit);
		}
		if (jz_button[i].scode)
		{
			jz_gpio_kbd.keycode[j++] = jz_button[i].scode;
			__set_bit(jz_button[i].scode, input_dev->keybit);
		}
	}

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP) | BIT_MASK(EV_SYN);
	input_dev->keycode = jz_gpio_kbd.keycode;
	input_dev->keycodesize = sizeof(jz_gpio_kbd.keycode[0]);
	input_dev->keycodemax = j;

	error = input_register_polled_device(jz_gpio_kbd.poll_dev);
	if (error) goto fail;

	res = create_proc_entry("jz/alt", 0, NULL);
	if(res)
	{
//		res->owner = THIS_MODULE;
		res->read_proc = proc_alt_read_proc;
		res->write_proc = proc_alt_write_proc;	
	}	

	return 0;

 fail:	input_free_polled_device(poll_dev);
	return error;
}

static void __exit jz_kbd_exit(void)
{
	input_unregister_polled_device(jz_gpio_kbd.poll_dev);
	input_free_polled_device(jz_gpio_kbd.poll_dev);
}

module_init(jz_kbd_init);
module_exit(jz_kbd_exit);

MODULE_AUTHOR("Ignacio Garcia Perez <iggarpe@gmail.com>");
MODULE_DESCRIPTION("JZ GPIO keys driver");
MODULE_LICENSE("GPLv2");
