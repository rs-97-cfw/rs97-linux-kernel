/*
 * linux/drivers/video/jz4760_lcd.c -- Ingenic Jz4760 LCD frame buffer device
 *
 * Copyright (C) 2005-2012, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * --------------------------------
 * NOTE:
 * This LCD driver support TFT16 TFT32 LCD, not support STN and Special TFT LCD
 * now.
 * 	It seems not necessory to support STN and Special TFT.
 * 	If it's necessary, update this driver in the future.
 * 	<Wolfgang Wang, Jun 10 2008>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>

#include "console/fbcon.h"

#include "jz4760_lcd.h"
#include "jz4760_tve.h"


#ifdef CONFIG_JZ4760_SLCD_KGM701A3_TFT_SPFD5420A
#include "jz_kgm_spfd5420a.h"
#endif

extern unsigned char *vmfbmem_addr;
extern u32 phy_vmfbmem_addr;
static int tvout_640_480 = 0;

//0: lcd   1 : 480X272->640X480    2:640X480
static unsigned int tvout_flag  = 0;

static unsigned int backlight_value = 80;
static int thread_init_end = 0;
static int resize_go_out = 0;
static int tvout_display_w = 320;
static int tvout_display_h = 240;
unsigned char *lcd_frame0,*lcd_frame01,*lcd_frame0_ipu_src;
#define TVOUT_2x

static unsigned int lcd_flush_flag = 1;
unsigned int panle_mode = 0;
static unsigned int tvout_flag2 = 0;
static unsigned int lcd_a320_flag = 0;
static struct timer_list avout_irq_timer;
static struct timer_list hdmi_irq_timer;
int panel_mode = PANEL_MODE_LCD_PANEL;
static int last_panel_mode = PANEL_MODE_LCD_PANEL;

extern unsigned int l009_globle_volume;

extern void ipu_driver_close_tv(void);
extern void ipu_driver_open_tv(int,int,int,int);
extern void ipu_driver_flush_tv(void);
extern void ipu_driver_wait_end();

static void fb_resize_a320_original_start();
static void fb_resize_a320_fullscreen_start();

#if 1
#define D(fmt, args...)
#else
#define D(fmt, args...) \
	printk(KERN_INFO "%s(): "fmt"\n", __func__, ##args)
#endif

#define JZ_FB_DEBUG 0
static int lcd_backlight_level = 102;
struct jz4760lcd_info jz4760_lcd_panel = {
#if defined(CONFIG_JZ4760_LCD_SAMSUNG_LTP400WQF02)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* General TFT panel */
		LCD_CFG_MODE_TFT_18BIT | 	/* output 18bpp */
		LCD_CFG_HSP | 	/* Hsync polarity: active low */
		LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		480, 272, 60, 41, 10, 2, 2, 2, 2,
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {32, 0, 0, 480, 272}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 720, 573}, /* bpp, x, y, w, h */
	 },
#elif defined(CONFIG_JZ4760_LCD_AUO_A043FL01V2)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* General TFT panel */
		LCD_CFG_MODE_TFT_24BIT | 	/* output 18bpp */
		LCD_CFG_HSP | 	/* Hsync polarity: active low */
		LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		480, 272, 60, 41, 10, 8, 4, 4, 2,
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
//		 LCD_OSDC_F1EN | /* enable Foreground1 */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {16, 0, 0, 480, 272}, /* bpp, x, y, w, h */
		 .fg1 = {16, 0, 0, 720, 573}, /* bpp, x, y, w, h */
	 },
#elif defined(CONFIG_JZ4760_LCD_TOPPOLY_TD043MGEB1)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* General TFT panel */
		LCD_CFG_MODE_TFT_24BIT | 	/* output 18bpp */
		LCD_CFG_HSP | 	/* Hsync polarity: active low */
		LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_64,	/* 16words burst, enable out FIFO underrun irq */
		800, 480, 60, 1, 1, 40, 215, 10, 34,
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
//		 LCD_OSDC_F1EN | /* enable Foreground1 */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x0, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {32, 0, 0, 800, 480}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 800, 480}, /* bpp, x, y, w, h */
	 },
#elif defined(CONFIG_JZ4760_LCD_UMIDO_L430)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* Serial TFT panel */
		LCD_CFG_MODE_TFT_24BIT | 	/* output 24bpp */
		LCD_CFG_HSP | 	/* Hsync polarity: active low */
		LCD_CFG_VSP ,	/* Vsync polarity: leading edge is falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
                //480, 272,  100, 1, 1, 40, 215, 0, 45,
                480, 272,  60, 41, 10, 2, 2, 2, 2,
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
		 //LCD_OSDC_ALPHAEN | /* enable alpha */
		 //LCD_OSDC_F1EN |	/* enable Foreground0 */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0x80000000, /* disable colorkey */
		 .colorkey1 = 0x80000000, /* disable colorkey */
		 .alpha = 0xa0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {16, 0, 0, 480, 272}, /* bpp, x, y, w, h */
		 .fg1 = {16, 0, 0, 480, 272}, /* bpp, x, y, w, h */
	 },

#elif defined(CONFIG_JZ4760_LCD_TM370_LN430_9)
    .panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */ 
			   LCD_CFG_MODE_SERIAL_TFT | /* General TFT panel */
			   LCD_CFG_MODE_TFT_16BIT |
			   LCD_CFG_PCP |
			   LCD_CFG_NEWDES, /* 8words descriptor */

		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
#if defined(RS97_AUO)
		320, 480,  60, 20, 1, 48, 40, 18,27, //auo
#elif defined(RS97_WEIRDSHIT)
		320, 480, 120, 20, 1, 48, 40, 10, 42,  //ILI8965
#elif defined(RS97_INNOLUX)
		320, 480, 120, 20, 1, 32, 40, 17, 27,	//INNOLUX
#endif
	},

	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
		 //LCD_OSDC_ALPHAEN | /* enable alpha */
		 //LCD_OSDC_F1EN |	/* enable Foreground0 */
		 LCD_OSDC_F0EN, /* enable Foreground0 */
		 .osd_ctrl = 0, 	/* disable ipu,  */
		 .rgb_ctrl = LCD_RGBC_EVEN_GBR << LCD_RGBC_EVENRGB_BIT,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0x80000000, /* disable colorkey */
		 .colorkey1 = 0x80000000, /* disable colorkey */
		 .alpha = 0xa0, /* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {16, 0, 0, 320,480}, /* bpp, x, y, w, h */
		 .fg1 = {16, 0, 0, 320,480}, /* bpp, x, y, w, h */
	 },
 
#elif defined(CONFIG_JZ4760_LCD_TRULY_TFT_GG1P0319LTSW_W)
	.panel = {
		 .cfg = LCD_CFG_LCDPIN_SLCD | /* Underrun recover*/
		 LCD_CFG_NEWDES | /* 8words descriptor */
		 LCD_CFG_MODE_SLCD, /* TFT Smart LCD panel */
		 .slcd_cfg = SLCD_CFG_DWIDTH_16BIT | SLCD_CFG_CWIDTH_16BIT | SLCD_CFG_CS_ACTIVE_LOW | SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING | SLCD_CFG_TYPE_PARALLEL,
		 .ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		 240, 320, 60, 0, 0, 0, 0, 0, 0,
	 },
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
//		 LCD_OSDC_F1EN | /* enable Foreground0 */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {32, 0, 0, 240, 320}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 240, 320}, /* bpp, x, y, w, h */
	 },

#elif defined(CONFIG_JZ4760_LCD_FOXCONN_PT035TN01)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* General TFT panel */
//		LCD_CFG_MODE_TFT_18BIT | 	/* output 18bpp */
		LCD_CFG_MODE_TFT_24BIT | 	/* output 24bpp */
		LCD_CFG_HSP | 	/* Hsync polarity: active low */
		LCD_CFG_VSP |	/* Vsync polarity: leading edge is falling edge */
		LCD_CFG_PCP,	/* Pix-CLK polarity: data translations at falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		320, 240, 80, 1, 1, 10, 50, 10, 13
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
//		 LCD_OSDC_F1EN |	/* enable Foreground1 */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {32, 0, 0, 320, 240}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 320, 240}, /* bpp, x, y, w, h */
	 },
#elif defined(CONFIG_JZ4760_LCD_INNOLUX_PT035TN01_SERIAL)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_SERIAL_TFT | /* Serial TFT panel */
		LCD_CFG_MODE_TFT_18BIT | 	/* output 18bpp */
		LCD_CFG_HSP | 	/* Hsync polarity: active low */
		LCD_CFG_VSP |	/* Vsync polarity: leading edge is falling edge */
		LCD_CFG_PCP,	/* Pix-CLK polarity: data translations at falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		320, 240, 60, 1, 1, 10, 50, 10, 13
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {32, 0, 0, 320, 240}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 320, 240}, /* bpp, x, y, w, h */
	 },
#elif defined(CONFIG_JZ4760_SLCD_KGM701A3_TFT_SPFD5420A)
	.panel = {
//		 .cfg = LCD_CFG_LCDPIN_SLCD | LCD_CFG_RECOVER | /* Underrun recover*/
		 .cfg = LCD_CFG_LCDPIN_SLCD | /* Underrun recover*/
//		 LCD_CFG_DITHER | /* dither */
		 LCD_CFG_NEWDES | /* 8words descriptor */
		 LCD_CFG_MODE_SLCD, /* TFT Smart LCD panel */
		 .slcd_cfg = SLCD_CFG_DWIDTH_18BIT | SLCD_CFG_CWIDTH_18BIT | SLCD_CFG_CS_ACTIVE_LOW | SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING | SLCD_CFG_TYPE_PARALLEL,
		 .ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		 400, 240, 60, 0, 0, 0, 0, 0, 0,
	 },
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
//		 LCD_OSDC_ALPHAMD | /* alpha blending mode */
//		 LCD_OSDC_F1EN | /* enable Foreground1 */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
//		 .fg0 = {32, 0, 0, 400, 240}, /* bpp, x, y, w, h */
		 .fg0 = {32, 0, 0, 320, 240}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 400, 240}, /* bpp, x, y, w, h */
	 },
#elif defined(CONFIG_JZ_VGA_DISPLAY_DEFAULT)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER |/* Underrun recover */
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* General TFT panel */
		LCD_CFG_MODE_TFT_24BIT | 	/* output 18bpp */
		LCD_CFG_HSP | 	/* Hsync polarity: active low */
		LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		800, 600, 60, 128, 4, 40, 88, 1, 23
//		640, 480, 60, 96, 2, 16, 48, 10, 33
//		1280, 720, 50, 152, 15, 22, 200, 14, 1
//		1024, 768, 62, 136, 6, 24, 160, 3, 29
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
//		 LCD_OSDC_F1EN | /* enable Foreground1 */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {32, 0, 0, 640, 480}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 640, 480}, /* bpp, x, y, w, h */
	 },
#elif defined(CONFIG_JZ_VGA_DISPLAY_1024x768_60)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER |/* Underrun recover */
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* General TFT panel */
		LCD_CFG_MODE_TFT_24BIT | 	/* output 18bpp */
		LCD_CFG_HSP | 	/* Hsync polarity: active low */
		LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
		.slcd_cfg = 0,
//		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_64,
//		1024, 768, 62, 136, 6, 24, 160, 3, 29
		1024, 768, 60, 136, 6, 24, 160, 3, 29
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
//		 LCD_OSDC_F1EN | /* enable Foreground1 */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
//		 .fg0 = {32, 224/2, 288/2, (1024-224), (768-288)}, /* bpp, x, y, w, h */
//		 .fg1 = {32, 224/2, 288/2, (1024-224), (768-288)}, /* bpp, x, y, w, h */
//		 .fg0 = {32, 224/2, 168/2, (1024-224), (768-168)}, /* bpp, x, y, w, h */
//		 .fg1 = {32, 224/2, 168/2, (1024-224), (768-168)}, /* bpp, x, y, w, h */
		 .fg0 = {32, 16/2, 12/2, (1024-16), (768-12)}, /* bpp, x, y, w, h */
		 .fg1 = {32, 16/2, 12/2, (1024-16), (768-12)}, /* bpp, x, y, w, h */
	 },
#elif defined(CONFIG_JZ4760_LCD_HYNIX_HT12X14)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */ 
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* General TFT panel */
		LCD_CFG_MODE_TFT_18BIT|(0<<9)|	/* output 16bpp */
		LCD_CFG_PCP |	/* PCLK polarity: the falling edge acts as data strobe*/
		LCD_CFG_HSP |	/* Hsync polarity: active low */
		LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_64,	/* 16words burst, enable out FIFO underrun irq */
		//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
		1024, 768, 65, 1, 1, 75, 0, 3, 0,
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
		 LCD_OSDC_ALPHAEN | /* enable alpha */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x0, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
//		 .fg0 = {32, 0, 0, 1024, 768}, /* bpp, x, y, w, h */
		 .fg0 = {32, 0, 0, 320, 240}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 320, 240}, /* bpp, x, y, w, h */
		},

#elif defined(CONFIG_JZ4760_LCD_KD101N2)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */ 
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* General TFT panel */
		LCD_CFG_MODE_TFT_18BIT|(0<<9)|	/* output 16bpp */
		LCD_CFG_PCP |	/* PCLK polarity: the falling edge acts as data strobe*/
		LCD_CFG_HSP |	/* Hsync polarity: active low */
		LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16_CTN,	/* 16words burst, enable out FIFO underrun irq */
		//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
		1024, 600, 45, 1, 1, 320, 0, 35, 0,
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
		 LCD_OSDC_ALPHAEN | /* enable alpha */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x0, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {32, 0, 0, 1024, 600}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 1024, 600}, /* bpp, x, y, w, h */
		},

#elif defined(CONFIG_JZ4760_LCD_KD101N4)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */ 
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* General TFT panel */
		LCD_CFG_MODE_TFT_18BIT|(0<<9)|	/* output 16bpp */
		LCD_CFG_PCP |	/* PCLK polarity: the falling edge acts as data strobe*/
		LCD_CFG_HSP |	/* Hsync polarity: active low */
		LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16_CTN,	/* 16words burst, enable out FIFO underrun irq */
		//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
		1280, 800, 27, 1, 1, 168, 32, 16, 0,
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
		 LCD_OSDC_ALPHAEN | /* enable alpha */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x0, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {32, 0, 0, 1280, 800}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 1280, 800}, /* bpp, x, y, w, h */
		},

#else
#error "Select LCD panel first!!!"
#endif
};

#if defined(CONFIG_JZ4760_HDMI_DISPLAY)  
#define   AIC_FR_TFTH_BIT         16
#define   AIC_FR_RFTH_BIT         24

#define PANEL_MODE_HDMI_480P   	 3
#define PANEL_MODE_HDMI_576P   	 4
#define PANEL_MODE_HDMI_720P50 	 5
#define PANEL_MODE_HDMI_720P60 	 6
#define PANEL_MODE_HDMI_1080P30  7
#define PANEL_MODE_HDMI_1080P50  8
#define PANEL_MODE_HDMI_1080P60  9

struct jz4760lcd_info jz4760_info_hdmi_480p = {                            
        .panel = {                                                     
                .cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
                       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
                       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
                .slcd_cfg = 0,                                            
		.ctrl = LCD_CTRL_BST_32,                                  
		//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
		640,480, 60, 96, 2,48,16,  33,10,       //HDMI-480P       
		//800,600,58,128,4,88,40,23,1,i
		//1024,768,60,136,6,160,24,29,3,
	},                                      
        .osd = {                                
                .osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
                 //LCD_OSDC_ALPHAEN                       |               // enable alpha
                 LCD_OSDC_F0EN                          ,               // enable Foreground0    
                // LCD_OSDC_F1EN,                                         // enable Foreground1    
		
                 .osd_ctrl = 0,                                         // disable ipu,          
                 .rgb_ctrl = 0,                                                                  
                 .bgcolor = 0x000000,                           // set background color Black    
                 .colorkey0 = 0,                                        // disable colorkey
                 .colorkey1 = 0,                                        // disable colorkey
                 .alpha = 0xa0,                                         // alpha value
                 .ipu_restart = 0x8000085d,                     // ipu restart
                 .fg_change = FG_CHANGE_ALL,            // change all initially
		 .fg0 = {32, 0, 0, 640, 480},   // bpp, x, y, w, h
                 .fg1 = {32, 0, 0, 640, 480},       // bpp, x, y, w, h
        },
};
struct jz4760lcd_info jz4760_info_hdmi_576p = {                            
        .panel = {                                                     
                .cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
                       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
                       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
                .slcd_cfg = 0,                                            
		.ctrl = LCD_CTRL_BST_32,                                  
		//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
		720,576,50,64,5,68,12,40,4,                               
		//800,600,58,128,4,88,40,23,1,i
		//1024,768,60,136,6,160,24,29,3,
	},                                      
        .osd = {                                
                .osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
                 LCD_OSDC_ALPHAEN                       |               // enable alpha
                 LCD_OSDC_F0EN                          ,               // enable Foreground0    
                // LCD_OSDC_F1EN,                                         // enable Foreground1    
		
                 .osd_ctrl = 0,                                         // disable ipu,          
                 .rgb_ctrl = 0,                                                                  
                 .bgcolor = 0x000000,                           // set background color Black    
                 .colorkey0 = 0,                                        // disable colorkey
                 .colorkey1 = 0,                                        // disable colorkey
                 .alpha = 0xa0,                                         // alpha value
                 .ipu_restart = 0x8000085d,                     // ipu restart
                 .fg_change = FG_CHANGE_ALL,            // change all initially
		 .fg0 = {32, 0, 0, 720, 576},   // bpp, x, y, w, h
                 .fg1 = {32, 0, 0, 720, 576},       // bpp, x, y, w, h
        },
};
struct jz4760lcd_info jz4760_info_hdmi_720p50 = {                            
        .panel = {                                                     
                .cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
                       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
                       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
                .slcd_cfg = 0,                                            
		.ctrl = LCD_CTRL_BST_32,                                  
		//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
                1280,720,50,40,5,440,220,20,5,
	//	1280, 720, 50, 152, 15, 22, 200, 14, 1
	//	1280,720,46,1,1,121,259,6,19,                       
		//800,600,58,128,4,88,40,23,1,i
		//1024,768,60,136,6,160,24,29,3,
	},                                      
        .osd = {                                
                .osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
                 LCD_OSDC_ALPHAEN                       |               // enable alpha
                 LCD_OSDC_F0EN                          ,               // enable Foreground0    
                // LCD_OSDC_F1EN,                                         // enable Foreground1    
		
                 .osd_ctrl = 0,                                         // disable ipu,          
                 .rgb_ctrl = 0,                                                                  
                 .bgcolor = 0x000000,                           // set background color Black    
                 .colorkey0 = 0,                                        // disable colorkey
                 .colorkey1 = 0,                                        // disable colorkey
		.alpha = 0xa0,                                         // alpha value
		.ipu_restart = 0x8000085d,                     // ipu restart
		.fg_change = FG_CHANGE_ALL,            // change all initially
		.fg0 = {32, 0, 0, 1280, 720},   // bpp, x, y, w, h
                 .fg1 = {32, 0, 0, 1280, 720},       // bpp, x, y, w, h
        },
};
struct jz4760lcd_info jz4760_info_hdmi_720p60 = {                            
        .panel = {                                                     
                .cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
                       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
                       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
                .slcd_cfg = 0,                                            
		.ctrl = LCD_CTRL_BST_32,                                  
		//  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
		1280,720,60,40,5,110,220,20,5,//74250000                  
		//800,600,58,128,4,88,40,23,1,i
		//1024,768,60,136,6,160,24,29,3,
	},                                      
        .osd = {                                
                .osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
                 LCD_OSDC_ALPHAEN                       |               // enable alpha
                 LCD_OSDC_F0EN                          ,               // enable Foreground0    
                // LCD_OSDC_F1EN,                                         // enable Foreground1    
		
                 .osd_ctrl = 0,                                         // disable ipu,          
                 .rgb_ctrl = 0,                                                                  
                 .bgcolor = 0x000000,                           // set background color Black    
                 .colorkey0 = 0,                                        // disable colorkey
                 .colorkey1 = 0,                                        // disable colorkey
                 .alpha = 0xa0,                                         // alpha value
                 .ipu_restart = 0x8000085d,                     // ipu restart
                 .fg_change = FG_CHANGE_ALL,            // change all initially
		 .fg0 = {32, 0, 0, 1280, 720},   // bpp, x, y, w, h
                 .fg1 = {32, 0, 0, 1280, 720},       // bpp, x, y, w, h
        },
};
struct jz4760lcd_info jz4760_info_hdmi_1080p30 = {
        .panel = {
                .cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
                       LCD_CFG_NEWDES | LCD_CFG_RECOVER |
                       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,
                .slcd_cfg = 0,
                .ctrl = LCD_CTRL_BST_32,
                //  width,height,freq,hsync,vsync,elw,blw,efw,bfw
                1920,1080,30,44,5,148,88,36,4
                //800,600,58,128,4,88,40,23,1,i
                //1024,768,60,136,6,160,24,29,3,
        },
        .osd = {
                .osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
                 LCD_OSDC_ALPHAEN                       |               // enable alpha
                 LCD_OSDC_F0EN                          ,               // enable Foreground0
                // LCD_OSDC_F1EN,                                         // enable Foreground1

                 .osd_ctrl = 0,                                         // disable ipu,
                 .rgb_ctrl = 0,
                 .bgcolor = 0x000000,                           // set background color Black
                 .colorkey0 = 0,                                        // disable colorkey
                 .colorkey1 = 0,                                        // disable colorkey
                 .alpha = 0xa0,                                         // alpha value
                 .ipu_restart = 0x8000085d,                     // ipu restart
                 .fg_change = FG_CHANGE_ALL,            // change all initially
                 .fg0 = {32, 0, 0, 1920, 1080},   // bpp, x, y, w, h
                 .fg1 = {32, 0, 0, 1920, 1080},       // bpp, x, y, w, h
        },
};




struct jz4760lcd_info jz4760_info_hdmi_1080p50 = {
        .panel = {
                .cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
                       LCD_CFG_NEWDES | LCD_CFG_RECOVER |
                       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,
                .slcd_cfg = 0,
                .ctrl = LCD_CTRL_BST_32|LCD_CTRL_OFUM,
                //  width,height,freq,hsync,vsync,elw,blw,efw,bfw
                1920,1080,50,44,5,148,528,36,4
                //800,600,58,128,4,88,40,23,1,i
                //1024,768,60,136,6,160,24,29,3,
        },
        .osd = {
                .osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
                 LCD_OSDC_ALPHAEN                       |               // enable alpha
                 LCD_OSDC_F0EN                          ,               // enable Foreground0
                // LCD_OSDC_F1EN,                                         // enable Foreground1

                 .osd_ctrl = 0,                                         // disable ipu,
                 .rgb_ctrl = 0,
                 .bgcolor = 0x000000,                           // set background color Black
                 .colorkey0 = 0,                                        // disable colorkey
                 .colorkey1 = 0,                                        // disable colorkey
                 .alpha = 0xa0,                                         // alpha value
                 .ipu_restart = 0x8000085d,                     // ipu restart
                 .fg_change = FG_CHANGE_ALL,            // change all initially
                 .fg0 = {32, 0, 0, 1920, 1080},   // bpp, x, y, w, h
                 .fg1 = {32, 0, 0, 1920, 1080},       // bpp, x, y, w, h
        },
};



struct jz4760lcd_info jz4760_info_hdmi_1080p60 = {                            
        .panel = {                                                     
                .cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
                       LCD_CFG_NEWDES | LCD_CFG_RECOVER |                 
                       LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,           
                .slcd_cfg = 0,                                            
		.ctrl = LCD_CTRL_BST_64|LCD_CTRL_OFUM,                                  //  width,height,freq,hsync,vsync,elw,blw,efw,bfw         
		1920,1080,60,44,5,148,88,37,4	                 
		//800,600,58,128,4,88,40,23,1,i
		//1024,768,60,136,6,160,24,29,3,
	},            
        .osd = {                                
                .osd_cfg =  LCD_OSDC_OSDEN      |               // Use OSD mode
                 LCD_OSDC_ALPHAEN                       |               // enable alpha
                 LCD_OSDC_F0EN                          ,               // enable Foreground0    
                // LCD_OSDC_F1EN,                                         // enable Foreground1    
		
                 .osd_ctrl = 0,                                         // disable ipu,          
                 .rgb_ctrl = 0,                                                                  
                 .bgcolor = 0x000000,                           // set background color Black    
                 .colorkey0 = 0,                                        // disable colorkey
                 .colorkey1 = 0,                                        // disable colorkey
                 .alpha = 0xa0,                                         // alpha value
                 .ipu_restart = 0x8000085d,                     // ipu restart
                 .fg_change = FG_CHANGE_ALL,            // change all initially
		 .fg0 = {32, 0, 0, 1920, 1080},   // bpp, x, y, w, h
                 .fg1 = {32, 0, 0, 1920, 1080},       // bpp, x, y, w, h
        },
};

extern void jz4760b_i2s_set_external_codec(void);
extern void jz4760b_i2s_set_internal_codec(void);
static int i2s_external_flag = 0;
static void set_i2s_external_codec(void)
{
	i2s_external_flag = 1;
	__gpio_as_func1(3*32 + 12); //blck
	__gpio_as_func0(3*32 + 13); //sync
	__gpio_as_func0(4*32 + 7);  //sd0
	jz4760b_i2s_set_external_codec();

}

static void set_i2s_internal_codec(void)
{
	if (i2s_external_flag == 1){
		jz4760b_i2s_set_internal_codec();
		i2s_external_flag = 0;
	}
}
#endif


struct jz4760lcd_info jz4760_info_tve = {
	.panel = {
		.cfg = LCD_CFG_TVEN | /* output to tve */
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_RECOVER | /* underrun protect */
		LCD_CFG_MODE_INTER_CCIR656, /* Interlace CCIR656 mode */
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst */
		TVE_WIDTH_PAL, TVE_HEIGHT_PAL, TVE_FREQ_PAL, 0, 0, 0, 0, 0, 0,
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
		// LCD_OSDC_ALPHAEN | /* enable alpha */
		// LCD_OSDC_F1EN|	/* enable Foreground0 */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = LCD_RGBC_YCC, /* enable RGB => YUV */
		 .bgcolor = 0x00000000, /* set background color Black */
		 .colorkey0 = 0x80000000, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80000100, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {16,},	/*  */
		 .fg1 = {16,},
	},
};

struct jz4760lcd_info *jz4760_lcd_info = &jz4760_lcd_panel; /* default output to lcd panel */
int LCD_SCREEN_W;
int LCD_SCREEN_H;

#if JZ_FB_DEBUG
void print_lcdc_registers(void)	/* debug */
{
	/* LCD Controller Resgisters */
	printk("REG_LCD_CFG:\t0x%08x\n", REG_LCD_CFG);
	printk("REG_LCD_CTRL:\t0x%08x\n", REG_LCD_CTRL);
	printk("REG_LCD_STATE:\t0x%08x\n", REG_LCD_STATE);
	printk("REG_LCD_OSDC:\t0x%08x\n", REG_LCD_OSDC);
	printk("REG_LCD_OSDCTRL:\t0x%08x\n", REG_LCD_OSDCTRL);
	printk("REG_LCD_OSDS:\t0x%08x\n", REG_LCD_OSDS);
	printk("REG_LCD_BGC:\t0x%08x\n", REG_LCD_BGC);
	printk("REG_LCD_KEK0:\t0x%08x\n", REG_LCD_KEY0);
	printk("REG_LCD_KEY1:\t0x%08x\n", REG_LCD_KEY1);
	printk("REG_LCD_ALPHA:\t0x%08x\n", REG_LCD_ALPHA);
	printk("REG_LCD_IPUR:\t0x%08x\n", REG_LCD_IPUR);
	printk("REG_LCD_VAT:\t0x%08x\n", REG_LCD_VAT);
	printk("REG_LCD_DAH:\t0x%08x\n", REG_LCD_DAH);
	printk("REG_LCD_DAV:\t0x%08x\n", REG_LCD_DAV);
	printk("REG_LCD_XYP0:\t0x%08x\n", REG_LCD_XYP0);
	printk("REG_LCD_XYP1:\t0x%08x\n", REG_LCD_XYP1);
	printk("REG_LCD_SIZE0:\t0x%08x\n", REG_LCD_SIZE0);
	printk("REG_LCD_SIZE1:\t0x%08x\n", REG_LCD_SIZE1);
	printk("REG_LCD_RGBC\t0x%08x\n", REG_LCD_RGBC);
	printk("REG_LCD_VSYNC:\t0x%08x\n", REG_LCD_VSYNC);
	printk("REG_LCD_HSYNC:\t0x%08x\n", REG_LCD_HSYNC);
	printk("REG_LCD_PS:\t0x%08x\n", REG_LCD_PS);
	printk("REG_LCD_CLS:\t0x%08x\n", REG_LCD_CLS);
	printk("REG_LCD_SPL:\t0x%08x\n", REG_LCD_SPL);
	printk("REG_LCD_REV:\t0x%08x\n", REG_LCD_REV);
	printk("REG_LCD_IID:\t0x%08x\n", REG_LCD_IID);
	printk("REG_LCD_DA0:\t0x%08x\n", REG_LCD_DA0);
	printk("REG_LCD_SA0:\t0x%08x\n", REG_LCD_SA0);
	printk("REG_LCD_FID0:\t0x%08x\n", REG_LCD_FID0);
	printk("REG_LCD_CMD0:\t0x%08x\n", REG_LCD_CMD0);
	printk("REG_LCD_OFFS0:\t0x%08x\n", REG_LCD_OFFS0);
	printk("REG_LCD_PW0:\t0x%08x\n", REG_LCD_PW0);
	printk("REG_LCD_CNUM0:\t0x%08x\n", REG_LCD_CNUM0);
	printk("REG_LCD_DESSIZE0:\t0x%08x\n", REG_LCD_DESSIZE0);
	printk("REG_LCD_DA1:\t0x%08x\n", REG_LCD_DA1);
	printk("REG_LCD_SA1:\t0x%08x\n", REG_LCD_SA1);
	printk("REG_LCD_FID1:\t0x%08x\n", REG_LCD_FID1);
	printk("REG_LCD_CMD1:\t0x%08x\n", REG_LCD_CMD1);
	printk("REG_LCD_OFFS1:\t0x%08x\n", REG_LCD_OFFS1);
	printk("REG_LCD_PW1:\t0x%08x\n", REG_LCD_PW1);
	printk("REG_LCD_CNUM1:\t0x%08x\n", REG_LCD_CNUM1);
	printk("REG_LCD_DESSIZE1:\t0x%08x\n", REG_LCD_DESSIZE1);
	printk("==================================\n");
	printk("REG_LCD_VSYNC:\t%d:%d\n", REG_LCD_VSYNC>>16, REG_LCD_VSYNC&0xfff);
	printk("REG_LCD_HSYNC:\t%d:%d\n", REG_LCD_HSYNC>>16, REG_LCD_HSYNC&0xfff);
	printk("REG_LCD_VAT:\t%d:%d\n", REG_LCD_VAT>>16, REG_LCD_VAT&0xfff);
	printk("REG_LCD_DAH:\t%d:%d\n", REG_LCD_DAH>>16, REG_LCD_DAH&0xfff);
	printk("REG_LCD_DAV:\t%d:%d\n", REG_LCD_DAV>>16, REG_LCD_DAV&0xfff);
	printk("==================================\n");

	/* Smart LCD Controller Resgisters */
	printk("REG_SLCD_CFG:\t0x%08x\n", REG_SLCD_CFG);
	printk("REG_SLCD_CTRL:\t0x%08x\n", REG_SLCD_CTRL);
	printk("REG_SLCD_STATE:\t0x%08x\n", REG_SLCD_STATE);
	printk("==================================\n");

	/* TVE Controller Resgisters */
	printk("REG_TVE_CTRL:\t0x%08x\n", REG_TVE_CTRL);
	printk("REG_TVE_FRCFG:\t0x%08x\n", REG_TVE_FRCFG);
	printk("REG_TVE_SLCFG1:\t0x%08x\n", REG_TVE_SLCFG1);
	printk("REG_TVE_SLCFG2:\t0x%08x\n", REG_TVE_SLCFG2);
	printk("REG_TVE_SLCFG3:\t0x%08x\n", REG_TVE_SLCFG3);
	printk("REG_TVE_LTCFG1:\t0x%08x\n", REG_TVE_LTCFG1);
	printk("REG_TVE_LTCFG2:\t0x%08x\n", REG_TVE_LTCFG2);
	printk("REG_TVE_CFREQ:\t0x%08x\n", REG_TVE_CFREQ);
	printk("REG_TVE_CPHASE:\t0x%08x\n", REG_TVE_CPHASE);
	printk("REG_TVE_CBCRCFG:\t0x%08x\n", REG_TVE_CBCRCFG);
	printk("REG_TVE_WSSCR:\t0x%08x\n", REG_TVE_WSSCR);
	printk("REG_TVE_WSSCFG1:\t0x%08x\n", REG_TVE_WSSCFG1);
	printk("REG_TVE_WSSCFG2:\t0x%08x\n", REG_TVE_WSSCFG2);
	printk("REG_TVE_WSSCFG3:\t0x%08x\n", REG_TVE_WSSCFG3);

	printk("==================================\n");
#if 0
	if ( 0 ) {
		unsigned int * pii = (unsigned int *)dma_desc_base;
		int i, j;
		for (j=0;j< DMA_DESC_NUM ; j++) {
			printk("dma_desc%d(0x%08x):\n", j, (unsigned int)pii);
			for (i =0; i<8; i++ ) {
				printk("\t\t0x%08x\n", *pii++);
			}
		}
	}
#endif
}
#else
void print_lcdc_registers(void)	/* debug */
{
}
#endif

struct lcd_cfb_info {
	struct fb_info		fb;
	struct {
		u16 red, green, blue;
	} palette[NR_PALETTE];

	int b_lcd_display;
	int b_lcd_pwm;
	int backlight_level;
};

static struct lcd_cfb_info *jz4760fb_info;
static struct jz4760_lcd_dma_desc *dma_desc_base;
static struct jz4760_lcd_dma_desc *dma0_desc_palette, *dma0_desc0, *dma0_desc1, *dma1_desc0, *dma1_desc1;

#define DMA_DESC_NUM 		6

static unsigned char *lcd_palette;
unsigned char *lcd_frame1;

static struct jz4760_lcd_dma_desc *dma0_desc_cmd0, *dma0_desc_cmd;
static unsigned char *lcd_cmdbuf;

static void jz4760fb_set_mode( struct jz4760lcd_info * lcd_info );
static void jz4760fb_deep_set_mode( struct jz4760lcd_info * lcd_info );

static int jz4760fb_set_backlight_level(int n);

static int screen_on(void);
static int screen_off(void);

int get_lcd_width(void)
{
	return LCD_SCREEN_W;
}
int get_lcd_hight(void)
{
	return LCD_SCREEN_H;
}
EXPORT_SYMBOL(get_lcd_width);
EXPORT_SYMBOL(get_lcd_hight);



static void ctrl_enable(void)
{
	REG_LCD_STATE = 0; /* clear lcdc status */
	__lcd_slcd_special_on();
	__lcd_clr_dis();
	__lcd_set_ena(); /* enable lcdc */

	return;
}

static void ctrl_disable(void)
{
	if ( jz4760_lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD ||
			jz4760_lcd_info->panel.cfg & LCD_CFG_TVEN ) /*  */
		__lcd_clr_ena(); /* Smart lcd and TVE mode only support quick disable */
	else {
		int cnt;
		/* when CPU main freq is 336MHz,wait for 30ms */
		cnt = 528000 * 3;
		__lcd_set_dis(); /* regular disable */
		//__lcd_clr_ena();
		while(!__lcd_disable_done() && cnt) {
			cnt--;
		}

		if (cnt == 0)
			printk("LCD disable timeout! REG_LCD_STATE=0x%08xx\n",REG_LCD_STATE);
		REG_LCD_STATE &= ~LCD_STATE_LDD;
	}

	return;
}

static inline u_int chan_to_field(u_int chan, struct fb_bitfield *bf)
{
        chan &= 0xffff;
        chan >>= 16 - bf->length;
        return chan << bf->offset;
}

static int jz4760fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			  u_int transp, struct fb_info *info)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	unsigned short *ptr, ctmp;

//	D("regno:%d,RGBt:(%d,%d,%d,%d)\t", regno, red, green, blue, transp);
	if (regno >= NR_PALETTE)
		return 1;

	cfb->palette[regno].red		= red ;
	cfb->palette[regno].green	= green;
	cfb->palette[regno].blue	= blue;
	if (cfb->fb.var.bits_per_pixel <= 16) {
		red	>>= 8;
		green	>>= 8;
		blue	>>= 8;

		red	&= 0xff;
		green	&= 0xff;
		blue	&= 0xff;
	}
	switch (cfb->fb.var.bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
		if (((jz4760_lcd_info->panel.cfg & LCD_CFG_MODE_MASK) == LCD_CFG_MODE_SINGLE_MSTN ) ||
		    ((jz4760_lcd_info->panel.cfg & LCD_CFG_MODE_MASK) == LCD_CFG_MODE_DUAL_MSTN )) {
			ctmp = (77L * red + 150L * green + 29L * blue) >> 8;
			ctmp = ((ctmp >> 3) << 11) | ((ctmp >> 2) << 5) |
				(ctmp >> 3);
		} else {
			/* RGB 565 */
			if (((red >> 3) == 0) && ((red >> 2) != 0))
			red = 1 << 3;
			if (((blue >> 3) == 0) && ((blue >> 2) != 0))
				blue = 1 << 3;
			ctmp = ((red >> 3) << 11)
				| ((green >> 2) << 5) | (blue >> 3);
		}

		ptr = (unsigned short *)lcd_palette;
		ptr = (unsigned short *)(((u32)ptr)|0xa0000000);
		ptr[regno] = ctmp;

		break;

	case 15:
		if (regno < 16)
			((u32 *)cfb->fb.pseudo_palette)[regno] =
				((red >> 3) << 10) |
				((green >> 3) << 5) |
				(blue >> 3);
		break;
	case 16:
		if (regno < 16) {
			((u32 *)cfb->fb.pseudo_palette)[regno] =
				((red >> 3) << 11) |
				((green >> 2) << 5) |
				(blue >> 3);
		}
		break;
	case 17 ... 32:
		if (regno < 16)
			((u32 *)cfb->fb.pseudo_palette)[regno] =
				(red << 16) |
				(green << 8) |
				(blue << 0);

/*		if (regno < 16) {
			unsigned val;
                        val  = chan_to_field(red, &cfb->fb.var.red);
                        val |= chan_to_field(green, &cfb->fb.var.green);
                        val |= chan_to_field(blue, &cfb->fb.var.blue);
			((u32 *)cfb->fb.pseudo_palette)[regno] = val;
		}
*/

		break;
	}
	return 0;
}

static struct task_struct * resize_task;
static struct task_struct * resize_a320_original_task = NULL;
static struct task_struct * resize_a320_full_screen_task = NULL;
unsigned int resize_a320_go_out = 0;
unsigned short *frame_dst;
unsigned short *frame_src;
//unsigned short frame_temp[640*480];

#define SCALE_WIDTH  160 //x_src = (x_dest * SCALE_WIDTH) >> 8
#define SCALE_HEIGHT  128 //y_src = (y_dest * SCALE_HEIGHT) >> 8
#define GET_R(temp) temp>>11
#define GET_G(temp) (temp&0x7e0)>>5
#define GET_B(temp) (temp&0x1f)
#define CREATE_RGB(r,g,b) (r << 11) | (g << 5) | b
//char r_pix[640*480];
//char g_pix[640*480];
//char b_pix[640*480];

static void fb_a320_thread(unsigned int src_weight, unsigned int src_height, unsigned int dst_weight, unsigned int dst_height,int mode)
{
	if (tvout_flag == 1)
		return;
  frame_dst = (unsigned short *)lcd_frame01;
  frame_src = (unsigned short *)lcd_frame0;
  unsigned short *lcd_frame_temp;
#define FRACTION_STEP 0x10000
  const unsigned int  x_fraction=src_weight*FRACTION_STEP/dst_weight;
  const unsigned int  y_fraction=src_height*FRACTION_STEP/dst_height;
  unsigned int x_temp = 0;
  unsigned int y_temp = 0;
  y_temp = y_fraction;
  unsigned int original_size_offset = ((dst_height - src_height) >> 1)*dst_weight;

  int i,j;
#define A320_FULLSCREEN 1
#define A320_ORIGINAL   2
  switch (mode)
  {
    case A320_FULLSCREEN:
      for(j = 0; j < dst_height; j++)
      {
        y_temp += y_fraction;
        if(y_temp >= FRACTION_STEP)
        {
          y_temp -= FRACTION_STEP;
          //scale horiontal
          x_temp = x_fraction;
          for(i = 0; i < dst_weight; i++)
          {
            x_temp += x_fraction;
            *frame_dst = *frame_src;
            if(x_temp >= FRACTION_STEP)
            {
              frame_src++;
              x_temp -= FRACTION_STEP;
            }
            frame_dst++;
          }
          frame_src += dst_weight-320;
        }
        else
        {
          lcd_frame_temp = frame_dst - dst_weight;
          for(i = 0; i < dst_weight; i++)
          {
            *frame_dst++ = *lcd_frame_temp++;
          }
        }
      }
      break;
    case A320_ORIGINAL:
      frame_dst += original_size_offset;
      for(j = 0; j < src_height; j++)
      {
        frame_dst +=(dst_weight - src_weight)/2;
        for(i = 0; i < src_weight; i++)
        {
          *frame_dst = *frame_src;
          frame_dst++;
          frame_src++;
        }
        frame_src += dst_weight-320;
        frame_dst +=(dst_weight - src_weight)/2;
      }
      break;
  }
}

static int fb_resize_thread(void *unused)
{
	printk("kernel frame buffer resize thread start!\n");
	while(1)
	{
		//printk("zzzz\n");
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ/20);

		ipu_driver_flush_tv();

		if(resize_go_out)
			break;
	}
	thread_init_end = 0;

	return 0;
}

void fb_resize_start(void)
{
	if (tvout_flag != 1 || thread_init_end)
		return;
//	printk("..............zz.........%s \n", __func__);

#ifdef TVOUT_2x
	resize_go_out = 0;
	thread_init_end = 0;
	resize_task = kthread_run(fb_resize_thread, NULL, "fb_resize");
	if(IS_ERR(resize_task))
	{
		printk("Kernel fb resize thread start error!\n");
		return;
	}
	thread_init_end = 1;
#endif
}

EXPORT_SYMBOL(fb_resize_start);

/*
 * switch to tve mode from lcd mode
 * mode:
 * 	PANEL_MODE_TVE_PAL: switch to TVE_PAL mode
 * 	PANEL_MODE_TVE_NTSC: switch to TVE_NTSC mode
 */
static void jz4760lcd_info_switch_to_TVE(int mode)
{
	struct jz4760lcd_info *info;
	struct jz4760lcd_osd_t *osd_lcd;
	int x, y, w, h;

	info = jz4760_lcd_info = &jz4760_info_tve;
	osd_lcd = &jz4760_lcd_panel.osd;

	switch ( mode ) {
	case PANEL_MODE_TVE_PAL:
		info->panel.cfg |= LCD_CFG_TVEPEH; /* TVE PAL enable extra halfline signal */
		info->panel.w = TVE_WIDTH_PAL;
		info->panel.h = TVE_HEIGHT_PAL;
		info->panel.fclk = TVE_FREQ_PAL;

		w = 640;
		h = 480;
		x = (TVE_WIDTH_PAL - 640)/2;
		y = (TVE_HEIGHT_PAL - 480)/2;

		info->osd.fg0.bpp = osd_lcd->fg0.bpp;
		info->osd.fg0.x = x;
		info->osd.fg0.y = y;
		info->osd.fg0.w = w;
		info->osd.fg0.h = h;


		info->osd.fg1.bpp = 16;	/* use RGB888 in TVE mode*/
		info->osd.fg1.x = x;
		info->osd.fg1.y = y;
		info->osd.fg1.w = w;
		info->osd.fg1.h = h;
		break;
	case PANEL_MODE_TVE_NTSC:
		info->panel.cfg &= ~LCD_CFG_TVEPEH; /* TVE NTSC disable extra halfline signal */
		info->panel.w = TVE_WIDTH_NTSC;
		info->panel.h = TVE_HEIGHT_NTSC;
		info->panel.fclk = TVE_FREQ_NTSC;

#ifdef TVOUT_2x
		w = 640;
		h = 480;
#else
		w = tvout_display_w;
		h = tvout_display_h;
#endif
	
#ifdef TVOUT_2x
		x = (TVE_WIDTH_NTSC - 640)/2;
		//y = (TVE_HEIGHT_NTSC- 480)/2;
        y = 2;//y must be even
#else
		x = (TVE_WIDTH_PAL - w)/2;
		y = (TVE_HEIGHT_PAL - h)/2;
#endif

		info->osd.fg0.bpp = osd_lcd->fg0.bpp;
		info->osd.fg0.x = x;
		info->osd.fg0.y = y;
		info->osd.fg0.w = w;
		info->osd.fg0.h = h;
		info->osd.fg1.bpp = 16;	/* use RGB888 int TVE mode */
		info->osd.fg1.x = x;
		info->osd.fg1.y = y;
		info->osd.fg1.w = w;
		info->osd.fg1.h = h;
		break;
	default:
		printk("%s, %s: Unknown tve mode\n", __FILE__, __FUNCTION__);
	}
}


static int get_ipu_restart_trigger(void)//from lihuawu
{
	int frond_porch;
	int third_of_vblank;
	int trigger_value;
	int ht, vt, vds, vde, hde;
	/* LCDC spec: ipu_restart_trigger = frond_porch + ((HT-0)x(VPE-VPS))/3 */

	vt = (REG_LCD_VAT&LCD_VAT_VT_MASK)>>LCD_VAT_VT_BIT;
	ht = (REG_LCD_VAT&LCD_VAT_HT_MASK)>>LCD_VAT_HT_BIT;
	vds = (REG_LCD_DAV&LCD_DAV_VDS_MASK)>>LCD_DAV_VDS_BIT; /* Vsync start at 0. */
	vde = (REG_LCD_DAV&LCD_DAV_VDE_MASK)>>LCD_DAV_VDE_BIT;
	hde = (REG_LCD_DAH&LCD_DAH_HDE_MASK)>>LCD_DAV_VDE_BIT;

	if (hde>ht) {
		printk("LCDC config error hde>ht\n");
		hde = ht;
	}
	if (vde>vt) {
		printk("LCDC config error vde>vt\n");
		vde = vt;
	}

	third_of_vblank = (ht*vds)/3;
	frond_porch = (ht-hde) + (vt-vde)*ht;
	trigger_value = (frond_porch + third_of_vblank);
#if JZ_FB_DEBUG
	printk("\n");
	printk("\n REG_LCD_VAT=0x%08X", REG_LCD_VAT);
	printk("\n REG_LCD_DAV=0x%08X", REG_LCD_DAV);
	printk("\n REG_LCD_DAH=0x%08X", REG_LCD_DAH);
	printk("\n");
	printk("\t ht = %d", ht);
	printk("\t vt = %d", vt);
	printk("\t vds = %d",vds );
	printk("\t vde = %d", vde);
	printk("\t hde = %d", hde);
	printk("\n third_of_vblank = %d", third_of_vblank);
	printk("\n frond_porch = %d", frond_porch);
	printk("\n trigger_value = %d", trigger_value);
	printk("\n");

#endif

	return trigger_value;
}

static int jz4760fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

        void __user *argp = (void __user *)arg;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		printk("FBIO_WAITFORVSYNC");
		break;
    case FBIOENBALECRTL:
        ctrl_enable();
        break;

	case FBIODISPON:
		ctrl_enable();
        screen_on();
        break;

	case FBIODISPOFF:
		screen_off();
		ctrl_disable();
		memset(lcd_frame1,0,640*480*2);
		break;

	case FBIOPRINT_REG:
		print_lcdc_registers();
		break;

	case FBIO_GET_MODE:
		D("fbio get mode\n");

		if (copy_to_user(argp, jz4760_lcd_info, sizeof(struct jz4760lcd_info)))
			return -EFAULT;

		break;

	case FBIO_SET_MODE:
		D("fbio set mode\n");

		if (copy_from_user(jz4760_lcd_info, argp, sizeof(struct jz4760lcd_info)))
			return -EFAULT;

		/* set mode */
		jz4760fb_set_mode(jz4760_lcd_info);
		break;

	case FBIO_DEEP_SET_MODE:
		D("fbio deep set mode\n");

		if (copy_from_user(jz4760_lcd_info, argp, sizeof(struct jz4760lcd_info)))
			return -EFAULT;

		jz4760fb_deep_set_mode(jz4760_lcd_info);

		break;
    case FBIO_GET_IPU_RESTART_VALUE:
		if((panel_mode == PANEL_MODE_TVE_PAL) || (last_panel_mode == PANEL_MODE_TVE_NTSC))//tvout
		    return 0x4000;
		else//lcd and hdmi
		    return get_ipu_restart_trigger();

		return 0;
		break;

	case FBIO_MODE_SWITCH:
        D("FBIO_MODE_SWITCH");
        panel_mode = arg;
		switch (arg) {
#ifdef CONFIG_FB_JZ4760_TVE
			case PANEL_MODE_TVE_PAL: 	/* switch to TVE_PAL mode */
			case PANEL_MODE_TVE_NTSC: 	/* switch to TVE_NTSC mode */
				jz4760lcd_info_switch_to_TVE(arg);
				jz4760tve_init(arg); /* tve controller init */
				udelay(100);
				cpm_start_clock(CGM_TVE);
				jz4760tve_enable_tve();
				/* turn off lcd backlight */
				screen_off();
				break;
#endif
#if defined(CONFIG_JZ4760_HDMI_DISPLAY)
 			case PANEL_MODE_HDMI_480P:
                set_i2s_external_codec();
				/* turn off TVE, turn off DACn... */
				//jz4760tve_disable_tve();
				jz4760_lcd_info =&jz4760_info_hdmi_480p ;
				/* turn on lcd backlight */
				screen_off();
				break;

			case PANEL_MODE_HDMI_576P:
   				set_i2s_external_codec();
				/* turn off TVE, turn off DACn... */
				//jz4760tve_disable_tve();
				jz4760_lcd_info =&jz4760_info_hdmi_576p ;
				/* turn on lcd backlight */
				screen_off();
				break;

			case PANEL_MODE_HDMI_720P50:
#if defined(CONFIG_SOC_JZ4760B)
				REG_LCD_PCFG = 0xc0000888;
                                REG_GPIO_PXSLC(2) = 0;
                                REG_GPIO_PXDS1S(2) |= 1 << 8;
                                REG_GPIO_PXDS0S(2) = 0x0fffffff;

#endif
				set_i2s_external_codec();
				/* turn off TVE, turn off DACn... */
				//jz4760tve_disable_tve();
				jz4760_lcd_info =&jz4760_info_hdmi_720p50 ;
				/* turn on lcd backlight */
				screen_off();
				break;

			case PANEL_MODE_HDMI_720P60:
#if defined(CONFIG_SOC_JZ4760B)
				REG_LCD_PCFG = 0xc0000888;
                                REG_GPIO_PXSLC(2) = 0;
                                REG_GPIO_PXDS1S(2) |= 1 << 8;
                                REG_GPIO_PXDS0S(2) = 0x0fffffff;
#endif
   				set_i2s_external_codec();
				/* turn off TVE, turn off DACn... */
				//jz4760tve_disable_tve();
				jz4760_lcd_info =&jz4760_info_hdmi_720p60 ;
				/* turn on lcd backlight */
				screen_off();
				break;
			case PANEL_MODE_HDMI_1080P30:
                                REG_LCD_PCFG = 0xc4010000;
                                set_i2s_external_codec();
                                /* turn off TVE, turn off DACn... */
                                //jz4760tve_disable_tve();
                                jz4760_lcd_info =&jz4760_info_hdmi_1080p30;
                                /* turn on lcd backlight */
                                screen_off();
                                break;

                        case PANEL_MODE_HDMI_1080P50:
                                REG_LCD_PCFG = 0xc4010000;
                                set_i2s_external_codec();
                                /* turn off TVE, turn off DACn... */
                                //jz4760tve_disable_tve();
                                jz4760_lcd_info =&jz4760_info_hdmi_1080p50;
                                /* turn on lcd backlight */
                                screen_off();
                                break;

			case PANEL_MODE_HDMI_1080P60:
                                REG_LCD_PCFG = 0xc4010000;
   				set_i2s_external_codec();
				/* turn off TVE, turn off DACn... */
				//jz4760tve_disable_tve();
				jz4760_lcd_info =&jz4760_info_hdmi_1080p60;
				/* turn on lcd backlight */
				screen_off();
				break;
#endif	//CONFIG_JZ4760_HDMI_DISPLAY
			case PANEL_MODE_LCD_PANEL: 	/* switch to LCD mode */
			default :
				/* turn off TVE, turn off DACn... */
#ifdef CONFIG_FB_JZ4760_TVE
				jz4760tve_disable_tve();
				cpm_stop_clock(CGM_TVE);
#endif
				//medive change for hdmi
#if defined(CONFIG_JZ4760_HDMI_DISPLAY)
				if (panle_mode != 2)
					set_i2s_internal_codec();
#endif

				jz4760_lcd_info = &jz4760_lcd_panel;
				/* turn on lcd backlight */
				//screen_on();   //medive
				break;
		}

		jz4760fb_deep_set_mode(jz4760_lcd_info);
		break;

#ifdef CONFIG_FB_JZ4760_TVE
	case FBIO_GET_TVE_MODE:
		D("fbio get TVE mode\n");
		if (copy_to_user(argp, jz4760_tve_info, sizeof(struct jz4760tve_info)))
			return -EFAULT;
		break;
	case FBIO_SET_TVE_MODE:
		D("fbio set TVE mode\n");
		if (copy_from_user(jz4760_tve_info, argp, sizeof(struct jz4760tve_info)))
			return -EFAULT;
		/* set tve mode */
		jz4760tve_set_tve_mode(jz4760_tve_info);
		break;
#endif
	default:
		printk("%s, unknown command(0x%x)", __FILE__, cmd);
		break;
	}

	return ret;
}

/* Use mmap /dev/fb can only get a non-cacheable Virtual Address. */
static int jz4760fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	unsigned long start;
	unsigned long off;
	u32 len;
	D("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	off = vma->vm_pgoff << PAGE_SHIFT;
	//fb->fb_get_fix(&fix, PROC_CONSOLE(info), info);

	/* frame buffer memory */
	start = cfb->fb.fix.smem_start;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + cfb->fb.fix.smem_len);
	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);	/* Uncacheable */

#if 1
 	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
 	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;		/* Uncacheable */
//	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;	/* Write-Back */
#endif

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}

/* checks var and eventually tweaks it to something supported,
 * DO NOT MODIFY PAR */
static int jz4760fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	//printk("jz4760fb_check_var, not implement\n");
	return 0;
}


/*
 * set the video mode according to info->var
 */
static int jz4760fb_set_par(struct fb_info *info)
{
	struct myfb_par *par = info->par;
	printk("YRES_VIRTUAL = %d, YRES = %d\n",info->var.yres_virtual,info->var.yres);
	if (info->var.yres_virtual != info->var.yres)
		printk("Required double buffer\n");

	// printk("jz4760fb_set_par, not implemented\n");
	return 0;
}


/*
 * (Un)Blank the display.
 * Fix me: should we use VESA value?
 */
static int jz4760fb_blank(int blank_mode, struct fb_info *info)
{
	D("jz4760 fb_blank %d %p", blank_mode, info);
	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		//case FB_BLANK_NORMAL:
			/* Turn on panel */
		__lcd_set_ena();
		screen_on();

		break;

	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
#if 0
		/* Turn off panel */
		__lcd_display_off();
		__lcd_set_dis();
#endif
		break;
	default:
		break;

	}
	return 0;
}

/*
 * pan display
 */
static int jz4760fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;

	if (!var || !cfb) {
		return -EINVAL;
	}

	dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)lcd_frame0 + 
	((var->xoffset * info->var.bits_per_pixel) / 8) +
	(var->yoffset * cfb->fb.fix.line_length));
	dma_cache_wback((unsigned int)(dma0_desc0), sizeof(struct jz4760_lcd_dma_desc));

	return 0;
}


/* use default function cfb_fillrect, cfb_copyarea, cfb_imageblit */
static struct fb_ops jz4760fb_ops = {
	.owner			= THIS_MODULE,
	.fb_setcolreg		= jz4760fb_setcolreg,
	.fb_check_var 		= jz4760fb_check_var,
	.fb_set_par 		= jz4760fb_set_par,
	.fb_blank		= jz4760fb_blank,
	.fb_pan_display		= jz4760fb_pan_display,
	.fb_fillrect		= cfb_fillrect,
	.fb_copyarea		= cfb_copyarea,
	.fb_imageblit		= cfb_imageblit,
	.fb_mmap		= jz4760fb_mmap,
	.fb_ioctl		= jz4760fb_ioctl,
};

static int jz4760fb_set_var(struct fb_var_screeninfo *var, int con,
			struct fb_info *info)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	struct jz4760lcd_info *lcd_info = jz4760_lcd_info;
	int chgvar = 0;

	var->height	            = lcd_info->osd.fg0.h;	/* tve mode */
	var->width	            = lcd_info->osd.fg0.w;
	//printk("fg0.h %d,fg0.w %d fg1.h %d fg1.w %d\n",var->height,var->width,lcd_info->osd.fg1.h,lcd_info->osd.fg1.w);


	if (tvout_640_480 == 0)
	{
		var->height 		= LCD_SCREEN_H;
		var->width			= LCD_SCREEN_W;
		//printk("ccc....var->height %d,var->width %d \n",var->height,var->width);
	}
	else if (tvout_640_480 == 1)
	{
		var->height = 480;
		var->width = 640;
	}

	var->bits_per_pixel	    = lcd_info->osd.fg0.bpp;

	var->vmode                  = FB_VMODE_NONINTERLACED;
	var->activate               = cfb->fb.var.activate;
	var->xres                   = var->width;
	var->yres                   = var->height;
	var->xres_virtual           = var->width;
	var->yres_virtual           = var->height;
	var->xoffset                = 0;
	var->yoffset                = 0;
	var->pixclock               = 0;
	var->left_margin            = 0;
	var->right_margin           = 0;
	var->upper_margin           = 0;
	var->lower_margin           = 0;
	var->hsync_len              = 0;
	var->vsync_len              = 0;
	var->sync                   = 0;
	var->activate              &= ~FB_ACTIVATE_TEST;

	/*
	 * CONUPDATE and SMOOTH_XPAN are equal.  However,
	 * SMOOTH_XPAN is only used internally by fbcon.
	 */
	if (var->vmode & FB_VMODE_CONUPDATE) {
		var->vmode |= FB_VMODE_YWRAP;
		var->xoffset = cfb->fb.var.xoffset;
		var->yoffset = cfb->fb.var.yoffset;
	}

	if (var->activate & FB_ACTIVATE_TEST)
		return 0;

	if ((var->activate & FB_ACTIVATE_MASK) != FB_ACTIVATE_NOW)
		return -EINVAL;

	if (cfb->fb.var.xres != var->xres)
		chgvar = 1;
	if (cfb->fb.var.yres != var->yres)
		chgvar = 1;
	if (cfb->fb.var.xres_virtual != var->xres_virtual)
		chgvar = 1;
	if (cfb->fb.var.yres_virtual != var->yres_virtual)
		chgvar = 1;
	if (cfb->fb.var.bits_per_pixel != var->bits_per_pixel)
		chgvar = 1;

	//display = fb_display + con;

	var->red.msb_right	= 0;
	var->green.msb_right	= 0;
	var->blue.msb_right	= 0;

	switch(var->bits_per_pixel){
	case 1:	/* Mono */
		cfb->fb.fix.visual	= FB_VISUAL_MONO01;
		cfb->fb.fix.line_length	= (var->xres * var->bits_per_pixel) / 8;
		break;
	case 2:	/* Mono */
		var->red.offset		= 0;
		var->red.length		= 2;
		var->green.offset	= 0;
		var->green.length	= 2;
		var->blue.offset	= 0;
		var->blue.length	= 2;

		cfb->fb.fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		cfb->fb.fix.line_length	= (var->xres * var->bits_per_pixel) / 8;
		break;
	case 4:	/* PSEUDOCOLOUR*/
		var->red.offset		= 0;
		var->red.length		= 4;
		var->green.offset	= 0;
		var->green.length	= 4;
		var->blue.offset	= 0;
		var->blue.length	= 4;

		cfb->fb.fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		cfb->fb.fix.line_length	= var->xres / 2;
		break;
	case 8:	/* PSEUDOCOLOUR, 256 */
		var->red.offset		= 0;
		var->red.length		= 8;
		var->green.offset	= 0;
		var->green.length	= 8;
		var->blue.offset	= 0;
		var->blue.length	= 8;

		cfb->fb.fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		cfb->fb.fix.line_length	= var->xres ;
		break;
	case 15: /* DIRECTCOLOUR, 32k */
		var->bits_per_pixel	= 15;
		var->red.offset		= 10;
		var->red.length		= 5;
		var->green.offset	= 5;
		var->green.length	= 5;
		var->blue.offset	= 0;
		var->blue.length	= 5;

		cfb->fb.fix.visual	= FB_VISUAL_DIRECTCOLOR;
		cfb->fb.fix.line_length	= var->xres_virtual * 2;
		break;
	case 16: /* DIRECTCOLOUR, 64k */
		var->bits_per_pixel	= 16;
		var->red.offset		= 11;
		var->red.length		= 5;
		var->green.offset	= 5;
		var->green.length	= 6;
		var->blue.offset	= 0;
		var->blue.length	= 5;

		cfb->fb.fix.visual	= FB_VISUAL_TRUECOLOR;
		cfb->fb.fix.line_length	= var->xres_virtual * 2;
		break;
	case 17 ... 32:
		/* DIRECTCOLOUR, 256 */
		var->bits_per_pixel	= 32;

		var->red.offset		= 16;
		var->red.length		= 8;
		var->green.offset	= 8;
		var->green.length	= 8;
		var->blue.offset	= 0;
		var->blue.length	= 8;
		var->transp.offset  	= 24;
		var->transp.length 	= 8;

		cfb->fb.fix.visual	= FB_VISUAL_TRUECOLOR;
		cfb->fb.fix.line_length	= var->xres_virtual * 4;
		break;

	default: /* in theory this should never happen */
		printk(KERN_WARNING "%s: don't support for %dbpp\n",
		       cfb->fb.fix.id, var->bits_per_pixel);
		break;
	}

	cfb->fb.var = *var;
	cfb->fb.var.activate &= ~FB_ACTIVATE_ALL;

	/*
	 * Update the old var.  The fbcon drivers still use this.
	 * Once they are using cfb->fb.var, this can be dropped.
	 *					--rmk
	 */
	//display->var = cfb->fb.var;
	/*
	 * If we are setting all the virtual consoles, also set the
	 * defaults used to create new consoles.
	 */
	fb_set_cmap(&cfb->fb.cmap, &cfb->fb);
	
	return 0;
}

static struct lcd_cfb_info * jz4760fb_alloc_fb_info(void)
{
 	struct lcd_cfb_info *cfb;

	cfb = kmalloc(sizeof(struct lcd_cfb_info) + sizeof(u32) * 16, GFP_KERNEL);

	if (!cfb)
		return NULL;

	jz4760fb_info = cfb;

	memset(cfb, 0, sizeof(struct lcd_cfb_info) );

	cfb->backlight_level		= LCD_DEFAULT_BACKLIGHT;

	strcpy(cfb->fb.fix.id, "jz-lcd");
	cfb->fb.fix.type	= FB_TYPE_PACKED_PIXELS;
	cfb->fb.fix.type_aux	= 0;
	cfb->fb.fix.xpanstep	= 1;
	cfb->fb.fix.ypanstep	= 1;
	cfb->fb.fix.ywrapstep	= 0;
	cfb->fb.fix.accel	= FB_ACCEL_NONE;

	cfb->fb.var.nonstd	= 0;
	cfb->fb.var.activate	= FB_ACTIVATE_NOW;
	cfb->fb.var.height	= -1;
	cfb->fb.var.width	= -1;
	cfb->fb.var.accel_flags	= FB_ACCELF_TEXT;

	cfb->fb.fbops		= &jz4760fb_ops;
	cfb->fb.flags		= FBINFO_FLAG_DEFAULT;

	cfb->fb.pseudo_palette	= (void *)(cfb + 1);

	switch (jz4760_lcd_info->osd.fg0.bpp) {
	case 1:
		fb_alloc_cmap(&cfb->fb.cmap, 4, 0);
		break;
	case 2:
		fb_alloc_cmap(&cfb->fb.cmap, 8, 0);
		break;
	case 4:
		fb_alloc_cmap(&cfb->fb.cmap, 32, 0);
		break;
	case 8:

	default:
		fb_alloc_cmap(&cfb->fb.cmap, 256, 0);
		break;
	}
	D("fb_alloc_cmap,fb.cmap.len:%d....\n", cfb->fb.cmap.len);

	return cfb;
}

static int bpp_to_data_bpp(int bpp)
{
	switch (bpp) {
		case 1:
		case 2:
		case 4:
		case 8:
		case 16:
		case 32:
			break;

		case 15:
			bpp = 16;
			break;

		case 18:
		case 24:
			bpp = 32;
			break;

		default:
			bpp = -EINVAL;
	}

	return bpp;
}

/*
 * Map screen memory
 */
static int jz4760fb_map_smem(struct lcd_cfb_info *cfb)
{
	unsigned long page;
	unsigned int page_shift, needroom, needroom1, bpp, w, h;

	bpp = bpp_to_data_bpp(jz4760_lcd_info->osd.fg0.bpp);

	D("FG0 BPP: %d, Data BPP: %d.", jz4760_lcd_info->osd.fg0.bpp, bpp);

#ifndef CONFIG_FB_JZ4760_TVE
	w = jz4760_lcd_info->osd.fg0.w;
	h = jz4760_lcd_info->osd.fg0.h;
#else
	w = ( jz4760_lcd_info->osd.fg0.w > TVE_WIDTH_PAL )?jz4760_lcd_info->osd.fg0.w:TVE_WIDTH_PAL;
	h = ( jz4760_lcd_info->osd.fg0.h > TVE_HEIGHT_PAL )?jz4760_lcd_info->osd.fg0.h:TVE_HEIGHT_PAL;
        printk("%s %d %d \n",__func__,w,h);
#endif
	needroom1 = needroom = ((w * bpp + 7) >> 3) * h * 3;

#if defined(CONFIG_FB_JZ4760_LCD_USE_2LAYER_FRAMEBUFFER)
	bpp = bpp_to_data_bpp(jz4760_lcd_info->osd.fg1.bpp);

	D("FG1 BPP: %d, Data BPP: %d.", jz4760_lcd_info->osd.fg1.bpp, bpp);

#ifndef CONFIG_FB_JZ4760_TVE
	w = jz4760_lcd_info->osd.fg1.w;
	h = jz4760_lcd_info->osd.fg1.h;
#else
	w = ( jz4760_lcd_info->osd.fg1.w > TVE_WIDTH_PAL )?jz4760_lcd_info->osd.fg1.w:TVE_WIDTH_PAL;
	h = ( jz4760_lcd_info->osd.fg1.h > TVE_HEIGHT_PAL )?jz4760_lcd_info->osd.fg1.h:TVE_HEIGHT_PAL;

#endif
        printk("%s %d %d \n",__func__,w,h);
	needroom += ((w * bpp + 7) >> 3) * h * 3;
#endif // two layer

	for (page_shift = 0; page_shift < 13; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;
	printk("page count is %d\n",page_shift);
#if defined(CONFIG_JZ4760_HDMI_DISPLAY)
	page_shift = 11;
#endif
	lcd_palette = (unsigned char *)__get_free_pages(GFP_KERNEL | GFP_DMA, 0);
	lcd_frame0 = (unsigned char *)__get_free_pages(GFP_KERNEL | GFP_DMA, page_shift);
    lcd_frame01 = (unsigned char *)__get_free_pages(GFP_KERNEL | GFP_DMA, page_shift);

	//maddrone add for mplayer trans fb
	vmfbmem_addr = lcd_frame01;
	memset(vmfbmem_addr,0xff,LCD_SCREEN_W*LCD_SCREEN_H*2);
    phy_vmfbmem_addr = virt_to_phys((void *)vmfbmem_addr);

	if ((!lcd_palette) || (!lcd_frame0)|| (!lcd_frame01))
		return -ENOMEM;
	memset((void *)lcd_palette, 0, PAGE_SIZE);
	memset((void *)lcd_frame0, 0, PAGE_SIZE << page_shift);

	dma_desc_base = (struct jz4760_lcd_dma_desc *)((void*)lcd_palette + ((PALETTE_SIZE+3)/4)*4);

#if defined(CONFIG_FB_JZ4760_SLCD)
	lcd_cmdbuf = (unsigned char *)__get_free_pages(GFP_KERNEL, 0);
	memset((void *)lcd_cmdbuf, 0, PAGE_SIZE);

	{	int data, i, *ptr;
		ptr = (unsigned int *)lcd_cmdbuf;
		data = WR_GRAM_CMD;
		data = ((data & 0xff) << 1) | ((data & 0xff00) << 2);
		for(i = 0; i < 3; i++){
			ptr[i] = data;
		}
	}
#endif

#if defined(CONFIG_FB_JZ4760_LCD_USE_2LAYER_FRAMEBUFFER)
	lcd_frame1 = lcd_frame0 + needroom1;
#endif

	/*
	 * Set page reserved so that mmap will work. This is necessary
	 * since we'll be remapping normal memory.
	 */
	page = (unsigned long)lcd_palette;
	SetPageReserved(virt_to_page((void*)page));

	for (page = (unsigned long)lcd_frame0;
	     page < PAGE_ALIGN((unsigned long)lcd_frame0 + (PAGE_SIZE<<page_shift));
	     page += PAGE_SIZE) {
		SetPageReserved(virt_to_page((void*)page));
	}
        for (page = (unsigned long)lcd_frame01;
            page < PAGE_ALIGN((unsigned long)lcd_frame01 + (PAGE_SIZE<<page_shift));
            page += PAGE_SIZE) {
		SetPageReserved(virt_to_page((void*)page));
	}

	cfb->fb.fix.smem_start = virt_to_phys((void *)lcd_frame0);
	cfb->fb.fix.smem_len = (PAGE_SIZE << page_shift); /* page_shift/2 ??? */
	cfb->fb.screen_base =
		(unsigned char *)(((unsigned int)lcd_frame0&0x1fffffff) | 0xa0000000);

	if (!cfb->fb.screen_base) {
		printk("jz4760fb, %s: unable to map screen memory\n", cfb->fb.fix.id);
		return -ENOMEM;
	}

	return 0;
}

static void jz4760fb_free_fb_info(struct lcd_cfb_info *cfb)
{
	if (cfb) {
		fb_alloc_cmap(&cfb->fb.cmap, 0, 0);
		kfree(cfb);
	}
}

static void jz4760fb_unmap_smem(struct lcd_cfb_info *cfb)
{
	struct page * map = NULL;
	unsigned char *tmp;
	unsigned int page_shift, needroom, bpp, w, h;

	bpp = jz4760_lcd_info->osd.fg0.bpp;
	if ( bpp == 18 || bpp == 24)
		bpp = 32;
	if ( bpp == 15 )
		bpp = 16;
	w = jz4760_lcd_info->osd.fg0.w;
	h = jz4760_lcd_info->osd.fg0.h;
	needroom = ((w * bpp + 7) >> 3) * h;
#if defined(CONFIG_FB_JZ4760_LCD_USE_2LAYER_FRAMEBUFFER)
	bpp = jz4760_lcd_info->osd.fg1.bpp;
	if ( bpp == 18 || bpp == 24)
		bpp = 32;
	if ( bpp == 15 )
		bpp = 16;
	w = jz4760_lcd_info->osd.fg1.w;
	h = jz4760_lcd_info->osd.fg1.h;
	needroom += ((w * bpp + 7) >> 3) * h;
#endif

	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;

	if (cfb && cfb->fb.screen_base) {
		iounmap(cfb->fb.screen_base);
		cfb->fb.screen_base = NULL;
		release_mem_region(cfb->fb.fix.smem_start,
				   cfb->fb.fix.smem_len);
	}

	if (lcd_palette) {
		map = virt_to_page(lcd_palette);
		clear_bit(PG_reserved, &map->flags);
		free_pages((int)lcd_palette, 0);
	}

	if (lcd_frame0) {
		for (tmp=(unsigned char *)lcd_frame0;
		     tmp < lcd_frame0 + (PAGE_SIZE << page_shift);
		     tmp += PAGE_SIZE) {
			map = virt_to_page(tmp);
			clear_bit(PG_reserved, &map->flags);
		}
		free_pages((int)lcd_frame0, page_shift);
	}
}

/* initial dma descriptors */
static void jz4760fb_descriptor_init( struct jz4760lcd_info * lcd_info )
{
	unsigned int pal_size;
	unsigned int frame_size0;

	switch ( lcd_info->osd.fg0.bpp ) {
	case 1:
		pal_size = 4;
		break;
	case 2:
		pal_size = 8;
		break;
	case 4:
		pal_size = 32;
		break;
	case 8:
	default:
		pal_size = 512;
	}

	pal_size /= 4;

	dma0_desc_palette 	= dma_desc_base + 0;
	dma0_desc0 		= dma_desc_base + 1;
	dma0_desc1 		= dma_desc_base + 2;
	dma0_desc_cmd0 		= dma_desc_base + 3; /* use only once */
	dma0_desc_cmd 		= dma_desc_base + 4;
	dma1_desc0 		= dma_desc_base + 5;
	dma1_desc1 		= dma_desc_base + 6;

	/*
	 * Normal TFT panel's DMA Chan0:
	 *	TO LCD Panel:
	 * 		no palette:	dma0_desc0 <<==>> dma0_desc0
	 * 		palette :	dma0_desc_palette <<==>> dma0_desc0
	 *	TO TV Encoder:
	 * 		no palette:	dma0_desc0 <<==>> dma0_desc1
	 * 		palette:	dma0_desc_palette --> dma0_desc0
	 * 				--> dma0_desc1 --> dma0_desc_palette --> ...
	 *
	 * SMART LCD TFT panel(dma0_desc_cmd)'s DMA Chan0:
	 *	TO LCD Panel:
	 * 		no palette:	dma0_desc_cmd <<==>> dma0_desc0
	 * 		palette :	dma0_desc_palette --> dma0_desc_cmd
	 * 				--> dma0_desc0 --> dma0_desc_palette --> ...
	 *	TO TV Encoder:
	 * 		no palette:	dma0_desc_cmd --> dma0_desc0
	 * 				--> dma0_desc1 --> dma0_desc_cmd --> ...
	 * 		palette:	dma0_desc_palette --> dma0_desc_cmd
	 * 				--> dma0_desc0 --> dma0_desc1
	 * 				--> dma0_desc_palette --> ...
	 * DMA Chan1:
	 *	TO LCD Panel:
	 * 		dma1_desc0 <<==>> dma1_desc0
	 *	TO TV Encoder:
	 * 		dma1_desc0 <<==>> dma1_desc1
	 */

#if defined(CONFIG_FB_JZ4760_SLCD)
	/* First CMD descriptors, use only once, cmd_num isn't 0 */
	dma0_desc_cmd0->next_desc 	= (unsigned int)virt_to_phys(dma0_desc0);
	dma0_desc_cmd0->databuf 	= (unsigned int)virt_to_phys((void *)lcd_cmdbuf);
	dma0_desc_cmd0->frame_id 	= (unsigned int)0x0da0cad0; /* dma0's cmd0 */
	dma0_desc_cmd0->cmd 		= LCD_CMD_CMD | 3; /* command */
	dma0_desc_cmd0->offsize 	= 0;
	dma0_desc_cmd0->page_width 	= 0;
	dma0_desc_cmd0->cmd_num 	= 3;

	/* Dummy Command Descriptor, cmd_num is 0 */
	dma0_desc_cmd->next_desc 	= (unsigned int)virt_to_phys(dma0_desc0);
	dma0_desc_cmd->databuf 		= 0;
	dma0_desc_cmd->frame_id 	= (unsigned int)0x0da000cd; /* dma0's cmd0 */
	dma0_desc_cmd->cmd 		= LCD_CMD_CMD | 0; /* dummy command */
	dma0_desc_cmd->cmd_num 		= 0;
	dma0_desc_cmd->offsize 		= 0;
	dma0_desc_cmd->page_width 	= 0;

	/* Palette Descriptor */
	dma0_desc_palette->next_desc 	= (unsigned int)virt_to_phys(dma0_desc_cmd0);
#else
	/* Palette Descriptor */
	dma0_desc_palette->next_desc 	= (unsigned int)virt_to_phys(dma0_desc0);
#endif
	dma0_desc_palette->databuf 	= (unsigned int)virt_to_phys((void *)lcd_palette);
	dma0_desc_palette->frame_id 	= (unsigned int)0xaaaaaaaa;
	dma0_desc_palette->cmd 		= LCD_CMD_PAL | pal_size; /* Palette Descriptor */

	/* DMA0 Descriptor0 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) /* TVE mode */
		dma0_desc0->next_desc 	= (unsigned int)virt_to_phys(dma0_desc1);
	else{			/* Normal TFT LCD */
#if defined(CONFIG_FB_JZ4760_SLCD)
			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd);
#else
			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
#endif
	}

	//maddrone change here
	if(lcd_info->panel.cfg & LCD_CFG_TVEN)
	{
#ifdef TVOUT_2x
                printk("tvout_640_480 is %d\n",tvout_640_480);
		if(tvout_640_480)
		dma0_desc0->databuf = virt_to_phys((void *)lcd_frame0);
		else
		dma0_desc0->databuf = virt_to_phys((void *)lcd_frame01);
		
		dma0_desc0->frame_id = (unsigned int)0x0000da00; /* DMA0'0 */
		
		frame_size0 = (640 * 480 * 16) >> 3;
		frame_size0 /= 4;
		dma0_desc0->cmd = frame_size0;
		dma0_desc0->desc_size = (480 << 16) | 640;
		dma0_desc0->offsize = 0;
		dma0_desc0->cmd_num = 0;
#else
		dma0_desc0->databuf = virt_to_phys((void *)lcd_frame0);
		dma0_desc0->frame_id = (unsigned int)0x0000da00; /* DMA0'0 */
	
		frame_size0 = (tvout_display_w * tvout_display_h * 16) >> 3;
		frame_size0 /= 4;
		dma0_desc0->cmd = frame_size0;
		dma0_desc0->desc_size = (tvout_display_h << 16) | tvout_display_w;
		dma0_desc0->offsize = 0;
		dma0_desc0->cmd_num = 0;
#endif
	}
	else
	{
		dma0_desc0->databuf = virt_to_phys((void *)lcd_frame0);
		dma0_desc0->frame_id = (unsigned int)0x0000da00; /* DMA0'0 */
	
		frame_size0 = (LCD_SCREEN_W * LCD_SCREEN_H * 16) >> 3;
		frame_size0 /= 4;
		dma0_desc0->cmd = frame_size0;
		dma0_desc0->desc_size = (LCD_SCREEN_H << 16) | LCD_SCREEN_W;
		dma0_desc0->offsize = 0;
		dma0_desc0->cmd_num = 0;
	}

	/* DMA0 Descriptor1 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* TVE mode */


		if (lcd_info->osd.fg0.bpp <= 8) /* load palette only once at setup */
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc_palette);
		else
#if defined(CONFIG_FB_JZ4760_SLCD)  /* for smatlcd */
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd);
#else
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
#endif
		dma0_desc1->frame_id = (unsigned int)0x0000da01; /* DMA0'1 */
	}

	if (lcd_info->osd.fg0.bpp <= 8) /* load palette only once at setup */
		REG_LCD_DA0 = virt_to_phys(dma0_desc_palette);
	else {
#if defined(CONFIG_FB_JZ4760_SLCD)  /* for smartlcd */
		REG_LCD_DA0 = virt_to_phys(dma0_desc_cmd0); //smart lcd
#else
		REG_LCD_DA0 = virt_to_phys(dma0_desc0); //tft
#endif
	}

	/* DMA1 Descriptor0 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) /* TVE mode */
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc1);
	else			/* Normal TFT LCD */
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc0);

	dma1_desc0->databuf = virt_to_phys((void *)lcd_frame1);
	dma1_desc0->frame_id = (unsigned int)0x0000da10; /* DMA1'0 */

	/* DMA1 Descriptor1 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* TVE mode */
		dma1_desc1->next_desc = (unsigned int)virt_to_phys(dma1_desc0);
		dma1_desc1->frame_id = (unsigned int)0x0000da11; /* DMA1'1 */
	}

	REG_LCD_DA1 = virt_to_phys(dma1_desc0);	/* set Dma-chan1's Descripter Addrress */
	dma_cache_wback_inv((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz4760_lcd_dma_desc));

}

static void jz4760fb_set_panel_mode( struct jz4760lcd_info * lcd_info )
{
	struct jz4760lcd_panel_t *panel = &lcd_info->panel;
#ifdef CONFIG_JZ4760_VGA_DISPLAY
	REG_TVE_CTRL |= TVE_CTRL_DAPD;
	REG_TVE_CTRL &= ~( TVE_CTRL_DAPD1 | TVE_CTRL_DAPD2 | TVE_CTRL_DAPD3);
#endif
	/* set bpp */
	lcd_info->panel.ctrl &= ~LCD_CTRL_BPP_MASK;
	if ( lcd_info->osd.fg0.bpp == 1 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_1;
	else if ( lcd_info->osd.fg0.bpp == 2 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_2;
	else if ( lcd_info->osd.fg0.bpp == 4 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_4;
	else if ( lcd_info->osd.fg0.bpp == 8 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_8;
	else if ( lcd_info->osd.fg0.bpp == 15 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB555;
	else if ( lcd_info->osd.fg0.bpp == 16 ){
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_16;
		lcd_info->panel.ctrl &= ~LCD_CTRL_RGB555;
	}else if ( lcd_info->osd.fg0.bpp > 16 && lcd_info->osd.fg0.bpp < 32+1 ) {
		lcd_info->osd.fg0.bpp = 32;
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_18_24;
	}
	else {
		printk("The BPP %d is not supported\n", lcd_info->osd.fg0.bpp);
		lcd_info->osd.fg0.bpp = 32;
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_18_24;
	}

	lcd_info->panel.cfg |= LCD_CFG_NEWDES; /* use 8words descriptor always */

	REG_LCD_CTRL = lcd_info->panel.ctrl; /* LCDC Controll Register */
	REG_LCD_CFG = lcd_info->panel.cfg; /* LCDC Configure Register */
	REG_SLCD_CFG = lcd_info->panel.slcd_cfg; /* Smart LCD Configure Register */

	if ( lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD ) /* enable Smart LCD DMA */
		REG_SLCD_CTRL = SLCD_CTRL_DMA_EN;

	switch ( lcd_info->panel.cfg & LCD_CFG_MODE_MASK ) {
	case LCD_CFG_MODE_GENERIC_TFT:
	case LCD_CFG_MODE_INTER_CCIR656:
	case LCD_CFG_MODE_NONINTER_CCIR656:
	case LCD_CFG_MODE_SLCD:
	default:		/* only support TFT16 TFT32, not support STN and Special TFT by now(10-06-2008)*/
		REG_LCD_VAT = (((panel->blw + panel->w + panel->elw + panel->hsw)) << 16) | (panel->vsw + panel->bfw + panel->h + panel->efw);
		REG_LCD_DAH = ((panel->hsw + panel->blw) << 16) | (panel->hsw + panel->blw + panel->w);
		REG_LCD_DAV = ((panel->vsw + panel->bfw) << 16) | (panel->vsw + panel->bfw + panel->h);
		REG_LCD_HSYNC = (0 << 16) | panel->hsw;
		REG_LCD_VSYNC = (0 << 16) | panel->vsw;
		break;
	}
}


static void jz4760fb_set_osd_mode( struct jz4760lcd_info * lcd_info )
{
	D("%s, %d\n", __FILE__, __LINE__ );
	lcd_info->osd.osd_ctrl &= ~(LCD_OSDCTRL_OSDBPP_MASK);
	if ( lcd_info->osd.fg1.bpp == 15 )
		lcd_info->osd.osd_ctrl |= LCD_OSDCTRL_OSDBPP_15_16|LCD_OSDCTRL_RGB555;
	else if ( lcd_info->osd.fg1.bpp == 16 )
		lcd_info->osd.osd_ctrl |= LCD_OSDCTRL_OSDBPP_15_16|LCD_OSDCTRL_RGB565;
	else {
		lcd_info->osd.fg1.bpp = 32;
		lcd_info->osd.osd_ctrl |= LCD_OSDCTRL_OSDBPP_18_24;
	}

	REG_LCD_OSDC 	= lcd_info->osd.osd_cfg; /* F0, F1, alpha, */

	REG_LCD_OSDCTRL = lcd_info->osd.osd_ctrl; /* IPUEN, bpp */
	REG_LCD_RGBC  	= lcd_info->osd.rgb_ctrl;
	REG_LCD_BGC  	= lcd_info->osd.bgcolor;
	REG_LCD_KEY0 	= lcd_info->osd.colorkey0;
	REG_LCD_KEY1 	= lcd_info->osd.colorkey1;
	REG_LCD_ALPHA 	= lcd_info->osd.alpha;
	REG_LCD_IPUR 	= lcd_info->osd.ipu_restart;
}

static void jz4760fb_foreground_resize( struct jz4760lcd_info * lcd_info )
{
	int fg0_line_size, fg0_frm_size, fg1_line_size, fg1_frm_size;
	/*
	 * NOTE:
	 * Foreground change sequence:
	 * 	1. Change Position Registers -> LCD_OSDCTL.Change;
	 * 	2. LCD_OSDCTRL.Change -> descripter->Size
	 * Foreground, only one of the following can be change at one time:
	 * 	1. F0 size;
	 *	2. F0 position
	 * 	3. F1 size
	 *	4. F1 position
	 */

	/*
	 * The rules of f0, f1's position:
	 * 	f0.x + f0.w <= panel.w;
	 * 	f0.y + f0.h <= panel.h;
	 *
	 * When output is LCD panel, fg.y and fg.h can be odd number or even number.
	 * When output is TVE, as the TVE has odd frame and even frame,
	 * to simplified operation, fg.y and fg.h should be even number always.
	 *
	 */

	/* Foreground 0  */
	if ( lcd_info->osd.fg0.x >= lcd_info->panel.w )
		lcd_info->osd.fg0.x = lcd_info->panel.w;
	if ( lcd_info->osd.fg0.y >= lcd_info->panel.h )
		lcd_info->osd.fg0.y = lcd_info->panel.h;
	if ( lcd_info->osd.fg0.x + lcd_info->osd.fg0.w > lcd_info->panel.w )
		lcd_info->osd.fg0.w = lcd_info->panel.w - lcd_info->osd.fg0.x;
	if ( lcd_info->osd.fg0.y + lcd_info->osd.fg0.h > lcd_info->panel.h )
		lcd_info->osd.fg0.h = lcd_info->panel.h - lcd_info->osd.fg0.y;

#if 0
	/* Foreground 1 */
	/* Case TVE ??? TVE 720x573 or 720x480*/
	if ( lcd_info->osd.fg1.x >= lcd_info->panel.w )
		lcd_info->osd.fg1.x = lcd_info->panel.w;
	if ( lcd_info->osd.fg1.y >= lcd_info->panel.h )
		lcd_info->osd.fg1.y = lcd_info->panel.h;
	if ( lcd_info->osd.fg1.x + lcd_info->osd.fg1.w > lcd_info->panel.w )
		lcd_info->osd.fg1.w = lcd_info->panel.w - lcd_info->osd.fg1.x;
	if ( lcd_info->osd.fg1.y + lcd_info->osd.fg1.h > lcd_info->panel.h )
		lcd_info->osd.fg1.h = lcd_info->panel.h - lcd_info->osd.fg1.y;
#endif
//	fg0_line_size = lcd_info->osd.fg0.w*((lcd_info->osd.fg0.bpp+7)/8);
	fg0_line_size = (lcd_info->osd.fg0.w*(lcd_info->osd.fg0.bpp)/8);
	fg0_line_size = ((fg0_line_size+3)>>2)<<2; /* word aligned */
	fg0_frm_size = fg0_line_size * lcd_info->osd.fg0.h;

	fg1_line_size = lcd_info->osd.fg1.w*((lcd_info->osd.fg1.bpp+7)/8);
	fg1_line_size = ((fg1_line_size+3)>>2)<<2; /* word aligned */
	fg1_frm_size = fg1_line_size * lcd_info->osd.fg1.h;

	//printk("fg1 osd x: %d y: %d w: %d h: %d\n",lcd_info->osd.fg1.x,lcd_info->osd.fg1.y,lcd_info->osd.fg1.w,lcd_info->osd.fg1.h);

	if ( lcd_info->osd.fg_change ) {
		if ( lcd_info->osd.fg_change & FG0_CHANGE_POSITION ) { /* F1 change position */
			REG_LCD_XYP0 = lcd_info->osd.fg0.y << 16 | lcd_info->osd.fg0.x;
		}
		if ( lcd_info->osd.fg_change & FG1_CHANGE_POSITION ) { /* F1 change position */
			REG_LCD_XYP1 = lcd_info->osd.fg1.y << 16 | lcd_info->osd.fg1.x;
		}

		/* set change */
		if ( !(lcd_info->osd.osd_ctrl & LCD_OSDCTRL_IPU) &&
		     (lcd_info->osd.fg_change != FG_CHANGE_ALL) )
			REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;

		/* wait change ready??? */
//		while ( REG_LCD_OSDS & LCD_OSDS_READY )	/* fix in the future, Wolfgang, 06-20-2008 */
		D("wait LCD_OSDS_READY\n");

		if ( lcd_info->osd.fg_change & FG0_CHANGE_SIZE ) { /* change FG0 size */
			if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* output to TV */
                                //fg0_line_size = fg1_line_size;
                                //fg0_frm_size = fg1_frm_size;
                                dma0_desc0->cmd = dma0_desc1->cmd = (fg0_frm_size/4)/2;
				dma0_desc0->offsize = dma0_desc1->offsize
					= fg0_line_size/4;
				dma0_desc0->page_width = dma0_desc1->page_width
					= fg0_line_size/4;
				#ifdef TVOUT_2x
				if(tvout_640_480)
				dma0_desc1->databuf = virt_to_phys((void *)(lcd_frame0 + fg0_line_size));  //maddrone
				else
				dma0_desc1->databuf = virt_to_phys((void *)(lcd_frame01 + fg0_line_size));  //maddrone
				#else
				dma0_desc1->databuf = virt_to_phys((void *)(lcd_frame0 + fg0_line_size));
				#endif
				REG_LCD_DA0 = virt_to_phys(dma0_desc0); //tft
			}
			else {
				dma0_desc0->cmd = dma0_desc1->cmd = fg0_frm_size/4;
				dma0_desc0->offsize = dma0_desc1->offsize =0;
				dma0_desc0->page_width = dma0_desc1->page_width = 0;
			}

			dma0_desc0->desc_size = dma0_desc1->desc_size
				= lcd_info->osd.fg0.h << 16 | lcd_info->osd.fg0.w;
			REG_LCD_SIZE0 = (lcd_info->osd.fg0.h<<16)|lcd_info->osd.fg0.w;

		}

		if ( lcd_info->osd.fg_change & FG1_CHANGE_SIZE ) { /* change FG1 size*/
			if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* output to TV */
				dma1_desc0->cmd = dma1_desc1->cmd = (fg1_frm_size/4)/2;
				dma1_desc0->offsize = dma1_desc1->offsize = fg1_line_size/4;
				dma1_desc0->page_width = dma1_desc1->page_width = fg1_line_size/4;
				dma1_desc1->databuf = virt_to_phys((void *)(lcd_frame1 + fg1_line_size));
				REG_LCD_DA1 = virt_to_phys(dma1_desc0); 

			}
			else {
				dma1_desc0->cmd = dma1_desc1->cmd = fg1_frm_size/4;
				dma1_desc0->offsize = dma1_desc1->offsize = 0;
				dma1_desc0->page_width = dma1_desc1->page_width = 0;
			}

			dma1_desc0->desc_size = dma1_desc1->desc_size
				= lcd_info->osd.fg1.h << 16 | lcd_info->osd.fg1.w;
			REG_LCD_SIZE1 = lcd_info->osd.fg1.h << 16|lcd_info->osd.fg1.w;
		}

		dma_cache_wback((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz4760_lcd_dma_desc));
		lcd_info->osd.fg_change = FG_NOCHANGE; /* clear change flag */
	}
}

static void jz4760fb_change_clock( struct jz4760lcd_info * lcd_info )
{

#if defined(CONFIG_FPGA)
	REG_LCD_REV = 0x00000004;
	printk("FPGA test, pixclk divide REG_LCD_REV=0x%08x\n", REG_LCD_REV);
	printk("FPGA test, pixclk %d\n", JZ_EXTAL/(((REG_LCD_REV&0xFF)+1)*2));
#else
	unsigned int val = 0;
	unsigned int pclk;
	/* Timing setting */
	__cpm_stop_lcd();

	val = lcd_info->panel.fclk; /* frame clk */

	if ( (lcd_info->panel.cfg & LCD_CFG_MODE_MASK) != LCD_CFG_MODE_SERIAL_TFT) {
		pclk = val * (lcd_info->panel.w + lcd_info->panel.hsw + lcd_info->panel.elw + lcd_info->panel.blw) * (lcd_info->panel.h + lcd_info->panel.vsw + lcd_info->panel.efw + lcd_info->panel.bfw); /* Pixclk */
	}
	else {
		/* serial mode: Hsync period = 3*Width_Pixel */
		pclk = val * (lcd_info->panel.w*3 + lcd_info->panel.hsw + lcd_info->panel.elw + lcd_info->panel.blw) * (lcd_info->panel.h + lcd_info->panel.vsw + lcd_info->panel.efw + lcd_info->panel.bfw); /* Pixclk */
	}

#ifdef CONFIG_FB_JZ4760_TVE
	/********* In TVE mode PCLK = 27MHz ***********/
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { 		/* LCDC output to TVE */
#if 1
//	   	__cpm_stop_tve();
		OUTREG32(CPM_CPPCR0,((90<< CPPCR0_PLLM_LSB)|(2<<CPPCR0_PLLN_LSB)|(1<<CPPCR0_PLLOD_LSB)|(0x20<<CPPCR0_PLLST_LSB)|CPPCR0_PLLEN));	
		REG_CPM_LPCDR  |= LPCDR_LTCS;
		pclk = 27000000;
		val = __cpm_get_pllout2() / pclk; /* pclk */
		val--;
		__cpm_set_pixdiv(val);
//		__cpm_start_tve();

		D("REG_CPM_LPCDR = 0x%08x\n", REG_CPM_LPCDR);
#if 0
#if defined(CONFIG_SOC_JZ4760) /* Jz4760D don't use LCLK */
		val = pclk * 3 ;	/* LCDClock > 2.5*Pixclock */

		val =(__cpm_get_pllout()) / val;
		if ( val > 0x1f ) {
			printk("lcd clock divide is too large, set it to 0x1f\n");
			val = 0x1f;
		}
		__cpm_set_ldiv( val );
#endif
#endif
		__cpm_select_pixclk_tve();

		REG_CPM_CPCCR |= CPCCR_CE ; /* update divide */
#else
		cpm_stop_clock(CGM_TVE);
		pclk = 27000000;
		cpm_set_clock(CGU_TVECLK,pclk);
		cpm_start_clock(CGM_TVE);
#endif
	}
	else 
#endif
	{ 		/* LCDC output to  LCD panel */
                D("pll out is %d pclk is %d\n",__cpm_get_pllout2(),pclk);
#ifdef CONFIG_JZ4760_LCD_UMIDO_L430
        //        pclk = 22000000;
#endif
		val = __cpm_get_pllout2() / pclk; /* pclk */
		val--;
		D("ratio: val = %d\n", val);
		if ( val > 0x7ff ) {
			printk("pixel clock divid is too large, set it to 0x7ff\n");
			val = 0x7ff;
		}

		__cpm_set_pixdiv(val);
		D("REG_CPM_LPCDR = 0x%08x\n", REG_CPM_LPCDR);
#if 0
#if defined(CONFIG_SOC_JZ4760) /* Jz4760D don't use LCLK */
		val = pclk * 3 ;	/* LCDClock > 2.5*Pixclock */
		val =__cpm_get_pllout2() / val;
		if ( val > 0x1f ) {
			printk("lcd clock divide is too large, set it to 0x1f\n");
			val = 0x1f;
		}
		__cpm_set_ldiv( val );
#endif
#endif
		__cpm_select_pixclk_lcd();
		REG_CPM_CPCCR |= CPCCR_CE ; /* update divide */

	}

	D("REG_CPM_LPCDR=0x%08x\n", REG_CPM_LPCDR);
	D("REG_CPM_CPCCR=0x%08x\n", REG_CPM_CPCCR);

	jz_clocks.pixclk = __cpm_get_pixclk();
	printk("LCDC: PixClock:%d\n", jz_clocks.pixclk);
#if 0
#if defined(CONFIG_SOC_JZ4760) /* Jz4760D don't use LCLK */
	jz_clocks.lcdclk = __cpm_get_lcdclk();
	printk("LCDC: LcdClock:%d\n", jz_clocks.lcdclk);
#endif
#endif
	__cpm_start_lcd();
	udelay(1000);
	/*
	 * set lcd device clock and lcd pixel clock.
	 * what about TVE mode???
	 *
	 */
#endif

}

/*
 * jz4760fb_set_mode(), set osd configure, resize foreground
 *
 */
static void jz4760fb_set_mode( struct jz4760lcd_info * lcd_info )
{
	struct lcd_cfb_info *cfb = jz4760fb_info;

	jz4760fb_set_osd_mode(lcd_info);
	jz4760fb_foreground_resize(lcd_info);
	jz4760fb_set_var(&cfb->fb.var, -1, &cfb->fb);
}

/*
 * jz4760fb_deep_set_mode,
 *
 */
static void jz4760fb_deep_set_mode( struct jz4760lcd_info * lcd_info )
{
	/* configurate sequence:
	 * 1. disable lcdc.
	 * 2. init frame descriptor.
	 * 3. set panel mode
	 * 4. set osd mode
	 * 5. start lcd clock in CPM
	 * 6. enable lcdc.
	 */

	__lcd_clr_ena();	/* Quick Disable */

	lcd_info->osd.fg_change = FG_CHANGE_ALL; /* change FG0, FG1 size, postion??? */
	jz4760fb_descriptor_init(lcd_info);
	jz4760fb_set_panel_mode(lcd_info);
	jz4760fb_set_mode(lcd_info);
	jz4760fb_change_clock(lcd_info);

	__lcd_set_ena();	/* enable lcdc */
}


static irqreturn_t jz4760fb_interrupt_handler(int irq, void *dev_id)
{
	unsigned int state;
	static int irqcnt=0;

	state = REG_LCD_STATE;
	D("In the lcd interrupt handler, state=0x%x\n", state);

	if (state & LCD_STATE_EOF) /* End of frame */
		REG_LCD_STATE = state & ~LCD_STATE_EOF;

	if (state & LCD_STATE_IFU0) {
		printk("%s, InFiFo0 underrun\n", __FUNCTION__);
		REG_LCD_STATE = state & ~LCD_STATE_IFU0;
	}

	if (state & LCD_STATE_IFU1) {
		printk("%s, InFiFo1 underrun\n", __FUNCTION__);
		REG_LCD_STATE = state & ~LCD_STATE_IFU1;
	}

	if (state & LCD_STATE_OFU) { /* Out fifo underrun */
		REG_LCD_STATE = state & ~LCD_STATE_OFU;
		if ( irqcnt++ > 100 ) {
			__lcd_disable_ofu_intr();
			printk("disable Out FiFo underrun irq.\n");
		}
		printk("%s, Out FiFo underrun.\n", __FUNCTION__);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM

/*
 * Suspend the LCDC.
 */
static int jz4760_fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("%s(): called.\n", __func__);

	screen_off();
	//ctrl_disable();
#if 0 //allen del
	__lcd_clr_ena();
#else
	__lcd_clr_dis();
#endif

	__cpm_stop_lcd();

	return 0;
}

/*
 * Resume the LCDC.
 */
static int jz4760_fb_resume(struct platform_device *pdev)
{
	__cpm_start_lcd();
	screen_on();
	__lcd_set_ena();

	return 0;
}

#else
#define jzfb_suspend      NULL
#define jzfb_resume       NULL
#endif /* CONFIG_PM */


/* Backlight Control Interface via sysfs
 *
 * LCDC:
 * Enabling LCDC when LCD backlight is off will only affects cfb->display.
 *
 * Backlight:
 * Changing the value of LCD backlight when LCDC is off will only affect the cfb->backlight_level.
 *
 * - River.
 */
static int screen_off(void)
{
	struct lcd_cfb_info *cfb = jz4760fb_info;

	__lcd_close_backlight();
	__lcd_display_off();

#ifdef HAVE_LCD_PWM_CONTROL
	if (cfb->b_lcd_pwm) {
		__lcd_pwm_stop();
		cfb->b_lcd_pwm = 0;
	}
#endif

	cfb->b_lcd_display = 0;

	return 0;
}

static int screen_on(void)
{
	struct lcd_cfb_info *cfb = jz4760fb_info;
    if(panle_mode != PANEL_MODE_LCD_PANEL)
      return 0;
	//__lcd_display_on();//allen del


	/* Really restore LCD backlight when LCD backlight is turned on. */
	if (cfb->backlight_level) {
#ifdef HAVE_LCD_PWM_CONTROL
		if (!cfb->b_lcd_pwm) {
			__lcd_pwm_start();
			cfb->b_lcd_pwm = 1;
		}
#endif
		__lcd_set_backlight_level(cfb->backlight_level);
	}

	cfb->b_lcd_display = 1;

	return 0;
}

static int screen_on_new(void)
{
	struct lcd_cfb_info *cfb = jz4760fb_info;
	//__lcd_display_on();  //allen del

	/* Really restore LCD backlight when LCD backlight is turned on. */
	if (cfb->backlight_level) {
#ifdef HAVE_LCD_PWM_CONTROL
		if (!cfb->b_lcd_pwm) {
			__lcd_pwm_start();
			cfb->b_lcd_pwm = 1;
		}
#endif
		__lcd_set_backlight_level(cfb->backlight_level);
	}

	cfb->b_lcd_display = 1;

	return 0;
}


static int jz4760fb_set_backlight_level(int n)
{
	struct lcd_cfb_info *cfb = jz4760fb_info;

	if (n) {
		if (n > LCD_MAX_BACKLIGHT)
			n = LCD_MAX_BACKLIGHT;

		if (n < LCD_MIN_BACKLIGHT)
			n = LCD_MIN_BACKLIGHT;

		/* Really change the value of backlight when LCDC is enabled. */
		if (cfb->b_lcd_display) {
#ifdef HAVE_LCD_PWM_CONTROL
			if (!cfb->b_lcd_pwm) {
				__lcd_pwm_start();
				cfb->b_lcd_pwm = 1;
			}
#endif
			__lcd_set_backlight_level(n);
		}
	}else{
		/* Turn off LCD backlight. */
		__lcd_close_backlight();

#ifdef HAVE_LCD_PWM_CONTROL
		if (cfb->b_lcd_pwm) {
			__lcd_pwm_stop();
			cfb->b_lcd_pwm = 0;
		}
#endif
	}

	cfb->backlight_level = n;

	return 0;
}

static ssize_t show_bl_level(struct device *device,
			     struct device_attribute *attr, char *buf)
{
	struct lcd_cfb_info *cfb = jz4760fb_info;

	return snprintf(buf, PAGE_SIZE, "%d\n", cfb->backlight_level);
}

static ssize_t store_bl_level(struct device *device,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int n;
	char *ep;

	n = simple_strtoul(buf, &ep, 0);
	if (*ep && *ep != '\n')
		return -EINVAL;

	jz4760fb_set_backlight_level(n);

	return count;
}

static struct device_attribute device_attrs[] = {
	__ATTR(backlight_level, S_IRUGO | S_IWUSR, show_bl_level, store_bl_level),
};

static int jz4760fb_device_attr_register(struct fb_info *fb_info)
{
	int error = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(device_attrs); i++) {
		error = device_create_file(fb_info->dev, &device_attrs[i]);

		if (error)
			break;
	}

	if (error) {
		while (--i >= 0)
			device_remove_file(fb_info->dev, &device_attrs[i]);
	}

	return 0;
}

static int jz4760fb_device_attr_unregister(struct fb_info *fb_info)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(device_attrs); i++)
		device_remove_file(fb_info->dev, &device_attrs[i]);

	return 0;
}
/* End */

static void gpio_init(void)
{
	__lcd_display_pin_init();  //LCD REST
	if (jz4760_lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD)
		__gpio_as_lcd_8bit();
	else if (jz4760_lcd_info->panel.cfg & LCD_CFG_MODE_SERIAL_TFT)
	{ //__gpio_as_lcd_8bit no use lcd_de
		REG_GPIO_PXFUNS(2) = 0x000c31fc;
		REG_GPIO_PXTRGC(2) = 0x000c31fc;
		REG_GPIO_PXSELC(2) = 0x000c31fc;
		REG_GPIO_PXPES(2)  = 0x000c31fc;
	}
	else
	{
		/* gpio init __gpio_as_lcd */
		if (jz4760_lcd_info->panel.cfg & LCD_CFG_MODE_TFT_16BIT)
			__gpio_as_lcd_16bit();
		else if (jz4760_lcd_info->panel.cfg & LCD_CFG_MODE_TFT_24BIT)
			__gpio_as_lcd_24bit();
		else
	 		__gpio_as_lcd_18bit();
	}

	return;
}

static void set_bpp_to_ctrl_bpp(void)
{
	switch (jz4760_lcd_info->osd.fg0.bpp) {
		case 15:
		case 16:
			break;

		case 17 ... 32:
			jz4760_lcd_info->osd.fg0.bpp = 32;
			break;

		default:
			jz4760_lcd_info->osd.fg0.bpp = 32;
			break;
	}

	switch (jz4760_lcd_info->osd.fg1.bpp) {
		case 15:
		case 16:
			break;

		case 17 ... 32:
			jz4760_lcd_info->osd.fg1.bpp = 32;
			break;

		default:
			jz4760_lcd_info->osd.fg1.bpp = 32;
			break;
	}

	return;
}

static void slcd_init(void)
{
	/* Configure SLCD module for setting smart lcd control registers */
#if defined(CONFIG_FB_JZ4760_SLCD)
	__lcd_as_smart_lcd();
	__slcd_disable_dma();
#endif
}

static void open_ausd_me()
{
#ifdef HP_POWER_EN
	__gpio_as_func0(HP_POWER_EN);
	__gpio_as_output(HP_POWER_EN);
	__gpio_enable_pull(HP_POWER_EN);

 #ifdef EARPHONE_DETE
	if(__gpio_get_pin(EARPHONE_DETE) != DETE_ACTIV_LEVEL)
		if (l009_globle_volume)
		__gpio_set_pin(HP_POWER_EN);
	else
		__gpio_clear_pin(HP_POWER_EN);
 #else
	if (l009_globle_volume)
		__gpio_set_pin(HP_POWER_EN);

 #endif
#endif
}

static void close_ausd_me()
{
#ifdef HP_POWER_EN
	__gpio_as_func0(HP_POWER_EN);
	__gpio_as_output(HP_POWER_EN);
	__gpio_clear_pin(HP_POWER_EN);
#endif
}

void clear_tv_framebuff(void)
{
	memset(lcd_frame0,0,640*480*2);
}

void clear_framebuff(void)
{
	memset(lcd_frame0,0,LCD_SCREEN_W*LCD_SCREEN_H*2);
}


int get_tvout_flag(void)
{
	return tvout_flag;
}
EXPORT_SYMBOL(get_tvout_flag);


#ifdef  AV_OUT_DETE
void avout_ack_timer(unsigned long data)
{

	int tmp_flag = 0;

	if(__gpio_get_pin(AV_OUT_DETE) != 0)
	{
		tmp_flag = 0;
		panle_mode = 0;
		open_ausd_me();
	}
  	else
  	{
		panle_mode = 1;
		if (tvout_flag2 == 2)
			tmp_flag = 2;
		else if (tvout_flag2 == 3)
			tmp_flag = 3;
		else
			tmp_flag = 1;

	    /* close ausd */
	    close_ausd_me();
  	}

	//printk("avout_ack_time tvout_flag %d,tmp_flag %d,tvout_flag2 %d \n",tvout_flag,tmp_flag,tvout_flag2);

	if(tvout_flag==0  && tmp_flag==1)	//lcd to pal
	{
		jz4760lcd_info_switch_to_TVE(PANEL_MODE_TVE_NTSC);
		jz4760tve_init(PANEL_MODE_TVE_NTSC); /* tve controller init */
		udelay(100);
		cpm_start_clock(CGM_TVE);
		jz4760tve_enable_tve();
		/* turn off lcd backlight */
		__lcd_display_off();
        tvout_640_480 = 0;
		jz4760fb_deep_set_mode(jz4760_lcd_info);
		//fb_resize_start();

		ipu_driver_open_tv(LCD_SCREEN_W,LCD_SCREEN_H,640,480);


		printk("1231\n");
	}
	else if((tvout_flag==1 || tvout_flag == 2) && tmp_flag==0)  //tvout to lcd
	{
		if (tvout_flag == 2){
			clear_tv_framebuff();
		}

		jz4760tve_disable_tve();
		cpm_stop_clock(CGM_TVE);
		udelay(100);
		jz4760_lcd_info = &jz4760_lcd_panel;
		/* turn off lcd backlight */
		tvout_640_480 = 0;
		jz4760fb_deep_set_mode(jz4760_lcd_info);
		resize_go_out = 1;
		__lcd_slcd_special_on();

		if (tvout_flag == 1)
			ipu_driver_close_tv();

		if (tvout_flag == 1){
			if (lcd_a320_flag > 0){
				memset(lcd_frame0,0x00,LCD_SCREEN_W*LCD_SCREEN_H*2);
				memset(lcd_frame01,0x00,640*480*2);
				dma0_desc0->databuf = virt_to_phys((void *)lcd_frame01);
			}
			mdelay(1000);
		}
		screen_on();
		//__lcd_display_on();
		//__lcd_set_backlight_level(backlight_value);	/* We support 8 levels here. */
	}
	else if(tvout_flag==1 && tmp_flag==2)  //tvout_2x to tvout_640x480
	{
			clear_framebuff();
			resize_go_out = 1;
			mdelay(500);
			tvout_640_480 = 1;
			jz4760fb_deep_set_mode(jz4760_lcd_info);
	}
	else if(tvout_flag==2 && tmp_flag==1)  //tvout_640x480 to tvout_2x
	{
			//fb_resize_start();
			clear_tv_framebuff();
			tvout_640_480 = 0;
			jz4760fb_deep_set_mode(jz4760_lcd_info);
	}
	else if(tvout_flag==0 && tmp_flag==2)  //tvout_2x to tvout_640x480
	{
		jz4760lcd_info_switch_to_TVE(PANEL_MODE_TVE_NTSC);
		jz4760tve_init(PANEL_MODE_TVE_NTSC); /* tve controller init */
		udelay(100);
		cpm_start_clock(CGM_TVE);
		jz4760tve_enable_tve();
		/* turn off lcd backlight */
		__lcd_display_off();
		tvout_640_480 = 1;
		resize_go_out = 1;
		jz4760fb_deep_set_mode(jz4760_lcd_info);

	}
	else if (tvout_flag == 0 && tmp_flag == 3)
	{
	//	__lcd_display_off();

	}else if (tvout_flag == 3 && tmp_flag == 0){
	//	screen_on();
	}

	if (tvout_flag != tmp_flag)
		tvout_flag = tmp_flag;
}

static irqreturn_t avout_pnp_irq(int irq, void *dev_id)
{

 // /* mask interrupt */
  //__gpio_mask_irq(AV_OUT_DETE);

  //printk("avout dete irq ............ \n");
  avout_irq_timer.expires = jiffies + 1*HZ;
  del_timer(&avout_irq_timer);
  add_timer(&avout_irq_timer);
  __gpio_unmask_irq(AV_OUT_DETE);
  return IRQ_HANDLED;
}

#endif

static int __devinit jz4760_fb_probe(struct platform_device *dev)
{
	struct lcd_cfb_info *cfb;

    int rv = 0;
    cpm_start_clock(CGM_IPU);
	cfb = jz4760fb_alloc_fb_info();
	if (!cfb)
		goto failed;

	screen_off();
	ctrl_disable();

	gpio_init();
	slcd_init();

	set_bpp_to_ctrl_bpp();

	/* init clk */
	//jz4760fb_change_clock(jz4760_lcd_info); //Lcd parame

	rv = jz4760fb_map_smem(cfb);
	if (rv)
		goto failed;

	jz4760fb_deep_set_mode(jz4760_lcd_info);

	rv = register_framebuffer(&cfb->fb);
	if (rv < 0) {
		D("Failed to register framebuffer device.");
		goto failed;
	}

	printk("fb%d: %s frame buffer device, using %dK of video memory\n",
	       cfb->fb.node, cfb->fb.fix.id, cfb->fb.fix.smem_len>>10);

	jz4760fb_device_attr_register(&cfb->fb);

	if (request_irq(IRQ_LCD, jz4760fb_interrupt_handler, IRQF_DISABLED,"lcd", 0))
	{
		D("Faield to request LCD IRQ.\n");
		rv = -EBUSY;
		goto failed;
	}

	ctrl_enable();

	__lcd_display_on();
	//fill black
#if (RGB_TEST)
	int i = 0;
	unsigned short*ptr;
	ptr = (unsigned short*)lcd_frame0;

	for(i = 0 ; i < LCD_SCREEN_W*LCD_SCREEN_H/3; i++)
	{
		*ptr++ = 0xf800;
	}
	
	for(i = 0 ; i < LCD_SCREEN_W*LCD_SCREEN_H/3; i++)
	{
		*ptr++ = 0x07e0;
	}
	
	for(i = 0 ; i < LCD_SCREEN_W*LCD_SCREEN_H/3; i++)
	{
		*ptr++ = 0x001f;
	}
	
	dma_cache_wback((unsigned int)lcd_frame0, LCD_SCREEN_W * LCD_SCREEN_H * 2);
	mdelay(3000);
#else
	memset(lcd_frame0, 0x00, LCD_SCREEN_W * LCD_SCREEN_W * 2);
	dma_cache_wback((unsigned int)lcd_frame0, LCD_SCREEN_W * LCD_SCREEN_W * 2);
	mdelay(50);	//needed to avoid flash white screen

#endif
	/* Really restore LCD backlight when LCD backlight is turned on. */
	if (cfb->backlight_level) {
#ifdef HAVE_LCD_PWM_CONTROL
		if (!cfb->b_lcd_pwm) {
			__lcd_pwm_start();
			cfb->b_lcd_pwm = 1;
		}
#endif
		__lcd_set_backlight_level(cfb->backlight_level);
	}

	cfb->b_lcd_display = 1;

#ifdef AV_OUT_DETE
        init_timer(&avout_irq_timer);
        avout_irq_timer.function = avout_ack_timer;
        avout_irq_timer.data = 0;
        avout_irq_timer.expires = jiffies + 2*HZ;
        add_timer(&avout_irq_timer);

        __gpio_as_func0(AV_OUT_DETE);
        __gpio_as_input(AV_OUT_DETE);
        __gpio_disable_pull(AV_OUT_DETE);

		printk("\n.................\n");


        if(__gpio_get_pin(AV_OUT_DETE) != 0)
        {
          	__gpio_as_irq_fall_edge(AV_OUT_DETE);
        }
        else
        {
          	__gpio_as_irq_rise_edge(AV_OUT_DETE);
        }
        int ret;
        ret = request_irq(AV_OUT_DETE_IRQ, avout_pnp_irq,
            IRQF_DISABLED, "avout_pnp", NULL);
        if (ret) {
          printk("%s %d Could not get avout irq %d\n",__FILE__,__LINE__, AV_OUT_DETE_IRQ);
          return ret;
        }
		printk("\n........xxx........\n");
#endif


	return 0;

failed:
	jz4760fb_unmap_smem(cfb);
	jz4760fb_free_fb_info(cfb);

	return rv;
}

static int __devexit jz4760_fb_remove(struct platform_device *pdev)
{
	struct lcd_cfb_info *cfb = jz4760fb_info;

	jz4760fb_device_attr_unregister(&cfb->fb);

	return 0;
}

static struct platform_driver jz4760_fb_driver = {
	.probe	= jz4760_fb_probe,
	.remove = jz4760_fb_remove,
	.suspend = jz4760_fb_suspend,
	.resume = jz4760_fb_resume,
	.driver = {
		.name = "jz-lcd",
		.owner = THIS_MODULE,
	},
};
static int proc_lcd_flush_write_proc(
			struct file *file, const char *buffer,
			unsigned long count, void *data)
{
	unsigned int framebuffer_size;

	return count;
	
  	//printk("%s %d\n",__FILE__,tvout_flag);
  if(tvout_flag == 0)
  {
    lcd_flush_flag =  simple_strtoul(buffer, 0, 10);

    //printk("%s %d  lcd_flush_flag is %d\n",__FILE__,__LINE__,lcd_flush_flag);
  
    framebuffer_size = jz4760_lcd_info->panel.w*jz4760_lcd_info->panel.h*(jz4760_lcd_info->osd.fg0.bpp/8);
    if(lcd_flush_flag == 1)  //start flush	
    { 
      D("\n");
      //memcpy(lcd_frame0,lcd_frame01,framebuffer_size);
      D("\n");
      //mdelay(500);
      D("\n");
      //dma_cache_wback((unsigned int)(lcd_frame0),framebuffer_size);
      dma0_desc0->databuf = virt_to_phys((void *)lcd_frame0);
    }
    else
    {
      D("\n");
      memcpy(lcd_frame01,lcd_frame0,framebuffer_size);
      D("\n");
      //mdelay(500);
      D("\n");
      dma_cache_wback((unsigned int)(lcd_frame01),framebuffer_size);
      dma0_desc0->databuf = virt_to_phys((void *)lcd_frame01);
    }
    //jz4750fb_deep_set_mode(jz4750_lcd_info);
  }
  
  return count;
}
static int proc_lcd_flush_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%u\n", panle_mode);
}

static int proc_lcd_backlight_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%u\n", backlight_value);
}

static int proc_lcd_backlight_write_proc(
			struct file *file, const char *buffer,
			unsigned long count, void *data)
{
    if (tvout_flag)
        return count;

	backlight_value =  simple_strtoul(buffer, 0, 10);
	//printk("backlight_value is %d \n",backlight_value);
	if(backlight_value == 0)
	{
		__gpio_clear_pin(GPIO_LCD_VCC_EN_N);
		__lcd_close_backlight();

		//__gpio_clear_pin(GPIO_TP_VCC_EN); //allen add for touch
	}
	else
	{
		__gpio_set_pin(GPIO_LCD_VCC_EN_N);
		__lcd_set_backlight_level(backlight_value);

		//__gpio_set_pin(GPIO_TP_VCC_EN); //allen add for touch
	}
	return count;
}

#define GPIO_WIFI_PW (32*5+7) //(32*3+3)
static int wifi_pw_on_flag = 0;
static int proc_wifi_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%lu\n", wifi_pw_on_flag);
}


static int proc_wifi_write_proc(
		struct file *file, const char *buffer,
		unsigned long count, void *data)
{

	wifi_pw_on_flag =  simple_strtoul(buffer, 0, 10);
	printk("\n wifi pw flag = %d\n",wifi_pw_on_flag);
	__gpio_as_func0(GPIO_WIFI_PW);
	__gpio_as_output(GPIO_WIFI_PW);
	if (wifi_pw_on_flag){
		__gpio_set_pin(GPIO_WIFI_PW);
	}else{
		__gpio_clear_pin(GPIO_WIFI_PW);
	}

	return count;
}

//============ allen add ===================
int app_set_tv_flag =0;
int old_tv_flag;
static void allen_tv_out()
{
	int tmp_flag = 0;
	
	//printk("%s %d %d\n",__func__,__LINE__,app_set_tv_flag);

    if(0 == app_set_tv_flag)
    {
	    tmp_flag = 0;

		panle_mode = 0;
		old_tv_flag = 0;
	    open_ausd_me();
    }
  	else
  	{
		panle_mode = 1;
	   	tmp_flag = 1;
	    close_ausd_me();
  	}

	if((old_tv_flag !=app_set_tv_flag) && tmp_flag)	//lcd to pal
	{
		if(old_tv_flag)
		{
			ipu_driver_close_tv();
			jz4760tve_disable_tve();
			cpm_stop_clock(CGM_TVE);
			udelay(100);

			memset(lcd_frame0,0x00,LCD_SCREEN_W*LCD_SCREEN_H*2);
			memset(lcd_frame01,0x00,640*480*2);

			mdelay(5000);
		}

		old_tv_flag = app_set_tv_flag;

		if(1 == app_set_tv_flag)
		{
			jz4760lcd_info_switch_to_TVE(PANEL_MODE_TVE_PAL);
			jz4760tve_init(PANEL_MODE_TVE_PAL); /* tve controller init */
			printk("entern PAL ...\n");
		}
		else
		{
			jz4760lcd_info_switch_to_TVE(PANEL_MODE_TVE_NTSC);
			jz4760tve_init(PANEL_MODE_TVE_NTSC); /* tve controller init */
			printk("entern NTSC ...\n");
		}
		udelay(100);

		cpm_start_clock(CGM_TVE);
		jz4760tve_enable_tve();
		__lcd_display_off();
        tvout_640_480 = 0;
		jz4760fb_deep_set_mode(jz4760_lcd_info);

		ipu_driver_open_tv(LCD_SCREEN_W,LCD_SCREEN_H,640,480);

	}
	else if((tvout_flag==1 || tvout_flag == 2) && tmp_flag==0)  //tvout to lcd
	{
		printk("tv_out exit to lcd\n");
		if (tvout_flag == 2)
			clear_tv_framebuff();

		jz4760tve_disable_tve();
		cpm_stop_clock(CGM_TVE);
		udelay(100);
		jz4760_lcd_info = &jz4760_lcd_panel;
		tvout_640_480 = 0;
		jz4760fb_deep_set_mode(jz4760_lcd_info);
		resize_go_out = 1;
		__lcd_slcd_special_on();

		if (tvout_flag == 1)
			ipu_driver_close_tv();

		if (tvout_flag == 1){
			if (lcd_a320_flag > 0){
				memset(lcd_frame0,0x00,LCD_SCREEN_W*LCD_SCREEN_H*2);
				memset(lcd_frame01,0x00,640*480*2);
				dma0_desc0->databuf = virt_to_phys((void *)lcd_frame01);
			}
			mdelay(1000);
		}
	#if 0 //allen del
		screen_on();
		//__lcd_display_on();
	#else
		clear_framebuff();
		__lcd_set_backlight_level(backlight_value);
		//printk("backlight_value is %d \n",backlight_value);
	#endif
	}

	if (tvout_flag != tmp_flag)
		tvout_flag = tmp_flag;
}

static int proc_tvselect_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%lu\n", app_set_tv_flag);
}

static int proc_tvselect_write_proc(struct file *file, const char *buffer,
			unsigned long count, void *data)
{
	app_set_tv_flag = simple_strtoul(buffer, 0, 10);
	//printk("app_set_tv_flag is %d \n",app_set_tv_flag);
	allen_tv_out();
	return count;
}

static int fb_resize_a320_original_thread(void *unused)
{
	printk("kernel frame buffer fb_resize_a320_original_thread start!\n");

	while(1)
	{
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ/20);
		fb_a320_thread(320,240,LCD_SCREEN_W,LCD_SCREEN_H,A320_ORIGINAL);
		if(resize_a320_go_out)
		break;
	}
	resize_a320_original_task = 0;

}
static void fb_resize_a320_original_start()
{
	if (resize_a320_original_task != 0)
		return;
	resize_a320_go_out= 0;
	resize_a320_original_task= kthread_run(fb_resize_a320_original_thread, NULL, "fb_a320_original");
	if(IS_ERR(resize_a320_original_task))
	{
		printk("Kernel fb_a320_originalstart error!\n");
		return;
	}
}
static int fb_resize_a320_fullscreen_thread(void *unused)
{
  printk("kernel frame buffer resize thread start!\n");
  while(1)
  {
    set_current_state(TASK_INTERRUPTIBLE);
    schedule_timeout(HZ/20);
    fb_a320_thread(320,240,LCD_SCREEN_W,LCD_SCREEN_H,A320_FULLSCREEN);
    if(resize_a320_go_out)
      break;
  }
	resize_a320_full_screen_task = 0;
}
static void fb_resize_a320_fullscreen_start()
{
	if (resize_a320_full_screen_task != 0)
		return;
	resize_a320_go_out = 0;
	resize_a320_full_screen_task = kthread_run(fb_resize_a320_fullscreen_thread, NULL, "fb_a320_fullscreen");
	if(IS_ERR(resize_a320_full_screen_task))
	{
		printk("Kernel fb_a320_fullscreen start error!\n");
		return;
	}
}

static int proc_lcd_a320_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%lu\n", lcd_a320_flag);
}

static int proc_lcd_a320_write_proc(
			struct file *file, const char *buffer,
			unsigned long count, void *data)
{
  int tmp_a320_flag;

  tmp_a320_flag =  simple_strtoul(buffer, 0, 10);

  if (tmp_a320_flag != lcd_a320_flag){
	  lcd_a320_flag = tmp_a320_flag;
  }else{
	  return count;
  }

  if(lcd_a320_flag == 0)
  {
	  resize_a320_go_out = 1;
	  if (!tvout_flag)
	  	dma0_desc0->databuf = virt_to_phys((void *)lcd_frame0);
  }
  else if(lcd_a320_flag == 1)
  {
	  resize_a320_go_out = 1;
	  memset(lcd_frame0,0x00,LCD_SCREEN_W*LCD_SCREEN_H*2);
	  memset(lcd_frame01,0x00,LCD_SCREEN_W*LCD_SCREEN_H*2);
	  dma0_desc0->databuf = virt_to_phys((void *)lcd_frame01);
	  fb_resize_a320_original_start();
  }
  else if(lcd_a320_flag == 2)
  {
	  resize_a320_go_out = 1;
	  memset(lcd_frame0,0x00,LCD_SCREEN_W*LCD_SCREEN_H*2);
	  memset(lcd_frame01,0x00,LCD_SCREEN_W*LCD_SCREEN_H*2);
	  dma0_desc0->databuf = virt_to_phys((void *)lcd_frame01);
	  fb_resize_a320_fullscreen_start();
  }
  return count;
}

static int proc_tvout_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%u\n", tvout_flag);
}

//when display in tvout,this proc called
static int proc_tvout_write_proc(
			struct file *file, const char *buffer,
			unsigned long count, void *data)
{
	tvout_flag2 =  simple_strtoul(buffer, 0, 10);

	if ((tvout_flag == 1) && (tvout_flag2 == 2))
	{
		resize_go_out = 1;
		clear_framebuff();
		mdelay(500);
		tvout_640_480 = 1;
		jz4760fb_deep_set_mode(jz4760_lcd_info);
		tvout_flag = 2;

		ipu_driver_close_tv();
	}
	else if ((tvout_flag == 2) && (tvout_flag2 == 0 || tvout_flag2 == 1))
	{
		clear_tv_framebuff();
		tvout_640_480 = 0;
		jz4760fb_deep_set_mode(jz4760_lcd_info);
		tvout_flag = 1;

		ipu_driver_open_tv(LCD_SCREEN_W,LCD_SCREEN_H,640,480);
	}

	//cim no suppor tvout!
	if (tvout_flag2 == 3 && tvout_flag == 1)
	{
		jz4760tve_disable_tve();
		cpm_stop_clock(CGM_TVE);
		udelay(100);
		jz4760_lcd_info = &jz4760_lcd_panel;
		/* turn off lcd backlight */
		tvout_640_480 = 0;
		jz4760fb_deep_set_mode(jz4760_lcd_info);
		resize_go_out = 1;
		__lcd_slcd_special_on();
		ipu_driver_close_tv();

		if (tvout_flag == 1)
			mdelay(1000);
		screen_on_new();

		tvout_flag = 3;
	}

	if (tvout_flag2 == 1 && tvout_flag == 3)
	{
		if(1 == app_set_tv_flag)
		{
			jz4760lcd_info_switch_to_TVE(PANEL_MODE_TVE_PAL);
			jz4760tve_init(PANEL_MODE_TVE_PAL); /* tve controller init */
		}
		else
		{
			jz4760lcd_info_switch_to_TVE(PANEL_MODE_TVE_NTSC);
			jz4760tve_init(PANEL_MODE_TVE_NTSC); /* tve controller init */
		}
		udelay(100);
		cpm_start_clock(CGM_TVE);
		jz4760tve_enable_tve();
		/* turn off lcd backlight */
		__lcd_display_off();
		tvout_640_480 = 0;
		jz4760fb_deep_set_mode(jz4760_lcd_info);

		ipu_driver_open_tv(LCD_SCREEN_W,LCD_SCREEN_H,640,480);

		tvout_flag = 1;
	}

	return count;
}

static int __init jz4760_fb_init(void)
{
	struct proc_dir_entry *res;

	//app maybe mode jz4760_lcd_panel.osd.fg0,so use global  Variable
	LCD_SCREEN_W = jz4760_lcd_panel.panel.w;
	LCD_SCREEN_H = jz4760_lcd_panel.panel.h;//allen add

	res = create_proc_entry("jz/lcd_backlight", 0, NULL);
	if(res)
	{
		res->read_proc = proc_lcd_backlight_read_proc;
		res->write_proc = proc_lcd_backlight_write_proc;
	}

	res = create_proc_entry("jz/tvout", 0, NULL);
	if(res)
	{
		res->read_proc = proc_tvout_read_proc;
		res->write_proc = proc_tvout_write_proc;
	}
	res = create_proc_entry("jz/lcd_flush", 0, NULL);
	if(res)
	{
		res->read_proc = proc_lcd_flush_read_proc;
		res->write_proc = proc_lcd_flush_write_proc;
	}

	res = create_proc_entry("jz/wifi_pw", 0, NULL);
	if(res)
	{
		res->read_proc = proc_wifi_read_proc;
		res->write_proc = proc_wifi_write_proc;
		res->data = NULL;
	}

	res = create_proc_entry("jz/lcd_a320", 0, NULL);
	if(res)
	{
		res->read_proc = proc_lcd_a320_read_proc;
		res->write_proc = proc_lcd_a320_write_proc;
		res->data = NULL;
	}
	res = create_proc_entry("jz/tvselect", 0, NULL);
	if(res)
	{
		res->data = NULL;
		res->read_proc = proc_tvselect_read_proc;
		res->write_proc = proc_tvselect_write_proc;
	}

  	return platform_driver_register(&jz4760_fb_driver);
}

static void __exit jz4760_fb_cleanup(void)
{
	platform_driver_unregister(&jz4760_fb_driver);
}


module_init(jz4760_fb_init);
module_exit(jz4760_fb_cleanup);

MODULE_DESCRIPTION("Jz4760 LCD Controller driver");
MODULE_AUTHOR("Wolfgang Wang, <lgwang@ingenic.cn>");
MODULE_LICENSE("GPL");

