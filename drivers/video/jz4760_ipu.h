#ifndef __IPU_H_ME__
#define __IPU_H_ME__

// IPU Control Register
#define BUS_OPT						( 1 << 22 ) //
#define CONF_MOD					( 1 << 21 ) //
#define ADDR_SEL                    ( 1 << 20 )
#define BURST_SEL					( 1 << 19 ) //
#define ZOOM_SEL 					( 1 << 18 ) //
#define DFIX_SEL					( 1 << 17 )
#define FIELD_SEL                   ( 1 << 16 )
#define FIELD_CONF_EN               ( 1 << 15 )
#define DISP_SEL   	                ( 1 << 14 )
#define DPAGE_MAP                   ( 1 << 13 )
#define SPAGE_MAP                   ( 1 << 12 )
#define LCDC_SEL                    ( 1 << 11 )
#define SPKG_SEL                    ( 1 << 10 )
//#define V_SCALE                     ( 1 << 9 ) //???
//#define H_SCALE                     ( 1 << 8 ) //???
#define IPU_RST                     ( 1 << 6 )
#define FM_IRQ_EN                   ( 1 << 5 )
#define CSC_EN                      ( 1 << 4 )
#define VRSZ_EN                     ( 1 << 3 )
#define HRSZ_EN                     ( 1 << 2 )
#define IPU_RUN                     ( 1 << 1 )
#define CHIP_EN                     ( 1 << 0 )

//#define V_SCALE_BIT					( 9 )
//#define H_SCALE_BIT					( 8 )


#define IPU_OUTDATA_FMT_888                    ( 2 << 19 )
#define IPU_OUTDATA_FMT_565                    ( 1 << 19 )
#define IPU_OUTDATA_FMT_YUV422PACKG            ( 3 << 19 )
#define IPU_YUV_PKG_OUT_OFT_VY1UY0             ( 3 << 16 )
#define IPU_INDATA_PACKAGE_OFFSIZE             ( 3 << 2 )
#define IPU_INDATA_FMT_YUV422                  ( 1 << 0 )

#define IPU_INDATA_FMT_RGB565                  ( 3 << 0 )
#define IPU_INDATA_FMT_RGB555                  ( 0 << 0 )
#define IPU_INDATA_FMT_RGB888                  ( 2 << 0 )

// IPU Status Register
#define SIZE_ERR                    ( 1 << 2 )
#define FMT_ERR                     ( 1 << 1 )
#define OUT_END                     ( 1 << 0 )

// IPU address control register
#define PTD_READY                   (1 << 5)
#define PTS_READY                   (1 << 4)

//no need more
#define PTV_READY                   (1 << 6) 
#define PTU_READY                   (1 << 5)
#define PTY_READY                   (1 << 4)

/***wyang***/
#define D_READY                     (1 << 3)
//#define Y_READY                   (1 << 2)
#define V_READY                     (1 << 2)
#define U_READY                     (1 << 1)
//#define V_READY                   (1 << 0)
#define Y_READY                     (1 << 0)

#define YUV_READY                   (Y_READY | U_READY | V_READY)

// Input Geometric Size Register
#define IN_FM_W_BIT					( 16 )
#define IN_FM_W_MASK				( 0xfff << IN_FM_W_BIT )

#define IN_FM_H_BIT					( 0 )
#define IN_FM_H_MASK				( 0xfff << IN_FM_H_BIT )

#define IN_FM_W(val)				((val) << IN_FM_W_BIT)
#define IN_FM_H(val)				((val) << IN_FM_H_BIT)

// Input UV Data Line Stride Register
/***wyang***/
#define U_S_BIT						( 16 )
//#define U_S_MASK					( 0xfff << U_S_BIT )
#define U_S_MASK					( 0x1fff << U_S_BIT )

#define V_S_BIT						( 0 )
//#define V_S_MASK					( 0xfff << V_S_BIT )
#define V_S_MASK					( 0x1fff << V_S_BIT )

#define U_STRIDE(val)				((val) << U_S_BIT)
#define V_STRIDE(val)				((val) << V_S_BIT)

// Output Geometric Size Register
/***wyang***/
#define OUT_FM_W_BIT				( 16 )
//#define OUT_FM_W_MASK				( 0x7fff << OUT_FM_W_BIT ) //
#define OUT_FM_W_MASK				( 0x1fff << OUT_FM_W_BIT ) //

#define OUT_FM_H_BIT				( 0 )
#define OUT_FM_H_MASK				( 0x1fff << OUT_FM_H_BIT ) //

#define OUT_FM_W(val)				((val) << OUT_FM_W_BIT)
#define OUT_FM_H(val)				((val) << OUT_FM_H_BIT)

// Resize Coefficients Table Index Register
#define HE_IDX_W_BIT				( 16 )
#define HE_IDX_W_MASK				( 0x1f << HE_IDX_W_BIT )

#define VE_IDX_H_BIT				( 0 )
#define VE_IDX_H_MASK				( 0x1f << VE_IDX_H_BIT )

// common parameter Resize Coefficients Look Up Table Register group
//#define OUT_EN_BIT					( 0 )
//#define IN_EN_BIT					( 1 )
//#define W_COEF_BIT					( 2 )
//#define W_COEF_MASK					(0x3ff << W_COEF_BIT)
//#define LUT_START					(1 << 12)

//H & V Resize Coefficients Look Up Table Register group
/*common define*/
/***wyang***/
#define W_COEF_20_BIT				(6)
//#define W_COEF_20_MASK		(0x7ff << W_COEF_20_BIT)
#define W_COEF_20_MASK			(0x3ff << W_COEF_20_BIT)
#define W_COEF_31_BIT				(17)
//#define W_COEF_31_MASK		(0x7ff << W_COEF_31_BIT)
#define W_COEF_31_MASK			(0x3ff << W_COEF_31_BIT)
#define W_COEF0_BIT					(6)
//#define W_COEF0_MSK			(0x7ff)
#define W_COEF0_MSK				(0x3ff)
//#define W_COEF0_MASK				(0x7ff << W_COEF0_BIT)
#define W_COEF0_MASK				(0x3ff << W_COEF0_BIT)

/*H conf*/
#define H_CONF_BIT					(0)
#define H_CONF_MASK					(1 << H_CONF_BIT)

/*H bi-cube*/
#define HRSZ_OFT_BIT				(1)
#define HRSZ_OFT_MASK				(0x1f << HRSZ_OFT_BIT)

/*H bi-linear*/
#define H_OFT_BIT					(1)
#define H_OFT_MSK					(0x1f)
#define H_OFT_MASK					(0x1f << H_OFT_BIT)

/*V conf*/
#define V_CONF_BIT					(0)
#define V_CONF_MASK					(1 << V_CONF_BIT)
/*bi-cube*/
#define VRSZ_OFT_BIT				(1)
#define VRSZ_OFT_MASK				(0x1f << VRSZ_OFT_BIT)
/*bi-linear*/
#define V_OFT_BIT					(1)
#define V_OFT_MSK					(0x1f)
#define V_OFT_MASK					(0x1f << V_OFT_BIT)


// CSC Offset Parameter Register
#define CHROM_OF_W_BIT				( 16 )
#define CHROM_OF_W_MASK				( 0xff << CHROM_OF_W_BIT )
#define CHROM(x)					(x << CHROM_OF_W_BIT)

#define LUMA_OF_H_BIT				( 0 )
#define LUMA_OF_H_MASK				( 0xff << LUMA_OF_H_BIT )
#define LUMA(x)						(x << LUMA_OF_H_BIT)

#define W_CUBE_COEF0_SFT 0x6
#define W_CUBE_COEF0_MSK 0x7FF
#define W_CUBE_COEF1_SFT 0x11
#define W_CUBE_COEF1_MSK 0x7FF
#define ZOOM_SEL_SFT 18



typedef struct
{
	unsigned int	coef;
	unsigned short	in_n;
	unsigned short	out_n;
} rsz_lut;


// Data Format Register
#define RGB_888_OUT_FMT				( 1 << 25 )

#define RGB_OUT_OFT_BIT				( 22 )
#define RGB_OUT_OFT_MASK			( 7 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_RGB				( 0 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_RBG				( 1 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_GBR				( 2 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_GRB				( 3 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_BRG				( 4 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_BGR				( 5 << RGB_OUT_OFT_BIT )

#define OUT_FMT_BIT					( 19 )
#define OUT_FMT_MASK				( 7 <<  OUT_FMT_BIT )
#define OUT_FMT_RGB555				( 0 <<  OUT_FMT_BIT )
#define OUT_FMT_RGB565				( 1 <<  OUT_FMT_BIT )
#define OUT_FMT_RGB888				( 2 <<  OUT_FMT_BIT )
#define OUT_FMT_YUV422				( 3 <<  OUT_FMT_BIT )

#define YUV_PKG_OUT_OFT_BIT			( 16 )
#define YUV_PKG_OUT_OFT_MASK		( 7 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y1UY0V		( 0 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y1VY0U		( 1 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_UY1VY0		( 2 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_VY1UY0		( 3 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y0UY1V		( 4 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y0VY1U		( 5 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_UY0VY1		( 6 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_VY0UY1		( 7 << YUV_PKG_OUT_OFT_BIT )

#define IN_OFT_BIT					( 2 )
#define IN_OFT_MASK					( 3 << IN_OFT_BIT )
#define IN_OFT_Y1UY0V				( 0 << IN_OFT_BIT )
#define IN_OFT_Y1VY0U				( 1 << IN_OFT_BIT )
#define IN_OFT_UY1VY0				( 2 << IN_OFT_BIT )
#define IN_OFT_VY1UY0				( 3 << IN_OFT_BIT )

#define IN_FMT_BIT					( 0 )
#define IN_FMT_MASK					( 3 << IN_FMT_BIT )
#define IN_FMT_YUV420				( 0 << IN_FMT_BIT )
#define IN_FMT_YUV422				( 1 << IN_FMT_BIT )
#define IN_FMT_YUV444				( 2 << IN_FMT_BIT )
#define IN_FMT_YUV411				( 3 << IN_FMT_BIT )
#define IN_FMT_PKG_RGB565			( 3 << IN_FMT_BIT )

struct Ration2m
{
	unsigned int ratio;
	int n, m;
};


// parameter
// R = 1.164 * (Y - 16) + 1.596 * (cr - 128)    {C0, C1}
// G = 1.164 * (Y - 16) - 0.392 * (cb -128) - 0.813 * (cr - 128)  {C0, C2, C3}
// B = 1.164 * (Y - 16) + 2.017 * (cb - 128)    {C0, C4}
#define USE_CRCB_MODE 		1
#if USE_CRCB_MODE
#define YUV_CSC_C0					0x4A8        /* 1.164 * 1024 */
#define YUV_CSC_C1					0x662        /* 1.596 * 1024 */
#define YUV_CSC_C2					0x191        /* 0.392 * 1024 */
#define YUV_CSC_C3					0x341        /* 0.813 * 1024 */
#define YUV_CSC_C4					0x811        /* 2.017 * 1024 */
#define YUV_CSC_CHROM				128
#define YUV_CSC_LUMA				16
#else
#define YUV_CSC_C0					0x400
#define YUV_CSC_C1					0x59C
#define YUV_CSC_C2					0x161
#define YUV_CSC_C3					0x2DC
#define YUV_CSC_C4					0x718
#define YUV_CSC_CHROM				128
#define YUV_CSC_LUMA				0
#endif

struct YuvCsc
{									// YUV(default)	or	YCbCr
	unsigned int csc0;				//	0x400			0x4A8
	unsigned int csc1;              //	0x59C   		0x662
	unsigned int csc2;              //	0x161   		0x191
	unsigned int csc3;              //	0x2DC   		0x341
	unsigned int csc4;              //	0x718   		0x811
	unsigned int chrom;             //	128				128
	unsigned int luma;              //	0				16
};

struct YuvStride
{
	unsigned int y;
	unsigned int u;
	unsigned int v;
	unsigned int out;
};

typedef struct 
//struct img_param_t
{
	unsigned int		ipu_ctrl;				// IPU Control Register
	unsigned int		ipu_d_fmt;				// IPU Data Format Register
	unsigned int		in_width;
	unsigned int		in_height;
	unsigned int		in_bpp;
	// unsigned int		out_x;
	// unsigned int		out_y;
//	unsigned int		in_fmt;
//	unsigned int		out_fmt;
	unsigned int		out_width;
	unsigned int		out_height;
	unsigned char*		y_buf;
	unsigned char*		u_buf;
	unsigned char*		v_buf;
	// unsigned char*		out_buf;
	unsigned char*		y_t_addr;				// table address
	unsigned char*		u_t_addr;
	unsigned char*		v_t_addr;
	// unsigned char*		out_t_addr;
	struct Ration2m*	ratio_table;
	struct YuvCsc*		csc;
	struct YuvStride*	stride;
} img_param_t;

typedef volatile struct _IPU_CTRL2
{
	unsigned int chip_en		:1;
	unsigned int ipu_run		:1;
	unsigned int hrsz_en		:1;
	unsigned int vrsz_en		:1;
	unsigned int csc_en			:1;
	unsigned int fm_irq_en		:1;
	unsigned int ipu_rst		:1;
	/***wyang***/
	unsigned int ipu_stop		:1;
	unsigned int rsv0			:2;
	
	//unsigned int h_scale		:1;
	//unsigned int v_scale		:1;
	unsigned int spkg_sel		:1;
	unsigned int lcdc_sel		:1;
	unsigned int spage_map		:1;
	unsigned int dpage_map		:1;
	unsigned int disp_sel		:1;
	unsigned int field_conf_en	:1;
	unsigned int field_sel		:1;
	unsigned int dfix_sel		:1;
	unsigned int zoom_sel		:1;
	unsigned int burst_sel		:1;
	unsigned int add_sel		:1;
	unsigned int conf_mode		:1;
	unsigned int bus_opt		:1;
	unsigned int rsv1			:9;
}IPU_CTRL2, *PIPU_CTRL2;

typedef volatile struct _IPU_D_FMT2
{
	unsigned int in_fmt			:2;
	unsigned int in_oft			:2;
	unsigned int in_rgb_fmt		:2;
	unsigned int rsv0			:10;
	unsigned int pkg_out_oft	:3;
	unsigned int out_fmt		:3;
	unsigned int rgb_out_oft	:3;
	unsigned int out_ftm_24b	:1;
	unsigned int rsv1			:6;
}IPU_DFMT2, *PIPU_DFMT2;

// Function prototype
int ipu_open(void);
int ipu_close(void);
int ipu_init(img_param_t *pimg);
int ipu_deinit(void);
int ipu_poweroff(void);
int ipu_poweron(void);
int ipu_ioctl(void *buff, unsigned int cmd);

#define IOCTL_IPU_SET_BUFF					0x0010
#define IOCTL_IPU_CHANGE_BUFF				0x0011
#define IOCTL_IPU_START						0x0012
#define IOCTL_IPU_STOP						0x0013
#define IOCTL_IPU_FB_SIZE					0x0014
#define IOCTL_IPU_SET_CTRL_REG				0x0015
#define IOCTL_IPU_SET_FMT_REG				0x0016
#define IOCTL_IPU_SET_CSC					0x0017
#define IOCTL_IPU_SET_STRIDE				0x0018
#define IOCTL_IPU_SET_OUTSIZE				0x0019
#define IOCTL_IPU_PENDING_OUTEND			0x0020

#define IPU_LUT_LEN 32

typedef struct {
	int width_up, height_up, width_resize_enable, height_resize_enable;
	int width_lut_size, height_lut_size;
	int outW, outH, Wsel, Hsel; 
	unsigned int width_lut_coef [IPU_LUT_LEN];
	unsigned int height_lut_coef [IPU_LUT_LEN];
	unsigned int width_bicube_lut_coef	[IPU_LUT_LEN * 2];
	unsigned int height_bicube_lut_coef [IPU_LUT_LEN * 2];
}IPU_rsize_para;


typedef struct {
	int rsize_algorithm;  
	float rsize_bicube_level;
	IPU_rsize_para resize_para;
}JZ47_IPU_MOD;

#endif

