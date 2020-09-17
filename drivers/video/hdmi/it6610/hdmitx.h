///*****************************************
//  Copyright (C) 2009-2014
//  ITE Tech. Inc. All Rights Reserved
//  Proprietary and Confidential
///*****************************************
//   @file   >hdmitx.h<
//   @author Jau-Chih.Tseng@ite.com.tw
//   @date   2009/08/24
//   @fileversion: CAT6611_SAMPLEINTERFACE_1.12
//******************************************/

#ifndef _HDMITX_H_
#define _HDMITX_H_

#ifdef CONFIG_SOC_JZ4770
//#define SUPPORT_DEGEN 1
//#define SUPPORT_HDCP 1
#else
#define SUPPORT_DEGEN 1
#define SUPPORT_HDCP 1
#endif
//#define SUPPORT_SYNCEMB 1
#define SUPPORT_EDID 1 
#define SUPPORT_INPUTRGB 1
#define SUPPORT_INPUTYUV444 1 
#define SUPPORT_INPUTYUV422  1

#if defined(SUPPORT_INPUTYUV444) || defined(SUPPORT_INPUTYUV422)
#define SUPPORT_INPUTYUV
#endif

#include "typedef.h"
#include "chip_sys.h"
#include "chip_drv.h"
#ifdef SUPPORT_EDID
typedef struct {
	BYTE b16bit:1 ;
	BYTE b20bit:1 ;
	BYTE b24bit:1 ;
	BYTE Rsrv:5 ;
} LPCM_BitWidth ;

typedef union {
	struct {
		BYTE channel:3 ;
		BYTE AudioFormatCode:4 ;
		BYTE Rsrv1:1 ;

		BYTE b32KHz:1 ;
		BYTE b44_1KHz:1 ;
		BYTE b48KHz:1 ;
		BYTE b88_2KHz:1 ;
		BYTE b96KHz:1 ;
		BYTE b176_4KHz:1 ;
		BYTE b192KHz:1 ;
		BYTE Rsrv2:1 ;
		BYTE ucCode ;
	} s ;
	BYTE uc[3] ;

} AUDDESCRIPTOR ;

typedef union {
	struct {
		BYTE FL_FR:1 ;
		BYTE LFE:1 ;
		BYTE FC:1 ;
		BYTE RL_RR:1 ;
		BYTE RC:1 ;
		BYTE FLC_FRC:1 ;
		BYTE RLC_RRC:1 ;
		BYTE Reserve:1 ;
		BYTE Unuse[2] ;
	} s ;
	BYTE uc[3] ;
} SPK_ALLOC ;
typedef struct {
	BYTE ValidCEA ;
	BYTE ValidHDMI ;
	BYTE VideoMode ;
	BYTE VDOModeCount ;
	BYTE idxNativeVDOMode ;
	BYTE VDOMode[32] ;
	BYTE AUDDesCount ;
	AUDDESCRIPTOR AUDDes[10] ;
	ULONG IEEEOUI ;
	SPK_ALLOC   SpeakerAllocBlk ;
} RX_CAP ;
#endif

#endif // _HDMITX_H_

