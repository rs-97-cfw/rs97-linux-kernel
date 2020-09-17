////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) Newport Media Inc.  All rights reserved.
//
// Module Name:  NMITYPES.H
//
// Author : K.Yu
//
// Date : 6th June. 2006
//
//////////////////////////////////////////////////////////////////////////////

#ifndef _NMI_TYPES_
#define _NMI_TYPES_

/********************************************
	Internat Data Types
********************************************/
#if 0
typedef unsigned char			uint8_t;
typedef unsigned short     		uint16_t;
typedef short               	int16_t;
typedef unsigned long       	uint32_t;
typedef long                	int32_t;

#ifndef _HAVE_FLOAT_
typedef unsigned long long 		uint64_t;
#endif
#endif

typedef enum {
	DVB = 1,
	ISDBT
} NMITV;

#endif

