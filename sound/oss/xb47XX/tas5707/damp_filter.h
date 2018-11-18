/*======================================================
;	DAMP Filter (TAS5707)
;
;	Proprietary and Confidential Infomation
;
;	Do Not Copy or Distribute
;
;	Copyright, GGEC Audio Team
;	Author: SQ Huang
;	Created:  Jul 29, 2010
;=======================================================*/
#ifndef _DAMP_FILTER_H
#define	_DAMP_FILTER_H

void DAMP_Filter_Init(void);
void DAMP_DRC_Init(void);
void DAMP_DRC_Control(unsigned char on);
void DAMP_Dynamic_EQ(unsigned char vol);

#endif

