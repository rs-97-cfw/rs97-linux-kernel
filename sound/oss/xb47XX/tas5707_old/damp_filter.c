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
#include	"damp.h"
#include	"damp_filter.h"

#define	SUPPORT_DYNAMIC_EQ
//#define	DYNAMIC_EQ_Q_1_8
//#define	DYNAMIC_EQ_5_LEVEL

#define	CH1_BQ_ADDR(i)				( REG_CH1_BQ0+i)  // ch1:0x29+i
#define	CH2_BQ_ADDR(i)				( REG_CH2_BQ0+i)  // ch2:0x30+i

unsigned int  default_BQ_table[][5] =
{
// M6 V01 eq  
#if 0
	0x007F2C58, 0x0F01A750, 0x007F2C58, 0x00FE5752, 0x0F81A5F2,     //BQ0   70 hz highpass 
	//0x0080E91E, 0x0F01DB35, 0x007D426B, 0x00FE24CB, 0x0F81D476,   //BQ1   110hz 6db 
	0x0080BE85, 0x0F01F0F8, 0x007D57E1, 0x00FE0F08, 0x0F81E999,     //BQ1  115Hz 5db
	0x007EA931, 0x0F15BDD0, 0x0076A181, 0x00EA4230, 0x0F8AB54E,     //BQ2 2300hz -2.5db 
	// 0x007B7842, 0x0F57D9E7, 0x00587A28, 0x00A82619, 0x0FAC0D95,   //BQ3  5000hz -2db q=2
	0x0070A9C0, 0x0F7040D4, 0x003C383D, 0x008FBF2C, 0x0FD31E02,     //BQ3   4500hz -4db Q=1 
	0x0085B198, 0x000DDDE6, 0x004E539C, 0x0FF2221A, 0x0FABFACC,     //BQ4
	0x008AA994, 0x0F8FF9CB, 0x00282145, 0x00603613, 0x0FDD0548,     //BQ5
	//0x007F0E38, 0x0F01E390, 0x007F0E38, 0x00FE1AA7, 0x0F81E1C8,   //BQ6 80hz highpass 
	0x007F9C79, 0x0F05C630, 0x007E4826, 0x00FA39D0, 0x0F821B61,     //BQ6 1300 -4db bandwith 80 
#endif

#if 1  //from harman 2012.08.01,  M6 EQ
  	0x007F4A7F, 0x0F016B02, 0x007F4A7F, 0x00FE93FD, 0x0F816A01 ,  //BQ0
	0x00806958, 0x0F0206BE, 0x007D97EF, 0x00FDF942, 0x0F81FEB8 ,  //BQ1
	0x007FD3DD, 0x0F01D2E0, 0x007E7EF6, 0x00FE2D20, 0x0F81AD2C ,  //BQ2
	0x007B7842, 0x0F57D9E7, 0x00587A28, 0x00A82619, 0x0FAC0D95 ,  //BQ3
	0x0085B198, 0x000DDDE6, 0x004E539C, 0x0FF2221A, 0x0FABFACC ,  //BQ4
	0x008AA994, 0x0F8FF9CB, 0x00282145, 0x00603613, 0x0FDD0548 ,  //BQ5
 	// 0x007F68AD, 0x0F012EA6, 0x007F68AD, 0x00FED0A7, 0x0F812DF3 ,	//BQ6
	0x007F5240, 0x0F0EFFA1, 0x007C07EA, 0x00F1005F, 0x0F84A5D6 , //BQ6 2200hz -3db bw=200
#endif 
 
#if 0
// bypass	
	0x00800000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,		//BQ0
	0x00800000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,		//BQ1
	0x00800000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,		//BQ2
	0x00800000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,		//BQ3
	0x00800000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,		//BQ4
	0x00800000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,		//BQ5
	0x00800000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,		//BQ6
#endif
};

// for volume related Biquad filter
// BQ1, 130Hz, Q=1.25
unsigned int dynamic_EQ_table[][5] =
{
   //correct the loudness setting base on relative gain in 08.02
	0x00828AB0, 0x0F0206BE, 0x007B7697, 0x00FDF942, 0x0F81FEB8,      //BQ1  8db  1-12 steps  
  	0x00818212, 0x0F0206BE, 0x007C7F35, 0x00FDF942, 0x0F81FEB8,       //BQ1  5db  13-21
	0x0080C6BD, 0x0F0206BE, 0x007D3A8A, 0x00FDF942, 0x0F81FEB8,       //BQ1  2db  22-27
	0x00806958, 0x0F0206BE, 0x007D97EF, 0x00FDF942, 0x0F81FEB8,      //BQ1  0db  28-30

};

// for volume related Biquad filter
// BQ6, 7KHz, 
unsigned int high_shelf_EQ_table[][5] =
{
//correct the loudness setting base on relative gain in 08.02  
	0x012E4DEB, 0x0E8FBF6A, 0x00873B10, 0x005CC3C7, 0x0FDDF3D2,    //BQ5  9db  1-12steps
	0x00E32FD8, 0x0F15FAED, 0x00531B9B, 0x00536E44, 0x0FE04B5B,    //BQ5  6db  13-21
	0x00B042B9, 0x0F63455F, 0x003784AA, 0x00550A80, 0x0FDFE8BC,    //BQ5  3db  22-27
	0x008AA994, 0x0F8FF9CB, 0x00282145, 0x00603613, 0x0FDD0548,    //BQ5  0db  28-3
	
};

void DAMP_BiquadWrite(unsigned char start_addr, unsigned int * coeff)
{
	//unsigned char i;
	//just cast to unsigned char due to big-endian
	Damp_Write_Coefficent(start_addr, (unsigned char*)coeff, 20);			// 20-byte write
}

void DAMP_BiquadInit(void)
{
	//init ch1 biquad
	DAMP_BiquadWrite(CH1_BQ_ADDR(0), default_BQ_table[0]);
	DAMP_BiquadWrite(CH1_BQ_ADDR(1), default_BQ_table[1]);
	DAMP_BiquadWrite(CH1_BQ_ADDR(2), default_BQ_table[2]);
	DAMP_BiquadWrite(CH1_BQ_ADDR(3), default_BQ_table[3]);
	DAMP_BiquadWrite(CH1_BQ_ADDR(4), default_BQ_table[4]);
	DAMP_BiquadWrite(CH1_BQ_ADDR(5), default_BQ_table[5]);
	DAMP_BiquadWrite(CH1_BQ_ADDR(6), default_BQ_table[6]);

	//init ch2 biquad
	DAMP_BiquadWrite(CH2_BQ_ADDR(0), default_BQ_table[0]);
	DAMP_BiquadWrite(CH2_BQ_ADDR(1), default_BQ_table[1]);
	DAMP_BiquadWrite(CH2_BQ_ADDR(2), default_BQ_table[2]);
	DAMP_BiquadWrite(CH2_BQ_ADDR(3), default_BQ_table[3]);
	DAMP_BiquadWrite(CH2_BQ_ADDR(4), default_BQ_table[4]);
	DAMP_BiquadWrite(CH2_BQ_ADDR(5), default_BQ_table[5]);
	DAMP_BiquadWrite(CH2_BQ_ADDR(6), default_BQ_table[6]);
}

void DAMP_Filter_Init(void)
{
	DAMP_BiquadInit();
}

/*
*  Anti-clipping
*/
#if 1 	
unsigned int  DRC_ae_aa_ad_table[][2] =
{
        // ae,		1-ae
//	0x00009443, 0x007F6BB9,			//0x3A, default  of GDE, energy time
//	0x0001BCC9, 0x007E4337,			//0x3A, 1/3 of the default time
//	0x0000883F, 0x007F77C0,			//5ms
//	0x0002A39C, 0x007D5C64,			// 1ms

//	0x00800000, 0x00000000,			// 0ms
//	0x0002A399, 0x007D5C65,      //0x3A,1.01ms   
//      0x0002A39A, 0x007D5C65,      //0x3A,1ms   

//	0x0002A399, 0x007D5C65,     //             
	// aa		1-aa
//	0x00004A37, 0x007FB5C5,			//0x3B, default of GDE, attack time
//	0x0000DEA5, 0x007F215B,			//0x3B, 1/3 of the default attack time
//	0x0000883F, 0x007F77C0,			//5ms
//	0x0000E2C1, 0x007F1D3F,			// 3ms
//	0x00015393, 0x007EAC6D,			// 2ms

//	0x0002A39C, 0x007D5C64,			// 1ms
//	0x0000883F, 0x007F77C0,      //0x3B, 250ms£¬
//      0x0001538F, 0x007EAC70,      //0x3B,2ms 
   
//      0x0001538F, 0x007EAC70,      //  ..2012.07.09
	// ad		1-ad
//	0x00004A37, 0x007FB5C5			//0x3C, default of GDE, decay time 
//	0x000006D0, 0x007FF930			//0x3C, 100ms	
//	0x00000091, 0x007FFF6E,			 //0x3C, 1200ms
//      0x000000D8, 0x007FFF24      //0x3C, 2024ms 
//	0x00000091, 0x007FFF6E			 //0x3C, 
//	0x00000091, 0x007FFF6E    //   2012.07.09 from XI
  
//2012.08.02 from harman Xi

#if 0
	0x0002A399, 0x007D5C65,       //0x3A
	0x0001538F, 0x007EAC70,       //0x3B 
	0x00000091, 0x007FFF6E        //0x3C
#else
	0x007FFF03, 0x000000FB,       //0x3A  guo_guang value
	0x007FFF03, 0x000000FB,	      //0x3B
	0x007FFF03, 0x000000FB,       //0x3C
#endif
};

unsigned int  DRC_TKO_table[] =
{
//	0xFD57AB4C,						//0x40, T, -8db, 2.x w
//      0xFDC1F889,						
//	0xFD823098,						//0x40, T, -6db, 3.1w
//	0xFD97733D,						//0x40, T, -5db, 3.7w
//	0xFDACB5E3,						//0x40,  T,  -4db,
//      0xFDE1DC81,             //0x40 ,T ,-1.5dB
//      0xFDEC7DD4,             //0x40 ,T,-1db 
//      0xFD4268A7,            //0x40, T, -2db   
//	0x0F844444,						
//      0x0F866668,             //0x41 ,K ,compression rate 5.999    
//	0x00084210 ,						//0x42, O	
//      0x00084210,              //0x42,O  ,offset 8.999          
		  
		
//	0xFD97733D,         //0x40,T,-5db        
//	0x0F8CCCCD,         //0x41,K, compress rate 10
//      0x00084210          //0x42,O,offset 0		
		
//	0xFD97733D,
//	0x0F866668,
//	0x00084210
	
#if 0	
	0xFD778F45,        // 0x40  , T ,-6.5db
        0x0F866668,        // 0x41  , K ,compress rate 19.9999
        0x00084210         // 0x42  , O, offset 0	

#else
	0xFC8310D4,       //0x40 , guo_guang value 
	0x0FC00000,	  //0x41	
	0x00084210,       //0x42
#endif
};

#endif
void DAMP_DRC_Init(void)
{
	Damp_Write_Coefficent(0x3A, (unsigned char*)DRC_ae_aa_ad_table[0], 8);		// 8-byte write  
	Damp_Write_Coefficent(0x3B, (unsigned char*)DRC_ae_aa_ad_table[1], 8);		// 8-byte write
	Damp_Write_Coefficent(0x3C, (unsigned char*)DRC_ae_aa_ad_table[2], 8);		// 8-byte write

	Damp_Write_Coefficent(0x40, (unsigned char*)&DRC_TKO_table[0], 4);		// 4-byte write
	Damp_Write_Coefficent(0x41, (unsigned char*)&DRC_TKO_table[1], 4);		// 4-byte write
	Damp_Write_Coefficent(0x42, (unsigned char*)&DRC_TKO_table[2], 4);		// 4-byte write
}

unsigned int  DRC_on_off_table[] =
{
	0x00000000,					// DRC off
	0x00000001					// DRC on
};

void DAMP_DRC_Control(unsigned char on)     //DRC on and off control
{
	if( on )
	{
	  Damp_Write_Coefficent(REG_DRC_CTRL, (unsigned char*)&DRC_on_off_table[1], 4);	// 4-byte write
	}
	else
	{
	  Damp_Write_Coefficent(REG_DRC_CTRL, (unsigned char*)&DRC_on_off_table[0], 4);	// 4-byte write
	}
}

unsigned char dynamic_eq_level_table[] =
{
  12,
  21,
  27,
  30,
};

#define	MAX_DEQ_LEVEL			sizeof(dynamic_eq_level_table)/1  //=4 here

unsigned char get_dynamic_eq_level(unsigned char vol)
{
	unsigned char i;
	for(i = 0; i < MAX_DEQ_LEVEL; i++ )
	{
		if( vol <= dynamic_eq_level_table[i] )
		{
			//__asm__ volatile("nop");           // tjin delete it
			break;
		}
	}
	if(i>3) 
                i=3;
	return i;
}

void DAMP_Dynamic_EQ(unsigned char vol)
{
#ifdef SUPPORT_DYNAMIC_EQ  
	unsigned char level;

	level = get_dynamic_eq_level(vol);

	//ch1 biquad based on vol level
	DAMP_BiquadWrite(CH1_BQ_ADDR(1), dynamic_EQ_table[level]);
	//ch2 biquad based on vol level
	DAMP_BiquadWrite(CH2_BQ_ADDR(1), dynamic_EQ_table[level]);
// high shelf eq
	//ch1 biquad based on vol level
	DAMP_BiquadWrite(CH1_BQ_ADDR(5), high_shelf_EQ_table[level]);
	//ch2 biquad based on vol level
	DAMP_BiquadWrite(CH2_BQ_ADDR(5), high_shelf_EQ_table[level]);
#endif
}

