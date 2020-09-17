#ifndef _IT6610_DRV_H_
#define _IT6610_DRV_H_

typedef enum tagHDMIVideoType {
	V_HDMI_Unkown = 0 ,
	V_HDMI_640x480p60 = 1 ,
	V_HDMI_480p60,
	V_HDMI_480p60_16x9,
	V_HDMI_720p60,
	V_HDMI_1080i60,
	V_HDMI_480i60,
	V_HDMI_480i60_16x9,
	V_HDMI_1080p60 = 16,
	V_HDMI_576p50,
	V_HDMI_576p50_16x9,
	V_HDMI_720p50,
	V_HDMI_1080i50,
	V_HDMI_576i50,
	V_HDMI_576i50_16x9,
	V_HDMI_1080p50 = 31,
	V_HDMI_1080p24,
	V_HDMI_1080p25,
	V_HDMI_1080p30,
} HDMIVideoType ;

typedef enum tagHDMIOutputColorMode {
	V_HDMI_RGB444,
	V_HDMI_YUV444,
	V_HDMI_YUV422,
} HDMIOutputColorMode ;

#define IT6610_OPEN_DRV 		0x00 	//open it6610 power ,close lcd 
#define IT6610_CLOSE_DRV 		0x01 	//close it6610 power,open lcd
#define IT6610_SET_VIDEO_MODE 		0x02    //set initial video mode
#define IT6610_GET_VIDEO_MODE           0x03    //get initial video mode
#define IT6610_SET_OUTPUT_COLOR_MODE    0x04    //set output color mode
#define IT6610_GET_OUTPUT_COLOR_MODE    0x05    //get output color mode
#define IT6610_START			0x06    //it6610 start,add timer,init it6610,checkHMDI etc.
#define IT6610_STOP			0x07	//it6610 stop, del time.
#endif

