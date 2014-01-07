/*
 * Copyright (c) 2008 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef YUV_SENSOR_OV2655_TAB_H
#define YUV_SENSOR_OV2655_TAB_H
#if defined(__cplusplus)
extern "C"
{
#endif

#define  YUV_SENSOR_STROBE (1)

struct sensor_reg {
    u8  op;
	u16 addr;
	u16 val;
};

struct sensor_info {
	int mode;
	struct i2c_client *i2c_client;
	struct yuv_sensor_platform_data *pdata;
};

static struct sensor_info *info;

static struct sensor_reg ov2655_init[] ={
//sijichun parameter
{WRITE_REG_DATA8,0x3012, 0x80},
{SENSOR_WAIT_MS, 0x0010, 0x10},
{WRITE_REG_DATA8,0x308c, 0x80},
{WRITE_REG_DATA8,0x308d, 0x0e},
{WRITE_REG_DATA8,0x360b, 0x00},
{WRITE_REG_DATA8,0x30b0, 0xff},
{WRITE_REG_DATA8,0x30b1, 0xff},
{WRITE_REG_DATA8,0x30b2, 0x24},// IO drive ability

{WRITE_REG_DATA8,0x300e, 0x34},
{WRITE_REG_DATA8,0x300f, 0xa6},
{WRITE_REG_DATA8,0x3010, 0x82},
{WRITE_REG_DATA8,0x3082, 0x01},
{WRITE_REG_DATA8,0x30f4, 0x01},
{WRITE_REG_DATA8,0x3090, 0x33},
//{WRITE_REG_DATA8,0x3090, 0x3b},// mirror and flip reg
{WRITE_REG_DATA8,0x3091, 0xc0},
{WRITE_REG_DATA8,0x30ac, 0x42},

{WRITE_REG_DATA8,0x30d1, 0x08},
{WRITE_REG_DATA8,0x30a8, 0x56},
{WRITE_REG_DATA8,0x3015, 0x34},//VAEC ceiling, 3 frames????
{WRITE_REG_DATA8,0x3093, 0x00},
{WRITE_REG_DATA8,0x307e, 0xe5},
{WRITE_REG_DATA8,0x3079, 0x00},
{WRITE_REG_DATA8,0x30aa, 0x42},
{WRITE_REG_DATA8,0x3017, 0x40},
{WRITE_REG_DATA8,0x30f3, 0x83},
{WRITE_REG_DATA8,0x306a, 0x0c},
{WRITE_REG_DATA8,0x306d, 0x00},
{WRITE_REG_DATA8,0x336a, 0x3c},
{WRITE_REG_DATA8,0x3076, 0x6a},
{WRITE_REG_DATA8,0x30d9, 0x8c},
{WRITE_REG_DATA8,0x3016, 0x82},
{WRITE_REG_DATA8,0x3601, 0x30},
{WRITE_REG_DATA8,0x304e, 0x04},
{WRITE_REG_DATA8,0x30f1, 0x82},
{WRITE_REG_DATA8,0x3011, 0x03},
//AEC/AGC
{WRITE_REG_DATA8,0x3013,0xf7},
{WRITE_REG_DATA8,0x301c, 0x12},
{WRITE_REG_DATA8,0x301d, 0x16},
{WRITE_REG_DATA8,0x3070, 0x3e},
{WRITE_REG_DATA8,0x3072, 0x34},
//D5060
{WRITE_REG_DATA8,0x30af, 0x00},
{WRITE_REG_DATA8,0x3048, 0x1f},
{WRITE_REG_DATA8,0x3049, 0x4e},
{WRITE_REG_DATA8,0x304a, 0x40},
{WRITE_REG_DATA8,0x304f, 0x40},
{WRITE_REG_DATA8,0x304b, 0x02},
{WRITE_REG_DATA8,0x304c, 0x00},
{WRITE_REG_DATA8,0x304d, 0x42},
//{WRITE_REG_DATA8,0x304f, 0x20},
{WRITE_REG_DATA8,0x30a3, 0x10},
//{WRITE_REG_DATA8,0x3013,0xf7},// fast AEC, big step, Banding filter on, auto banding disable under strong light, less than 1 line off
{WRITE_REG_DATA8,0x3014, 0x2c},//;a4 for normal;// ; manual 60Hz, band depend on 50/60 detect, night mode off, 50/60 smooth switch,
{WRITE_REG_DATA8,0x3071, 0x00},
//{WRITE_REG_DATA8,0x3070, 0x3e},
{WRITE_REG_DATA8,0x3073, 0x00},
//{WRITE_REG_DATA8,0x3072, 0x34},
//{WRITE_REG_DATA8,0x301c, 0x12},
//{WRITE_REG_DATA8,0x301d, 0x16},
//{WRITE_REG_DATA8,0x304d, 0x42},
//{WRITE_REG_DATA8,0x304a, 0x40},
//{WRITE_REG_DATA8,0x304f, 0x40},
{WRITE_REG_DATA8,0x3095, 0x07},
{WRITE_REG_DATA8,0x3096, 0x16},
{WRITE_REG_DATA8,0x3097, 0x1d},
//Window Setup
//{WRITE_REG_DATA8,0x300e, 0x38},
{WRITE_REG_DATA8,0x3020, 0x01},
{WRITE_REG_DATA8,0x3021, 0x18}, 
{WRITE_REG_DATA8,0x3022, 0x00},
{WRITE_REG_DATA8,0x3023, 0x0a},
{WRITE_REG_DATA8,0x3024, 0x06},
{WRITE_REG_DATA8,0x3025, 0x58},
{WRITE_REG_DATA8,0x3026, 0x04},
{WRITE_REG_DATA8,0x3027, 0xbc},
{WRITE_REG_DATA8,0x3088, 0x06},
{WRITE_REG_DATA8,0x3089, 0x40},
{WRITE_REG_DATA8,0x308a, 0x04},
{WRITE_REG_DATA8,0x308b, 0xb0},
{WRITE_REG_DATA8,0x3316, 0x64},
{WRITE_REG_DATA8,0x3317, 0x4b},
{WRITE_REG_DATA8,0x3318, 0x00},
{WRITE_REG_DATA8,0x331a, 0x64},
{WRITE_REG_DATA8,0x331b, 0x4b},
{WRITE_REG_DATA8,0x331c, 0x00},
{WRITE_REG_DATA8,0x3100, 0x00},

//Lens correction
{WRITE_REG_DATA8,0x3350, 0x35},
{WRITE_REG_DATA8,0x3351, 0x26},
{WRITE_REG_DATA8,0x3352, 0x00},
{WRITE_REG_DATA8,0x3353, 0x2e},
{WRITE_REG_DATA8,0x3354, 0x00},
{WRITE_REG_DATA8,0x3355, 0x85},
{WRITE_REG_DATA8,0x3356, 0x34},
{WRITE_REG_DATA8,0x3357, 0x26},
{WRITE_REG_DATA8,0x3358, 0x00},
{WRITE_REG_DATA8,0x3359, 0x2d},
{WRITE_REG_DATA8,0x335a, 0x00},
{WRITE_REG_DATA8,0x335b, 0x85},
{WRITE_REG_DATA8,0x335c, 0x35},
{WRITE_REG_DATA8,0x335d, 0x26},
{WRITE_REG_DATA8,0x335e, 0x00},
{WRITE_REG_DATA8,0x335f, 0x2a},
{WRITE_REG_DATA8,0x3360, 0x00},
{WRITE_REG_DATA8,0x3361, 0x85},
{WRITE_REG_DATA8,0x3363, 0x70},
{WRITE_REG_DATA8,0x3364, 0x7f},
{WRITE_REG_DATA8,0x3365, 0x00},
{WRITE_REG_DATA8,0x3366, 0x00},


{WRITE_REG_DATA8,0x3069,0x86},// BLC target
{WRITE_REG_DATA8,0x307c,0x10},//mirror off, flip off
//{WRITE_REG_DATA8,0x307c, 0x13},

{WRITE_REG_DATA8,0x3300, 0xfc},
{WRITE_REG_DATA8,0x3302,0x01},
{WRITE_REG_DATA8,0x3400, 0x02},
//{WRITE_REG_DATA8,0x3606, 0x20},
//{WRITE_REG_DATA8,0x3601, 0x30},
//{WRITE_REG_DATA8,0x300e, 0x34},
//{WRITE_REG_DATA8,0x30f3, 0x83},
//{WRITE_REG_DATA8,0x304e, 0x88},

//shen zhen OV param
//AWB
{WRITE_REG_DATA8, 0x3320, 0xfa},
{WRITE_REG_DATA8, 0x3321, 0x11},
{WRITE_REG_DATA8, 0x3322, 0x92},
{WRITE_REG_DATA8, 0x3323, 0x01},
{WRITE_REG_DATA8, 0x3324, 0x97},
{WRITE_REG_DATA8, 0x3325, 0x02},
{WRITE_REG_DATA8, 0x3326, 0xff},
{WRITE_REG_DATA8, 0x3327, 0x10},
{WRITE_REG_DATA8, 0x3328, 0x10}, 
{WRITE_REG_DATA8, 0x3329, 0x1f},
{WRITE_REG_DATA8, 0x332a, 0x56},
{WRITE_REG_DATA8, 0x332b, 0x54},
{WRITE_REG_DATA8, 0x332c, 0xbe},
{WRITE_REG_DATA8, 0x332d, 0xce},
{WRITE_REG_DATA8, 0x332e, 0x2e},
{WRITE_REG_DATA8, 0x332f, 0x30},
{WRITE_REG_DATA8, 0x3330, 0x4d},
{WRITE_REG_DATA8, 0x3331, 0x44},
{WRITE_REG_DATA8, 0x3332, 0xf0},
{WRITE_REG_DATA8, 0x3333, 0x0a},
{WRITE_REG_DATA8, 0x3334, 0xf0},
{WRITE_REG_DATA8, 0x3335, 0xf0},
{WRITE_REG_DATA8, 0x3336, 0xf0},
{WRITE_REG_DATA8, 0x3337, 0x40},
{WRITE_REG_DATA8, 0x3338, 0x40},
{WRITE_REG_DATA8, 0x3339, 0x40},
{WRITE_REG_DATA8, 0x333a, 0x00},
{WRITE_REG_DATA8, 0x333b, 0x00},
	
//Color Matrix
{WRITE_REG_DATA8, 0x3380, 0x28}, 
{WRITE_REG_DATA8, 0x3381, 0x48}, 
{WRITE_REG_DATA8, 0x3382, 0x12}, 
{WRITE_REG_DATA8, 0x3383, 0x15}, 
{WRITE_REG_DATA8, 0x3384, 0x9e}, 
{WRITE_REG_DATA8, 0x3385, 0xb3}, 
{WRITE_REG_DATA8, 0x3386, 0xb3}, 
{WRITE_REG_DATA8, 0x3387, 0xa7}, 
{WRITE_REG_DATA8, 0x3388, 0x0c}, 
{WRITE_REG_DATA8, 0x3389, 0x98}, 
{WRITE_REG_DATA8, 0x338a, 0x01},
{WRITE_REG_DATA8, 0x3340, 0x06},
{WRITE_REG_DATA8, 0x3341, 0x0c},
{WRITE_REG_DATA8, 0x3342, 0x1c},
{WRITE_REG_DATA8, 0x3343, 0x36},
{WRITE_REG_DATA8, 0x3344, 0x4e},
{WRITE_REG_DATA8, 0x3345, 0x5f},
{WRITE_REG_DATA8, 0x3346, 0x6d},
{WRITE_REG_DATA8, 0x3347, 0x78},
{WRITE_REG_DATA8, 0x3348, 0x84},
{WRITE_REG_DATA8, 0x3349, 0x95},
{WRITE_REG_DATA8, 0x334a, 0xa5},
{WRITE_REG_DATA8, 0x334b, 0xb4},
{WRITE_REG_DATA8, 0x334c, 0xc8},
{WRITE_REG_DATA8, 0x334d, 0xde},
{WRITE_REG_DATA8, 0x334e, 0xf0},
{WRITE_REG_DATA8, 0x334f, 0x15},

//;UV adjust
{WRITE_REG_DATA8, 0x3301, 0xff},
{WRITE_REG_DATA8, 0x338B, 0x10},
{WRITE_REG_DATA8, 0x338c, 0x0e},
{WRITE_REG_DATA8, 0x338d, 0x40},

//Sharpness/De-noise
{WRITE_REG_DATA8, 0x3370, 0xd0},
{WRITE_REG_DATA8, 0x3371, 0x00},
{WRITE_REG_DATA8, 0x3372, 0x00},
{WRITE_REG_DATA8, 0x3373, 0x50},
{WRITE_REG_DATA8, 0x3374, 0x10},
{WRITE_REG_DATA8, 0x3375, 0x10},
{WRITE_REG_DATA8, 0x3376, 0x06},
{WRITE_REG_DATA8, 0x3377, 0x01}, 
{WRITE_REG_DATA8, 0x3378, 0x10},
{WRITE_REG_DATA8, 0x3379, 0x80},


//{WRITE_REG_DATA8,0x363b,0x01},
//{WRITE_REG_DATA8,0x309e,0x08},
//{WRITE_REG_DATA8,0x3606,0x00},
{WRITE_REG_DATA8,0x3630,0x31},
//{WRITE_REG_DATA8,0x3086,0x0f},//soft sleep on
//{WRITE_REG_DATA8,0x3086,0x00},//soft sleep off
//{WRITE_REG_DATA8,0x304e,0x04},
{WRITE_REG_DATA8,0x363b,0x01},
{WRITE_REG_DATA8,0x309e,0x08},
{WRITE_REG_DATA8,0x3606,0x00},
{WRITE_REG_DATA8,0x3084,0x01},
//{WRITE_REG_DATA8,0x3010,0x82},
//{WRITE_REG_DATA8,0x3011,0x03},
{WRITE_REG_DATA8,0x3634,0x26},
{WRITE_REG_DATA8,0x3086,0x0f},
{WRITE_REG_DATA8,0x3086,0x00},
{WRITE_REG_DATA8,0x3087,0x22},
//{WRITE_REG_DATA8,0x3013,0xf2},
{WRITE_REG_DATA8,0x3002,0x01},
{WRITE_REG_DATA8,0x3003,0xce},
{SENSOR_TABLE_END, 0x0000, 0x00}
};

//origin param
static struct sensor_reg mode_640x480[] = {
//24M Hz input
//sijichun parameter
//{WRITE_REG_DATA8,0x3013,0xf2},
//{WRITE_REG_DATA8,0x3002,0x02},
//{WRITE_REG_DATA8,0x3003,0x2e},
{WRITE_REG_DATA8,0x3012, 0x10},
{WRITE_REG_DATA8,0x302a, 0x02},
{WRITE_REG_DATA8,0x302b, 0x6a},
{WRITE_REG_DATA8,0x306f, 0x14},
{WRITE_REG_DATA8,0x3020, 0x01},
{WRITE_REG_DATA8,0x3021, 0x18},
{WRITE_REG_DATA8,0x3022, 0x00},
{WRITE_REG_DATA8,0x3023, 0x06},
{WRITE_REG_DATA8,0x3024, 0x06},
{WRITE_REG_DATA8,0x3025, 0x58},
{WRITE_REG_DATA8,0x3026, 0x02},
{WRITE_REG_DATA8,0x3027, 0x61},
//{WRITE_REG_DATA8,0x3088, 0x02},
//{WRITE_REG_DATA8,0x3089, 0x80},
//{WRITE_REG_DATA8,0x308a, 0x01},
//{WRITE_REG_DATA8,0x308b, 0xe0},
{WRITE_REG_DATA8,0x3313, 0x00},
{WRITE_REG_DATA8,0x3314, 0x00},
{WRITE_REG_DATA8,0x3315, 0x00},
{WRITE_REG_DATA8,0x3316, 0x64},
{WRITE_REG_DATA8,0x3317, 0x25},
{WRITE_REG_DATA8,0x3318, 0x80},
{WRITE_REG_DATA8,0x3319, 0x08},
//{WRITE_REG_DATA8,0x331a, 0x64},
//{WRITE_REG_DATA8,0x331b, 0x4b},
//{WRITE_REG_DATA8,0x331c, 0x00},

{WRITE_REG_DATA8,0x3302, 0x11},

//shen zhen OV param
{WRITE_REG_DATA8, 0x3088, 0x02},
{WRITE_REG_DATA8, 0x3089, 0x80},
{WRITE_REG_DATA8, 0x308a, 0x01},
{WRITE_REG_DATA8, 0x308b, 0xe0},
{WRITE_REG_DATA8, 0x331a, 0x28},
{WRITE_REG_DATA8, 0x331b, 0x1e},
{WRITE_REG_DATA8, 0x331c, 0x00},
{WRITE_REG_DATA8, 0x331d, 0x38},
//origin param
//sijichun parameter
{WRITE_REG_DATA8,0x300e,0x34},
{WRITE_REG_DATA8,0x300f,0xa6},
{WRITE_REG_DATA8,0x3010,0x81},
{WRITE_REG_DATA8,0x3011,0x00},
{WRITE_REG_DATA8,0x302d,0x00},
{WRITE_REG_DATA8,0x302e,0x00},
{WRITE_REG_DATA8,0x302c,0x00},
//{WRITE_REG_DATA8,0x3013,0xf0},
{WRITE_REG_DATA8,0x3002,0x01},
{WRITE_REG_DATA8,0x3003,0xce},	   
{WRITE_REG_DATA8,0x3070,0xba},
{WRITE_REG_DATA8,0x3072,0x9a},
{WRITE_REG_DATA8,0x301c,0x02},
{WRITE_REG_DATA8,0x301d,0x03},
{WRITE_REG_DATA8,0x3015,0x04},
{WRITE_REG_DATA8,0x3014,0x20},
//{WRITE_REG_DATA8,0x3014,0x00},
{SENSOR_WAIT_MS,0x0010,0x50},
{WRITE_REG_DATA8,0x3013,0xf7},
//{WRITE_REG_DATA8,0x302d,0x00},
//{WRITE_REG_DATA8,0x302e,0x00},
{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg mode_800x600[] = {
{WRITE_REG_DATA8, 0x3012, 0x10},
{WRITE_REG_DATA8, 0x302a, 0x02},
{WRITE_REG_DATA8, 0x302b, 0x6a},
{WRITE_REG_DATA8, 0x306f, 0x14},   
{WRITE_REG_DATA8, 0x3020, 0x01},
{WRITE_REG_DATA8, 0x3021, 0x18},  
{WRITE_REG_DATA8, 0x3022, 0x00},
{WRITE_REG_DATA8, 0x3023, 0x06},
{WRITE_REG_DATA8, 0x3024, 0x06},
{WRITE_REG_DATA8, 0x3025, 0x58},
{WRITE_REG_DATA8, 0x3026, 0x02},
{WRITE_REG_DATA8, 0x3027, 0x61},
//{WRITE_REG_DATA8, 0x3088, 0x03},
//{WRITE_REG_DATA8, 0x3089, 0x20},
//{WRITE_REG_DATA8, 0x308a, 0x02},
//{WRITE_REG_DATA8, 0x308b, 0x58},
{WRITE_REG_DATA8, 0x3316, 0x64},
{WRITE_REG_DATA8, 0x3317, 0x25},
{WRITE_REG_DATA8, 0x3318, 0x80},
{WRITE_REG_DATA8, 0x3319, 0x08},
{WRITE_REG_DATA8, 0x331a, 0x64},
{WRITE_REG_DATA8, 0x331b, 0x4b},
{WRITE_REG_DATA8, 0x331c, 0x00},
{WRITE_REG_DATA8, 0x331d, 0x38},
{WRITE_REG_DATA8, 0x3302, 0x11},

{WRITE_REG_DATA8,0x3088,0x03},
{WRITE_REG_DATA8,0x3089,0x20},
{WRITE_REG_DATA8,0x308a,0x02},
{WRITE_REG_DATA8,0x308b,0x58},
{WRITE_REG_DATA8,0x3313,0x00},
{WRITE_REG_DATA8,0x3314,0x00},
{WRITE_REG_DATA8,0x3315,0x00},
{WRITE_REG_DATA8,0x331a,0x32},
{WRITE_REG_DATA8,0x331b,0x25},
{WRITE_REG_DATA8,0x331c,0x80},
{WRITE_REG_DATA8,0x300e,0x34},
{WRITE_REG_DATA8,0x300f,0xa6},
{WRITE_REG_DATA8,0x3010,0x81},
{WRITE_REG_DATA8,0x3011,0x00},
{WRITE_REG_DATA8,0x302d,0x00},
{WRITE_REG_DATA8,0x302e,0x00},

{WRITE_REG_DATA8,0x302c,0x00},
{WRITE_REG_DATA8,0x3070,0x5d},
{WRITE_REG_DATA8,0x3072,0x4d},
{WRITE_REG_DATA8,0x301c,0x05},
{WRITE_REG_DATA8,0x301d,0x06},
{WRITE_REG_DATA8,0x3013,0xf7},
{WRITE_REG_DATA8,0x3015,0x34},
{WRITE_REG_DATA8,0x3014,0x2c},
//{WRITE_REG_DATA8,0x3014,0x0c},
{SENSOR_TABLE_END, 0x0000, 0x00}
};

//shen zhen OV param
static struct sensor_reg mode_1600x1200[] = {
{WRITE_REG_DATA8,0x3012,0x00},
{WRITE_REG_DATA8,0x302a,0x04},
{WRITE_REG_DATA8,0x302b,0xd4},
{WRITE_REG_DATA8,0x306f,0x54}, 
{WRITE_REG_DATA8,0x3070,0x3e},
{WRITE_REG_DATA8,0x3072,0x34},
{WRITE_REG_DATA8,0x3020,0x01},
{WRITE_REG_DATA8,0x3021,0x18},
{WRITE_REG_DATA8,0x3022,0x00},
{WRITE_REG_DATA8,0x3023,0x0a},
{WRITE_REG_DATA8,0x3024,0x06},
{WRITE_REG_DATA8,0x3025,0x58},
{WRITE_REG_DATA8,0x3026,0x04},
{WRITE_REG_DATA8,0x3027,0xbc},
{WRITE_REG_DATA8,0x3088,0x06},
{WRITE_REG_DATA8,0x3089,0x40},
{WRITE_REG_DATA8,0x308a,0x04},
{WRITE_REG_DATA8,0x308b,0xb0},
{WRITE_REG_DATA8,0x3313,0x00},
{WRITE_REG_DATA8,0x3314,0x00},
{WRITE_REG_DATA8,0x3315,0x00},
{WRITE_REG_DATA8,0x3316,0x64},
{WRITE_REG_DATA8,0x3317,0x4b},
{WRITE_REG_DATA8,0x3318,0x00},
{WRITE_REG_DATA8,0x3319,0x2c},
{WRITE_REG_DATA8,0x331a,0x64},
{WRITE_REG_DATA8,0x331b,0x4b},
//{WRITE_REG_DATA8,0x331c,0x00},
//{WRITE_REG_DATA8,0x331d,0x4c},
{WRITE_REG_DATA8,0x3302,0x01},

//sijichun parameter
{WRITE_REG_DATA8,0x300e,0x34},
{WRITE_REG_DATA8,0x300f,0xa6},
{WRITE_REG_DATA8,0x3010,0x80},
{WRITE_REG_DATA8,0x3011,0x00},   
{WRITE_REG_DATA8,0x302d,0x00},
{WRITE_REG_DATA8,0x302e,0x00},
{WRITE_REG_DATA8,0x302c,0x00},
{WRITE_REG_DATA8,0x3070,0xbb},
{WRITE_REG_DATA8,0x3072,0x9b},
{WRITE_REG_DATA8,0x301c,0x06},
{WRITE_REG_DATA8,0x301d,0x07},
{SENSOR_TABLE_END,0x0000,0x00}   
};

static struct sensor_reg  ov2655_reg_mipi_control_tab[] = 
{//sijichun parameter
{WRITE_REG_DATA8,0x363b, 0x01},
{WRITE_REG_DATA8,0x309e, 0x08},
{WRITE_REG_DATA8,0x3606, 0x00},
{WRITE_REG_DATA8,0x3630, 0x35},
{WRITE_REG_DATA8,0x3086, 0x0f},
{WRITE_REG_DATA8,0x3086, 0x00},
{WRITE_REG_DATA8,0x304e, 0x04},  
{WRITE_REG_DATA8,0x363b, 0x01},   
{WRITE_REG_DATA8,0x309e, 0x08},   
{WRITE_REG_DATA8,0x3606, 0x00},   
{WRITE_REG_DATA8,0x3084, 0x01},  
{WRITE_REG_DATA8,0x3010, 0x81},  
{WRITE_REG_DATA8,0x3011, 0x00},   
{WRITE_REG_DATA8,0x3634, 0x26},  
{WRITE_REG_DATA8,0x3086, 0x0f},  
{WRITE_REG_DATA8,0x3086, 0x00},  
{SENSOR_TABLE_END, 0x0000, 0x00}   
};

static struct sensor_reg mode_176x144[] = {
};

static struct sensor_reg mode_1280x720[] = {

{SENSOR_TABLE_END, 0x0000, 0x00}
};


static struct sensor_reg ColorEffect_None[] = {
/* normal: */   
{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg ColorEffect_Mono[] = {
/* B&W: */
{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg ColorEffect_Sepia[] = {
/* Sepia(antique): */
{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg ColorEffect_Negative[] = {
/* Negative: */
{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg ColorEffect_Solarize[] = {		
{SENSOR_TABLE_END, 0x0000, 0x00}
};

//Sensor ISP Not Support this function

static struct sensor_reg ColorEffect_Posterize[] = {
{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg Whitebalance_Auto[] = {
/* Auto: */
{WRITE_REG_DATA8,0x3306, 0x00},
{SENSOR_WAIT_MS,0x0010,0x10},
{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg Whitebalance_Incandescent[] = {
/* Office: */
{WRITE_REG_DATA8,0x3306, 0x02},
{WRITE_REG_DATA8,0x3337, 0x52},
{WRITE_REG_DATA8,0x3338, 0x40},
{WRITE_REG_DATA8,0x3339, 0x58},
{SENSOR_WAIT_MS,0x0010,0x10},
{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg Whitebalance_Daylight[] = { 
/* Sunny: */
{WRITE_REG_DATA8,0x3306, 0x02}, 
{WRITE_REG_DATA8,0x3337, 0x5e},
{WRITE_REG_DATA8,0x3338, 0x40},
{WRITE_REG_DATA8,0x3339, 0x46},
{SENSOR_WAIT_MS,0x0010,0x10},
{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg Whitebalance_Fluorescent[] = {
/* Home: */
{WRITE_REG_DATA8,0x3306, 0x02},
{WRITE_REG_DATA8,0x3337, 0x44},
{WRITE_REG_DATA8,0x3338, 0x40},
{WRITE_REG_DATA8,0x3339, 0x70},
{SENSOR_WAIT_MS,0x0010,0x10},
{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg Whitebalance_CloudyDaylight[] = {	 
/* Cloudy: */
{WRITE_REG_DATA8,0x3306, 0x02},
{WRITE_REG_DATA8,0x3337, 0x68},
{WRITE_REG_DATA8,0x3338, 0x40},
{WRITE_REG_DATA8,0x3339, 0x4e},
{SENSOR_WAIT_MS,0x0010,0x10},
{SENSOR_TABLE_END, 0x0000, 0x00}
};


static struct sensor_reg exp_negative2[] = {
{WRITE_REG_DATA8,0x3018, 0x40},
{WRITE_REG_DATA8,0x3019, 0x30},
{WRITE_REG_DATA8,0x301a, 0xa2},	
{SENSOR_WAIT_MS,0x0010,0x10},
{SENSOR_TABLE_END, 0x0000, 0x00}
};
static struct sensor_reg exp_negative1[] = {
/* -1.0EV */
{WRITE_REG_DATA8,0x3018, 0x60}, 
{WRITE_REG_DATA8,0x3019, 0x50}, 
{WRITE_REG_DATA8,0x301a, 0xc3},
{SENSOR_WAIT_MS,0x0010,0x10},
{SENSOR_TABLE_END, 0x0000, 0x00}
};
static struct sensor_reg exp_zero[] = {
/* default */
{WRITE_REG_DATA8,0x3018, 0x80}, 
{WRITE_REG_DATA8,0x3019, 0x70},
{WRITE_REG_DATA8,0x301a, 0xd4},
{SENSOR_WAIT_MS,0x0010,0x10},
{SENSOR_TABLE_END, 0x0000, 0x00}
};
static struct sensor_reg exp_one[] = {
/* 1.0EV */
{WRITE_REG_DATA8,0x3018, 0xa0}, 
{WRITE_REG_DATA8,0x3019, 0x90}, 
{WRITE_REG_DATA8,0x301a, 0xe6}, 
{SENSOR_WAIT_MS,0x0010,0x10},
{SENSOR_TABLE_END, 0x0000, 0x00}
};
static struct sensor_reg exp_two[] = {
/* 2.0EV */
{WRITE_REG_DATA8,0x3018, 0xc8},
{WRITE_REG_DATA8,0x3019, 0xb8},
{WRITE_REG_DATA8,0x301a, 0xf7},
{SENSOR_WAIT_MS,0x0010,0x10},
{SENSOR_TABLE_END, 0x0000, 0x00}
};


static struct sensor_reg scene_auto[] = {

{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg scene_action[] = {

{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg scene_portrait[] = {

{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg scene_landscape[] = {

{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg scene_night[] = {

{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg scene_nightportrait[] = {

{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg scene_theatre[] = {

{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg scene_beach[] = {

{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg scene_snow[] = {
{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg scene_sunset[] = {
{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg scene_steadyphoto[] = {
{SENSOR_TABLE_END, 0x0000, 0x00}
};

static struct sensor_reg scene_fireworks[] = {
{SENSOR_TABLE_END, 0x0000, 0x00}
};


enum {
	YUV_Contrast_Negative_2 = -1, //-100
	YUV_Contrast_Negative_1 =-5,  //-50
	YUV_Contrast_0=0,             //0
	YUV_Contrast_1=50,            //50
	YUV_Contrast_2=10             //100
};

enum {
	YUV_Saturation_0 =0,
	YUV_Saturation_1,
	YUV_Saturation_2,
	YUV_Saturation_3,
	YUV_Saturation_4
};


enum {
	SENSOR_MODE_176x144=0,
	SENSOR_MODE_640x480,
	SENSOR_MODE_800x600,
	SENSOR_MODE_1280x720,
	SENSOR_MODE_1600x1200

};

	
static struct sensor_reg *mode_table[] = {
	[SENSOR_MODE_176x144]	= mode_176x144,
	[SENSOR_MODE_640x480]	= mode_640x480,	
	[SENSOR_MODE_800x600]	= mode_800x600,	
	[SENSOR_MODE_1280x720]	= mode_1280x720,	
	[SENSOR_MODE_1600x1200]	= mode_1600x1200
};
	
enum {
	SENSOR_AF_INIFINITY = 0 ,
	SENSOR_AF_FULLTRIGER,
};

enum {
      YUV_Exposure_Negative_2 = -2,
      YUV_Exposure_Negative_1 = -1,
      YUV_Exposure_0=0,
      YUV_Exposure_1=1,
      YUV_Exposure_2=2
};

#if defined(__cplusplus)
}
#endif

#endif  //SENSOR_YUV_H

