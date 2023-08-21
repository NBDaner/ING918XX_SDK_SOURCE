#ifndef _STK8BA58_H_
#define _STK8BA58_H_

//文件引用

/*signed integer types*/
typedef	signed char  s8;/**< used for signed 8bit */
typedef	signed short int s16;/**< used for signed 16bit */
typedef	signed int s32;/**< used for signed 32bit */
typedef	signed long long int s64;/**< used for signed 64bit */

/*unsigned integer types*/
typedef	unsigned char u8;/**< used for unsigned 8bit */
typedef	unsigned short int u16;/**< used for unsigned 16bit */
typedef	unsigned int u32;/**< used for unsigned 32bit */
typedef	unsigned long long int u64;/**< used for unsigned 64bit */

//宏定义区域
#define STK8BA58_ADDR   (0x18)

/**************************************************************/
/**\name          REGISTER ADDRESS DEFINITIONS                */
/**************************************************************/
#define STK8BA58_CHIP_ID_ADDR                    (0x00)
/** DATA ADDRESS DEFINITIONS */
#define STK8BA58_X_OUT_LSB_ADDR                  (0x02)
#define STK8BA58_X_OUT_MSB_ADDR                  (0x03)
#define STK8BA58_Y_OUT_LSB_ADDR                  (0x04)
#define STK8BA58_Y_OUT_MSB_ADDR                  (0x05)
#define STK8BA58_Z_OUT_LSB_ADDR                  (0x06)
#define STK8BA58_Z_OUT_MSB_ADDR                  (0x07)
#define STK8BA58_INISTS1_ADDR                    (0x09) 
#define STK8BA58_INISTS2_ADDR                    (0x0A)  
#define STK8BA58_EVENTINFO1_ADDR                 (0x0B)
#define STK8BA58_RANGESEL_ADDR                   (0x0F)
#define STK8BA58_BWSEL_ADDR                      (0x10)
#define STK8BA58_POWMODE_ADDR                    (0x11)
#define STK8BA58_DATASETUP_ADDR                  (0x13)
#define STK8BA58_SWRST_ADDR                      (0x14)
#define STK8BA58_INTEN1_ADDR                     (0x16)
#define STK8BA58_INTEN2_ADDR                     (0x17)
#define STK8BA58_INTMAP1_ADDR                    (0x19)
#define STK8BA58_INTMAP2_ADDR                    (0x1A)
#define STK8BA58_INTCFG1_ADDR                    (0x20)
#define STK8BA58_INTCFG2_ADDR                    (0x21)
#define STK8BA58_SLOPEDLY_ADDR                   (0x27)
#define STK8BA58_SLOPETHD_ADDR                   (0x28)
#define STK8BA58_SIGMOT1_ADDR                    (0x29)
#define STK8BA58_SIGMOT2_ADDR                    (0x2A)
#define STK8BA58_SIGMOT3_ADDR                    (0x2B)
#define STK8BA58_INTFCFG_ADDR                    (0x34)
#define STK8BA58_OFSTCOMP_ADDR                   (0x36) 
#define STK8BA58_X_OFST_ADDR                     (0x38)
#define STK8BA58_Y_OFST_ADDR                     (0x39)
#define STK8BA58_Z_OFST_ADDR                     (0x3A)

//
typedef enum{
    STK8BA58_SLEEP_DURN_0_5MS = 0x05,
    STK8BA58_SLEEP_DURN_1MS = 0x06,
    STK8BA58_SLEEP_DURN_2MS = 0x07,
    STK8BA58_SLEEP_DURN_4MS = 0x08,
    STK8BA58_SLEEP_DURN_6MS = 0x09,
    STK8BA58_SLEEP_DURN_10MS = 0x0A,
    STK8BA58_SLEEP_DURN_25MS = 0x0B,
    STK8BA58_SLEEP_DURN_50MS = 0x0C,
    STK8BA58_SLEEP_DURN_100MS = 0x0D,
    STK8BA58_SLEEP_DURN_500MS = 0x0E,
    STK8BA58_SLEEP_DURN_1S = 0x0F,
}stk8ba58_slp_durn;


/**************************************************************/
/**\name             STRUCTURE DEFINITIONS                    */
/**************************************************************/
struct stk8ba58_accel_data
{
    s16 x,/**< accel x data 12 resolution*/
    y,/**< accel y data 12 resolution*/
    z;/**< accel z data 12 resolution*/
};

/**************************************************************/
/**\name	            RETURN TYPE DEFINITION                */
/**************************************************************/
#define	STK8BA58_RETURN_FUNCTION_TYPE        s8

/****************************************************/
/**\name	ARRAY PARAMETERS      */
/***************************************************/

#define STK8BA58_SENSOR_DATA_ACCEL_LSB	(0)
#define STK8BA58_SENSOR_DATA_ACCEL_MSB	(1)

/**************************************************************/
/**\name             FUNCTION DECLARATION                     */
/**************************************************************/
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_init(void);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_write_reg(void);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_read_reg(void);

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_get_range(void);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_set_range(void);

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_get_bw(void);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_set_bw(void);

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_get_power_mode(void);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_set_power_mode(void);

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_get_sleep_durn(void);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_set_sleep_durn(void);

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_get_offset(void);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_set_offset(void);

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_get_protect_dis(void);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_set_protect_dis(void);

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_get_whether_filtered(void);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_set_whether_filtered(void);

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_soft_rst(void);



#endif  //_STK8BA58_H_