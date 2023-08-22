#ifndef _STK8BA58_H_
#define _STK8BA58_H_

//文件引用
#include "stdint.h"

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


#define STK8BA58_WR_FUNC_PTR int8_t(*bus_write)\
(uint8_t, uint8_t, uint8_t *, uint8_t)

#define STK8BA58_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
bus_write(dev_addr, reg_addr, reg_data, wr_len)

#define STK8BA58_RD_FUNC_PTR int8_t(*bus_read)\
(uint8_t, uint8_t, uint8_t *, uint8_t)
#define STK8BA58_BRD_FUNC_PTR int8_t(*burst_read)\
(uint8_t, uint8_t, uint8_t *, uint32_t)

#define STK8BA58_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len)\
bus_read(dev_addr, reg_addr, reg_data, r_len)
#define STK8BA58_BURST_READ_FUNC(device_addr,\
register_addr, register_data, rd_len)\
burst_read(device_addr, register_addr, register_data, rd_len)

//宏定义区域
#define STK8BA58_I2C_ADDR   (0x18)

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


/******************************************/
/**\name         MODE SETTINGS            */
/******************************************/
#define STK8BA58_MODE_NORMAL         (0)
#define STK8BA58_MODE_LOW_POWER     (1)
#define STK8BA58_MODE_SUSPEND       (2)
#define STK8BA58_MODE_POWER_OFF     (3)

/******************************************/
/**\name         AXIS SELECTION           */
/******************************************/
#define STK8BA58_X_AXIS           (0)
/**< It refers STK8BA58 X-axis */
#define STK8BA58_Y_AXIS           (1)
/**< It refers STK8BA58 Y-axis */
#define STK8BA58_Z_AXIS           (2)
/**< It refers STK8BA58 Z-axis */

/**************************************************************/
/**\name	            ERROR CODE DEFINITIONS                */
/**************************************************************/
#define E_OUT_OF_RANGE          ((int8_t)-2)
#define E_STK8BA58_NULL_PTR       ((int8_t)-127)
#define STK8BA58_NULL             ((void *)0)
#define ERROR			((int8_t)-1)
#define	SUCCESS			((uint8_t)0)


typedef enum
{
    STK8BA58_RANGE_2G   = 0x03,
    STK8BA58_RANGE_4G   = 0x05,
    STK8BA58_RANGE_8G   = 0x08,
}stk8ba58_range_sel;

typedef enum
{
    STK8BA58_BW_7_81HZ  = 0x08,
    STK8BA58_BW_15_63HZ = 0x09,
    STK8BA58_BW_31_25HZ = 0x0A,
    STK8BA58_BW_62_50HZ = 0x0B,
    STK8BA58_BW_125HZ   = 0x0C,
    STK8BA58_BW_250HZ   = 0x0D,
    STK8BA58_BW_500HZ   = 0x0E,
    STK8BA58_BW_1000HZ  = 0x0F,
}stk8ba58_bw_sel;

typedef enum
{
    STK8BA58_SLEEP_DURN_0_5MS   = 0x05,
    STK8BA58_SLEEP_DURN_1MS     = 0x06,
    STK8BA58_SLEEP_DURN_2MS     = 0x07,
    STK8BA58_SLEEP_DURN_4MS     = 0x08,
    STK8BA58_SLEEP_DURN_6MS     = 0x09,
    STK8BA58_SLEEP_DURN_10MS    = 0x0A,
    STK8BA58_SLEEP_DURN_25MS    = 0x0B,
    STK8BA58_SLEEP_DURN_50MS    = 0x0C,
    STK8BA58_SLEEP_DURN_100MS   = 0x0D,
    STK8BA58_SLEEP_DURN_500MS   = 0x0E,
    STK8BA58_SLEEP_DURN_1S      = 0x0F,
}stk8ba58_slp_durn;

typedef enum
{
    STK8BA58_LATCH_INT_NON_LATCH0   = 0x00,
    STK8BA58_LATCH_INT_TEMP_250MS   = 0x01,
    STK8BA58_LATCH_INT_TEMP_500MS   = 0x02,
    STK8BA58_LATCH_INT_TEMP_1S      = 0x03,
    STK8BA58_LATCH_INT_TEMP_2S      = 0x04,
    STK8BA58_LATCH_INT_TEMP_4S      = 0x05,
    STK8BA58_LATCH_INT_TEMP_8S      = 0x06,
    STK8BA58_LATCH_INT_LATCH0       = 0x07,
    STK8BA58_LATCH_INT_NON_LATCH1   = 0x08,
    STK8BA58_LATCH_INT_TEMP_250US   = 0x09,
    STK8BA58_LATCH_INT_TEMP_500US   = 0x0A,
    STK8BA58_LATCH_INT_TEMP_1MS     = 0x0B,
    STK8BA58_LATCH_INT_TEMP_12_5MS  = 0x0C,
    STK8BA58_LATCH_INT_TEMP_25MS    = 0x0D,
    STK8BA58_LATCH_INT_TEMP_50MS    = 0x0E,
    STK8BA58_LATCH_INT_LATCH1       = 0x0F,
}stk8ba58_int_latch;

/**************************************************************/
/**\name             STRUCTURE DEFINITIONS                    */
/**************************************************************/
struct stk8ba58_accel_data
{
    int16_t x,/**< accel x data 12 resolution*/
    y,/**< accel y data 12 resolution*/
    z;/**< accel z data 12 resolution*/
};

struct stk8ba58_t {
	uint8_t power_mode;
	uint8_t chip_id;
	uint8_t ctrl_mode_reg;
	uint8_t low_mode_reg;
	uint8_t dev_addr;
	uint8_t fifo_config;
    STK8BA58_WR_FUNC_PTR;
	STK8BA58_RD_FUNC_PTR;
	STK8BA58_BRD_FUNC_PTR;
	// void (*delay_msec)(STK8BA58_MDELAY_DATA_TYPE);
};


/*********************************************************************/
/**\name REGISTER BIT MASK, BIT LENGTH, BIT POSITION DEFINITIONS  */
/********************************************************************/
/******************************/
/**\name CHIP ID  */
/******************************/
#define STK8BA58_CHIP_ID_POS             (0)
#define STK8BA58_CHIP_ID_MSK             (0xFF)
#define STK8BA58_CHIP_ID_LEN             (8)
#define STK8BA58_CHIP_ID_REG             STK8BA58_CHIP_ID_ADDR


/**************************************************************/
/**\name	            RETURN TYPE DEFINITION                */
/**************************************************************/
#define	STK8BA58_RETURN_FUNCTION_TYPE        int8_t

/****************************************************/
/**\name	ARRAY PARAMETERS      */
/***************************************************/

#define STK8BA58_SENSOR_DATA_ACCEL_LSB	(0)
#define STK8BA58_SENSOR_DATA_ACCEL_MSB	(1)

/**************************************************************/
/**\name             FUNCTION DECLARATION                     */
/**************************************************************/
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_init(struct stk8ba58_t *stk8ba58);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_write_reg(uint8_t addr, uint8_t *data, uint8_t len);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_read_reg(uint8_t addr, uint8_t *data, uint8_t len);

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_read_accel_x(int16_t *accel_x);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_read_accel_y(int16_t *accel_y);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_read_accel_z(int16_t *accel_z);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_read_accel_xyz(struct stk8ba58_accel_data *accel);

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_get_range(void);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_set_range(void);

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_get_bw(void);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_set_bw(void);

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_get_power_mode(void);
STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_set_power_mode();

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