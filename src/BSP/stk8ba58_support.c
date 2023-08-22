#include "stk8ba58.h"
#include "iic.h"

int8_t STK8BA58_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);

int8_t STK8BA58_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);

struct stk8ba58_t stk8ba58 =
{
    	.dev_addr = STK8BA58_I2C_ADDR,
		.bus_write = STK8BA58_I2C_bus_write,
        .bus_read = STK8BA58_I2C_bus_read,
        // .delay_msec = STK8BA58_delay_msek,
};

#define	I2C_BUFFER_LEN 8

int32_t stk8ba58_power_on()
{
	/* Local variable used to assign the bandwidth value*/
	uint8_t bw_value_u8 = 0;
	/* Local variable used to set the bandwidth value*/
	uint8_t banwid = 0;
    /* status of communication*/
	int32_t com_rslt = ERROR;

	com_rslt = stk8ba58_init(&stk8ba58);

	com_rslt += stk8ba58_set_power_mode(STK8BA58_MODE_NORMAL);

	bw_value_u8 = 0x08;/* set bandwidth of 7.81Hz*/
	com_rslt += stk8ba58_set_bw(bw_value);

	/* This API used to read back the written value of bandwidth*/
	com_rslt += stk8ba58_get_bw(&banwid);

    return com_rslt;
}

s32 stk8ba58_power_down(void)
{
    return stk8ba58_set_power_mode(STK8BA58_MODE_POWER_OFF);
}

int8_t STK8BA58_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int32_t iError = 0;
    uint8_t array[I2C_BUFFER_LEN];
	uint8_t stringpos = 0;
	array[0] = reg_addr;
	for (stringpos = 0; stringpos < cnt; stringpos++) {
		array[stringpos + 1] = reg_data[stringpos];
	}

	i2c_write(I2C_PORT_0, dev_addr, array, cnt + 1);
	return (s8)iError;
}


int8_t STK8BA58_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int32_t iError = 0;

    //reg_data[0] = I2C_READ_BYTE(dev_addr, reg_addr);
	i2c_read(I2C_PORT_0, dev_addr, &reg_addr, 1, reg_data, cnt);
	return (s8)iError;
}