#include "stk8ba58.h"

static struct stk8ba58_t *p_stk8ba58;

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_init(struct stk8ba58_t *stk8ba58)
{
    STK8BA58_RETURN_FUNCTION_TYPE com_rslt = ERROR; 
	uint8_t data = 0;
	uint8_t config_data = 0;
    /* assign stk8ba58 ptr */
    p_stk8ba58 = stk8ba58;
	if (p_stk8ba58 == STK8BA58_NULL)
    {
		/* Check the struct p_stk8ba58 is empty */
		com_rslt = E_STK8BA58_NULL_PTR;
	} 
    else 
    {
 		/* read Chip Id */ 
        com_rslt = p_stk8ba58->STK8BA58_BUS_READ_FUNC(  p_stk8ba58->dev_addr,
		                                                STK8BA58_CHIP_ID_REG, 
                                                        &data, 1);
        p_stk8ba58->chip_id = data;
    }

    return com_rslt;
}

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_write_reg(uint8_t addr, uint8_t *data, uint8_t len)
{
	/*  Variable used to return value of
	communication routine*/
	STK8BA58_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_stk8ba58 == STK8BA58_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return E_STK8BA58_NULL_PTR;
		} else {
		/* Write the data to the register*/
		com_rslt = p_stk8ba58->STK8BA58_BUS_WRITE_FUNC
		(p_stk8ba58->dev_addr, addr, data, len);

		if (p_stk8ba58->power_mode != STK8BA58_MODE_NORMAL) {
			/*A minimum interface idle time delay
			of atleast 450us is required as per the data sheet.*/
			p_bma2x2->delay_msec(STK8BA58_INTERFACE_IDLE_TIME_DELAY);
		}
	}
	return com_rslt;
}

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_read_reg(uint8_t addr, uint8_t *data, uint8_t len)
{
	/*  Variable used to return value of
	communication routine*/
	STK8BA58_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_stk8ba58 == STK8BA58_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return E_STK8BA58_NULL_PTR;
		} else {
			/*Read the data from the register*/
			com_rslt = p_stk8ba58->STK8BA58_BUS_READ_FUNC
			(p_stk8ba58->dev_addr, addr, data, len);
		}
	return com_rslt;
}

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_read_accel_x(int16_t *accel_x)
{
	/*  Variable used to return value of
	communication routine*/
	STK8BA58_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the accel x value
	data_u8[0] - x->LSB
	data_u8[1] - x->MSB
	*/
	uint8_t	data[STK8BA58_ACCEL_DATA_SIZE] = {0};
	if (p_stk8ba58 == STK8BA58_NULL) {
		/* Check the struct p_stk8ba58 is empty */
		return E_STK8BA58_NULL_PTR;
		} 
        else 
        {
		/* This case used for the resolution bit 12*/
			com_rslt = p_stk8ba58->STK8BA58_BUS_READ_FUNC
			(p_stk8ba58->dev_addr,
			STK8BA58_ACCEL_X12_LSB_REG, data,
			STK8BA58_LSB_MSB_READ_LENGTH);
			*accel_x = (int16_t)((((int32_t)((int8_t)
			data[STK8BA58_SENSOR_DATA_ACCEL_MSB]))
			<< STK8BA58_SHIFT_EIGHT_BITS) |
			(data[STK8BA58_SENSOR_DATA_ACCEL_LSB] &
			STK8BA58_RESOLUTION_12_MASK));
			*accel_x = *accel_x >>
			STK8BA58_SHIFT_FOUR_BITS;
		}
	}
	return com_rslt;
}

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_read_accel_y(int16_t *accel_y)
{

}

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_read_accel_z(int16_t *accel_z)
{

}

STK8BA58_RETURN_FUNCTION_TYPE stk8ba58_read_accel_xyz(struct stk8ba58_accel_data *accel)
{

}
