/**
 *************************************************************************************
 * @file    audio_sbc_encoder.h
 * @brief   An implementation of SBC decoder for bluetooth.
 *
 * @author  LIU Liang
 * @version V1.0.0 
 *
 * Date       Author          Revision  Change Description                     
 * ========== =============== ========= =======================================
 * 2023-02-28 LIU Liang        v1.0.0   Create
 *
 *************************************************************************************
 */

#include "audio_sbc_common.h"



/**
 * error code
 */
typedef enum _SBC_ENC_ERROR_CODE
{
    SBC_ENC_ERRS = -6,
    SBC_ENC_ERR_INVALID_SAMPLE_RATE,      /**< invalid sample rate   */
    SBC_ENC_ERR_INVALID_CHANNLES,         /**< invalid channels      */
    SBC_ENC_ERR_INVALID_CTRL_CMD,         /**< invalid ctrl cmd      */
    SBC_ENC_ERR_INVALID_CTRL_ARG,         /**< invalid ctrl arg      */
    SBC_ENC_ERR_BITPOOL_OUT_BOUNDS,       /**< bitpool out of bounds */
    SBC_ENCODER_ERR_OK,                   /**< no error              */

}SBC_ENC_ERROR_CODE;

/**
 * @brief  SBC encoder initialzie
 * @param  sbc          SBC encoder context pointer
 * @param  sample_rate  sample rate
 * @param  num_channels number of channels
 * @return error code, @see SBC_ENC_ERR_CODE
 */
int32_t sbc_encoder_init(SbcEncoderContext* sbc, int32_t sample_rate, int32_t num_channels);

struct sbc_encoder_state *state,
						const struct sbc_frame *frame
                        
/**
 * @brief  SBC encoder parameters config
 * @param  sbc SBC encoder context pointer
 * @param  cmd @see SBC_ENCODER_CTRL_CMD
 * @param  arg the argument or result address for the cmd
 * @return error code, @see SBC_ENC_ERR_CODE
 */
int32_t sbc_encoder_ctrl(SbcEncoderContext* sbc, uint32_t cmd, uint32_t arg);

/**
 * @brief  SBC encoder encode one frame
 * @param  sbc SBC encoder context pointer
 * @param  pcm input PCM samples to be encoded,
 * @note   the number of input PCM samples must be sbc->pcm_length !!!
 * 
 * @return encoded buffer length by encoder if no error ocurs,
 *         else error code (always small than 0) will be return, @see SBC_ENC_ERR_CODE
 *         the output encoded buffer refer to sbc->stream.
 */
int32_t sbc_encoder_encode(SbcEncoderContext* sbc, const int16_t* pcm);