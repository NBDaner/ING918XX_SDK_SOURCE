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
#ifndef _AUDIO_SBC_H_
#define _AUDIO_SBC_H_

#ifdef __cplusplus
extern "C" {
#endif//__cplusplus

#include <stdbool.h>
#include "audio_sbc_common.h"

/**
 * error code list
 */
typedef enum _sbc_enc_err_code
{
    SBC_ENC_ERRS = -6,
    SBC_ENC_ERR_INVALID_SAMPLE_RATE,      /**<invalid sample rate   */
    SBC_ENC_ERR_INVALID_CHANNLES,         /**<invalid channels      */
    SBC_ENC_ERR_INVALID_CTRL_CMD,         /**<invalid ctrl cmd      */
    SBC_ENC_ERR_INVALID_CTRL_ARG,         /**<invalid ctrl arg      */
    SBC_ENC_ERR_BITPOOL_OUT_BOUNDS,       /**<bitpool out of bounds */

    SBC_ENC_ERR_NONE,                      /**<no error              */

}sbc_enc_err_code;

/**
 * SBC encoder control cmd list
 */
typedef enum _sbc_enc_ctrl_cmd
{
    SBC_ENCODER_CTRL_CMD_SET_ALLOCATION_METHOD,    /**<arg: 0:loundness, 1:SNR                       */
    SBC_ENCODER_CTRL_CMD_SET_BITPOOL,              /**<arg: 2-250                                    */
    SBC_ENCODER_CTRL_CMD_SET_BLOCK_MODE,           /**<arg: 0:4, 1:8, 2:12, 3:16                     */
    SBC_ENCODER_CTRL_CMD_SET_CHANNEL_MODE,         /**<arg: 0:MONO, 1:DUAL, 2:STEREO, 3:JOINT STEREO */
    SBC_ENCODER_CTRL_CMD_SET_SAMPLE_RATE_INDEX,    /**<arg: 0:16000, 1:32000, 2:44100, 3:48000       */
    SBC_ENCODER_CTRL_CMD_SET_SUBBAND_MODE,         /**<arg: 0:4, 1:8                                 */
    SBC_ENCODER_CTRL_CMD_SET_MSBC_ENCODE_MODE,     /**<arg: NULL                                     */

    SBC_ENCODER_CTRL_CMD_GET_ALLOCATION_METHOD,    /**<get allcation method                          */
    SBC_ENCODER_CTRL_CMD_GET_BITPOOL,              /**<get bitpool value                             */
    SBC_ENCODER_CTRL_CMD_GET_BLOCK_MODE,           /**<get block mode                                */
    SBC_ENCODER_CTRL_CMD_GET_CHANNEL_MODE,         /**<get channel mode                              */
    SBC_ENCODER_CTRL_CMD_GET_SAMPLE_RATE_INDEX,    /**<get sample rate index                         */
    SBC_ENCODER_CTRL_CMD_GET_SUBBAND_MODE,         /**<get sunband mode                              */

}sbc_enc_ctrl_cmd;


/**
 * @brief SBC encoder parament structure
 */
typedef struct _sbc_encoder
{
    bool init;
    sbc_frame_info frame;
    sbc_frame_header header;

    int8_t   num_channels;              /**<channels number    */
    uint8_t  pcm_length;                /**<PCM length         */
    uint16_t sample_rate;               /**<sample rate        */

    int32_t  sb_sample_f[2][16][8];     /**<subband sample     */

    uint8_t  reserved;
    uint8_t  frame_index;
    uint8_t  frame_id[2];
    uint8_t  stream[512];               /**<encoded buffer     */

    int32_t  position[2];
    int32_t  xfifo[2][160];
}sbc_encoder;

/**
 * @brief  SBC encoder initialzie
 * @param  sbc          SBC encoder parameter structure pointer
 * @param  sample_rate  sample rate
 * @param  num_channels number of channels
 * @return error code, @see sbc_enc_err_code
 */
int32_t sbc_enc_init(sbc_encoder* sbc, int32_t sample_rate, int32_t num_channels);
                    
/**
 * @brief  SBC encoder parameters config
 * @param  sbc SBC encoder parameter structure pointer
 * @param  cmd @see sbc_enc_ctrl_cmd
 * @param  arg the argument or result address for the cmd
 * @return error code, @see sbc_enc_err_code
 */
int32_t sbc_encoder_ctrl(sbc_encoder* sbc, uint32_t cmd, uint32_t arg);

/**
 * @brief  SBC encoder entry with one frame at a time
 * @param  sbc SBC encoder parameter structure pointer
 * @param  pcm input PCM samples to be encoded,
 * @note   the number of input PCM samples must be sbc->pcm_length !!!
 * 
 * @return encoded buffer length by encoder if no error ocurs,
 *         else error code (always small than 0) will be return, @see sbc_enc_err_code
 *         the output encoded buffer refer to sbc->stream.
 */
int32_t sbc_encode(sbc_encoder* sbc, const int16_t* pcm);

/**
 * @brief 
 * 
 * @param sbc 
 * @return int32_t 
 */
uint32_t sbc_enc_get_codesize(sbc_encoder* sbc);

/**
 * @brief 
 * 
 * @param sbc 
 * @return int32_t 
 */
uint32_t sbc_enc_get_frame_length(sbc_encoder* sbc);

#ifdef __cplusplus
}
#endif//__cplusplus

#endif