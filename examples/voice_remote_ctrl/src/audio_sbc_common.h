/**
 *************************************************************************************
 * @file    audio_sbc_common.h
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
#ifndef _AUDIO_SBC_COMMON_H
#define _AUDIO_SBC_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif//__cplusplus

#include <stdint.h>

#define SBC_SYNCWORD                    0x9C        /* SBC synchronize word             */
#define MSBC_SYNCWORD                   0xAD        /* MSBC synchronize word            */
#define SBC_SAMPLE_RATE_16000           0x0         /* SBC sampling frequency : 16.0KHz */
#define SBC_SAMPLE_RATE_32000           0x1         /* SBC sampling frequency : 32.0KHz */
#define SBC_SAMPLE_RATE_44100           0x2         /* SBC sampling frequency : 44.1KHz */
#define SBC_SAMPLE_RATE_48000           0x3         /* SBC sampling frequency : 48.0KHz */
#define SBC_BLOCKS_4                    0x0         /* SBC blocks number 4              */
#define SBC_BLOCKS_8                    0x1         /* SBC blocks number 8              */
#define SBC_BLOCKS_12                   0x2         /* SBC blocks number 12             */
#define SBC_BLOCKS_16                   0x3         /* SBC blocks number 16             */
#define SBC_CHANNEL_MODE_MONO           0x0         /* SBC channel mode : MONO          */
#define SBC_CHANNEL_MODE_DUAL_CHANNEL   0x1         /* SBC channel mode : Dual Channels */
#define SBC_CHANNEL_MODE_STEREO         0x2         /* SBC channel mode : Stereo        */
#define SBC_CHANNEL_MODE_JOINT_STEREO   0x3         /* SBC channel mode : Joint Stereo  */
#define SBC_ALLOCATION_METHOD_LOUDNESS  0x0         /* SBC allocation method : Loudness */
#define SBC_ALLOCATION_METHOD_SNR       0x1         /* SBC allocation method : SNR      */
#define SBC_SUBBANDS_4                  0x0         /* SBC subbands number 4            */
#define SBC_SUBBANDS_8                  0x1         /* SBC subbands number 8            */

#define SCALE_PROTO4_TBL                (15)        /* coefficient            */
#define SCALE_ANA4_TBL                  (17)        /* anamatrix coefficient            */
#define SCALE_PROTO8_TBL                (16)        /* coefficient            */
#define SCALE_ANA8_TBL                  (17)        /* anamatrix coefficient            */
#define SCALE_SPROTO4_TBL               (12)        /* coefficient            */
#define SCALE_SPROTO8_TBL               (14)        /* coefficient            */
#define SCALE_NPROTO4_TBL               (11)        /* coefficient            */
#define SCALE_NPROTO8_TBL               (11)        /* coefficient            */
#define SCALE4_STAGE1_BITS              (15)        /* coefficient            */
#define SCALE4_STAGE2_BITS              (15)        /* coefficient            */
#define SCALE8_STAGE1_BITS              (15)        /* coefficient            */
#define SCALE8_STAGE2_BITS              (15)        /* coefficient            */
#define SCALE4_STAGED1_BITS             (15)        /* coefficient            */
#define SCALE4_STAGED2_BITS             (16)        /* coefficient            */
#define SCALE8_STAGED1_BITS             (15)        /* coefficient            */
#define SCALE8_STAGED2_BITS             (16)        /* coefficient            */

#define ASR(val, bits)                  ((int32_t)(val)) >> (bits)
#define SP4(val)                        ASR(val, SCALE_PROTO4_TBL)
#define SA4(val)                        ASR(val, SCALE_ANA4_TBL)
#define SP8(val)                        ASR(val, SCALE_PROTO8_TBL)
#define SA8(val)                        ASR(val, SCALE_ANA8_TBL)
#define SS4(val)                        ASR(val, SCALE_SPROTO4_TBL)
#define SS8(val)                        ASR(val, SCALE_SPROTO8_TBL)
#define SN4(val)                        ASR(val, SCALE_NPROTO4_TBL)
#define SN8(val)                        ASR(val, SCALE_NPROTO8_TBL)
#define SCALE4_STAGE1(src)              ASR(src, SCALE4_STAGE1_BITS)
#define SCALE4_STAGE2(src)              ASR(src, SCALE4_STAGE2_BITS)
#define SCALE8_STAGE1(src)              ASR(src, SCALE8_STAGE1_BITS)
#define SCALE8_STAGE2(src)              ASR(src, SCALE8_STAGE2_BITS)
#define SCALE4_STAGED1(src)             ASR(src, SCALE4_STAGED1_BITS)
#define SCALE4_STAGED2(src)             ASR(src, SCALE4_STAGED2_BITS)
#define SCALE8_STAGED1(src)             ASR(src, SCALE8_STAGED1_BITS)
#define SCALE8_STAGED2(src)             ASR(src, SCALE8_STAGED2_BITS)

#ifndef inline
#define inline __inline
#endif//inline

/**
 * @brief SBC frame header context
 */
typedef struct sbc_frame_header
{
    #if defined(__BIG_ENDIAN__)
    //big endianness
    uint32_t crc_check          :8;
    uint32_t bitpool            :8;
    uint32_t subband_mode       :1;
    uint32_t allocation_method  :1;
    uint32_t channel_mode       :2;
    uint32_t block_mode         :2;
    uint32_t sample_rate_index  :2;
    uint32_t syncword           :8;
    #else
    //little endianness
    uint32_t syncword           :8;
    uint32_t subband_mode       :1;
    uint32_t allocation_method  :1;
    uint32_t channel_mode       :2;
    uint32_t block_mode         :2;
    uint32_t sample_rate_index  :2;
    uint32_t bitpool            :8;
    uint32_t crc_check          :8;
    #endif
}sbc_frame_header_t;

#ifdef __cplusplus
}
#endif//__cplusplus

#endif