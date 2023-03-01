/**
 *************************************************************************************
 * @file    audio_sbc_encoder.c
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

#include <string.h>
#include "audio_sbc_encoder.h"

static const uint8_t _sbc_enc_frame_id[] =
{
    0x08, 0x38, 0xC8, 0xF8
};

static const int32_t _sbc_enc_proto_4[20] =
{
    SP4(0x02cb3e8c), SP4(0x22b63dc0), SP4(0x002329cc), SP4(0x053b7548),
    SP4(0x31eab940), SP4(0xec1f5e60), SP4(0xff3773a8), SP4(0x0061c5a7),
    SP4(0x07646680), SP4(0x3f239480), SP4(0xf89f23a8), SP4(0x007a4737),
    SP4(0x00b32807), SP4(0x083ddc80), SP4(0x4825e480), SP4(0x0191e578),
    SP4(0x00ff11ca), SP4(0x00fb7991), SP4(0x069fdc58), SP4(0x4b584000)
};

static const int32_t _sbc_enc_proto_8[40] =
{
    SP8(0x02e5cd20), SP8(0x22d0c200), SP8(0x006bfe27), SP8(0x07808930),
    SP8(0x3f1c8800), SP8(0xf8810d70), SP8(0x002cfdc6), SP8(0x055acf28),
    SP8(0x31f566c0), SP8(0xebfe57e0), SP8(0xff27c437), SP8(0x001485cc),
    SP8(0x041c6e58), SP8(0x2a7cfa80), SP8(0xe4c4a240), SP8(0xfe359e4c),
    SP8(0x0048b1f8), SP8(0x0686ce30), SP8(0x38eec5c0), SP8(0xf2a1b9f0),
    SP8(0xffe8904a), SP8(0x0095698a), SP8(0x0824a480), SP8(0x443b3c00),
    SP8(0xfd7badc8), SP8(0x00d3e2d9), SP8(0x00c183d2), SP8(0x084e1950),
    SP8(0x4810d800), SP8(0x017f43fe), SP8(0x01056dd8), SP8(0x00e9cb9f),
    SP8(0x07d7d090), SP8(0x4a708980), SP8(0x0488fae8), SP8(0x0113bd20),
    SP8(0x0107b1a8), SP8(0x069fb3c0), SP8(0x4b3db200), SP8(0x00763f48)
};

static const int32_t _sbc_enc_anamatrix_4[4] =
{
    SA4(0x2d413cc0), SA4(0x3b20d780), SA4(0x40000000), SA4(0x187de2a0)
};

static const int32_t _sbc_enc_anamatrix_8[8] =
{
    SA8(0x3b20d780), SA8(0x187de2a0), SA8(0x3ec52f80), SA8(0x3536cc40),
    SA8(0x238e7680), SA8(0x0c7c5c20), SA8(0x2d413cc0), SA8(0x40000000)
};

static inline void _sbc_enc_analyze_four(const int32_t* in, int32_t* out)
{
    int32_t t0, t1, t2, t3, t4, t5, t7;
    int32_t s0, s1, s2, s3, s4;

    t0  = _sbc_enc_proto_4[0] * (in[8]  - in[32]);
    t0 += _sbc_enc_proto_4[1] * (in[16] - in[24]);
    t0  = SCALE4_STAGE1(t0);

    t1  = _sbc_enc_proto_4[2] * in[1];
    t1 += _sbc_enc_proto_4[3] * in[9];
    t1 += _sbc_enc_proto_4[4] * in[17];
    t1 += _sbc_enc_proto_4[5] * in[25];
    t1 += _sbc_enc_proto_4[6] * in[33];
    t1  = SCALE4_STAGE1(t1);

    t2  = _sbc_enc_proto_4[7]  * in[2];
    t2 += _sbc_enc_proto_4[8]  * in[10];
    t2 += _sbc_enc_proto_4[9]  * in[18];
    t2 += _sbc_enc_proto_4[10] * in[26];
    t2 += _sbc_enc_proto_4[11] * in[34];
    t2  = SCALE4_STAGE1(t2);

    t3  = _sbc_enc_proto_4[12] * in[3];
    t3 += _sbc_enc_proto_4[13] * in[11];
    t3 += _sbc_enc_proto_4[14] * in[19];
    t3 += _sbc_enc_proto_4[15] * in[27];
    t3 += _sbc_enc_proto_4[16] * in[35];
    t3  = SCALE4_STAGE1(t3);

    t4  = _sbc_enc_proto_4[17] * (in[4] + in[36]);
    t4 += _sbc_enc_proto_4[18] * (in[12] + in[28]);
    t4 += _sbc_enc_proto_4[19] * in[20];
    t4  = SCALE4_STAGE1(t4);

    t5  = _sbc_enc_proto_4[16] * in[5];
    t5 += _sbc_enc_proto_4[15] * in[13];
    t5 += _sbc_enc_proto_4[14] * in[21];
    t5 += _sbc_enc_proto_4[13] * in[29];
    t5 += _sbc_enc_proto_4[12] * in[37];
    t5  = SCALE4_STAGE1(t5);

    /* don't compute t[6]... this term always multiplies with cos(pi/2) = 0 */

    t7  = _sbc_enc_proto_4[6] * in[7];
    t7 += _sbc_enc_proto_4[5] * in[15];
    t7 += _sbc_enc_proto_4[4] * in[23];
    t7 += _sbc_enc_proto_4[3] * in[31];
    t7 += _sbc_enc_proto_4[2] * in[39];
    t7  = SCALE4_STAGE1(t7);

    s0 = _sbc_enc_anamatrix_4[0] * (t0 + t4);
    s1 = _sbc_enc_anamatrix_4[2] * t2;
    s2 = _sbc_enc_anamatrix_4[1] * (t1 + t3) + _sbc_enc_anamatrix_4[3] * t5;
    s3 = _sbc_enc_anamatrix_4[3] * (t1 + t3) + _sbc_enc_anamatrix_4[1] * (-t5 + t7);
    s4 = _sbc_enc_anamatrix_4[3] * t7;

    *out++ = SCALE4_STAGE2( s0 + s1 + s2 - s4);
    *out++ = SCALE4_STAGE2(-s0 + s1 + s3);
    *out++ = SCALE4_STAGE2(-s0 + s1 - s3);
    *out++ = SCALE4_STAGE2( s0 + s1 - s2 + s4);
}

static inline void sbc_enc_analyze_four(SbcEncoderContext* sbc)
{
    int32_t ch, blk;

    for (ch = 0; ch < sbc->num_channels; ch++)
    {
        for (blk = 0; blk < sbc->frame.blocks; blk++)
        {
            int32_t* x   = &sbc->xfifo[ch][sbc->position[ch]];
            int32_t* pcm = sbc->sb_sample_f[ch][blk];

            /* Input 4 Audio Samples */
            x[40] = x[0] = pcm[3];
            x[41] = x[1] = pcm[2];
            x[42] = x[2] = pcm[1];
            x[43] = x[3] = pcm[0];

            _sbc_enc_analyze_four(x, pcm);

            sbc->position[ch] -= 4;

            if(sbc->position[ch] < 0)
            {
                sbc->position[ch] = 36;
            }
        }
    }
}

static inline void _sbc_enc_analyze_eight(const int32_t *in, int32_t *out)
{
    int32_t t0, t1, t2, t3, t4, t5, t6, t7;
    int32_t res;

    t0  = _sbc_enc_proto_8[0]  * (in[16] - in[64]);
    t0 += _sbc_enc_proto_8[1]  * (in[32] - in[48]);
    t0 += _sbc_enc_proto_8[36] * (in[8]  + in[72]);
    t0 += _sbc_enc_proto_8[37] * (in[24] + in[56]);
    t0 += _sbc_enc_proto_8[38] * in[40];
    t0  = SCALE8_STAGE1(t0);

    t1  = _sbc_enc_proto_8[11] * in[1];
    t1 += _sbc_enc_proto_8[12] * in[17];
    t1 += _sbc_enc_proto_8[13] * in[33];
    t1 += _sbc_enc_proto_8[14] * in[49];
    t1 += _sbc_enc_proto_8[15] * in[65];
    t1 += _sbc_enc_proto_8[31] * in[7];
    t1 += _sbc_enc_proto_8[32] * in[23];
    t1 += _sbc_enc_proto_8[33] * in[39];
    t1 += _sbc_enc_proto_8[34] * in[55];
    t1 += _sbc_enc_proto_8[35] * in[71];
    t1  = SCALE8_STAGE1(t1);

    t2  = _sbc_enc_proto_8[6 ] * in[2];
    t2 += _sbc_enc_proto_8[7 ] * in[18];
    t2 += _sbc_enc_proto_8[8 ] * in[34];
    t2 += _sbc_enc_proto_8[9 ] * in[50];
    t2 += _sbc_enc_proto_8[10] * in[66];
    t2 += _sbc_enc_proto_8[26] * in[6];
    t2 += _sbc_enc_proto_8[27] * in[22];
    t2 += _sbc_enc_proto_8[28] * in[38];
    t2 += _sbc_enc_proto_8[29] * in[54];
    t2 += _sbc_enc_proto_8[30] * in[70];
    t2  = SCALE8_STAGE1(t2);

    t3  =  _sbc_enc_proto_8[16] * in[3];
    t3 +=  _sbc_enc_proto_8[17] * in[19];
    t3 +=  _sbc_enc_proto_8[18] * in[35];
    t3 +=  _sbc_enc_proto_8[19] * in[51];
    t3 +=  _sbc_enc_proto_8[20] * in[67];
    t3 +=  _sbc_enc_proto_8[21] * in[5];
    t3 +=  _sbc_enc_proto_8[22] * in[21];
    t3 +=  _sbc_enc_proto_8[23] * in[37];
    t3 +=  _sbc_enc_proto_8[24] * in[53];
    t3 +=  _sbc_enc_proto_8[25] * in[69];
    t3  =  SCALE8_STAGE1(t3);

    t4  =  _sbc_enc_proto_8[2 ] * in[4];
    t4 +=  _sbc_enc_proto_8[3 ] * in[20];
    t4 +=  _sbc_enc_proto_8[4 ] * in[36];
    t4 +=  _sbc_enc_proto_8[5 ] * in[52];
    t4 +=  _sbc_enc_proto_8[39] * in[68];
    t4  =  SCALE8_STAGE1(t4);

    t5  =  _sbc_enc_proto_8[35] * in[9];
    t5 +=  _sbc_enc_proto_8[34] * in[25];
    t5 +=  _sbc_enc_proto_8[33] * in[41];
    t5 +=  _sbc_enc_proto_8[32] * in[57];
    t5 +=  _sbc_enc_proto_8[31] * in[73];
    t5 += -_sbc_enc_proto_8[15] * in[15];
    t5 += -_sbc_enc_proto_8[14] * in[31];
    t5 += -_sbc_enc_proto_8[13] * in[47];
    t5 += -_sbc_enc_proto_8[12] * in[63];
    t5 += -_sbc_enc_proto_8[11] * in[79];
    t5  =  SCALE8_STAGE1(t5);

    t6  =  _sbc_enc_proto_8[30] * in[10];
    t6 +=  _sbc_enc_proto_8[29] * in[26];
    t6 +=  _sbc_enc_proto_8[28] * in[42];
    t6 +=  _sbc_enc_proto_8[27] * in[58];
    t6 +=  _sbc_enc_proto_8[26] * in[74];
    t6 += -_sbc_enc_proto_8[10] * in[14];
    t6 += -_sbc_enc_proto_8[9 ] * in[30];
    t6 += -_sbc_enc_proto_8[8 ] * in[46];
    t6 += -_sbc_enc_proto_8[7 ] * in[62];
    t6 += -_sbc_enc_proto_8[6 ] * in[78];
    t6  =  SCALE8_STAGE1(t6);

    t7  =  _sbc_enc_proto_8[25] * in[11];
    t7 +=  _sbc_enc_proto_8[24] * in[27];
    t7 +=  _sbc_enc_proto_8[23] * in[43];
    t7 +=  _sbc_enc_proto_8[22] * in[59];
    t7 +=  _sbc_enc_proto_8[21] * in[75];
    t7 += -_sbc_enc_proto_8[20] * in[13];
    t7 += -_sbc_enc_proto_8[19] * in[29];
    t7 += -_sbc_enc_proto_8[18] * in[45];
    t7 += -_sbc_enc_proto_8[17] * in[61];
    t7 += -_sbc_enc_proto_8[16] * in[77];
    t7  =  SCALE8_STAGE1(t7);

    res  =  _sbc_enc_anamatrix_8[6] * t0;
    res +=  _sbc_enc_anamatrix_8[3] * t1;
    res +=  _sbc_enc_anamatrix_8[0] * t2;
    res +=  _sbc_enc_anamatrix_8[2] * t3;
    res +=  _sbc_enc_anamatrix_8[7] * t4;
    res +=  _sbc_enc_anamatrix_8[4] * t5;
    res +=  _sbc_enc_anamatrix_8[1] * t6;
    res +=  _sbc_enc_anamatrix_8[5] * t7;
    *out++ = SCALE8_STAGE2(res);

    res  = -_sbc_enc_anamatrix_8[6] * t0;
    res += -_sbc_enc_anamatrix_8[5] * t1;
    res +=  _sbc_enc_anamatrix_8[1] * t2;
    res +=  _sbc_enc_anamatrix_8[3] * t3;
    res +=  _sbc_enc_anamatrix_8[7] * t4;
    res += -_sbc_enc_anamatrix_8[2] * t5;
    res += -_sbc_enc_anamatrix_8[0] * t6;
    res += -_sbc_enc_anamatrix_8[4] * t7;
    *out++ = SCALE8_STAGE2(res);

    res  = -_sbc_enc_anamatrix_8[6] * t0;
    res += -_sbc_enc_anamatrix_8[2] * t1;
    res += -_sbc_enc_anamatrix_8[1] * t2;
    res +=  _sbc_enc_anamatrix_8[4] * t3;
    res +=  _sbc_enc_anamatrix_8[7] * t4;
    res +=  _sbc_enc_anamatrix_8[5] * t5;
    res +=  _sbc_enc_anamatrix_8[0] * t6;
    res +=  _sbc_enc_anamatrix_8[3] * t7;
    *out++ = SCALE8_STAGE2(res);

    res  =  _sbc_enc_anamatrix_8[6] * t0;
    res += -_sbc_enc_anamatrix_8[4] * t1;
    res += -_sbc_enc_anamatrix_8[0] * t2;
    res +=  _sbc_enc_anamatrix_8[5] * t3;
    res +=  _sbc_enc_anamatrix_8[7] * t4;
    res +=  _sbc_enc_anamatrix_8[3] * t5;
    res += -_sbc_enc_anamatrix_8[1] * t6;
    res += -_sbc_enc_anamatrix_8[2] * t7;
    *out++ = SCALE8_STAGE2(res);

    res  =  _sbc_enc_anamatrix_8[6] * t0;
    res +=  _sbc_enc_anamatrix_8[4] * t1;
    res += -_sbc_enc_anamatrix_8[0] * t2;
    res += -_sbc_enc_anamatrix_8[5] * t3;
    res +=  _sbc_enc_anamatrix_8[7] * t4;
    res += -_sbc_enc_anamatrix_8[3] * t5;
    res += -_sbc_enc_anamatrix_8[1] * t6;
    res +=  _sbc_enc_anamatrix_8[2] * t7;
    *out++ = SCALE8_STAGE2(res);

    res  = -_sbc_enc_anamatrix_8[6] * t0;
    res +=  _sbc_enc_anamatrix_8[2] * t1;
    res += -_sbc_enc_anamatrix_8[1] * t2;
    res += -_sbc_enc_anamatrix_8[4] * t3;
    res +=  _sbc_enc_anamatrix_8[7] * t4;
    res += -_sbc_enc_anamatrix_8[5] * t5;
    res +=  _sbc_enc_anamatrix_8[0] * t6;
    res += -_sbc_enc_anamatrix_8[3] * t7;
    *out++ = SCALE8_STAGE2(res);

    res  = -_sbc_enc_anamatrix_8[6] * t0;
    res +=  _sbc_enc_anamatrix_8[5] * t1;
    res +=  _sbc_enc_anamatrix_8[1] * t2;
    res += -_sbc_enc_anamatrix_8[3] * t3;
    res +=  _sbc_enc_anamatrix_8[7] * t4;
    res +=  _sbc_enc_anamatrix_8[2] * t5;
    res += -_sbc_enc_anamatrix_8[0] * t6;
    res +=  _sbc_enc_anamatrix_8[4] * t7;
    *out++ = SCALE8_STAGE2(res);

    res  =  _sbc_enc_anamatrix_8[6] * t0;
    res += -_sbc_enc_anamatrix_8[3] * t1;
    res +=  _sbc_enc_anamatrix_8[0] * t2;
    res += -_sbc_enc_anamatrix_8[2] * t3;
    res +=  _sbc_enc_anamatrix_8[7] * t4;
    res += -_sbc_enc_anamatrix_8[4] * t5;
    res +=  _sbc_enc_anamatrix_8[1] * t6;
    res += -_sbc_enc_anamatrix_8[5] * t7;
    *out++ = SCALE8_STAGE2(res);
}

static inline void sbc_encoder_analyze_eight(SbcEncoderContext* sbc)
{
    int32_t ch, blk;

    for (ch = 0; ch < sbc->num_channels; ch++)
    {
        for (blk = 0; blk < sbc->frame.blocks; blk++)
        {
            int32_t *x   = &sbc->xfifo[ch][sbc->position[ch]];
            int32_t *pcm = sbc->sb_sample_f[ch][blk];

            /* Input 8 Audio Samples */
            x[80] = x[0] = pcm[7];
            x[81] = x[1] = pcm[6];
            x[82] = x[2] = pcm[5];
            x[83] = x[3] = pcm[4];
            x[84] = x[4] = pcm[3];
            x[85] = x[5] = pcm[2];
            x[86] = x[6] = pcm[1];
            x[87] = x[7] = pcm[0];

            _sbc_encoder_analyze_eight(x, pcm);

            sbc->position[ch] -= 8;

            if(sbc->position[ch] < 0)
            {
                sbc->position[ch] = 72;
            }
        }
    }
}

static void sbc_encoder_subband_analyze_filter(SbcEncoderContext* sbc)
{
    switch(sbc->frame.subbands)
    {
    case 4:
        sbc_encoder_analyze_four(sbc);
        break;

    case 8:
        sbc_encoder_analyze_eight(sbc);
        break;

    default:
        break;
    }
}

int32_t sbc_encoder_init(SbcEncoderContext* sbc, int32_t sample_rate, int32_t num_channels)
{
    SbcCommonContext* frame  = &sbc->frame;
    SbcFrameHeader*   header = &sbc->header;

    memset(sbc, 0, sizeof(SbcEncoderContext));

    //Check input arguments
    switch(sample_rate)
    {
    case 16000:
        frame->sample_rate_index = 0;
        break;
    case 32000:
        frame->sample_rate_index = 1;
        break;
    case 44100:
        frame->sample_rate_index = 2;
        break;
    case 48000:
        frame->sample_rate_index = 3;
        break;
    default:
        return SBC_ENCODER_ERROR_INVALID_SAMPLE_RATE;
    }

    switch(num_channels)
    {
    case 1:
        frame->channel_mode = 0;
        break;
    case 2:
        frame->channel_mode = 2;
        break;
    default:
        return SBC_ENCODER_ERROR_INVALID_CHANNLES;
    }

    frame->blocks             = 16;
    frame->allocation_method  = SBC_ALLOCATION_METHOD_LOUDNESS;
    frame->subbands           = 8;
    frame->bitpool            = 32;

    header->syncword          = SBC_SYNCWORD;
    header->sample_rate_index = frame->sample_rate_index;
    header->block_mode        = SBC_BLOCKS_16;
    header->channel_mode      = frame->channel_mode;
    header->allocation_method = frame->allocation_method;
    header->subband_mode      = SBC_SUBBANDS_8;
    header->bitpool           = frame->bitpool;
    header->crc_check         = 0;

    sbc->num_channels = num_channels;
    sbc->sample_rate  = sample_rate;
    sbc->pcm_length   = frame->blocks * frame->subbands;
    sbc->position[0]  = sbc->position[1] = 9 * frame->subbands;
    sbc->frame_id[0]  = 0x01;
    sbc->frame_id[1]  = 0x08;
    sbc->frame_index  = 0x00;

    return SBC_ENCODER_ERROR_OK;
}
