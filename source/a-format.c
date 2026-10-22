/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include "a-format.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*
 * NOT REENTRANT. Every API in this file communicates through the file-scope state
 * below: the *_CMD half stores cmd/cdf/nEncoder/is_MultiTrans and the *_Parse half
 * reads them back, and res2/res3 are shared response buffers. So call these APIs
 * from one context only, and follow a *_CMD with its matching *_Parse.
 */
static bool is_MultiTrans;
static uint8_t cmd, cmdErr, crc, nEncoder;
static uint16_t crc_data, cdf;
/*
 * Address the current command was sent to, and the ES field of its response. Valid
 * only until the next A_Format_* call. reqAddr is 0xFF for the 1-to-1 commands,
 * which are not addressed, so the responder address is not checked.
 */
static uint8_t reqAddr = 0xFFU;
static uint8_t lastES;
/*
 * ALM word of the last status readout (A_Format_Readout_Encoder_status() or
 * A_Format_ABS_Readout_Single_with_status()). Unlike lastES it is not refreshed by every
 * command, because only those two responses carry ALM.
 */
static uint16_t lastALM;
static a_format_crc_para_t crc3_para = {
    .message       = (uint8_t *)(&crc_data),
    .type          = A_FORMAT_CRC3,
    .message_len   = 2,
    .polynomial    = A_FORMAT_CRC_POLY_COMMAND_DATA,
    .inputBitSwap  = true,
    .outputBitSwap = true
};
static a_format_crc_para_t crc8_para = {
    .type          = A_FORMAT_CRC8,
    .polynomial    = A_FORMAT_CRC_POLY_ENCODER_DATA,
    .inputBitSwap  = true,
    .outputBitSwap = true
};
static a_format_res2_t res2[8];
static a_format_res3_t res3[8];

/*
 * Mark every expected output slot as "no valid response frame", for the case where
 * a transfer is aborted before the parser can run. Without it an aborted transfer
 * leaves es at 0 (kFLEXIO_A_FORMAT_ES_NoErr), which is indistinguishable from a
 * valid response.
 */
#define A_FORMAT_MARK_NO_FRAME(array, count)                       \
    do                                                             \
    {                                                              \
        for (uint8_t idx = 0; idx < (count); idx++)                \
        {                                                          \
            (array)[idx].es = kFLEXIO_A_FORMAT_ES_FrameErr;        \
        }                                                          \
    } while (0)

/*******************************************************************************
 * Codes
 ******************************************************************************/
/* Sign-extend a right-aligned two's-complement field of `bits` width to int32_t. */
static int32_t A_Format_SignExtend(uint32_t value, uint8_t bits)
{
    uint32_t signBit = (uint32_t)1U << (bits - 1U);

    return (int32_t)((value ^ signBit) - signBit);
}

static uint8_t Swap_Byte(uint8_t data)
{
    data = (data << 4) | (data >> 4);
    data = ((data << 2) & 0xCC) | ((data >> 2) & 0x33);
    data = ((data << 1) & 0xAA) | ((data >> 1) & 0x55);
    return data;
}

static uint8_t CRC_Calc(a_format_crc_para_t *crc)
{
    uint8_t remainder = 0;
    uint8_t i = 0, j = 0;
    uint8_t poly = crc->polynomial << (8 - crc->type);

    for (j = 0; j < crc->message_len; j++)
    {
        remainder ^= (crc->inputBitSwap ? Swap_Byte(crc->message[j]) : crc->message[j]);

        for (i = 0; i < 8; i++)
        {
            if (remainder & 0x80)
            {
                remainder = (remainder << 1) ^ poly;
            }
            else
            {
                remainder <<= 1;
            }
        }
    }

    return crc->outputBitSwap ? Swap_Byte(remainder) : (remainder >> (8 - crc->type));
}

/*
 * Check the responder address and the ES field of a response IF word, and latch ES
 * for A_Format_GetLastES(). The callers verify the command code, the CRC8 and any
 * echoed payload themselves.
 *
 * A wrong responder address is a frame error. A non-zero ES is returned as
 * kStatus_FLEXIO_A_FORMAT_EncoderStatErr so the caller can tell a bad link from a
 * bad encoder. The address check is skipped when reqAddr is 0xFF.
 */
static status_t A_Format_CheckIF(uint16_t IF)
{
    lastES = A_FORMAT_GET_ENC_STAT_IF(IF);

    if ((reqAddr != 0xFFU) && (A_FORMAT_GET_ENC_ADDR_IF(IF) != reqAddr))
    {
        return kStatus_FLEXIO_A_FORMAT_FrameErr;
    }
    if (lastES != (uint8_t)kFLEXIO_A_FORMAT_ES_NoErr)
    {
        return kStatus_FLEXIO_A_FORMAT_EncoderStatErr;
    }

    return kStatus_Success;
}

uint8_t A_Format_GetLastES(void)
{
    return lastES;
}

uint16_t A_Format_GetLastALM(void)
{
    return lastALM;
}

/*
 * Decode the ALM word bit by bit. ES carries only the four OR-ed groups, so it cannot
 * tell Busy from MemBusy, or OvTemp from a real over-speed fault; ALM can.
 */
void A_Format_PrintfALM(logFunc logALM, uint16_t alm)
{
    static const char *const names[11] = {
        "Batt: backup battery error, multi-turn count lost",
        "MTErr: multi-turn counter error",
        "OvFlow: multi-turn counter overflow",
        "OvSpd: over speed",
        "MemErr: EEPROM error",
        "STErr: single-turn data error",
        "PSErr: position comparison error",
        "Busy: determining the single-turn absolute value, position data reads as 0",
        "MemBusy: EEPROM access in progress",
        "OvTemp: temperature warning",
        "IncErr: incremental signal error"
    };

    if ((alm & (uint16_t)0x07FFU) == 0U)
    {
        logALM("\tALM: no alarm bit is set\r\n");
        return;
    }

    for (uint8_t i = 0; i < 11U; i++)
    {
        if ((alm & ((uint16_t)1U << i)) != 0U)
        {
            logALM("\tALM[%d] %s\r\n", i, names[i]);
        }
    }
}

void A_Format_PrintfES(logFunc logES, uint8_t es)
{
    if (es == kFLEXIO_A_FORMAT_ES_NoErr)
    {
        logES("No error in the ES field\r\n");
    }
    else
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            switch (es & (0x1 << i))
            {
            case kFLEXIO_A_FORMAT_ES_Busy_MemBusy:
                logES("Encoder or memory is busy!\r\n");
                break;
            case kFLEXIO_A_FORMAT_ES_Batt:
                logES("Battery error!\r\n");
                break;
            case kFLEXIO_A_FORMAT_ES_OvSpd_MemErr_OvTemp_OvFlow:
                logES("Over speed or memory error or temperature warning or over flow!\r\n");
                break;
            case kFLEXIO_A_FORMAT_ES_STErr_PSErr_MTErr_INCErr:
                logES("ST error or PS error or MT error or incremental signal Error\r\n");
                break;
            case kFLEXIO_A_FORMAT_ES_FrameErr:
                logES("Encoder frame error!\r\n");
                break;
            case kFLEXIO_A_FORMAT_ES_Anyone:
                logES("One or more of all errors!\r\n");
                break;
            default:
                break;
            }
        }
    }
}

status_t A_Format_Set_Encoder_Address_1to1(encoder_a_format_t *enc, uint8_t enc_addr)
{
    status_t status = kStatus_Success;
    status_t ifStatus = kStatus_Success;

    cmd = A_FORMAT_REQ_IT_SET_ENCODER_ADDR1;
    cdf = A_FORMAT_PACK_CDF((enc_addr & 0x7), A_FORMAT_REQ_IT_SET_ENCODER_ADDR1, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);

    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    reqAddr = 0xFFU;

    FLEXIO_A_Format_Config_DR_length(enc->controller, 1);

    crc8_para.message_len = 6;
    crc8_para.message     = (uint8_t const *)res2;

    for (uint8_t i = 0; i < 8; i++)
    {
        /* Cleared every round so a stale response cannot satisfy the checks below. */
        memset(res2, 0, sizeof(a_format_res2_t));

        status = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
        if (status == kStatus_Success)
        {
            status = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res2,
                                                  HALFWORD_NUM(a_format_res2_t));
        }
        if (status != kStatus_Success)
        {
            return status;
        }

        if ((A_FORMAT_GET_CMD_CODE_IF(res2[0].IF) != cmd) || (CRC_Calc(&crc8_para) != 0))
        {
            return kStatus_FLEXIO_A_FORMAT_FrameErr;
        }

        /* An ES complaint does not stop the loop: the address is assigned regardless. */
        ifStatus = A_Format_CheckIF(res2[0].IF);
    }

    return ifStatus;
}

status_t A_Format_Set_ID_1to1(encoder_a_format_t *enc, uint32_t id)
{
    status_t result;
    uint32_t temp_val;
    uint16_t mdf[3] = {
        A_FORMAT_PACK_MDF(A_FORMAT_FRAME_CODE_MDF0, id & 0x000000FF, 0),
        A_FORMAT_PACK_MDF(A_FORMAT_FRAME_CODE_MDF1, (id & 0x0000FF00) >> 8, 0),
        A_FORMAT_PACK_MDF(A_FORMAT_FRAME_CODE_MDF2, (id & 0x00FF0000) >> 16, 0)
    };

    cmd = A_FORMAT_REQ_IT_ID_CODE_WRITE2;
    cdf = A_FORMAT_PACK_CDF(0, A_FORMAT_REQ_IT_ID_CODE_WRITE2, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);
    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    reqAddr = 0xFFU;

    for (uint8_t i = 0; i < 3; i++)
    {
        crc_data = A_FORMAT_GET_CRC_DATA_MDF(mdf[i]);
        crc = CRC_Calc(&crc3_para);
        mdf[i] = A_FORMAT_SET_CRC_CODE_MDF(mdf[i], crc);
    }

    FLEXIO_A_Format_Config_DR_length(enc->controller, 4);

    crc8_para.message_len = 6;
    crc8_para.message     = (uint8_t const *)res2;

    memset(res2, 0, sizeof(a_format_res2_t));

    result = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
    for (uint8_t i = 0; (i < 3) && (result == kStatus_Success); i++)
    {
        SDK_DelayAtLeastUs(7, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
        result = FLEXIO_A_Format_WriteBlocking(enc->controller, &mdf[i], 1);
    }
    if (result == kStatus_Success)
    {
        result = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res2,
                                              HALFWORD_NUM(a_format_res2_t));
    }
    if (result != kStatus_Success)
    {
        return result;
    }

    memcpy(&temp_val, res2[0].DF, sizeof(uint32_t));
    if ((A_FORMAT_GET_CMD_CODE_IF(res2[0].IF) != cmd) || (CRC_Calc(&crc8_para) != 0) ||
        (GET_ENCODER_ID(temp_val) != id))
    {
        return kStatus_FLEXIO_A_FORMAT_FrameErr;
    }

    return A_Format_CheckIF(res2[0].IF);
}

status_t A_Format_Get_ID_1to1(encoder_a_format_t *enc, uint32_t *id)
{
    status_t result;
    uint32_t temp_val;

    cmd = A_FORMAT_REQ_IT_ID_CODE_READ2;
    cdf = A_FORMAT_PACK_CDF(0, A_FORMAT_REQ_IT_ID_CODE_READ2, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);
    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    reqAddr = 0xFFU;

    FLEXIO_A_Format_Config_DR_length(enc->controller, 1);

    crc8_para.message_len = 6;
    crc8_para.message     = (uint8_t const *)res2;

    memset(res2, 0, sizeof(a_format_res2_t));
    result = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
    if (result == kStatus_Success)
    {
        result = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res2,
                                              HALFWORD_NUM(a_format_res2_t));
    }
    if (result != kStatus_Success)
    {
        return result;
    }

    if ((A_FORMAT_GET_CMD_CODE_IF(res2[0].IF) != cmd) || (CRC_Calc(&crc8_para) != 0))
    {
        return kStatus_FLEXIO_A_FORMAT_FrameErr;
    }

    memcpy(&temp_val, res2[0].DF, sizeof(uint32_t));
    *id = GET_ENCODER_ID(temp_val);
    return A_Format_CheckIF(res2[0].IF);
}

status_t A_Format_Set_Encoder_Address_MATCH_ID(encoder_a_format_t *enc, uint32_t id, uint8_t enc_addr)
{
    status_t result;
    uint32_t temp_val;
    uint16_t mdf[3] = {
        A_FORMAT_PACK_MDF(A_FORMAT_FRAME_CODE_MDF0, id & 0x000000FF, 0),
        A_FORMAT_PACK_MDF(A_FORMAT_FRAME_CODE_MDF1, (id & 0x0000FF00) >> 8, 0),
        A_FORMAT_PACK_MDF(A_FORMAT_FRAME_CODE_MDF2, (id & 0x00FF0000) >> 16, 0)
    };

    cmd = A_FORMAT_REQ_IT_SET_ENCODER_ADDR2;
    cdf = A_FORMAT_PACK_CDF((enc_addr & 0x7), A_FORMAT_REQ_IT_SET_ENCODER_ADDR2, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);
    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    /* SetAddress: the address itself is in flux, so it cannot be checked. */
    reqAddr = 0xFFU;

    for (uint8_t i = 0; i < 3; i++)
    {
        crc_data = A_FORMAT_GET_CRC_DATA_MDF(mdf[i]);
        crc = CRC_Calc(&crc3_para);
        mdf[i] = A_FORMAT_SET_CRC_CODE_MDF(mdf[i], crc);
    }

    FLEXIO_A_Format_Config_DR_length(enc->controller, 4);

    crc8_para.message_len = 6;
    crc8_para.message     = (uint8_t const *)res2;

    memset(res2, 0, sizeof(a_format_res2_t));

    result = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
    for (uint8_t i = 0; (i < 3) && (result == kStatus_Success); i++)
    {
        SDK_DelayAtLeastUs(7, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
        result = FLEXIO_A_Format_WriteBlocking(enc->controller, &mdf[i], 1);
    }
    if (result == kStatus_Success)
    {
        result = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res2,
                                              HALFWORD_NUM(a_format_res2_t));
    }
    if (result != kStatus_Success)
    {
        return result;
    }

    memcpy(&temp_val, res2[0].DF, sizeof(uint32_t));
    if ((A_FORMAT_GET_CMD_CODE_IF(res2[0].IF) != cmd) || (CRC_Calc(&crc8_para) != 0) ||
        (GET_ENCODER_ID(temp_val) != id))
    {
        return kStatus_FLEXIO_A_FORMAT_FrameErr;
    }

    return A_Format_CheckIF(res2[0].IF);
}

status_t A_Format_Set_ID(encoder_a_format_t *enc, uint8_t enc_addr, uint32_t id)
{
    status_t result;
    uint32_t temp_val;
    uint16_t mdf[3] = {
        A_FORMAT_PACK_MDF(A_FORMAT_FRAME_CODE_MDF0, id & 0x000000FF, 0),
        A_FORMAT_PACK_MDF(A_FORMAT_FRAME_CODE_MDF1, (id & 0x0000FF00) >> 8, 0),
        A_FORMAT_PACK_MDF(A_FORMAT_FRAME_CODE_MDF2, (id & 0x00FF0000) >> 16, 0)
    };

    cmd = A_FORMAT_REQ_IT_ID_CODE_WRITE1;
    cdf = A_FORMAT_PACK_CDF((enc_addr & 0x7), A_FORMAT_REQ_IT_ID_CODE_WRITE1, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);
    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    reqAddr = enc_addr & 0x7;

    for (uint8_t i = 0; i < 3; i++)
    {
        crc_data = A_FORMAT_GET_CRC_DATA_MDF(mdf[i]);
        crc = CRC_Calc(&crc3_para);
        mdf[i] = A_FORMAT_SET_CRC_CODE_MDF(mdf[i], crc);
    }

    FLEXIO_A_Format_Config_DR_length(enc->controller, 4);

    crc8_para.message_len = 6;
    crc8_para.message     = (uint8_t const *)res2;

    memset(res2, 0, sizeof(a_format_res2_t));

    result = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
    for (uint8_t i = 0; (i < 3) && (result == kStatus_Success); i++)
    {
        SDK_DelayAtLeastUs(7, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
        result = FLEXIO_A_Format_WriteBlocking(enc->controller, &mdf[i], 1);
    }
    if (result == kStatus_Success)
    {
        result = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res2,
                                              HALFWORD_NUM(a_format_res2_t));
    }
    if (result != kStatus_Success)
    {
        return result;
    }

    memcpy(&temp_val, res2[0].DF, sizeof(uint32_t));
    if ((A_FORMAT_GET_CMD_CODE_IF(res2[0].IF) != cmd) || (CRC_Calc(&crc8_para) != 0) ||
        (GET_ENCODER_ID(temp_val) != id))
    {
        return kStatus_FLEXIO_A_FORMAT_FrameErr;
    }

    return A_Format_CheckIF(res2[0].IF);
}


status_t A_Format_Get_ID_CMD(uint8_t enc_addr)
{
    if (ENCODER_ADDRESS(enc_addr) >= A_FORMAT_ENCODER_MAX_NUM)
    {
        return kStatus_FLEXIO_A_FORMAT_OutOfIDRange;
    }

    nEncoder = 1;
    enc_addr &= 0x7;
    reqAddr = enc_addr;
    cmd = A_FORMAT_REQ_IT_ID_CODE_READ1;
    cdf = A_FORMAT_PACK_CDF(enc_addr, A_FORMAT_REQ_IT_ID_CODE_READ1, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);
    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    memset(res2, 0, sizeof(a_format_res2_t));

    return kStatus_Success;
}

status_t A_Format_Get_ID_Parse(encoder_a_format_t *enc, a_format_res2_t *res, uint32_t *id)
{
    uint32_t temp_val;

    crc8_para.message_len = 6;
    crc8_para.message     = (uint8_t const *)res;
    if ((A_FORMAT_GET_CMD_CODE_IF(res->IF) != cmd) || (CRC_Calc(&crc8_para) != 0))
    {
        return kStatus_FLEXIO_A_FORMAT_FrameErr;
    }

    memcpy(&temp_val, res->DF, sizeof(uint32_t));
    *id = GET_ENCODER_ID(temp_val);
    return A_Format_CheckIF(res->IF);
}

status_t A_Format_Get_ID(encoder_a_format_t *enc, uint8_t enc_addr, uint32_t *id)
{
    status_t result;

    if (A_Format_Get_ID_CMD(enc_addr) == kStatus_FLEXIO_A_FORMAT_OutOfIDRange)
    {
        return kStatus_FLEXIO_A_FORMAT_OutOfIDRange;
    }

    FLEXIO_A_Format_Config_DR_length(enc->controller, 1);

    result = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
    if (result == kStatus_Success)
    {
        result = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res2,
                                              HALFWORD_NUM(a_format_res2_t));
    }
    if (result != kStatus_Success)
    {
        return result;
    }

    return A_Format_Get_ID_Parse(enc, res2, id);
}

status_t A_Format_Memory_Write(encoder_a_format_t *enc, uint8_t enc_addr, a_format_eeprom_t *eeprom)
{
    status_t result;
    uint16_t mdf[3] = {
        A_FORMAT_PACK_MDF(A_FORMAT_FRAME_CODE_MDF0, eeprom->data & 0x00FF, 0),
        A_FORMAT_PACK_MDF(A_FORMAT_FRAME_CODE_MDF1, (eeprom->data & 0xFF00) >> 8, 0),
        A_FORMAT_PACK_MDF(A_FORMAT_FRAME_CODE_MDF2, eeprom->address, 0)
    };

    cmd = A_FORMAT_REQ_IT_MEMORY_WRITE;
    cdf = A_FORMAT_PACK_CDF((enc_addr & 0x7), A_FORMAT_REQ_IT_MEMORY_WRITE, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);
    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    reqAddr = enc_addr & 0x7;

    for (uint8_t i = 0; i < 3; i++)
    {
        crc_data = A_FORMAT_GET_CRC_DATA_MDF(mdf[i]);
        crc = CRC_Calc(&crc3_para);
        mdf[i] = A_FORMAT_SET_CRC_CODE_MDF(mdf[i], crc);
    }

    FLEXIO_A_Format_Config_DR_length(enc->controller, 4);

    crc8_para.message_len = 6;
    crc8_para.message     = (uint8_t const *)res2;

    memset(res2, 0, sizeof(a_format_res2_t));

    result = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
    for (uint8_t i = 0; (i < 3) && (result == kStatus_Success); i++)
    {
        SDK_DelayAtLeastUs(7, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
        result = FLEXIO_A_Format_WriteBlocking(enc->controller, &mdf[i], 1);
    }
    if (result == kStatus_Success)
    {
        result = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res2,
                                              HALFWORD_NUM(a_format_res2_t));
    }
    if (result != kStatus_Success)
    {
        return result;
    }

    if ((A_FORMAT_GET_CMD_CODE_IF(res2[0].IF) != cmd) || (CRC_Calc(&crc8_para) != 0) ||
        (res2[0].DF[0] != eeprom->data) || ((res2[0].DF[1] & 0x00FF) != eeprom->address))
    {
        return kStatus_FLEXIO_A_FORMAT_FrameErr;
    }

    /* kFLEXIO_A_FORMAT_ES_Busy_MemBusy is expected here while the encoder is still
     * programming the cell; the caller must wait up to 30 ms before another access. */
    return A_Format_CheckIF(res2[0].IF);
}

status_t A_Format_Memory_Read(encoder_a_format_t *enc, uint8_t enc_addr, a_format_eeprom_t *eeprom)
{
    status_t result;
    uint16_t mdf = A_FORMAT_PACK_MDF(A_FORMAT_FRAME_CODE_MDF2, eeprom->address, 0);
    /* F9h reads the temperature sensor rather than EEPROM and needs one request. */
    uint8_t readCount = (eeprom->address == 0xF9U) ? 1U : 2U;

    if (ENCODER_ADDRESS_IS_MT(enc_addr) || (ENCODER_ADDRESS(enc_addr) >= A_FORMAT_ENCODER_MAX_NUM))
    {
        return kStatus_InvalidArgument;
    }

    cmd = A_FORMAT_REQ_IT_MEMORY_READ;
    cdf = A_FORMAT_PACK_CDF((enc_addr & 0x7), A_FORMAT_REQ_IT_MEMORY_READ, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);
    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    reqAddr = enc_addr & 0x7;

    crc_data = A_FORMAT_GET_CRC_DATA_MDF(mdf);
    crc = CRC_Calc(&crc3_para);
    mdf = A_FORMAT_SET_CRC_CODE_MDF(mdf, crc);

    for (uint8_t readIndex = 0U; readIndex < readCount; readIndex++)
    {
        FLEXIO_A_Format_Config_DR_length(enc->controller, 2);
        crc8_para.message_len = 6;
        crc8_para.message     = (uint8_t const *)res2;

        memset(res2, 0, sizeof(a_format_res2_t));
        result = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
        if (result == kStatus_Success)
        {
            SDK_DelayAtLeastUs(7, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
            result = FLEXIO_A_Format_WriteBlocking(enc->controller, &mdf, 1);
        }
        if (result == kStatus_Success)
        {
            result = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res2,
                                                  HALFWORD_NUM(a_format_res2_t));
        }
        if (result != kStatus_Success)
        {
            return result;
        }

        if ((A_FORMAT_GET_CMD_CODE_IF(res2[0].IF) != cmd) || (CRC_Calc(&crc8_para) != 0) ||
            ((res2[0].DF[1] & 0x00FF) != eeprom->address))
        {
            return kStatus_FLEXIO_A_FORMAT_FrameErr;
        }
        result = A_Format_CheckIF(res2[0].IF);
        if ((readCount == 2U) && (readIndex == 0U))
        {
            /* The first response starts the access. Its data is undefined; only a
             * clean response or the expected Busy/MemBusy acknowledgement may proceed. */
            if ((result != kStatus_Success) &&
                !((result == kStatus_FLEXIO_A_FORMAT_EncoderStatErr) &&
                  (lastES == kFLEXIO_A_FORMAT_ES_Busy_MemBusy)))
            {
                return result;
            }
            SDK_DelayAtLeastUs(300, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
        }
        else
        {
            /* ES cannot tell Busy from MemBusy, so any non-zero ES fails the read
             * and an unfinished memory access never reaches the caller. */
            if (result != kStatus_Success)
            {
                return result;
            }
            eeprom->data = res2[0].DF[0];
        }
    }

    return kStatus_Success;
}

status_t A_Format_Readout_Encoder_status_CMD(uint8_t enc_addr)
{
    if (ENCODER_ADDRESS(enc_addr) >= A_FORMAT_ENCODER_MAX_NUM)
    {
        return kStatus_FLEXIO_A_FORMAT_OutOfIDRange;
    }

    is_MultiTrans = ENCODER_ADDRESS_IS_MT(enc_addr);
    enc_addr &= 0x7;

    cmd = is_MultiTrans ? A_FORMAT_REQ_MT_ENCODER_STAT : A_FORMAT_REQ_IT_ENCODER_STAT;
    cdf = A_FORMAT_PACK_CDF(enc_addr, cmd, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);
    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    nEncoder = is_MultiTrans ? (enc_addr + 1) : 1;
    /* Multiple Transmission answers with one frame per encoder, each carrying its
     * own address, so there is no single expected responder to check against. */
    reqAddr = is_MultiTrans ? 0xFFU : (enc_addr & 0x7);
    memset(res2, 0, sizeof(a_format_res2_t) * nEncoder);

    return kStatus_Success;
}

status_t A_Format_Readout_Encoder_status_Parse(encoder_a_format_t *enc, a_format_res2_t *res,
                                               a_format_status_t *statusData)
{
    cmdErr = 0;

    crc8_para.message_len = 6;
    for (uint8_t i = 0; i < nEncoder; i++)
    {
        crc8_para.message = (uint8_t const *)&res[i];
        if ((A_FORMAT_GET_CMD_CODE_IF(res[i].IF) != cmd) || (CRC_Calc(&crc8_para) != 0))
        {
            cmdErr++;
            statusData[i].es = kFLEXIO_A_FORMAT_ES_FrameErr;
            continue;
        }

        statusData[i].encID = A_FORMAT_GET_ENC_ADDR_IF(res[i].IF);
        statusData[i].es    = A_FORMAT_GET_ENC_STAT_IF(res[i].IF);
        if (statusData[i].es != kFLEXIO_A_FORMAT_ES_NoErr)
        {
            cmdErr++;
        }

        statusData[i].status = res[i].DF[0];
        if (!is_MultiTrans)
        {
            lastALM = statusData[i].status;
        }
    }

    return cmdErr ? kStatus_Fail : kStatus_Success;
}

status_t A_Format_Readout_Encoder_status(encoder_a_format_t *enc, uint8_t enc_addr,
                                         a_format_status_t *statusData)
{
    status_t result;

    if (A_Format_Readout_Encoder_status_CMD(enc_addr) == kStatus_FLEXIO_A_FORMAT_OutOfIDRange)
    {
        return kStatus_FLEXIO_A_FORMAT_OutOfIDRange;
    }

    FLEXIO_A_Format_Config_DR_length(enc->controller, 1);
    result = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
    if (result == kStatus_Success)
    {
        result = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res2,
                                              HALFWORD_NUM(a_format_res2_t) * nEncoder);
    }
    if (result != kStatus_Success)
    {
        A_FORMAT_MARK_NO_FRAME(statusData, nEncoder);
        return result;
    }

    return A_Format_Readout_Encoder_status_Parse(enc, res2, statusData);
}

/*
 * Clear the encoder status flags and reset the multi-turn counter
 * (A-format command CDF(10) = A_FORMAT_REQ_IT_CLEAR_STAT_MULTI).
 *
 * A multi-turn encoder that lost its backup battery powers up with
 * ES = battery-error | multi-turn-error (e.g. ES=0x0B), and every position/status
 * read then fails on ES != 0 until those flags are cleared.
 *
 * CDF(10) also resets the multi-turn counter, which destroys the machine's revolution
 * reference. Issue it only when ALM reports a latched fault that CDF(10) can clear,
 * that is when (ALM & A_FORMAT_ALM_MASK_CDF10_CLEARABLE) != 0. A non-zero ES is not a
 * sufficient condition: ES bit0 is the OR of the non-latched Busy and MemBusy bits,
 * which no clear command removes. Use CDF(8) to clear the flags without touching the
 * counter.
 *
 * Per the A-format spec the CLEAR command must be sent continuously eight times.
 * The encoder can take up to 400 us after the eighth command to finish clearing.
 *
 * @retval kStatus_Success                           No latched ALM bit is left.
 * @retval kStatus_FLEXIO_A_FORMAT_EncoderStatErr    The eight frames went out and were
 *                                                   answered, but a latched fault came
 *                                                   straight back; read
 *                                                   A_Format_GetLastALM() for which one.
 * @retval kStatus_FLEXIO_A_FORMAT_FrameErr          A clear frame or the verification
 *                                                   frame was answered with a bad
 *                                                   command code, CRC8 or address.
 * @retval kStatus_FLEXIO_A_FORMAT_Timeout           No answer at all.
 */
status_t A_Format_Clear_Stat_Multi(encoder_a_format_t *enc, uint8_t enc_addr)
{
    status_t result;
    a_format_status_t statusData = {0};
    uint8_t i;

    if (ENCODER_ADDRESS(enc_addr) >= A_FORMAT_ENCODER_MAX_NUM)
    {
        return kStatus_FLEXIO_A_FORMAT_OutOfIDRange;
    }

    enc_addr &= 0x7;

    cmd = A_FORMAT_REQ_IT_CLEAR_STAT_MULTI;
    cdf = A_FORMAT_PACK_CDF(enc_addr, A_FORMAT_REQ_IT_CLEAR_STAT_MULTI, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);
    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    nEncoder = 1;
    reqAddr = enc_addr;
    memset(res2, 0, sizeof(a_format_res2_t));

    FLEXIO_A_Format_Config_DR_length(enc->controller, 1);

    crc8_para.message_len = 6;
    for (i = 0; i < 8U; i++)
    {
        memset(res2, 0, sizeof(a_format_res2_t));
        result = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
        if (result == kStatus_Success)
        {
            result = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res2,
                                                  HALFWORD_NUM(a_format_res2_t));
        }
        if (result != kStatus_Success)
        {
            return result;
        }

        crc8_para.message = (uint8_t const *)res2;
        if ((A_FORMAT_GET_CMD_CODE_IF(res2[0].IF) != cmd) || (CRC_Calc(&crc8_para) != 0) ||
            (A_Format_CheckIF(res2[0].IF) == kStatus_FLEXIO_A_FORMAT_FrameErr))
        {
            return kStatus_FLEXIO_A_FORMAT_FrameErr;
        }
    }

    SDK_DelayAtLeastUs(400, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);

    /*
     * Verify that the latched flags are actually clear before reporting success.
     * The check runs on ALM, not on ES: Busy, MemBusy and OvTemp are not latched, so a
     * clear cannot remove them and ES stays non-zero as long as the condition lasts.
     *
     * A_Format_Readout_Encoder_status() returns kStatus_Fail whenever ES != 0, so its
     * return value only tells whether the verification frame arrived at all.
     */
    result = A_Format_Readout_Encoder_status(enc, ENCODER_ADDRESS_IT(enc_addr), &statusData);
    lastES = statusData.es;
    if (statusData.es == kFLEXIO_A_FORMAT_ES_FrameErr)
    {
        return (result != kStatus_Success) ? result : kStatus_FLEXIO_A_FORMAT_FrameErr;
    }

    lastALM = statusData.status;
    if ((statusData.status & A_FORMAT_ALM_MASK_LATCHED) != 0U)
    {
        /* A latched fault came straight back after the clear, so the underlying
         * fault is still active. */
        return kStatus_FLEXIO_A_FORMAT_EncoderStatErr;
    }

    return kStatus_Success;
}

status_t A_Format_Get_Temperature_CMD(uint8_t enc_addr)
{
    if (ENCODER_ADDRESS(enc_addr) >= A_FORMAT_ENCODER_MAX_NUM)
    {
        return kStatus_FLEXIO_A_FORMAT_OutOfIDRange;
    }

    nEncoder = 1;
    enc_addr &= 0x7;
    reqAddr = enc_addr;
    cmd = A_FORMAT_REQ_IT_TEMPERATURE_10BIT;
    cdf = A_FORMAT_PACK_CDF(enc_addr, A_FORMAT_REQ_IT_TEMPERATURE_10BIT, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);
    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    memset(res2, 0, sizeof(a_format_res2_t));

    return kStatus_Success;
}

status_t A_Format_Get_Temperature_Parse(encoder_a_format_t *enc, a_format_res2_t *res, float *temp)
{
    crc8_para.message_len = 6;
    crc8_para.message     = (uint8_t const *)res;
    if ((A_FORMAT_GET_CMD_CODE_IF(res->IF) != cmd) || (CRC_Calc(&crc8_para) != 0))
    {
        return kStatus_FLEXIO_A_FORMAT_FrameErr;
    }

    *temp = GET_TEMPERATURE_VALUE(res->DF[0]);
    /* ES may carry the over-temperature warning itself, so temp is still returned. */
    return A_Format_CheckIF(res->IF);
}

status_t A_Format_Get_Temperature(encoder_a_format_t *enc, uint8_t enc_addr, float *temp)
{
    status_t result;

    if (A_Format_Get_Temperature_CMD(enc_addr) == kStatus_FLEXIO_A_FORMAT_OutOfIDRange)
    {
        return kStatus_FLEXIO_A_FORMAT_OutOfIDRange;
    }
    FLEXIO_A_Format_Config_DR_length(enc->controller, 1);

    result = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
    if (result != kStatus_Success)
    {
        return result;
    }

    result = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res2, HALFWORD_NUM(a_format_res2_t));
    if (result != kStatus_Success)
    {
        return result;
    }

    return A_Format_Get_Temperature_Parse(enc, res2, temp);
}

status_t A_Format_ABS_Readout_Multi_CMD(uint8_t enc_addr)
{
    if (ENCODER_ADDRESS(enc_addr) >= A_FORMAT_ENCODER_MAX_NUM)
    {
        return kStatus_FLEXIO_A_FORMAT_OutOfIDRange;
    }

    is_MultiTrans = ENCODER_ADDRESS_IS_MT(enc_addr);
    enc_addr &= 0x7;

    cmd = is_MultiTrans ? A_FORMAT_REQ_MT_ABS_UPPER_24BIT : A_FORMAT_REQ_IT_ABS_UPPER_24BIT;
    cdf = A_FORMAT_PACK_CDF(enc_addr, cmd, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);
    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    nEncoder = is_MultiTrans ? (enc_addr + 1) : 1;
    reqAddr = is_MultiTrans ? 0xFFU : (enc_addr & 0x7);
    memset(res2, 0, sizeof(a_format_res2_t) * nEncoder);

    return kStatus_Success;
}

status_t A_Format_ABS_Readout_Multi_Parse(encoder_a_format_t *enc, a_format_res2_t *res,
                                          a_format_abs_multi_t *multiData)
{
    uint32_t temp_val;
    cmdErr = 0;

    crc8_para.message_len = 6;
    for (uint8_t i = 0; i < nEncoder; i++)
    {
        crc8_para.message = (uint8_t const *)&res[i];
        if ((A_FORMAT_GET_CMD_CODE_IF(res[i].IF) != cmd) || (CRC_Calc(&crc8_para) != 0))
        {
            cmdErr++;
            multiData[i].es = kFLEXIO_A_FORMAT_ES_FrameErr;
            continue;
        }

        multiData[i].encID = A_FORMAT_GET_ENC_ADDR_IF(res[i].IF);
        multiData[i].es    = A_FORMAT_GET_ENC_STAT_IF(res[i].IF);
        if (multiData[i].es != kFLEXIO_A_FORMAT_ES_NoErr)
        {
            cmdErr++;
        }

        memcpy(&temp_val, res[i].DF, sizeof(uint32_t));
        multiData[i].multiTurn = A_Format_SignExtend(
            (temp_val >> (enc->singleTurnRevolution - 16)) & enc->multi_turn_sign_mask,
            enc->multiTurnRevolution);
    }

    return cmdErr ? kStatus_Fail : kStatus_Success;
}

status_t A_Format_ABS_Readout_Multi(encoder_a_format_t *enc, uint8_t enc_addr, a_format_abs_multi_t *multiData)
{
    status_t result;

    if (A_Format_ABS_Readout_Multi_CMD(enc_addr) == kStatus_FLEXIO_A_FORMAT_OutOfIDRange)
    {
        return kStatus_FLEXIO_A_FORMAT_OutOfIDRange;
    }

    FLEXIO_A_Format_Config_DR_length(enc->controller, 1);
    result = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
    if (result == kStatus_Success)
    {
        result = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res2,
                                              HALFWORD_NUM(a_format_res2_t) * nEncoder);
    }
    if (result != kStatus_Success)
    {
        A_FORMAT_MARK_NO_FRAME(multiData, nEncoder);
        return result;
    }

    return A_Format_ABS_Readout_Multi_Parse(enc, res2, multiData);
}

status_t A_Format_ABS_Readout_Single_CMD(uint8_t enc_addr)
{
    if (ENCODER_ADDRESS(enc_addr) >= A_FORMAT_ENCODER_MAX_NUM)
    {
        return kStatus_FLEXIO_A_FORMAT_OutOfIDRange;
    }

    is_MultiTrans = ENCODER_ADDRESS_IS_MT(enc_addr);
    enc_addr &= 0x7;

    cmd = is_MultiTrans ? A_FORMAT_REQ_MT_ABS_LOWER_24BIT : A_FORMAT_REQ_IT_ABS_LOWER_24BIT;
    cdf = A_FORMAT_PACK_CDF(enc_addr, cmd, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);
    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    nEncoder = is_MultiTrans ? (enc_addr + 1) : 1;
    reqAddr = is_MultiTrans ? 0xFFU : (enc_addr & 0x7);
    memset(res2, 0, sizeof(a_format_res2_t) * nEncoder);

    return kStatus_Success;
}

status_t A_Format_ABS_Readout_Single_Parse(encoder_a_format_t *enc, a_format_res2_t *res,
                                           a_format_abs_single_t *single_data)
{
    uint32_t temp_val;
    cmdErr = 0;

    crc8_para.message_len = 6;
    for (uint8_t i = 0; i < nEncoder; i++)
    {
        crc8_para.message = (uint8_t const *)&res[i];
        if ((A_FORMAT_GET_CMD_CODE_IF(res[i].IF) != cmd) || (CRC_Calc(&crc8_para) != 0))
        {
            cmdErr++;
            single_data[i].es = kFLEXIO_A_FORMAT_ES_FrameErr;
            continue;
        }

        single_data[i].encID = A_FORMAT_GET_ENC_ADDR_IF(res[i].IF);
        single_data[i].es    = A_FORMAT_GET_ENC_STAT_IF(res[i].IF);
        if (single_data[i].es != kFLEXIO_A_FORMAT_ES_NoErr)
        {
            cmdErr++;
        }

        memcpy(&temp_val, res[i].DF, sizeof(uint32_t));
        single_data[i].singleTurn = temp_val & enc->single_turn_sign_mask;
    }

    return cmdErr ? kStatus_Fail : kStatus_Success;
}

status_t A_Format_ABS_Readout_Single(encoder_a_format_t *enc, uint8_t enc_addr, a_format_abs_single_t *singleData)
{
    status_t result;

    if (A_Format_ABS_Readout_Single_CMD(enc_addr) == kStatus_FLEXIO_A_FORMAT_OutOfIDRange)
    {
        return kStatus_FLEXIO_A_FORMAT_OutOfIDRange;
    }

    FLEXIO_A_Format_Config_DR_length(enc->controller, 1);
    result = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
    if (result == kStatus_Success)
    {
        result = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res2,
                                              HALFWORD_NUM(a_format_res2_t) * nEncoder);
    }
    if (result != kStatus_Success)
    {
        A_FORMAT_MARK_NO_FRAME(singleData, nEncoder);
        return result;
    }

    return A_Format_ABS_Readout_Single_Parse(enc, res2, singleData);
}

status_t A_Format_ABS_Readout_Single_with_status(encoder_a_format_t *enc, uint8_t enc_addr, a_format_single_stat_t *singleStat)
{
    status_t result;
    uint32_t temp_val;
    a_format_res3_t *res = res3;

    if (ENCODER_ADDRESS(enc_addr) >= A_FORMAT_ENCODER_MAX_NUM)
    {
        return kStatus_FLEXIO_A_FORMAT_OutOfIDRange;
    }

    cmdErr = 0;
    is_MultiTrans = ENCODER_ADDRESS_IS_MT(enc_addr);

    cmd = is_MultiTrans ? A_FORMAT_REQ_MT_ABS_LOWER_24BIT_STAT : A_FORMAT_REQ_IT_ABS_LOWER_24BIT_STAT;
    cdf = A_FORMAT_PACK_CDF((enc_addr & 0x7), cmd, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);
    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    nEncoder = is_MultiTrans ? ((enc_addr & 0x7) + 1) : 1;
    reqAddr = is_MultiTrans ? 0xFFU : (enc_addr & 0x7);
    memset(res, 0, sizeof(a_format_res3_t) * nEncoder);

    FLEXIO_A_Format_Config_DR_length(enc->controller, 1);
    result = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
    if (result == kStatus_Success)
    {
        result = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res,
                                              HALFWORD_NUM(a_format_res3_t) * nEncoder);
    }
    if (result != kStatus_Success)
    {
        A_FORMAT_MARK_NO_FRAME(singleStat, nEncoder);
        return result;
    }

    crc8_para.message_len = 8;
    for (uint8_t i = 0; i < nEncoder; i++)
    {
        crc8_para.message = (uint8_t const *)&res[i];
        if ((A_FORMAT_GET_CMD_CODE_IF(res[i].IF) != cmd) || (CRC_Calc(&crc8_para) != 0))
        {
            cmdErr++;
            singleStat[i].es = kFLEXIO_A_FORMAT_ES_FrameErr;
            continue;
        }

        singleStat[i].encID = A_FORMAT_GET_ENC_ADDR_IF(res[i].IF);
        singleStat[i].es    = A_FORMAT_GET_ENC_STAT_IF(res[i].IF);
        if (singleStat[i].es != kFLEXIO_A_FORMAT_ES_NoErr)
        {
            cmdErr++;
        }

        memcpy(&temp_val, res[i].DF, sizeof(uint32_t));
        singleStat[i].singleTurn = temp_val & enc->single_turn_sign_mask;
        singleStat[i].ALM = (res[i].DF[2] << 8) | (res[i].DF[1] >> 8);
        if (!is_MultiTrans)
        {
            lastALM = singleStat[i].ALM;
        }
    }

    return cmdErr ? kStatus_Fail : kStatus_Success;
}

status_t A_Format_ABS_Readout_Single_with_temperature(encoder_a_format_t *enc, uint8_t enc_addr, a_format_single_temp_t *singleTemp)
{
    status_t result;
    uint32_t temp_val;
    a_format_res3_t *res = res3;
    uint16_t temp;

    if (ENCODER_ADDRESS(enc_addr) >= A_FORMAT_ENCODER_MAX_NUM)
    {
        return kStatus_FLEXIO_A_FORMAT_OutOfIDRange;
    }

    cmdErr = 0;
    is_MultiTrans = ENCODER_ADDRESS_IS_MT(enc_addr) ? true : false;

    cmd = is_MultiTrans ? A_FORMAT_REQ_MT_ABS_LOWER_24BIT_TEMP : A_FORMAT_REQ_IT_ABS_LOWER_24BIT_TEMP;
    cdf = A_FORMAT_PACK_CDF((enc_addr & 0x7), cmd, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);
    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    nEncoder = is_MultiTrans ? ((enc_addr & 0x7) + 1) : 1;
    reqAddr = is_MultiTrans ? 0xFFU : (enc_addr & 0x7);
    memset(res, 0, sizeof(a_format_res3_t) * nEncoder);

    FLEXIO_A_Format_Config_DR_length(enc->controller, 1);
    result = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
    if (result == kStatus_Success)
    {
        result = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res,
                                              HALFWORD_NUM(a_format_res3_t) * nEncoder);
    }
    if (result != kStatus_Success)
    {
        A_FORMAT_MARK_NO_FRAME(singleTemp, nEncoder);
        return result;
    }

    crc8_para.message_len = 8;
    for (uint8_t i = 0; i < nEncoder; i++)
    {
        crc8_para.message = (uint8_t const *)&res[i];
        if ((A_FORMAT_GET_CMD_CODE_IF(res[i].IF) != cmd) || (CRC_Calc(&crc8_para) != 0))
        {
            cmdErr++;
            singleTemp[i].es = kFLEXIO_A_FORMAT_ES_FrameErr;
            continue;
        }

        singleTemp[i].encID = A_FORMAT_GET_ENC_ADDR_IF(res[i].IF);
        singleTemp[i].es    = A_FORMAT_GET_ENC_STAT_IF(res[i].IF);
        if (singleTemp[i].es != kFLEXIO_A_FORMAT_ES_NoErr)
        {
            cmdErr++;
        }

        memcpy(&temp_val, res[i].DF, sizeof(uint32_t));
        singleTemp[i].singleTurn = temp_val & enc->single_turn_sign_mask;
        temp = (res[i].DF[2] << 8) | (res[i].DF[1] >> 8);
        singleTemp[i].temperature = GET_TEMPERATURE_VALUE(temp);
    }

    return cmdErr ? kStatus_Fail : kStatus_Success;
}

status_t A_Format_ABS_Readout_Multi_Single_Parse(encoder_a_format_t *enc, a_format_res3_t *res,
                                                 a_format_abs_multi_single_t *abs_data)
{
    uint32_t temp_val1, temp_val2;
    cmdErr = 0;

    crc8_para.message_len = 8;
    for (uint8_t i = 0; i < nEncoder; i++)
    {
        crc8_para.message = (uint8_t const *)&res[i];
        if ((A_FORMAT_GET_CMD_CODE_IF(res[i].IF) != cmd) || (CRC_Calc(&crc8_para) != 0))
        {
            cmdErr++;
            abs_data[i].es = kFLEXIO_A_FORMAT_ES_FrameErr;
            continue;
        }

        abs_data[i].encID = A_FORMAT_GET_ENC_ADDR_IF(res[i].IF);
        abs_data[i].es    = A_FORMAT_GET_ENC_STAT_IF(res[i].IF);
        if (abs_data[i].es != kFLEXIO_A_FORMAT_ES_NoErr)
        {
            cmdErr++;
        }

        memcpy(&temp_val1, res[i].DF, sizeof(uint32_t));
        memcpy(&temp_val2, &(res[i].DF[1]), sizeof(uint32_t));
        abs_data[i].singleTurn = temp_val1 & enc->single_turn_sign_mask;
        abs_data[i].multiTurn  = A_Format_SignExtend(
            (temp_val2 >> (enc->singleTurnRevolution - 16)) & enc->multi_turn_sign_mask,
            enc->multiTurnRevolution);
    }

    return cmdErr ? kStatus_Fail : kStatus_Success;
}

status_t A_Format_ABS_Readout_Multi_Single_CMD(uint8_t enc_addr)
{
    if (ENCODER_ADDRESS(enc_addr) >= A_FORMAT_ENCODER_MAX_NUM)
    {
        return kStatus_FLEXIO_A_FORMAT_OutOfIDRange;
    }

    is_MultiTrans = ENCODER_ADDRESS_IS_MT(enc_addr);
    enc_addr &= 0x7;

    cmd = is_MultiTrans ? A_FORMAT_REQ_MT_ABS_FULL_40BIT : A_FORMAT_REQ_IT_ABS_FULL_40BIT;
    cdf = A_FORMAT_PACK_CDF(enc_addr, cmd, 0);
    crc_data = A_FORMAT_GET_CRC_DATA_CDF(cdf);
    crc = CRC_Calc(&crc3_para);
    cdf = (uint16_t)A_FORMAT_SET_CRC_CODE_CDF(cdf, crc);

    nEncoder = is_MultiTrans ? (enc_addr + 1) : 1;
    reqAddr = is_MultiTrans ? 0xFFU : (enc_addr & 0x7);
    memset(res3, 0, sizeof(a_format_res3_t) * nEncoder);

    return kStatus_Success;
}

status_t A_Format_ABS_Readout_Multi_Single(encoder_a_format_t *enc, uint8_t enc_addr,
                                           a_format_abs_multi_single_t *abs_data)
{
    status_t result;

    if (A_Format_ABS_Readout_Multi_Single_CMD(enc_addr) == kStatus_FLEXIO_A_FORMAT_OutOfIDRange)
    {
        return kStatus_FLEXIO_A_FORMAT_OutOfIDRange;
    }

    FLEXIO_A_Format_Config_DR_length(enc->controller, 1);
    result = FLEXIO_A_Format_WriteBlocking(enc->controller, &cdf, 1);
    if (result == kStatus_Success)
    {
        result = FLEXIO_A_Format_ReadBlocking(enc->controller, (uint16_t *)res3,
                                              HALFWORD_NUM(a_format_res3_t) * nEncoder);
    }
    if (result != kStatus_Success)
    {
        A_FORMAT_MARK_NO_FRAME(abs_data, nEncoder);
        return result;
    }

    return A_Format_ABS_Readout_Multi_Single_Parse(enc, res3, abs_data);
}
