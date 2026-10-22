/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <math.h>
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"
#include "a-format.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ENC_ADDR DEMO_ENCODER_ADDRESS
#define ENC_ID   0x030201

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
encoder_a_format_t encoder;
status_t status;

/* Free-running millisecond counter, see APP_InitUptime(). */
static volatile uint32_t s_uptimeMs = 0U;

/*******************************************************************************
 * Code
 ******************************************************************************/
/* 1 kHz time base for the loop-test timestamps. Overrides the WEAK stub in
 * startup_MCXA366.c. */
void SysTick_Handler(void)
{
    s_uptimeMs++;
}

static void APP_InitUptime(void)
{
    s_uptimeMs = 0U;
    /* Valid after BOARD_InitBootClocks(). */
    (void)SysTick_Config(SystemCoreClock / 1000U);
}

static uint32_t APP_GetUptimeMs(void)
{
    return s_uptimeMs;
}

/*
 * Judge a transfer that returns a status_t only (ID / EEPROM / temperature).
 * A non-zero ES fails the read, because the encoder is then flagging a condition
 * that makes the payload unreliable.
 */
static bool StatusOk(status_t result)
{
    uint8_t es;

    if (result == kStatus_FLEXIO_A_FORMAT_EncoderStatErr)
    {
        es = A_Format_GetLastES();
        PRINTF("failed (encoder reports ES 0x%02X)\r\n", es);
        A_Format_PrintfES(PRINTF, es);
        return false;
    }
    if (result != kStatus_Success)
    {
        PRINTF("failed (transfer status 0x%X)\r\n", result);
        return false;
    }
    es = A_Format_GetLastES();
    if (es != kFLEXIO_A_FORMAT_ES_NoErr)
    {
        PRINTF("failed (encoder reports ES 0x%02X)\r\n", es);
        return false;
    }
    return true;
}

/*
 * Judge an echo-verified write (address / ID / EEPROM). ES bit0 is MemBusy, the
 * expected answer while the encoder is still writing, so it passes here. Any
 * other non-zero ES is a fault and falls through to StatusOk().
 */
static bool WriteAckOk(status_t result)
{
    if ((result == kStatus_FLEXIO_A_FORMAT_EncoderStatErr) &&
        (A_Format_GetLastES() == kFLEXIO_A_FORMAT_ES_Busy_MemBusy))
    {
        PRINTF("[encoder ES 0x%02X] ", A_Format_GetLastES());
        return true;
    }
    return StatusOk(result);
}

/*
 * Read the encoder status for diagnostics. The parser returns kStatus_Fail for a
 * non-zero ES, so a valid frame carrying an alarm is accepted here and left for
 * the caller to judge on ALM.
 */
static bool ReadEncoderStatus(encoder_a_format_t *encoder, a_format_status_t *statusData)
{
    status_t result = A_Format_Readout_Encoder_status(encoder, ENCODER_ADDRESS_IT(ENC_ADDR), statusData);

    if ((statusData->es == kFLEXIO_A_FORMAT_ES_FrameErr) || (statusData->encID != ENC_ADDR) ||
        !((result == kStatus_Success) ||
          ((result == kStatus_Fail) && (statusData->es != kFLEXIO_A_FORMAT_ES_NoErr))))
    {
        PRINTF("no valid status response (transfer status 0x%X)\r\n", result);
        return false;
    }
    return true;
}

/*
 * Judge a readout that reports per-encoder results through the ES field.
 * kFLEXIO_A_FORMAT_ES_FrameErr is written by the parser when no usable frame
 * arrived; any other non-zero ES comes from the encoder. A stale ES left by an
 * aborted transfer is caught through result.
 */
static bool FrameOk(status_t result, uint8_t es)
{
    if (es == kFLEXIO_A_FORMAT_ES_FrameErr)
    {
        PRINTF("failed (no valid response frame)\r\n");
        return false;
    }
    if (es != kFLEXIO_A_FORMAT_ES_NoErr)
    {
        PRINTF("failed (encoder reports ES 0x%02X)\r\n", es);
        A_Format_PrintfES(PRINTF, es);
        return false;
    }
    if (result != kStatus_Success)
    {
        PRINTF("failed (transfer status 0x%X)\r\n", result);
        return false;
    }
    return true;
}

static void RunOneToOneTests(encoder_a_format_t *encoder)
{
    uint32_t enc_id = 0;

    PRINTF("\r\n******************** 1 to 1 mode (Only one encoder connects to the board) ********************\r\n");
    PRINTF("****************\r\n* Test case  1 *\r\n****************\r\n");
    PRINTF("> Set the encoder address to %d ==> ", ENC_ADDR);
    status = A_Format_Set_Encoder_Address_1to1(encoder, ENCODER_ADDRESS_IT(ENC_ADDR));
    SDK_DelayAtLeastUs(DEMO_MEMORY_WRITE_SETTLE_US, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    if (WriteAckOk(status))
    {
        PRINTF("acknowledged\r\n");
    }

    PRINTF("****************\r\n* Test case  2 *\r\n****************\r\n");
    PRINTF("> Set the encoder ID to 0x%06X ==> ", ENC_ID);
    status = A_Format_Set_ID_1to1(encoder, ENC_ID);
    SDK_DelayAtLeastUs(DEMO_MEMORY_WRITE_SETTLE_US, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    if (WriteAckOk(status))
    {
        PRINTF("acknowledged\r\n");
    }

    PRINTF("****************\r\n* Test case  3 *\r\n****************\r\n");
    PRINTF("> Get the encoder ID ==> ");
    status = A_Format_Get_ID_1to1(encoder, &enc_id);
    if (StatusOk(status))
    {
        PRINTF("0x%06X (successful)\r\n", enc_id);
    }
}

static void RunIndividualTransmissionTests(encoder_a_format_t *encoder)
{
    uint32_t enc_id = 0;
    a_format_eeprom_t eeprom = {
        .address = 0x01,
        .data = 0x5A2B
    };
    const uint16_t expectedData = eeprom.data;

    PRINTF("\r\n******************** Individual Transmission mode (The board accesses only one encoder on the bus) ********************\r\n");
    PRINTF("****************\r\n* Test case  4 *\r\n****************\r\n");
    PRINTF("> The encoder address %d is set by matching the ID 0x%06X ==> ", ENC_ADDR, ENC_ID);
    status = A_Format_Set_Encoder_Address_MATCH_ID(encoder, ENC_ID, ENCODER_ADDRESS_IT(ENC_ADDR));
    SDK_DelayAtLeastUs(DEMO_MEMORY_WRITE_SETTLE_US, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    if (WriteAckOk(status))
    {
        PRINTF("acknowledged\r\n");
    }

    PRINTF("****************\r\n* Test case  5 *\r\n****************\r\n");
    PRINTF("> Set the encoder ID to 0x%06X ==> ", ENC_ID);
    status = A_Format_Set_ID(encoder, ENCODER_ADDRESS_IT(ENC_ADDR), ENC_ID);
    SDK_DelayAtLeastUs(DEMO_MEMORY_WRITE_SETTLE_US, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    if (WriteAckOk(status))
    {
        PRINTF("acknowledged\r\n");
    }

    PRINTF("****************\r\n* Test case  6 *\r\n****************\r\n");
    PRINTF("> Get the encoder ID ==> ");
    status = A_Format_Get_ID(encoder, ENCODER_ADDRESS_IT(ENC_ADDR), &enc_id);
    if (StatusOk(status))
    {
        PRINTF("0x%06X (successful)\r\n", enc_id);
    }

    PRINTF("****************\r\n* Test case  7 *\r\n****************\r\n");
    PRINTF("> Write the data 0x%04X to the address 0x%02X of EEPROM ==> ", eeprom.data, eeprom.address);
    status = A_Format_Memory_Write(encoder, ENCODER_ADDRESS_IT(ENC_ADDR), &eeprom);
    if (WriteAckOk(status))
    {
        PRINTF("acknowledged\r\n");
    }

    PRINTF("****************\r\n* Test case  8 *\r\n****************\r\n");
    PRINTF("> Read the data from the address 0x%02X of EEPROM ==> ", eeprom.address);
    memset(&eeprom.data, 0, 2);
    SDK_DelayAtLeastUs(DEMO_MEMORY_WRITE_SETTLE_US, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    status = A_Format_Memory_Read(encoder, ENCODER_ADDRESS_IT(ENC_ADDR), &eeprom);
    if (StatusOk(status))
    {
        if (eeprom.data == expectedData)
        {
            PRINTF("0x%04X (readback verified)\r\n", eeprom.data);
        }
        else
        {
            PRINTF("failed (readback 0x%04X, expected 0x%04X)\r\n", eeprom.data, expectedData);
        }
    }
}

/*
 * Wait out the non-latched Busy and MemBusy conditions, then issue CDF 10 if ALM
 * reports a latched fault that CDF 10 can clear. CDF 10 also resets the multi-turn
 * counter, so ES bit0 on its own does not trigger it.
 */
static void PrepareEncoder(encoder_a_format_t *encoder)
{
    a_format_status_t statusData[A_FORMAT_ENCODER_MAX_NUM] = {0};
    uint32_t waitUs = 0U;

    PRINTF("\r\n******************** Preparing the encoder ********************\r\n");
    PRINTF("> Read the status of the encoder 0x%02X ==> ", ENC_ADDR);
    if (!ReadEncoderStatus(encoder, statusData))
    {
        return;
    }

    PRINTF("ES: 0x%02X, ALM: 0x%04X\r\n", statusData[0].es, statusData[0].status);
    A_Format_PrintfALM(PRINTF, statusData[0].status);
    if ((statusData[0].status & kFLEXIO_A_FORMAT_ALM_Busy) != 0U)
    {
        PRINTF("> Position not ready: keep the shaft stopped or at <= 250 rpm; Busy clears automatically\r\n");
    }

    while ((statusData[0].status & (kFLEXIO_A_FORMAT_ALM_Busy | kFLEXIO_A_FORMAT_ALM_MemBusy)) != 0U)
    {
        if (waitUs >= DEMO_ENCODER_READY_TIMEOUT_US)
        {
            PRINTF("> Ready wait timed out: ES 0x%02X, ALM 0x%04X; CDF10 skipped, multi-turn counter kept\r\n",
                   statusData[0].es, statusData[0].status);
            A_Format_PrintfALM(PRINTF, statusData[0].status);
            return;
        }
        SDK_DelayAtLeastUs(DEMO_ENCODER_READY_POLL_US, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
        waitUs += DEMO_ENCODER_READY_POLL_US;
        if (!ReadEncoderStatus(encoder, statusData))
        {
            return;
        }
    }

    if ((statusData[0].status & A_FORMAT_ALM_MASK_CDF10_CLEARABLE) == 0U)
    {
        PRINTF("> No CDF10-clearable ALM fault is set, the multi-turn counter is kept as is\r\n");
        return;
    }

    A_Format_PrintfALM(PRINTF, statusData[0].status);
    PRINTF("> Clear the status flags and reset the multi-turn counter (CDF 10) ==> ");
    status = A_Format_Clear_Stat_Multi(encoder, ENCODER_ADDRESS_IT(ENC_ADDR));
    if (status == kStatus_Success)
    {
        PRINTF("latched flags cleared; ES 0x%02X, ALM 0x%04X (multi-turn reset requested)\r\n",
               A_Format_GetLastES(), A_Format_GetLastALM());
    }
    else
    {
        PRINTF("failed (transfer status 0x%X)\r\n", status);
        if (status == kStatus_FLEXIO_A_FORMAT_EncoderStatErr)
        {
            A_Format_PrintfALM(PRINTF, A_Format_GetLastALM());
        }
    }
}

static void RunIndividualReadoutTests(encoder_a_format_t *encoder,
                                      a_format_abs_multi_single_t *enc_abs,
                                      float *temp)
{
    a_format_abs_multi_t multiData[A_FORMAT_ENCODER_MAX_NUM] = {0};
    a_format_abs_single_t singleData[A_FORMAT_ENCODER_MAX_NUM] = {0};
    a_format_single_stat_t singleStat[A_FORMAT_ENCODER_MAX_NUM] = {0};
    a_format_single_temp_t singleTemp[A_FORMAT_ENCODER_MAX_NUM] = {0};
    a_format_status_t statusData[A_FORMAT_ENCODER_MAX_NUM] = {0};

    PRINTF("****************\r\n* Test case  9 *\r\n****************\r\n");
    PRINTF("> Read the status of the encoder 0x%02X ==> ", ENC_ADDR);
    if (ReadEncoderStatus(encoder, statusData))
    {
        PRINTF("ES: 0x%02X, ALM: 0x%04X (status received)\r\n", statusData[0].es, statusData[0].status);
        A_Format_PrintfALM(PRINTF, statusData[0].status);
    }

    PRINTF("****************\r\n* Test case 10 *\r\n****************\r\n");
    PRINTF("> Get the temperature of the encoder ==> ");
    status = A_Format_Get_Temperature(encoder, ENCODER_ADDRESS_IT(ENC_ADDR), temp);
    if (StatusOk(status))
    {
        PRINTF("%f (successful)\r\n", (double)*temp);
    }

    PRINTF("****************\r\n* Test case 11 *\r\n****************\r\n");
    PRINTF("> Get the multi-turn data of the encoder ==> ");
    status = A_Format_ABS_Readout_Multi(encoder, ENCODER_ADDRESS_IT(ENC_ADDR), multiData);
    if (FrameOk(status, multiData[0].es))
    {
        PRINTF("ES: 0x%02X, Multi-turn: %d (successful)\r\n", multiData[0].es, multiData[0].multiTurn);
    }

    PRINTF("****************\r\n* Test case 12 *\r\n****************\r\n");
    PRINTF("> Get the single-turn data of the encoder ==> ");
    status = A_Format_ABS_Readout_Single(encoder, ENCODER_ADDRESS_IT(ENC_ADDR), singleData);
    if (FrameOk(status, singleData[0].es))
    {
        PRINTF("ES: 0x%02X, Single-turn: %u (successful)\r\n", singleData[0].es, singleData[0].singleTurn);
    }

    PRINTF("****************\r\n* Test case 13 *\r\n****************\r\n");
    PRINTF("> Get the single-turn data and status of the encoder ==> ");
    status = A_Format_ABS_Readout_Single_with_status(encoder, ENCODER_ADDRESS_IT(ENC_ADDR), singleStat);
    if (FrameOk(status, singleStat[0].es))
    {
        PRINTF("ES: 0x%02X, Single-turn: %u, status: 0x%04X (successful)\r\n",
               singleStat[0].es, singleStat[0].singleTurn, singleStat[0].ALM);
    }

    PRINTF("****************\r\n* Test case 14 *\r\n****************\r\n");
    PRINTF("> Get the single-turn data and temperature of the encoder ==> ");
    status = A_Format_ABS_Readout_Single_with_temperature(encoder, ENCODER_ADDRESS_IT(ENC_ADDR), singleTemp);
    if (FrameOk(status, singleTemp[0].es))
    {
        PRINTF("ES: 0x%02X, Single-turn: %u, temperature: %f (successful)\r\n",
               singleTemp[0].es, singleTemp[0].singleTurn, (double)singleTemp[0].temperature);
    }

    PRINTF("****************\r\n* Test case 15 *\r\n****************\r\n");
    PRINTF("> Get the single-turn data and the multi-turn data of the encoder ==> ");
    status = A_Format_ABS_Readout_Multi_Single(encoder, ENCODER_ADDRESS_IT(ENC_ADDR), &enc_abs[0]);
    if (FrameOk(status, enc_abs[0].es))
    {
        PRINTF("successful\r\n");
        PRINTF("\tMulti-turn data: %d, single-turn data: %u\r\n", enc_abs[0].multiTurn, enc_abs[0].singleTurn);
    }
}

static void RunMultipleTransmissionTest(encoder_a_format_t *encoder,
                                        a_format_abs_multi_single_t *enc_abs)
{
    PRINTF("\r\n******************** Multiple Transmission mode (The board can access multiple encoders on the bus simultaneously) ********************\r\n");
    PRINTF("****************\r\n* Test case 16 *\r\n****************\r\n");
#if DEMO_ENABLE_MT_TEST
    PRINTF("> Get the single-turn data and the multi-turn data of the encoder ==> \r\n");
    status = A_Format_ABS_Readout_Multi_Single(encoder, ENCODER_ADDRESS_MT(ENC_ADDR), &enc_abs[0]);
    PRINTF("\t(transfer status 0x%X)\r\n", status);
    for (uint8_t index = 0; index <= ENC_ADDR; index++)
    {
        if (enc_abs[index].es == kFLEXIO_A_FORMAT_ES_FrameErr)
        {
            PRINTF("\t[Encoder -] Disconnected\r\n");
        }
        else if (enc_abs[index].es != kFLEXIO_A_FORMAT_ES_NoErr)
        {
            PRINTF("\t[Encoder %d] ES 0x%02X, data not trustworthy\r\n", enc_abs[index].encID, enc_abs[index].es);
            A_Format_PrintfES(PRINTF, enc_abs[index].es);
        }
        else
        {
            PRINTF("\t[Encoder %d] Multi-turn data: %d, single-turn data: %u\r\n",
                   enc_abs[index].encID, enc_abs[index].multiTurn, enc_abs[index].singleTurn);
        }
    }
#else
    (void)encoder;
    (void)enc_abs;
    PRINTF("> Skipped: needs %d encoders on the bus (see DEMO_ENABLE_MT_TEST)\r\n", ENC_ADDR + 1);
#endif
}

static void RunLoopTest(encoder_a_format_t *encoder,
                        a_format_abs_multi_single_t *abs_save,
                        float temp_save)
{
    a_format_abs_multi_single_t enc_abs[A_FORMAT_ENCODER_MAX_NUM] = {0};
    float temp = temp_save;
    uint32_t iteration = 0;
    uint32_t nowMs = 0;
    uint32_t singleTurnDiff = 0;
    uint32_t failCount = 0;

    PRINTF("\r\n******************** Running the loop test ********************\r\n");
    while (1)
    {
        SDK_DelayAtLeastUs(DEMO_POLL_PERIOD_US, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
        iteration++;
        nowMs = APP_GetUptimeMs();
        status = A_Format_ABS_Readout_Multi_Single(encoder, ENCODER_ADDRESS_IT(ENC_ADDR), &enc_abs[0]);
        if (enc_abs[0].es == kFLEXIO_A_FORMAT_ES_FrameErr)
        {
            /*
             * Timeout means the encoder never answered; any other status means a
             * response arrived but its command code or CRC8 did not match.
             */
            if (status == kStatus_FLEXIO_A_FORMAT_Timeout)
            {
                PRINTF("[%lu.%03lus] Read timed out: no response frame\r\n",
                       (unsigned long)(nowMs / 1000U), (unsigned long)(nowMs % 1000U));
            }
            else
            {
                PRINTF("[%lu.%03lus] Frame error: response arrived, command code/CRC8 mismatch (status 0x%X)\r\n",
                       (unsigned long)(nowMs / 1000U), (unsigned long)(nowMs % 1000U), status);
            }
            failCount++;
            continue;
        }
        if ((enc_abs[0].es != kFLEXIO_A_FORMAT_ES_NoErr) || (status != kStatus_Success))
        {
            failCount++;
            /* ES does not separate Busy from MemBusy, so fetch the full ALM word.
             * Only on the first failure and once per temperature period, to keep
             * a persistent fault from flooding the UART. */
            if ((failCount == 1U) || ((iteration % DEMO_TEMPERATURE_PERIOD) == 0U))
            {
                a_format_status_t statusData = {0};
                PRINTF("Encoder reports ES 0x%02X (transfer status 0x%X), skipping this sample\r\n",
                       enc_abs[0].es, status);
                SDK_DelayAtLeastUs(DEMO_INTER_FRAME_GAP_US, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
                if (ReadEncoderStatus(encoder, &statusData))
                {
                    PRINTF("  -> ES 0x%02X, ALM 0x%04X\r\n", statusData.es, statusData.status);
                    A_Format_PrintfALM(PRINTF, statusData.status);
                }
            }
            continue;
        }

        if (failCount != 0U)
        {
            PRINTF("Recovered after %lu failed reads\r\n", (unsigned long)failCount);
            failCount = 0;
        }

        /* Temperature is refreshed every DEMO_TEMPERATURE_PERIOD-th iteration only.
         * A failed refresh keeps the last known value. */
        if ((iteration % DEMO_TEMPERATURE_PERIOD) == 0U)
        {
            SDK_DelayAtLeastUs(DEMO_INTER_FRAME_GAP_US, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
            status = A_Format_Get_Temperature(encoder, ENCODER_ADDRESS_IT(ENC_ADDR), &temp);
            if (status != kStatus_Success)
            {
                PRINTF("  -> temperature read failed (status 0x%X), keeping %f\r\n",
                       status, (double)temp_save);
                if (status == kStatus_FLEXIO_A_FORMAT_EncoderStatErr)
                {
                    A_Format_PrintfES(PRINTF, A_Format_GetLastES());
                }
                temp = temp_save;
            }
        }

        singleTurnDiff = (enc_abs[0].singleTurn > abs_save->singleTurn) ?
                         (enc_abs[0].singleTurn - abs_save->singleTurn) :
                         (abs_save->singleTurn - enc_abs[0].singleTurn);
        if ((singleTurnDiff > DEMO_ST_REPORT_THRESHOLD) ||
            (fabsf(temp - temp_save) > DEMO_TEMP_REPORT_DELTA_C))
        {
            PRINTF("[%lu.%03lus] Multi-turn data: %d, single-turn data: %u\r\n\tTemperature: %f\r\n\r\n",
                   (unsigned long)(nowMs / 1000U), (unsigned long)(nowMs % 1000U),
                   enc_abs[0].multiTurn, enc_abs[0].singleTurn, (double)temp);
            *abs_save = enc_abs[0];
            temp_save = temp;
        }
    }
}
/*!
 * @brief Main function
 */
int main(void)
{
    /* Structure of initialize A-format encoder */
    flexio_a_format_config_t devConfig;
    FLEXIO_A_FORMAT_Type encDev = {0};
    a_format_abs_multi_single_t enc_abs[A_FORMAT_ENCODER_MAX_NUM] = {0};
    uint32_t clock;
    float temp = 0.0f;

    BOARD_InitHardware();
    APP_InitUptime();
    clock = FLEXIO_CLOCK_FREQUENCY / 1000000;

    PRINTF("%s %s\r\n", __DATE__, __TIME__);
    PRINTF("Encoder A-format example\r\n");
    PRINTF("FlexIO Root Clock is %d MHz\r\n", clock);

    /*
     * Config->enableA_Format   = true;
     * Config->enableInDoze     = false;
     * Config->enableInDebug    = true;
     * Config->enableFastAccess = false;
     * Config->baudRate_bps     = kFLEXIO_A_FORMAT_2_5MHZ;
     * Config->userMode         = kFLEXIO_A_FORMAT_USERMODE_ONESHOT;
     */
    FLEXIO_A_Format_GetDefaultConfig(&devConfig);

    encDev.flexioBase                          = BOARD_FLEXIO_BASE;
    encDev.TxPinIndex                          = FLEXIO_A_FORMAT_TX_PIN;
    encDev.RxPinIndex                          = FLEXIO_A_FORMAT_RX_PIN;
    encDev.DRPinIndex                          = FLEXIO_A_FORMAT_DR_PIN;
    encDev.shifterIndex[0]                     = A_FORMAT_TX_SHIFTER_INDEX;
    encDev.shifterIndex[1]                     = A_FORMAT_RX_SHIFTER_INDEX;
    encDev.timerIndex[A_FORMAT_TIMER_TX_INDEX] = A_FORMAT_TX_TIMER_INDEX;
    encDev.timerIndex[A_FORMAT_TIMER_RX_INDEX] = A_FORMAT_RX_TIMER_INDEX;
    encDev.timerIndex[A_FORMAT_TIMER_DR_INDEX] = A_FORMAT_DR_TIMER_INDEX;
    /* The TX/RX clock timers are only used by the 16 Mbps path, which needs 5 FlexIO
     * timers; FLEXIO0 on MCXA366 has 4. Mark both slots unused. */
    encDev.timerIndex[A_FORMAT_TIMER_TX_CLOCK_INDEX] = 0xFFU;
    encDev.timerIndex[A_FORMAT_TIMER_RX_CLOCK_INDEX] = 0xFFU;

    encoder.controller = &encDev;
    encoder.singleTurnRevolution  = DEMO_ENCODER_ST_BITS;
    encoder.multiTurnRevolution   = DEMO_ENCODER_MT_BITS;
    encoder.single_turn_sign_mask = DEMO_ENCODER_ST_MASK;
    encoder.multi_turn_sign_mask  = DEMO_ENCODER_MT_MASK;

    status = FLEXIO_A_Format_Init(&encDev, &devConfig, FLEXIO_CLOCK_FREQUENCY);
    if (status != kStatus_Success)
    {
        PRINTF("FLEXIO_A_Format_Init() failed with status 0x%X\r\n", status);
        return -1;
    }

    RunOneToOneTests(&encoder);
    RunIndividualTransmissionTests(&encoder);
    PrepareEncoder(&encoder);
    RunIndividualReadoutTests(&encoder, enc_abs, &temp);
    RunMultipleTransmissionTest(&encoder, enc_abs);
    RunLoopTest(&encoder, &enc_abs[0], temp);

    return 0;
}
