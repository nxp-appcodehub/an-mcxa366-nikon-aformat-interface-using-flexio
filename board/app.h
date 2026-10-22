/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_H_
#define _APP_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define BOARD_FLEXIO_BASE         FLEXIO0
#define FLEXIO_A_FORMAT_DR_PIN    16U /* PORT2_8  / FLEXIO0_D16 */
#define FLEXIO_A_FORMAT_TX_PIN    17U /* PORT2_9  / FLEXIO0_D17 */
#define FLEXIO_A_FORMAT_RX_PIN    18U /* PORT2_10 / FLEXIO0_D18 */
#define A_FORMAT_TX_SHIFTER_INDEX 0U
#define A_FORMAT_RX_SHIFTER_INDEX 1U
#define A_FORMAT_DR_TIMER_INDEX   0U
#define A_FORMAT_TX_TIMER_INDEX   1U
#define A_FORMAT_RX_TIMER_INDEX   2U
#define FLEXIO_CLOCK_FREQUENCY    CLOCK_GetFlexioClkFreq()

/* A-format encoder address, 3 bits (0..7). */
#define DEMO_ENCODER_ADDRESS     3U
#define DEMO_ENCODER_ST_BITS     20U
#define DEMO_ENCODER_MT_BITS     16U
#define DEMO_ENCODER_ST_MASK     0x000FFFFFU
#define DEMO_ENCODER_MT_MASK     0x0000FFFFU
/* Position readout period of the loop test. */
#define DEMO_POLL_PERIOD_US      100000U

/* Temperature is read once every DEMO_TEMPERATURE_PERIOD-th loop iteration
 * instead of once per position sample. */
#define DEMO_TEMPERATURE_PERIOD  10U

/* Idle gap inserted between two back-to-back A-format transactions. */
#define DEMO_INTER_FRAME_GAP_US  10000U

/* Maximum MemBusy time after an encoder address, ID, or EEPROM write. */
#define DEMO_MEMORY_WRITE_SETTLE_US 30000U

/* Bounded status polling while Busy or MemBusy is active before readout tests. */
#define DEMO_ENCODER_READY_TIMEOUT_US 1000000U
#define DEMO_ENCODER_READY_POLL_US    10000U

/* Report thresholds of the loop test, in single-turn counts and in degC. */
#define DEMO_ST_REPORT_THRESHOLD 500U
#define DEMO_TEMP_REPORT_DELTA_C 0.3f

/* Multiple Transmission test (test case 16). That mode waits for
 * DEMO_ENCODER_ADDRESS+1 responses, so it needs that many encoders on the bus. */
#define DEMO_ENABLE_MT_TEST      0
/*${macro:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);
/*${prototype:end}*/

#endif /* _APP_H_ */
