/*
 * Copyright (c) 2023-2025 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*
 * QEO (Quadrature Encoder Output) ABZ Signal Generation Example
 *
 * This example demonstrates how QEO generates incremental ABZ signals based on position information:
 * 1. QEO takes position input (from software or hardware) and converts it to ABZ quadrature signals
 * 2. Based on the configured resolution lines, QEO generates A/B quadrature signals and Z index pulse
 * 3. The position range (0 to 0x100000000) is divided according to resolution lines
 * 4. Each line generates 4 states in A/B signals (4x resolution)
 * 5. Z signal generates one pulse per revolution
 * 6. Supports position synchronization for ABZ signals output
 */

#include "board.h"
#include <stdio.h>
#include "hpm_qeo_drv.h"
#include "hpm_qeiv2_drv.h"
#include "hpm_trgm_soc_drv.h"
#include "hpm_synt_drv.h"
#include "moto.h"

#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"

/* QEO configuration defines */
#define TEST_QEO BOARD_QEO
#define QEO_TRGM_POS BOARD_QEO_TRGM_POS

#define TEST_QEO_ABZ_LINES    (1024U)   /* Number of encoder lines per revolution */
#define TEST_QEO_ABZ_MAX_FREQ (250000U) /* Maximum frequency of ABZ signals in Hz */
#define QEO_POSITION_MAX_VAL  (0x100000000UL)  /* Maximum position value (32-bit) */


#define PULSE0_NUM 64
#define PULSE1_NUM 128

#ifndef APP_QEI_BASE
#define APP_QEI_BASE BOARD_BLDC_QEIV2_BASE
#endif
#ifndef APP_QEI_IRQ
#define APP_QEI_IRQ  BOARD_BLDC_QEIV2_IRQ
#endif
#ifndef APP_MOTOR_CLK
#define APP_MOTOR_CLK BOARD_BLDC_QEI_CLOCK_SOURCE
#endif
#ifndef APP_ENCODER_PHASE_COUNT_PER_REV
#define APP_ENCODER_PHASE_COUNT_PER_REV BOARD_BLDC_QEI_FOC_PHASE_COUNT_PER_REV
#endif

static volatile bool s_pos_cmp_matched;
static volatile bool s_pulse0_matched;
static volatile bool s_pulse1_matched;
static volatile bool s_cycle0_matched;
static volatile bool s_cycle1_matched;
static volatile uint32_t z, ph, spd, tm, phcnt, pos, angle;
static volatile uint32_t pulse_snap0, pulse_snap1;
static volatile uint32_t cycle_snap0, cycle_snap1;

/* Static function declaration */
static void qeiv2_init(void)
{
    qeiv2_mode_config_t mode_config = {0};
    qeiv2_phcnt_cmp_match_config_t phcnt_cmp_config = {0};

    /*  mode config */
    mode_config.work_mode = qeiv2_work_mode_abz;
    mode_config.spd_tmr_content_sel = qeiv2_spd_tmr_as_spd_tm;
    mode_config.z_count_inc_mode = qeiv2_z_count_inc_on_phase_count_max;
    mode_config.phcnt_max = APP_ENCODER_PHASE_COUNT_PER_REV;
    mode_config.z_cali_enable = false;
    mode_config.z_cali_ignore_ab = false;
    mode_config.phcnt_idx = 0;
    qeiv2_config_mode(APP_QEI_BASE, &mode_config);
    qeiv2_set_z_phase(APP_QEI_BASE, 100);       /* setting z phase init value */
    qeiv2_set_phase_cnt(APP_QEI_BASE, 500);     /* setting phase cnt init value */

//    /*  cmp config */
//    phcnt_cmp_config.phcnt_cmp_value = APP_ENCODER_PHASE_COUNT_PER_REV / 2;
//    phcnt_cmp_config.ignore_rotate_dir = true;
//    phcnt_cmp_config.ignore_zcmp = true;
//    qeiv2_config_phcnt_cmp_match_condition(APP_QEI_BASE, &phcnt_cmp_config);
//    qeiv2_enable_load_read_trigger_event(APP_QEI_BASE, QEIV2_EVENT_POSITION_COMPARE_FLAG_MASK);

//    /*  measure speed config */
//    qeiv2_set_pulse0_num(APP_QEI_BASE, PULSE0_NUM);
//    qeiv2_set_pulse1_num(APP_QEI_BASE, PULSE1_NUM);
//    qeiv2_set_cycle0_num(APP_QEI_BASE, 2500000);
//    qeiv2_set_cycle1_num(APP_QEI_BASE, 25000000);
//#if defined (HPM_IP_FEATURE_QEIV2_SW_RESTART_TRG) && HPM_IP_FEATURE_QEIV2_SW_RESTART_TRG
//    qeiv2_enable_trig_pulse0(APP_QEI_BASE);
//    qeiv2_enable_trig_pulse1(APP_QEI_BASE);
//    qeiv2_enable_trig_cycle0(APP_QEI_BASE);
//    qeiv2_enable_trig_cycle1(APP_QEI_BASE);
//#endif
//#if defined (HPM_IP_FEATURE_QEIV2_SW_RESTART_TRG) && HPM_IP_FEATURE_QEIV2_SW_RESTART_TRG
//    qeiv2_sw_restart_pulse0(APP_QEI_BASE);
//    qeiv2_sw_restart_pulse1(APP_QEI_BASE);
//    qeiv2_sw_restart_cycle0(APP_QEI_BASE);
//    qeiv2_sw_restart_cycle1(APP_QEI_BASE);
//#endif

//    /*  enable irq */
//    qeiv2_enable_irq(APP_QEI_BASE, QEIV2_EVENT_POSITION_COMPARE_FLAG_MASK | QEIV2_EVENT_PULSE0_FLAG_MASK
//                                 | QEIV2_EVENT_PULSE1_FLAG_MASK | QEIV2_EVENT_CYCLE0_FLAG_MASK | QEIV2_EVENT_CYCLE1_FLAG_MASK);
//    intc_m_enable_irq_with_priority(APP_QEI_IRQ, 1);
}



/*
 * QEO ABZ Signal Generation with Software Position Injection
 *
 * This function demonstrates ABZ signal generation using software-injected positions:
 * 1. Configures QEO for ABZ signal generation with specified resolution
 * 2. Sets maximum frequency to ensure proper signal timing
 * 3. Simulates forward rotation for 2 cycles:
 *    - Injects position values sequentially
 *    - Each position increment generates corresponding ABZ states
 *    - A/B signals generate quadrature waveforms
 *    - Z signal generates index pulse once per revolution
 */
void qeo_gen_abz_signal_software(void)
{
    qeo_abz_mode_t config;
    uint32_t delay_time;
    uint32_t post_unit;
    uint32_t period_us = 1000000 / TEST_QEO_ABZ_MAX_FREQ;  /* Period in microseconds */

    printf("QEO generate ABZ signal with software inject position\n");

    /* Initialize QEO with default ABZ mode configuration */
    qeo_abz_get_default_mode_config(TEST_QEO, &config);
    qeo_abz_config_mode(TEST_QEO, &config);

    /* Set encoder resolution (lines per revolution) */
    /* ABZ.RESOLUTION */
    qeo_abz_set_resolution_lines(TEST_QEO, TEST_QEO_ABZ_LINES);
    /* Configure maximum frequency based on system clock and target frequency */
    /* ABZ.LINE_WIDTH = src_freq / (250000 * 4U) */
    if (status_success != qeo_abz_set_max_frequency(TEST_QEO, clock_get_frequency(BOARD_MOTOR_CLK_NAME), TEST_QEO_ABZ_MAX_FREQ)) {
        printf("config QEO abz max frequency failed\n");
        return;
    }

    /* Calculate position increment per line (total position range / number of lines) */
    /** 0x100000000 / 1024 */
    post_unit = QEO_POSITION_MAX_VAL / TEST_QEO_ABZ_LINES;
    /* Calculate delay time to match target frequency */
    if (period_us == 0) {
        delay_time = 1U;
    } else {
        delay_time = period_us;
    }

    /* Enable software position injection and simulate rotation */
    /* base->POSTION_SEL = 0x01 */
    qeo_enable_software_position_inject(TEST_QEO);
    /* 1024 * 2 -> 转2圈, 16个step */
    for (uint32_t i = 0; i < TEST_QEO_ABZ_LINES * 2; i += 16) {
        qeo_software_position_inject(TEST_QEO, post_unit * i);  /* Inject new position */
        /* Wait for ABZ signals to reach injected position (16 steps per delay) */
        board_delay_us(delay_time * 16);
    }
    /* Reset position to 0 and disable software injection */
    qeo_software_position_inject(TEST_QEO, 0);
    qeo_disable_software_position_inject(TEST_QEO);
}

/*
 * QEO ABZ Signal Generation with Hardware Position Input
 *
 * This function demonstrates ABZ signal generation using hardware-provided positions:
 * 1. Configures QEO with same ABZ parameters as software mode
 * 2. Sets up hardware position input from MMC through TRGM
 * 3. QEO continuously generates ABZ signals based on MMC position
 * 4. Maximum speed is limited by configured frequency:
 *    - TEST_QEO_ABZ_LINES * 1s / TEST_QEO_ABZ_MAX_FREQ = 4000us per revolution
 *    - Maximum speed = 1s / 4000us = 250 revolutions/second
 */
void qeo_gen_abz_signal_hardware(void)
{
    qeo_abz_mode_t config, config1;

    printf("QEO generate ABZ signal with hardware(MMC) provide position\n");

    /* Initialize QEO with default ABZ mode configuration */
    qeo_abz_get_default_mode_config(TEST_QEO, &config);
    qeo_abz_get_default_mode_config(HPM_QEO1, &config1);

    config1.output_type = qeo_abz_output_pulse_revise;
    //config1.output_type = qeo_abz_output_up_down;
    qeo_abz_config_mode(TEST_QEO, &config);
    qeo_abz_config_mode(HPM_QEO1, &config1);

    /* Configure resolution and maximum frequency */
    /* for z index */
    qeo_abz_set_resolution_lines(TEST_QEO, TEST_QEO_ABZ_LINES);
    qeo_abz_set_resolution_lines(HPM_QEO1, TEST_QEO_ABZ_LINES);
    /* TEST_QEO_ABZ_LINES * 1s / TEST_QEO_ABZ_MAX_FREQ = 4000us, speed should less than 1s / 4000us = 250 r/s */
    /* 用于屏蔽过高脉冲？ src_freq / (250000 * 4U) */
    if (status_success != qeo_abz_set_max_frequency(TEST_QEO, clock_get_frequency(BOARD_MOTOR_CLK_NAME), TEST_QEO_ABZ_MAX_FREQ)) {
        printf("config QEO0 abz max frequency failed\n");
        return;
    }
    if (status_success != qeo_abz_set_max_frequency(HPM_QEO1, clock_get_frequency(BOARD_MOTOR_CLK_NAME), TEST_QEO_ABZ_MAX_FREQ)) {
        printf("config QEO1 abz max frequency failed\n");
        return;
    }

    /* Configure hardware position input from MMC */
    /* set pos_matrix from xx to yy 当前是2个输出都连接到 */
    trgm_pos_matrix_config(HPM_TRGM0, QEO_TRGM_POS, trgm_pos_matrix_in_from_mmc0_pos0, false);
    mmc_open_loop_pred();

    trgm_pos_matrix_config(HPM_TRGM0, trgm_pos_matrix_output_to_qeo1, trgm_pos_matrix_in_from_qei0, false);

    /* First enable MOTOR peripheral devices, such as MMC, and then enable timestamp for MOTOR */
    synt_enable_timestamp(HPM_SYNT, true);
    synt_enable_timestamp_debug_stop(HPM_SYNT, true);
}

int main(void)
{
    board_init();
    printf("QEO ABZ example cat!\n");
    HPM_IOC->PAD[IOC_PAD_PA00].FUNC_CTL = IOC_PA00_FUNC_CTL_GPIO_A_00;
    HPM_IOC->PAD[IOC_PAD_PA00].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(4) | IOC_PAD_PAD_CTL_OD_SET(0);





    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOA, 0, gpiom_soc_gpio0);
    gpio_set_pin_output(HPM_GPIO0, GPIO_OE_GPIOA, 0);
    gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, 0, 0);
    /* Demonstrate software position injection mode */
    //qeo_gen_abz_signal_software();

    //board_delay_ms(100U);

    /* Demonstrate hardware position input mode */
    init_qeo_pins(TEST_QEO);  /* Initialize QEO output pins for ABZ signals */
    init_qeo_pins(HPM_QEO1);
    qeo_gen_abz_signal_hardware();

    printf("qeiv2 abz encoder example\n");

    init_qeiv2_ab_pins(APP_QEI_BASE);
    qeiv2_init();

    for (uint32_t i = 0; i < 10; i++) {
        printf("z: 0x%x, phase: %d\n", qeiv2_get_current_count(APP_QEI_BASE, qeiv2_counter_type_z), qeiv2_get_phase_cnt(APP_QEI_BASE));
        board_delay_ms(3000);
    }

    while (1) {
        
    }

    return 0;
}
