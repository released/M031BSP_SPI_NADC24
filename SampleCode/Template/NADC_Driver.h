/**************************************************************************//**
 * @file     NADC_Driver.h
 * @version  V1.00
 * @brief    NADC24 register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef _ADC_REG_TABLE_H_
#define _ADC_REG_TABLE_H_

//------------------------------------------------------------------------------
#include <stdio.h>
#include <stdint.h>

//------------------------------------------------------------------------------
// Use SPIx Port
//------------------------------------------------------------------------------
#define SPI_PORT                                SPI0
#define nDRDY                                   PA4

// #define ADOUT_DIG18_2                           PH3
// #define ADOUT_DIG18_1                           PH4
// #define ADOUT_DIG18_0                           PH5

//------------------------------------------------------------------------------
// ADC Commands
//------------------------------------------------------------------------------
#define ADC_NOP_CMD                             0x00 /* No operation */
#define ADC_WAKE_UP_CMD                         0x02 /* Wake-up from Power-down mode */
#define ADC_POWER_DOWN_CMD                      0x04 /* Enter Power-down mode */
#define ADC_RESET_CMD                           0x06 /* RESET */
#define ADC_START_CONVERSION_CMD                0x08 /* Start ADC conversion */
#define ADC_STOP_CONVERSION_CMD                 0x0A /* Stop  ADC conversion */

#define ADC_SYSOC_CMD                           0x10 /* System offset error calibration */
//#define ADC_SELFOC_CMD                        0x12 /* Self offset error calibration */
#define ADC_TEMPSENSORP1C_CMD                   0x14 /* Temperature sensor point 1 calibration */
//#define ADC_TEMPSENSORP2C_CMD                 0x16 /* Temperature sensor point 2 calibration */

/* DATA READ COMMAND */
#define ADC_READ_DATA_CMD                       0x20 /* Read ADC data */

/* DATA READ COMMAND  */
#define ADC_READ_REG_ADDR_CMD                   0x40 /* Read  b000nnnnn registers from address b010aaaaa */
#define ADC_READ_REG_LEN_CMD                    0x1F /* Read  b000nnnnn registers from address b010aaaaa */

#define ADC_WRITE_REG_ADDR_CMD                  0x60 /* Write b000nnnnn registers from address b011aaaaa */
#define ADC_WRITE_REG_LEN_CMD                   0x1F /* Write b000nnnnn registers from address b011aaaaa */

/* DATA READ COMMAND  */
#define ADC_READ_COEF_ADDR_CMD                  0x9F /* Read coefficient from address b100aaaaa */
#define ADC_READ_COEF_LEN_CMD                   0x02 /* Read coefficient from address b100aaaaa */

#define ADC_WRITE_COEF_ADDR_CMD                 0xBF /* Write coefficient from address b101aaaaa */
#define ADC_WRITE_COEF_LEN_CMD                  0x02 /* Write coefficient from address b101aaaaa */

//------------------------------------------------------------------------------
// Bit Field Definition
//------------------------------------------------------------------------------
#define PWD_CTRL1_PWD_ADC_INT_REF_Pos           (2)
#define PWD_CTRL1_PWD_ADC_INT_REF_Msk           (1 << PWD_CTRL1_PWD_ADC_INT_REF_Pos)
#define PWD_CTRL1_PWD_ADC_INT_REF_POWER_UP      (0 << PWD_CTRL1_PWD_ADC_INT_REF_Pos)
#define PWD_CTRL1_PWD_ADC_INT_REF_POWER_DOWN    (1 << PWD_CTRL1_PWD_ADC_INT_REF_Pos)

#define PWD_CTRL1_PWD_OSC_Pos                           (5)
#define PWD_CTRL1_PWD_OSC_Msk                           (1 << PWD_CTRL1_PWD_OSC_Pos)
#define PWD_CTRL1_PWD_OSC_POWER_UP                      (0 << PWD_CTRL1_PWD_OSC_Pos)
#define PWD_CTRL1_PWD_OSC_POWER_DOWN                    (1 << PWD_CTRL1_PWD_OSC_Pos)

#define PWD_CTRL1_PWD_DAC_Pos                           (6)
#define PWD_CTRL1_PWD_DAC_Msk                           (1 << PWD_CTRL1_PWD_DAC_Pos)
#define PWD_CTRL1_PWD_DAC_POWER_UP                      (0 << PWD_CTRL1_PWD_DAC_Pos)
#define PWD_CTRL1_PWD_DAC_POWER_DOWN                    (1 << PWD_CTRL1_PWD_DAC_Pos)

#define PWD_CTRL1_PWD_DACBUF_Pos                    (7)
#define PWD_CTRL1_PWD_DACBUF_Msk                    (1 << PWD_CTRL1_PWD_DACBUF_Pos)
#define PWD_CTRL1_PWD_DACBUF_POWER_UP               (0 << PWD_CTRL1_PWD_DACBUF_Pos)
#define PWD_CTRL1_PWD_DACBUF_POWER_DOWN             (1 << PWD_CTRL1_PWD_DACBUF_Pos)

#define PWD_CTRL2_PWD_MOD_REFP_Pos                  (0)
#define PWD_CTRL2_PWD_MOD_REFP_Msk                  (1 << PWD_CTRL2_PWD_MOD_REFP_Pos)
#define PWD_CTRL2_PWD_MOD_REFP_POWER_UP             (0 << PWD_CTRL2_PWD_MOD_REFP_Pos)
#define PWD_CTRL2_PWD_MOD_REFP_POWER_DOWN           (1 << PWD_CTRL2_PWD_MOD_REFP_Pos)

#define PWD_CTRL2_PWD_PGA_BUFF_Pos                  (1)
#define PWD_CTRL2_PWD_PGA_BUFF_Msk                  (1 << PWD_CTRL2_PWD_PGA_BUFF_Pos)
#define PWD_CTRL2_PWD_PGA_BUFF_POWER_UP             (0 << PWD_CTRL2_PWD_PGA_BUFF_Pos)
#define PWD_CTRL2_PWD_PGA_BUFF_POWER_DOWN           (1 << PWD_CTRL2_PWD_PGA_BUFF_Pos)

#define PWD_CTRL2_PWD_CHIP_Pos                          (7)
#define PWD_CTRL2_PWD_CHIP_Msk                          (1 << PWD_CTRL2_PWD_CHIP_Pos)
#define PWD_CTRL2_PWD_CHIP_POWER_UP                     (0 << PWD_CTRL2_PWD_CHIP_Pos)
#define PWD_CTRL2_PWD_CHIP_POWER_DOWN                   (1 << PWD_CTRL2_PWD_CHIP_Pos)

#define SET_PGA_BIAS1_CURRENT_Pos               (0)
#define SET_PGA_BIAS1_CURRENT_Msk               (0x3)
#define SET_PGA_BIAS1_CURRENT_0_5X                  (0 << SET_PGA_BIAS1_CURRENT_Pos)
#define SET_PGA_BIAS1_CURRENT_1_0X                  (1 << SET_PGA_BIAS1_CURRENT_Pos)
#define SET_PGA_BIAS1_CURRENT_2_0X                  (2 << SET_PGA_BIAS1_CURRENT_Pos)
#define SET_PGA_BIAS1_CURRENT_3_0X                  (3 << SET_PGA_BIAS1_CURRENT_Pos)

#define SET_ADC_MODULATOR_OPAMP_2_BIAS_Pos      (0)
#define SET_ADC_MODULATOR_OPAMP_2_BIAS_Msk      (0x3)
#define SET_ADC_MODULATOR_OPAMP_2_BIAS_0_5X     (0 << SET_ADC_MODULATOR_OPAMP_2_BIAS_Pos)
#define SET_ADC_MODULATOR_OPAMP_2_BIAS_1_0X     (1 << SET_ADC_MODULATOR_OPAMP_2_BIAS_Pos)
#define SET_ADC_MODULATOR_OPAMP_2_BIAS_2_0X     (2 << SET_ADC_MODULATOR_OPAMP_2_BIAS_Pos)
#define SET_ADC_MODULATOR_OPAMP_2_BIAS_3_0X     (3 << SET_ADC_MODULATOR_OPAMP_2_BIAS_Pos)

#define SET_ADC_MODULATOR_OPAMP_1_BIAS_Pos      (2)
#define SET_ADC_MODULATOR_OPAMP_1_BIAS_Msk      (0xC)
#define SET_ADC_MODULATOR_OPAMP_1_BIAS_0_5X     (0 << SET_ADC_MODULATOR_OPAMP_1_BIAS_Pos)
#define SET_ADC_MODULATOR_OPAMP_1_BIAS_1_0X     (1 << SET_ADC_MODULATOR_OPAMP_1_BIAS_Pos)
#define SET_ADC_MODULATOR_OPAMP_1_BIAS_2_0X     (2 << SET_ADC_MODULATOR_OPAMP_1_BIAS_Pos)
#define SET_ADC_MODULATOR_OPAMP_1_BIAS_3_0X     (3 << SET_ADC_MODULATOR_OPAMP_1_BIAS_Pos)

#define SET_ADC_MODULATOR_WHOLE_BIAS_Pos        (4)
#define SET_ADC_MODULATOR_WHOLE_BIAS_Msk        (0x30)
#define SET_ADC_MODULATOR_WHOLE_BIAS_0_5X       (0 << SET_ADC_MODULATOR_WHOLE_BIAS_Pos)
#define SET_ADC_MODULATOR_WHOLE_BIAS_1_0X       (1 << SET_ADC_MODULATOR_WHOLE_BIAS_Pos)
#define SET_ADC_MODULATOR_WHOLE_BIAS_2_0X       (2 << SET_ADC_MODULATOR_WHOLE_BIAS_Pos)
#define SET_ADC_MODULATOR_WHOLE_BIAS_3_0X       (3 << SET_ADC_MODULATOR_WHOLE_BIAS_Pos)

#define SET_ADC_BUFFER_BIAS_Pos                     (6)
#define SET_ADC_BUFFER_BIAS_Msk                     (0xC0)
#define SET_ADC_BUFFER_BIAS_0_5X                    (0 << SET_ADC_BUFFER_BIAS_Pos)
#define SET_ADC_BUFFER_BIAS_1_0X                    (1 << SET_ADC_BUFFER_BIAS_Pos)
#define SET_ADC_BUFFER_BIAS_2_0X                    (2 << SET_ADC_BUFFER_BIAS_Pos)
#define SET_ADC_BUFFER_BIAS_3_0X                    (3 << SET_ADC_BUFFER_BIAS_Pos)

#define DF1_CTRL_OSR_SEL_Pos                            (0)
#define DF1_CTRL_OSR_SEL_Msk                            (0xF << DF1_CTRL_OSR_SEL_Pos)
#define DF1_CTRL_OSR_SEL_32768                          (0x0 << DF1_CTRL_OSR_SEL_Pos)
#define DF1_CTRL_OSR_SEL_16384                          (0x1 << DF1_CTRL_OSR_SEL_Pos)
#define DF1_CTRL_OSR_SEL_8192                           (0x2 << DF1_CTRL_OSR_SEL_Pos)
#define DF1_CTRL_OSR_SEL_4096                           (0x3 << DF1_CTRL_OSR_SEL_Pos)
#define DF1_CTRL_OSR_SEL_2048                           (0x4 << DF1_CTRL_OSR_SEL_Pos)
#define DF1_CTRL_OSR_SEL_1024                           (0x5 << DF1_CTRL_OSR_SEL_Pos)
#define DF1_CTRL_OSR_SEL_512                            (0x6 << DF1_CTRL_OSR_SEL_Pos)
#define DF1_CTRL_OSR_SEL_256                            (0x7 << DF1_CTRL_OSR_SEL_Pos)
#define DF1_CTRL_OSR_SEL_128                            (0x8 << DF1_CTRL_OSR_SEL_Pos)
#define DF1_CTRL_OSR_SEL_64                             (0x9 << DF1_CTRL_OSR_SEL_Pos)

#define DF1_CTRL_BYPASS_FIR_Pos                         (4)
#define DF1_CTRL_BYPASS_FIR_Msk                         (1 << DF1_CTRL_BYPASS_FIR_Pos)
#define DF1_CTRL_BYPASS_FIR_DISABLE                     (0 << DF1_CTRL_BYPASS_FIR_Pos)
#define DF1_CTRL_BYPASS_FIR_ENABLE                      (1 << DF1_CTRL_BYPASS_FIR_Pos)

#define DF1_CTRL_DF_GAIN_Pos                        (6)
#define DF1_CTRL_DF_GAIN_Msk                            (0x3 << DF1_CTRL_DF_GAIN_Pos)
#define DF1_CTRL_DF_GAIN_1x                             (0 << DF1_CTRL_DF_GAIN_Pos)
#define DF1_CTRL_DF_GAIN_2x                             (1 << DF1_CTRL_DF_GAIN_Pos)
#define DF1_CTRL_DF_GAIN_4x                             (2 << DF1_CTRL_DF_GAIN_Pos)
#define DF1_CTRL_DF_GAIN_8x                             (3 << DF1_CTRL_DF_GAIN_Pos)

#define DF2_CTRL_AVG_ADAP_SET_Pos       				(0)
#define DF2_CTRL_AVG_ADAP_SET_Msk       				(0x3 << DF2_CTRL_AVG_ADAP_SET_Pos)
#define DF2_CTRL_AVG_ADAP_SET_WEAKEST   				(0 << DF2_CTRL_AVG_ADAP_SET_Pos)
#define DF2_CTRL_AVG_ADAP_SET_LOW       				(1 << DF2_CTRL_AVG_ADAP_SET_Pos)
#define DF2_CTRL_AVG_ADAP_SET_MEDIUM    				(2 << DF2_CTRL_AVG_ADAP_SET_Pos)
#define DF2_CTRL_AVG_ADAP_SET_HIGH      				(3 << DF2_CTRL_AVG_ADAP_SET_Pos)

#define DF2_CTRL_AVG_ADAP_EN_Pos                        (2)
#define DF2_CTRL_AVG_ADAP_EN_Msk                        (1 << DF2_CTRL_AVG_ADAP_EN_Pos)
#define DF2_CTRL_AVG_ADAP_EN_DISABLE                    (0 << DF2_CTRL_AVG_ADAP_EN_Pos)
#define DF2_CTRL_AVG_ADAP_EN_ENABLE                     (1 << DF2_CTRL_AVG_ADAP_EN_Pos)

#define DF2_CTRL_LOW_LAT_EN_Pos                         (3)
#define DF2_CTRL_LOW_LAT_EN_Msk                         (1 << DF2_CTRL_LOW_LAT_EN_Pos)
#define DF2_CTRL_LOW_LAT_EN_DISABLE                     (0 << DF2_CTRL_LOW_LAT_EN_Pos)
#define DF2_CTRL_LOW_LAT_EN_ENABLE                      (1 << DF2_CTRL_LOW_LAT_EN_Pos)

#define DF2_CTRL_AVG_FILT_SET_Pos                       (4)
#define DF2_CTRL_AVG_FILT_SET_Msk                       (0x3 << DF2_CTRL_AVG_FILT_SET_Pos)
#define DF2_CTRL_AVG_FILT_SET_BYPASS                    (0 << DF2_CTRL_AVG_FILT_SET_Pos)
#define DF2_CTRL_AVG_FILT_SET_AVG_2                     (1 << DF2_CTRL_AVG_FILT_SET_Pos)
#define DF2_CTRL_AVG_FILT_SET_AVG_4                     (2 << DF2_CTRL_AVG_FILT_SET_Pos)
#define DF2_CTRL_AVG_FILT_SET_AVG_16                    (3 << DF2_CTRL_AVG_FILT_SET_Pos)

#define DF2_CTRL_AVG_FILT_EN_Pos                        (6)
#define DF2_CTRL_AVG_FILT_EN_Msk                        (1 << DF2_CTRL_AVG_FILT_EN_Pos)
#define DF2_CTRL_AVG_FILT_EN_DISABLE                    (0 << DF2_CTRL_AVG_FILT_EN_Pos)
#define DF2_CTRL_AVG_FILT_EN_ENABLE                     (1 << DF2_CTRL_AVG_FILT_EN_Pos)

#define ADC_CTRL_ADC_CLK_SET_Pos                        (2)
#define ADC_CTRL_ADC_CLK_SET_Msk                        (0x3 << ADC_CTRL_ADC_CLK_SET_Pos)
#define ADC_CTRL_ADC_CLK_SET_40960Hz                    (0x0 << ADC_CTRL_ADC_CLK_SET_Pos)
#define ADC_CTRL_ADC_CLK_SET_16384Hz                    (0x1 << ADC_CTRL_ADC_CLK_SET_Pos)
#define ADC_CTRL_ADC_CLK_SET_512KHz                     (0x2 << ADC_CTRL_ADC_CLK_SET_Pos)
#define ADC_CTRL_ADC_CLK_SET_1024KHz                    (0x3 << ADC_CTRL_ADC_CLK_SET_Pos)

#define ADC_CTRL_SETLSEL_Pos                            (5)
#define ADC_CTRL_SETLSEL_Msk                            (0x7 << ADC_CTRL_SETLSEL_Pos)
#define ADC_CTRL_SETLSEL_SETTLE_0                       (0x0 << ADC_CTRL_SETLSEL_Pos)
#define ADC_CTRL_SETLSEL_SETTLE_1                       (0x1 << ADC_CTRL_SETLSEL_Pos)
#define ADC_CTRL_SETLSEL_SETTLE_3                       (0x2 << ADC_CTRL_SETLSEL_Pos)
#define ADC_CTRL_SETLSEL_SETTLE_5                       (0x3 << ADC_CTRL_SETLSEL_Pos)
#define ADC_CTRL_SETLSEL_SETTLE_6                       (0x4 << ADC_CTRL_SETLSEL_Pos)
#define ADC_CTRL_SETLSEL_SETTLE_7                       (0x5 << ADC_CTRL_SETLSEL_Pos)
#define ADC_CTRL_SETLSEL_SETTLE_8                       (0x6 << ADC_CTRL_SETLSEL_Pos)
#define ADC_CTRL_SETLSEL_SETTLE_9                       (0x7 << ADC_CTRL_SETLSEL_Pos)

#define BG_CTRL_SEL_BG_OUT_Pos                          (4)
#define BG_CTRL_SEL_BG_OUT_Msk                          (0xF << BG_CTRL_SEL_BG_OUT_Pos)

#define OSC_CTRL_TUNE_OSC_Pos                           (0)
#define OSC_CTRL_TUNE_OSC_Msk                           (0x1F << OSC_CTRL_TUNE_OSC_Pos)

#define OSC_CTRL_SEL_INT_OSC_Pos                        (7)
#define OSC_CTRL_SEL_INT_OSC_Msk                        (1 << OSC_CTRL_SEL_INT_OSC_Pos)
#define OSC_CTRL_SEL_INT_OSC_Ext                    (0 << OSC_CTRL_SEL_INT_OSC_Pos)
#define OSC_CTRL_SEL_INT_OSC_Int                    (1 << OSC_CTRL_SEL_INT_OSC_Pos)

#define OSC_CTRL_OSC_BIAS_Pos                       (2)
#define OSC_CTRL_OSC_BIAS_Msk                       (0x3 << OSC_CTRL_OSC_BIAS_Pos)
#define OSC_CTRL_OSC_BIAS_0_5X                  (0 << OSC_CTRL_OSC_BIAS_Pos)
#define OSC_CTRL_OSC_BIAS_1_0X                      (1 << OSC_CTRL_OSC_BIAS_Pos)
#define OSC_CTRL_OSC_BIAS_2_0X                  (2 << OSC_CTRL_OSC_BIAS_Pos)
#define OSC_CTRL_OSC_BIAS_3_0X                      (3 << OSC_CTRL_OSC_BIAS_Pos)

#define OSC_CTRL_SEL_INT_OSC_F_Pos                      (4)
#define OSC_CTRL_SEL_INT_OSC_F_Msk                      (0x3 << OSC_CTRL_SEL_INT_OSC_F_Pos)
#define OSC_CTRL_SEL_INT_OSC_F_8192KHz                  (0 << OSC_CTRL_SEL_INT_OSC_F_Pos)
#define OSC_CTRL_SEL_INT_OSC_F_16384KHz                 (1 << OSC_CTRL_SEL_INT_OSC_F_Pos)
#define OSC_CTRL_SEL_INT_OSC_F_24576KHz                 (2 << OSC_CTRL_SEL_INT_OSC_F_Pos)
#define OSC_CTRL_SEL_INT_OSC_F_49152KHz                 (3 << OSC_CTRL_SEL_INT_OSC_F_Pos)

#define VREF_CTRL_SEL_24V_INT_REF_Pos                   (0)
#define VREF_CTRL_SEL_24V_INT_REF_Msk                   (1 << VREF_CTRL_SEL_24V_INT_REF_Pos)
#define VREF_CTRL_SEL_24V_INT_REF_1_2V                  (0 << VREF_CTRL_SEL_24V_INT_REF_Pos)
#define VREF_CTRL_SEL_24V_INT_REF_2_4V                  (1 << VREF_CTRL_SEL_24V_INT_REF_Pos)

#define VREF_CTRL_EN_LDO_CAPLESS_Pos                    (1)
#define VREF_CTRL_EN_LDO_CAPLESS_Msk                    (1 << VREF_CTRL_EN_LDO_CAPLESS_Pos)

#define VREF_CTRL_SEL_INT_REF_Pos                       (2)
#define VREF_CTRL_SEL_INT_REF_Msk                       (0xF << VREF_CTRL_SEL_INT_REF_Pos)

#define VREF_CTRL_SEL_INT_REF_DRV_Pos               (6)
#define VREF_CTRL_SEL_INT_REF_DRV_Msk               (0x3 << VREF_CTRL_SEL_INT_REF_DRV_Pos)
#define VREF_CTRL_SEL_INT_REF_LOWEST                (0 << VREF_CTRL_SEL_INT_REF_DRV_Pos)
#define VREF_CTRL_SEL_INT_REF_MEDIUM_LOW            (1 << VREF_CTRL_SEL_INT_REF_DRV_Pos)
#define VREF_CTRL_SEL_INT_REF_MEDIUM_HIGH           (2 << VREF_CTRL_SEL_INT_REF_DRV_Pos)
#define VREF_CTRL_SEL_INT_REF_HIGHEST               (3 << VREF_CTRL_SEL_INT_REF_DRV_Pos)

#define DAC_DATA2_MISC_WP_Pos                           (6)
#define DAC_DATA2_MISC_WP_Msk                           (0x3 << DAC_DATA2_MISC_WP_Pos)
#define DAC_DATA2_MISC_WP_ENABLE                        (0x1 << DAC_DATA2_MISC_WP_Pos)
#define DAC_DATA2_MISC_WP_DISABLE                       (0x2 << DAC_DATA2_MISC_WP_Pos)

#define PGA_CTRL1_SEL_PGA_INP_Pos                       (2)
#define PGA_CTRL1_SEL_PGA_INP_Msk                       (0x7 << PGA_CTRL1_SEL_PGA_INP_Pos)
#define PGA_CTRL1_SEL_PGA_INN_Pos                       (5)
#define PGA_CTRL1_SEL_PGA_INN_Msk                       (0x7 << PGA_CTRL1_SEL_PGA_INN_Pos)

#define PGA_CTRL1_SEL_PGA_INP_CH_1                      (0x0 << PGA_CTRL1_SEL_PGA_INP_Pos)
#define PGA_CTRL1_SEL_PGA_INP_CH_3                      (0x1 << PGA_CTRL1_SEL_PGA_INP_Pos)
#define PGA_CTRL1_SEL_PGA_INP_CH_5                      (0x2 << PGA_CTRL1_SEL_PGA_INP_Pos)
#define PGA_CTRL1_SEL_PGA_INP_CH_7                      (0x3 << PGA_CTRL1_SEL_PGA_INP_Pos)
#define PGA_CTRL1_SEL_PGA_INP_TP_P                      (0x4 << PGA_CTRL1_SEL_PGA_INP_Pos)
#define PGA_CTRL1_SEL_PGA_INP_AIN_COM                   (0x6 << PGA_CTRL1_SEL_PGA_INP_Pos)
#define PGA_CTRL1_SEL_PGA_INP_VCM                       (0x7 << PGA_CTRL1_SEL_PGA_INP_Pos)

#define PGA_CTRL1_SEL_PGA_INN_CH_0                      (0x0 << PGA_CTRL1_SEL_PGA_INN_Pos)
#define PGA_CTRL1_SEL_PGA_INN_CH_2                      (0x1 << PGA_CTRL1_SEL_PGA_INN_Pos)
#define PGA_CTRL1_SEL_PGA_INN_CH_4                      (0x2 << PGA_CTRL1_SEL_PGA_INN_Pos)
#define PGA_CTRL1_SEL_PGA_INN_CH_6                      (0x3 << PGA_CTRL1_SEL_PGA_INN_Pos)
#define PGA_CTRL1_SEL_PGA_INN_TP_N                      (0x4 << PGA_CTRL1_SEL_PGA_INN_Pos)
#define PGA_CTRL1_SEL_PGA_INN_AIN_COM                   (0x6 << PGA_CTRL1_SEL_PGA_INN_Pos)
#define PGA_CTRL1_SEL_PGA_INN_VCM                       (0x7 << PGA_CTRL1_SEL_PGA_INN_Pos)

#define PGA_CTRL2_PGA_BUF_GAIN_Pos                      (4)
#define PGA_CTRL2_PGA_BUF_GAIN_Msk                      (1 << PGA_CTRL2_PGA_BUF_GAIN_Pos)
#define PGA_CTRL2_PGA_BUF_GAIN_1x                       (0 << PGA_CTRL2_PGA_BUF_GAIN_Pos)
#define PGA_CTRL2_PGA_BUF_GAIN_2x                       (1 << PGA_CTRL2_PGA_BUF_GAIN_Pos)

#define PGA_CTRL2_BUF_BYPASS_Pos                      (5)
#define PGA_CTRL2_BUF_BYPASS_Msk                      (1 << PGA_CTRL2_BUF_BYPASS_Pos)
#define PGA_CTRL2_BUF_BYPASS_DISABLE                  (0 << PGA_CTRL2_BUF_BYPASS_Pos)
#define PGA_CTRL2_BUF_BYPASS_ENABLE                   (1 << PGA_CTRL2_BUF_BYPASS_Pos)

#define PGA_CTRL2_BUF_AUTO_EN_Pos                     (6)
#define PGA_CTRL2_BUF_AUTO_EN_Msk                       (1 << PGA_CTRL2_BUF_AUTO_EN_Pos)
#define PGA_CTRL2_BUF_AUTO_DISABLE                      (0 << PGA_CTRL2_BUF_AUTO_EN_Pos)
#define PGA_CTRL2_BUF_AUTO_ENABLE                       (1 << PGA_CTRL2_BUF_AUTO_EN_Pos)

#define PGA_CTRL2_PGA_GAIN_Pos                        (1)
#define PGA_CTRL2_PGA_GAIN_Msk                          (0x7 << PGA_CTRL2_PGA_GAIN_Pos)
#define PGA_CTRL2_PGA_GAIN_1x                           (0x0 << PGA_CTRL2_PGA_GAIN_Pos)
#define PGA_CTRL2_PGA_GAIN_2x                           (0x1 << PGA_CTRL2_PGA_GAIN_Pos)
#define PGA_CTRL2_PGA_GAIN_4x                           (0x2 << PGA_CTRL2_PGA_GAIN_Pos)
#define PGA_CTRL2_PGA_GAIN_8x                           (0x3 << PGA_CTRL2_PGA_GAIN_Pos)
#define PGA_CTRL2_PGA_GAIN_16x                          (0x4 << PGA_CTRL2_PGA_GAIN_Pos)
#define PGA_CTRL2_PGA_GAIN_32x                          (0x5 << PGA_CTRL2_PGA_GAIN_Pos)
#define PGA_CTRL2_PGA_GAIN_64x                          (0x6 << PGA_CTRL2_PGA_GAIN_Pos)

#define PGA_CTRL2_OFFSETDIS_Pos                         (7)
#define PGA_CTRL2_OFFSETDIS_Msk                         (1 << PGA_CTRL2_OFFSETDIS_Pos)
#define PGA_CTRL2_OFFSETDIS_ENABLE                      (0 << PGA_CTRL2_OFFSETDIS_Pos)
#define PGA_CTRL2_OFFSETDIS_DISABLE                     (1 << PGA_CTRL2_OFFSETDIS_Pos)

#define OTP_CTRL_OTP_CS_Pos                             (6)
#define OTP_CTRL_OTP_CS_Msk                             (3 << OTP_CTRL_OTP_CS_Pos)

#define MISC_SRAM_BISTEN_Pos                            (4)
#define MISC_SRAM_BISTEN_Msk                            (1 << MISC_SRAM_BISTEN_Pos)

#define MISC_DF_BISTEN_Pos                              (5)
#define MISC_DF_BISTEN_Msk                              (1 << MISC_DF_BISTEN_Pos)

#define MISC_HIRCTEN_Pos                                (6)
#define MISC_HIRCTEN_Msk                                (1 << MISC_HIRCTEN_Pos)
#define MISC_HIRCTEN_Disable                            (0 << MISC_HIRCTEN_Pos)
#define MISC_HIRCTEN_Enable                             (1 << MISC_HIRCTEN_Pos)

#define MISC_LDOTEN_Pos                                 (7)
#define MISC_LDOTEN_Msk                                 (1 << MISC_LDOTEN_Pos)
#define MISC_TESTMODE_Msk                               (0xF0)

#define STATUS_DF_BISTFAIL_Pos                          (4)
#define STATUS_DF_BISTFAIL_Msk                          (1 << STATUS_DF_BISTFAIL_Pos)
#define STATUS_DF_BISTEND_Pos                           (5)
#define STATUS_DF_BISTEND_Msk                           (1 << STATUS_DF_BISTEND_Pos)
#define STATUS_SRAM_BISTFAIL_Pos                        (6)
#define STATUS_SRAM_BISTFAIL_Msk                        (1 << STATUS_SRAM_BISTFAIL_Pos)
#define STATUS_SRAM_BISTEND_Pos                         (7)
#define STATUS_SRAM_BISTEND_Msk                         (1 << STATUS_SRAM_BISTEND_Pos)

//------------------------------------------------------------------------------
// ADC register
//------------------------------------------------------------------------------
#define REG_ADDR_PWD_CTRL1                      0x00 /* Power Down Control 1 */
#define REG_ADDR_PWD_CTRL2                      0x01 /* Power Down Control 2 */
#define REG_ADDR_DF1_CTRL                       0x02 /* Digital Filter Control 1 */
#define REG_ADDR_DF2_CTRL                       0x03 /* Digital Filter Control 2 */
#define REG_ADDR_ADC_CTRL                       0x04 /* ADC Control */
#define REG_ADDR_BIAS1_CTRL                     0x05 /* Bias Control 1 */
#define REG_ADDR_BIAS2_CTRL                     0x06 /* Bias Control 2 */
#define REG_ADDR_CHOP_CTRL                      0x07 /* Chop Control Register */
#define REG_ADDR_BG_CTRL1                       0x08 /* Band-gap Control 1 */
//#define REG_ADDR_BG_CTRL2                     0x08 /* Band-gap Control 2 */
#define REG_ADDR_OSC_CTRL1                      0x09 /* Internal Oscillator Control Register 1 */
#define REG_ADDR_OSC_CTRL2                      0x0A /* Internal Oscillator Control Register 2 */
//#define REG_ADDR_OSC_CTRL3                    0x0A /* Internal Oscillator Control Register 3 */
#define REG_ADDR_REF_CTRL                       0x0B /* Reference Voltage Control  */
#define REG_ADDR_DAC_DATA1                      0x0C /* DAC Data Register 1 */
#define REG_ADDR_DAC_DATA2                      0x0D /* DAC Data Register 2 */
#define REG_ADDR_DAC_CTRL                       0x0E /* DAC Control Register */
#define REG_ADDR_PGA_CTRL1                      0x0F /* PGA Control Register 1*/
#define REG_ADDR_PGA_CTRL2                      0x10 /* PGA Control Register 2 */
#define REG_ADDR_ADC_OFFSET_CAL1                0x11 /* ADC Offset Calibration Data Register 1 */
#define REG_ADDR_ADC_OFFSET_CAL2                0x12 /* ADC Offset Calibration Data Register 2 */
#define REG_ADDR_ADC_OFFSET_CAL3                0x13 /* ADC Offset Calibration Data Register 3 */
#define REG_ADDR_ADC_TEMP_SENSOR_CAL1           0x14 /* Temperature Sensor Calibration Data Register 1 */
#define REG_ADDR_ADC_TEMP_SENSOR_CAL2           0x15 /* Temperature Sensor Calibration Data Register 2 */
#define REG_ADDR_ADC_TEMP_SENSOR_CAL3           0x16 /* Temperature Sensor Calibration Data Register 3 */
//#define REG_ADDR_ADC_TEMP_SENSOR_CAL4         0x13 /* Temperature Sensor Calibration Data Register 4 */
//#define REG_ADDR_ADC_TEMP_SENSOR_CAL5         0x14 /* Temperature Sensor Calibration Data Register 5 */
//#define REG_ADDR_ADC_TEMP_SENSOR_CAL6         0x15 /* Temperature Sensor Calibration Data Register 6 */
#define REG_ADDR_ADC_STATUS                     0x17 /* Status Register  */

#define REG_ADDR_ADC_OTP_DOUT0                  0x18 /* OTP Data Output Register 0 */
#define REG_ADDR_ADC_OTP_DOUT1                  0x19 /* OTP Data Output Register 1 */
#define REG_ADDR_ADC_OTP_DOUT2                  0x1A /* OTP Data Output Register 2 */
#define REG_ADDR_ADC_OTP_DOUT3                  0x1B /* OTP Data Output Register 3 */
#define REG_ADDR_ADC_OTP_CTRL                   0x1C /* OTP Control Register       */
#define REG_ADDR_ADC_OTP_DIN                    0x1D /* OTP Data Input Register    */
#define REG_ADDR_MISC_CTRL                      0x1E /* OTP Data Input Register    */
#define REG_ADDR_ADC_VER_ID                     0x1F /* TC58xx RTL Design Version Number Device ID Register*/

//------------------------------------------------------------------------------
// ADC OTP operation register address
//------------------------------------------------------------------------------
#define WRITE_OTP1_DOUT0_CMD                    0x10 /* Write the OTP1 DOUT0 Register OTP_CS:00b PRD:0b PPROG:1b PTM:00b PA:00b  */
#define WRITE_OTP1_DOUT1_CMD                    0x11 /* Write the OTP1 DOUT1 Register OTP_CS:00b PRD:0b PPROG:1b PTM:00b PA:01b  */
#define WRITE_OTP1_DOUT2_CMD                    0x12 /* Write the OTP1 DOUT2 Register OTP_CS:00b PRD:0b PPROG:1b PTM:00b PA:10b  */
#define WRITE_OTP1_DOUT3_CMD                    0x13 /* Write the OTP1 DOUT3 Register OTP_CS:00b PRD:0b PPROG:1b PTM:00b PA:11b  */
#define READ_OTP1_DOUT_REG_CMD                  0x20 /* Read the OTP1 DOUT0-DOUT3 Register OTP_CS:00b PRD:1b PPROG:0b PTM:00b PA:00b*/

#define WRITE_OTP2_DOUT0_CMD                    0x50 /* Write the OTP2 DOUT0 Register OTP_CS:01b PRD:0b PPROG:1b PTM:00b PA:00b  */
#define WRITE_OTP2_DOUT1_CMD                    0x51 /* Write the OTP2 DOUT1 Register OTP_CS:01b PRD:0b PPROG:1b PTM:00b PA:01b  */
#define WRITE_OTP2_DOUT2_CMD                    0x52 /* Write the OTP2 DOUT2 Register OTP_CS:01b PRD:0b PPROG:1b PTM:00b PA:10b  */
#define WRITE_OTP2_DOUT3_CMD                    0x53 /* Write the OTP2 DOUT3 Register OTP_CS:01b PRD:0b PPROG:1b PTM:00b PA:11b  */
#define READ_OTP2_DOUT_REG_CMD                  0x60 /* Read the OTP2 DOUT0-DOUT3 Register OTP_CS:01b PRD:1b PPROG:0b PTM:00b PA:00b*/

#define WRITE_OTP3_DOUT0_CMD                    0x90 /* Write the OTP3 DOUT0 Register OTP_CS:10b PRD:0b PPROG:1b PTM:00b PA:00b  */
#define WRITE_OTP3_DOUT1_CMD                    0x91 /* Write the OTP3 DOUT1 Register OTP_CS:10b PRD:0b PPROG:1b PTM:00b PA:01b  */
#define WRITE_OTP3_DOUT2_CMD                    0x92 /* Write the OTP3 DOUT2 Register OTP_CS:10b PRD:0b PPROG:1b PTM:00b PA:10b  */
#define WRITE_OTP3_DOUT3_CMD                    0x93 /* Write the OTP3 DOUT3 Register OTP_CS:10b PRD:0b PPROG:1b PTM:00b PA:11b  */
#define READ_OTP3_DOUT_REG_CMD                  0xA0 /* Read the OTP3 DOUT0-DOUT3 Register OTP_CS:10b PRD:1b PPROG:0b PTM:00b PA:00b*/

#define CLEAR_OTP1_DOUT0_WRITE_CMD              0x00 /* Clear the write command for OTP1 DOUT0 Register OTP_CS:00b PRD:0b PPROG:0b PTM:00b PA:00b  */
#define CLEAR_OTP1_DOUT1_WRITE_CMD              0x01 /* Clear the write command for OTP1 DOUT1 Register OTP_CS:00b PRD:0b PPROG:0b PTM:00b PA:01b  */
#define CLEAR_OTP1_DOUT2_WRITE_CMD              0x02 /* Clear the write command for OTP1 DOUT2 Register OTP_CS:00b PRD:0b PPROG:0b PTM:00b PA:10b  */
#define CLEAR_OTP1_DOUT3_WRITE_CMD              0x03 /* Clear the write command for OTP1 DOUT3 Register OTP_CS:00b PRD:0b PPROG:0b PTM:00b PA:11b  */
#define CLEAR_OTP1_DOUT_READ_CMD                0x00 /* Clear the read command for OTP1 DOUT0-DOUT3 Register OTP_CS:00b PRD:0b PPROG:0b PTM:00b PA:00b*/

#define CLEAR_OTP2_DOUT0_WRITE_CMD              0x40 /* Clear the write command for OTP2 DOUT0 Register OTP_CS:01b PRD:0b PPROG:0b PTM:00b PA:00b  */
#define CLEAR_OTP2_DOUT1_WRITE_CMD              0x41 /* Clear the write command for OTP2 DOUT1 Register OTP_CS:01b PRD:0b PPROG:0b PTM:00b PA:01b  */
#define CLEAR_OTP2_DOUT2_WRITE_CMD              0x42 /* Clear the write command for OTP2 DOUT2 Register OTP_CS:01b PRD:0b PPROG:0b PTM:00b PA:10b  */
#define CLEAR_OTP2_DOUT3_WRITE_CMD              0x43 /* Clear the write command for OTP2 DOUT3 Register OTP_CS:01b PRD:0b PPROG:0b PTM:00b PA:11b  */
#define CLEAR_OTP2_DOUT_READ_CMD                0x40 /* Clear the read command for OTP2 DOUT0-DOUT3 Register OTP_CS:01b PRD:0b PPROG:0b PTM:00b PA:00b*/

#define CLEAR_OTP3_DOUT0_WRITE_CMD              0x80 /* Clear the write command for OTP3 DOUT0 Register OTP_CS:10b PRD:0b PPROG:0b PTM:00b PA:00b  */
#define CLEAR_OTP3_DOUT1_WRITE_CMD              0x81 /* Clear the write command for OTP3 DOUT1 Register OTP_CS:10b PRD:0b PPROG:0b PTM:00b PA:01b  */
#define CLEAR_OTP3_DOUT2_WRITE_CMD              0x82 /* Clear the write command for OTP3 DOUT2 Register OTP_CS:10b PRD:0b PPROG:0b PTM:00b PA:10b  */
#define CLEAR_OTP3_DOUT3_WRITE_CMD              0x83 /* Clear the write command for OTP3 DOUT3 Register OTP_CS:10b PRD:0b PPROG:0b PTM:00b PA:11b  */
#define CLEAR_OTP3_DOUT_READ_CMD                0x40 /* Clear the read command for OTP3 DOUT0-DOUT3 Register OTP_CS:10b PRD:0b PPROG:0b PTM:00b PA:00b*/

enum
{
    eOTP1           = 0,
    eOTP2,
    eOTP_CNT
};

enum
{
    eOTP_DOUT0      = 0,
    eOTP_DOUT1,
    eOTP_DOUT2,
    eOTP_DOUT3,
    eOTP_DOUT_CNT
};

//------------------------------------------------------------------------------
typedef struct
{
    char *chName;
} S_CommonTables;

typedef struct
{
    char *chName;
    uint32_t u32CMD;
} S_ADCCMDTables;

//------------------------------------------------------------------------------
// ADC register
//------------------------------------------------------------------------------
void delay_10us(uint32_t nDelay);
void delay_1ms(uint32_t nDelay);
int8_t Reset_NADC24(void);
int32_t NADC24_Calibration_and_Initial(int8_t Selet_Cali_Init);

void SPI_WriteReg(uint8_t u8RegAddr, uint8_t u8WriteData);
uint8_t SPI_ReadReg(uint8_t u8RegAddr);

void ADC_RegisterByOR(uint8_t u8Reg, uint32_t u32Mask);
void ADC_RegisterByAND(uint8_t u8Reg, uint32_t u32Mask);

//------------------------------------------------------------------------------
// ADC Commands
//------------------------------------------------------------------------------
void SPI_Send_ADC_Commnad(uint32_t u32Cmd);

//------------------------------------------------------------------------------
// ADC OTP operation register address
//------------------------------------------------------------------------------
void SPI_OTP_Write(uint32_t u32Idx1, uint32_t u32Idx2, uint32_t i32DinValue);
void SPI_OTP_Read(void);
uint32_t SPI_Read_OTP(uint32_t u32OTPIdx, uint32_t u32DOUTIdx);
int32_t SPI_Write_OTP(uint32_t u32OTPIdx, uint32_t u32DOUTIdx, uint32_t u32OTPData);

//------------------------------------------------------------------------------
// Read ADC Data
//------------------------------------------------------------------------------
uint32_t SPI_ReadADCData(void);
uint32_t SPI_ReadADCDataWithReadCMD(void);
uint32_t SPI_SetChannel_and_ReadADCData(uint8_t u8Channel);

//------------------------------------------------------------------------------
// ADC Coefficient Data
//-----------------------------------------------------------------------------
void SPI_ReadAdcCoffData(uint8_t u8Cmd, uint8_t *pu8AdcBuf, uint8_t u8ReadCnt);
void SPI_WriteAdcCoffData(uint8_t u8Cmd, uint8_t *pu8AdcBuf, uint8_t u8WriteCnt);

uint32_t SPI_ReadCofficientData(uint8_t u8Addr);
uint32_t SPI_WriteCofficientData(uint8_t u8Addr, uint32_t u32Value);

//------------------------------------------------------------------------------
// Common function
//------------------------------------------------------------------------------
uint32_t sysGetNum(void);

#endif /* _ADC_REG_TABLE_H_ */
/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
