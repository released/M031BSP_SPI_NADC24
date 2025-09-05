/******************************************************************************
 * @file     NADC_Driver.c
 * @version  V1.00
 * @brief    To drive the NADC24 analog-to-digital converter by using SPI interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"
#include "NADC_Driver.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
// Delay for 10us
void delay_10us(uint32_t nDelay)
{
    uint32_t nIndex;

    for (nIndex = 0; nIndex < nDelay; nIndex++)
    {
        CLK_SysTickDelay(10);
    }
}

// Delay for 1ms
void delay_1ms(uint32_t nDelay)
{
    uint32_t nIndex;

    for (nIndex = 0; nIndex < nDelay; nIndex++)
    {
        CLK_SysTickDelay(1000);
    }
}

void Read_Voltage_Channel(void)
{
    uint8_t u8RegVal = 0xFF;

    while (SPI_ReadReg(REG_ADDR_PGA_CTRL1) != u8RegVal)
    {
        u8RegVal = (SPI_ReadReg(REG_ADDR_PGA_CTRL1) & ~(PGA_CTRL1_SEL_PGA_INP_Msk | PGA_CTRL1_SEL_PGA_INN_Msk)) |
                   (PGA_CTRL1_SEL_PGA_INN_CH_0 | PGA_CTRL1_SEL_PGA_INP_CH_1);
        SPI_WriteReg(REG_ADDR_PGA_CTRL1, u8RegVal);
    }
}

void Read_Current_Channel(void)
{
    uint8_t u8RegVal = 0xFF;

    while (SPI_ReadReg(REG_ADDR_PGA_CTRL1) != u8RegVal)
    {
        u8RegVal = (SPI_ReadReg(REG_ADDR_PGA_CTRL1) & ~(PGA_CTRL1_SEL_PGA_INP_Msk | PGA_CTRL1_SEL_PGA_INN_Msk)) |
                   (PGA_CTRL1_SEL_PGA_INN_CH_2 | PGA_CTRL1_SEL_PGA_INP_CH_3);
        SPI_WriteReg(REG_ADDR_PGA_CTRL1, u8RegVal);
    }
}

int8_t Reset_NADC24(void)
{
    uint8_t u8RegVal;

    /* Disable chip power-down */
    SPI_WriteReg(REG_ADDR_PWD_CTRL2, 0x00);

    /* Read chip ID through SPI I/F firstly */
    if ((u8RegVal = SPI_ReadReg(REG_ADDR_ADC_VER_ID)) != 0xDA)
    {
        printf("REG_ADDR_ADC_VER_ID: 0x%X\n", u8RegVal);
        return -1;
    }

    /* Command Reset */
    SPI_Send_ADC_Commnad(ADC_RESET_CMD);

    SPI_WriteReg(REG_ADDR_PWD_CTRL2, 0x00);
    SPI_WriteReg(REG_ADDR_CHOP_CTRL, 0x00);

    return 0;
}

int32_t NADC24_Calibration_and_Initial(int8_t Selet_Cali_Init)
{
    int32_t i;
    volatile uint8_t u8RegVal;
    int32_t s32AdcData = 0;
    int32_t s32AdcDataSum = 0;
    int32_t s32CalibData = 0;
    uint32_t u32AdcData = 0;
    uint32_t u32CalibData = 0;

    u8RegVal = 0xFF;

    while (SPI_ReadReg(REG_ADDR_PWD_CTRL1) != u8RegVal)
    {
        u8RegVal = (SPI_ReadReg(REG_ADDR_PWD_CTRL1) & ~(PWD_CTRL1_PWD_DAC_Msk | PWD_CTRL1_PWD_DACBUF_Msk)) |
                   (PWD_CTRL1_PWD_DAC_POWER_DOWN | PWD_CTRL1_PWD_DACBUF_POWER_DOWN);
        SPI_WriteReg(REG_ADDR_PWD_CTRL1, u8RegVal);
    }

    /* Disable PGA bandwidth buffer block & Modulator REFP buffer block */
    u8RegVal = 0xFF;

    while (SPI_ReadReg(REG_ADDR_PWD_CTRL2) != u8RegVal)
    {
        u8RegVal = (SPI_ReadReg(REG_ADDR_PWD_CTRL2) & ~(PWD_CTRL2_PWD_MOD_REFP_Msk | PWD_CTRL2_PWD_PGA_BUFF_Msk)) |
                   (PWD_CTRL2_PWD_MOD_REFP_POWER_DOWN | PWD_CTRL2_PWD_PGA_BUFF_POWER_DOWN);
        SPI_WriteReg(REG_ADDR_PWD_CTRL2, u8RegVal);
    }

    /* Enable Internal Oscillator Clock & 8.192MHz */
    u8RegVal = 0xFF;

    while (SPI_ReadReg(REG_ADDR_OSC_CTRL2) != u8RegVal)
    {
        u8RegVal = (SPI_ReadReg(REG_ADDR_OSC_CTRL2) & ~(OSC_CTRL_SEL_INT_OSC_Msk | OSC_CTRL_OSC_BIAS_Msk | OSC_CTRL_SEL_INT_OSC_F_Msk)) |
                   (OSC_CTRL_SEL_INT_OSC_Int | OSC_CTRL_OSC_BIAS_1_0X | OSC_CTRL_SEL_INT_OSC_F_8192KHz);
        SPI_WriteReg(REG_ADDR_OSC_CTRL2, u8RegVal);
    }

    /* Enable Internal Voltage 2.4V */
    u8RegVal = 0xFF;

    while (SPI_ReadReg(REG_ADDR_REF_CTRL) != u8RegVal)
    {
        u8RegVal = (SPI_ReadReg(REG_ADDR_REF_CTRL) & ~(VREF_CTRL_SEL_24V_INT_REF_Msk | VREF_CTRL_SEL_INT_REF_DRV_Msk)) |
                   (VREF_CTRL_SEL_24V_INT_REF_2_4V | VREF_CTRL_SEL_INT_REF_MEDIUM_HIGH);
        SPI_WriteReg(REG_ADDR_REF_CTRL, u8RegVal);
    }

    /* Disable FIR, OSR = 4096 */
    u8RegVal = 0xFF;

    while (SPI_ReadReg(REG_ADDR_DF1_CTRL) != u8RegVal)
    {
        u8RegVal = (SPI_ReadReg(REG_ADDR_DF1_CTRL) & ~(DF1_CTRL_OSR_SEL_Msk | DF1_CTRL_BYPASS_FIR_Msk | DF1_CTRL_DF_GAIN_Msk)) |
                   (DF1_CTRL_OSR_SEL_32768 | DF1_CTRL_BYPASS_FIR_ENABLE | DF1_CTRL_DF_GAIN_1x);
        SPI_WriteReg(REG_ADDR_DF1_CTRL, u8RegVal);
    }

    /* FIR Average 1/16 */
    u8RegVal = 0xFF;

    while (SPI_ReadReg(REG_ADDR_DF2_CTRL) != u8RegVal)
    {
        u8RegVal = (SPI_ReadReg(REG_ADDR_DF2_CTRL) & ~(DF2_CTRL_AVG_ADAP_EN_Msk | DF2_CTRL_LOW_LAT_EN_Msk | DF2_CTRL_AVG_FILT_SET_Msk | DF2_CTRL_AVG_FILT_EN_Msk)) |
                   (DF2_CTRL_AVG_ADAP_EN_ENABLE | DF2_CTRL_LOW_LAT_EN_ENABLE | DF2_CTRL_AVG_FILT_SET_AVG_16 | DF2_CTRL_AVG_FILT_EN_ENABLE);

        SPI_WriteReg(REG_ADDR_DF2_CTRL, u8RegVal);
    }

    /* ADC CLOCK = 1.024 MHz, Settling Time = 0 */
    u8RegVal = 0xFF;

    while (SPI_ReadReg(REG_ADDR_ADC_CTRL) != u8RegVal)
    {
        u8RegVal = (SPI_ReadReg(REG_ADDR_ADC_CTRL) & ~(ADC_CTRL_ADC_CLK_SET_Msk | ADC_CTRL_SETLSEL_Msk)) |
                   (ADC_CTRL_ADC_CLK_SET_1024KHz | ADC_CTRL_SETLSEL_SETTLE_9);
        SPI_WriteReg(REG_ADDR_ADC_CTRL, u8RegVal);
    }

    /* Bias 1 Setting */
    u8RegVal = 0xFF;

    while (SPI_ReadReg(REG_ADDR_BIAS1_CTRL) != u8RegVal)
    {
        u8RegVal = (SPI_ReadReg(REG_ADDR_BIAS1_CTRL) & ~SET_PGA_BIAS1_CURRENT_Msk) | (SET_PGA_BIAS1_CURRENT_1_0X);
        SPI_WriteReg(REG_ADDR_BIAS1_CTRL, u8RegVal);
    }

    /* Bias 2 Setting */
    u8RegVal = 0xFF;

    while (SPI_ReadReg(REG_ADDR_BIAS2_CTRL) != u8RegVal)
    {
        u8RegVal = (SPI_ReadReg(REG_ADDR_BIAS2_CTRL) & ~(SET_ADC_MODULATOR_OPAMP_2_BIAS_Msk | SET_ADC_MODULATOR_OPAMP_1_BIAS_Msk | SET_ADC_MODULATOR_WHOLE_BIAS_Msk | SET_ADC_BUFFER_BIAS_Msk)) |
                   (SET_ADC_MODULATOR_OPAMP_2_BIAS_1_0X) | (SET_ADC_MODULATOR_OPAMP_1_BIAS_1_0X) | (SET_ADC_MODULATOR_WHOLE_BIAS_1_0X) | (SET_ADC_BUFFER_BIAS_1_0X);
        SPI_WriteReg(REG_ADDR_BIAS2_CTRL, u8RegVal);
    }


    if (Selet_Cali_Init == 0)   /* Calibration */
    {
        /* Analog PGA Gain = 1, PGA BUF GAIN 2x, Offset Calibration Enable */
        u8RegVal = 0xFF;

        while (SPI_ReadReg(REG_ADDR_PGA_CTRL2) != u8RegVal)
        {
            u8RegVal = (SPI_ReadReg(REG_ADDR_PGA_CTRL2) & ~(PGA_CTRL2_OFFSETDIS_Msk | PGA_CTRL2_PGA_BUF_GAIN_Msk | PGA_CTRL2_PGA_GAIN_Msk | PGA_CTRL2_BUF_BYPASS_Msk | PGA_CTRL2_BUF_AUTO_EN_Msk)) |
                       (PGA_CTRL2_OFFSETDIS_ENABLE | PGA_CTRL2_PGA_GAIN_1x | PGA_CTRL2_PGA_BUF_GAIN_1x | PGA_CTRL2_BUF_BYPASS_DISABLE | PGA_CTRL2_BUF_AUTO_ENABLE);
            SPI_WriteReg(REG_ADDR_PGA_CTRL2, u8RegVal);
        }

        /* Input: INN_VCM, INP_VCM ,VREF=2.4V */
        u8RegVal = 0xFF;

        while (SPI_ReadReg(REG_ADDR_PGA_CTRL1) != u8RegVal)
        {
            u8RegVal = (SPI_ReadReg(REG_ADDR_PGA_CTRL1) & ~(PGA_CTRL1_SEL_PGA_INP_Msk | PGA_CTRL1_SEL_PGA_INN_Msk)) |
                       (PGA_CTRL1_SEL_PGA_INN_VCM | PGA_CTRL1_SEL_PGA_INP_VCM);
            SPI_WriteReg(REG_ADDR_PGA_CTRL1, u8RegVal);
        }

        /* Read current adc value */
				SPI_Send_ADC_Commnad(ADC_START_CONVERSION_CMD);

				s32AdcDataSum = 0;
        for (i = 0; i < 4; i++)
        {
            while (nDRDY == 1) ;
            u32AdcData = SPI_ReadADCDataWithReadCMD();

            if (u32AdcData & 0x800000)
                u32AdcData += 0xFF000000;

            s32AdcData = (int32_t)u32AdcData;
            s32AdcDataSum += s32AdcData;
        }

        s32CalibData = s32AdcDataSum / 4;

        if (DEBUG_ENABLE) printf("     Before Calibration : %d\n", s32CalibData);
				
				/* Stop conversion */
				SPI_Send_ADC_Commnad(ADC_STOP_CONVERSION_CMD);


        /* Write CalibData to REG_ADDR_ADC_OFFSET_CAL1/2/3 */
        u8RegVal = 0xFF;

        while (SPI_ReadReg(REG_ADDR_ADC_OFFSET_CAL1) != u8RegVal)
        {
            u8RegVal = (uint8_t)(s32CalibData >> 0);
            SPI_WriteReg(REG_ADDR_ADC_OFFSET_CAL1, u8RegVal);
        }

        u8RegVal = 0xFF;

        while (SPI_ReadReg(REG_ADDR_ADC_OFFSET_CAL2) != u8RegVal)
        {
            u8RegVal = (uint8_t)(s32CalibData >> 8);
            SPI_WriteReg(REG_ADDR_ADC_OFFSET_CAL2, u8RegVal);
        }

        u8RegVal = 0xFF;

        while (SPI_ReadReg(REG_ADDR_ADC_OFFSET_CAL3) != u8RegVal)
        {
            u8RegVal = (uint8_t)(s32CalibData >> 16);
            SPI_WriteReg(REG_ADDR_ADC_OFFSET_CAL3, u8RegVal);
        }

        /* Read current adc value */
				SPI_Send_ADC_Commnad(ADC_START_CONVERSION_CMD);

				s32AdcDataSum = 0;
        for (i = 0; i < 4; i++)
        {
            while (nDRDY == 1) ;
            u32AdcData = SPI_ReadADCDataWithReadCMD();

            if (u32AdcData & 0x800000)
                u32AdcData += 0xFF000000;

            s32AdcData = (int32_t)u32AdcData;
            s32AdcDataSum += s32AdcData;
        }

        s32CalibData = s32AdcDataSum / 4;

        if (DEBUG_ENABLE) printf("      After Calibration : %d\n", s32CalibData);
				
				/* Stop conversion */
				SPI_Send_ADC_Commnad(ADC_STOP_CONVERSION_CMD);

        /* Check ADC data */
        if ((s32CalibData > 500) || (s32CalibData < -500))
        {
            printf("[%s] Invalid u32AdcData: 0x%X\n", __func__, u32AdcData);
            return -1;
        }

        return u32CalibData;
    }
    else        /* Initialize */
    {
        // Input: N_0, P_1 ,VREF=2.4V
        u8RegVal = 0xFF;

        while (SPI_ReadReg(REG_ADDR_PGA_CTRL1) != u8RegVal)
        {
            u8RegVal = (SPI_ReadReg(REG_ADDR_PGA_CTRL1) & ~(PGA_CTRL1_SEL_PGA_INP_Msk | PGA_CTRL1_SEL_PGA_INN_Msk)) |
                       (PGA_CTRL1_SEL_PGA_INN_CH_0 | PGA_CTRL1_SEL_PGA_INP_CH_1);
            SPI_WriteReg(REG_ADDR_PGA_CTRL1, u8RegVal);
        }

        return 0;
    }
}


uint8_t SPI_TransferByte(uint8_t u8data)
{
    while (SPI_GET_TX_FIFO_FULL_FLAG(SPI_PORT));

    SPI_WRITE_TX(SPI_PORT, u8data);

    while (SPI_IS_BUSY(SPI_PORT));

    return SPI_READ_RX(SPI_PORT);
}

void SPI_WriteReg(uint8_t u8RegAddr, uint8_t u8WriteData)
{
    uint8_t u8AdcCmd = ((u8RegAddr & 0x1F) | 0x60);

    SPI_SET_SS_LOW(SPI_PORT);

    SPI_TransferByte(u8AdcCmd);
    SPI_TransferByte(0x00);

    SPI_TransferByte(u8WriteData);
    SPI_TransferByte(0x00);

    SPI_SET_SS_HIGH(SPI_PORT);
}

uint8_t SPI_ReadReg(uint8_t u8RegAddr)
{
    uint8_t u8ReadValue = 0;
    uint8_t u8AdcCmd = ((u8RegAddr & 0x1F) | 0x40);

    SPI_SET_SS_LOW(SPI_PORT);

    SPI_TransferByte(u8AdcCmd);
    SPI_TransferByte(0x00);

    u8ReadValue = SPI_TransferByte(0x0);
    SPI_TransferByte(0x00);

    SPI_SET_SS_HIGH(SPI_PORT);

    return (u8ReadValue & 0xFF);
}

void ADC_RegisterByOR(uint8_t u8Reg, uint32_t u32Mask)
{
    uint8_t u8RegValue = 0;

    u8RegValue = SPI_ReadReg(u8Reg);
    u8RegValue |= u32Mask;
    SPI_WriteReg(u8Reg, u8RegValue);
}

void ADC_RegisterByAND(uint8_t u8Reg, uint32_t u32Mask)
{
    uint8_t u8RegValue = 0;

    u8RegValue = SPI_ReadReg(u8Reg);
    u8RegValue &= u32Mask;
    SPI_WriteReg(u8Reg, u8RegValue);
}

uint32_t SPI_ReadADCDataWithReadCMD(void)
{
    uint32_t u32AdcValue = 0;

    SPI_SET_SS_LOW(SPI_PORT);

    SPI_TransferByte(ADC_READ_DATA_CMD);

    SPI_PORT->CTL &= (~SPI_CTL_DWIDTH_Msk);
    SPI_PORT->CTL |= (24 << SPI_CTL_DWIDTH_Pos);

    /* receive 24-bit */
    SPI_WRITE_TX(SPI_PORT, (uint32_t)0x00000000);

    while (SPI_IS_BUSY(SPI_PORT));

    u32AdcValue = SPI_READ_RX(SPI_PORT);

    SPI_SET_SS_HIGH(SPI_PORT);

    SPI_PORT->CTL &= (~SPI_CTL_DWIDTH_Msk);
    SPI_PORT->CTL |= (8 << SPI_CTL_DWIDTH_Pos);

    return u32AdcValue;
}

uint32_t SPI_ReadADCData(void)
{
    uint32_t u32AdcValue = 0;

    SPI_PORT->CTL &= (~SPI_CTL_DWIDTH_Msk);
    SPI_PORT->CTL |= (24 << SPI_CTL_DWIDTH_Pos);

    SPI_SET_SS_LOW(SPI_PORT);

    /* receive 24-bit */
    SPI_WRITE_TX(SPI_PORT, (uint32_t)0x00000000);

    while (SPI_IS_BUSY(SPI_PORT));

    u32AdcValue = SPI_READ_RX(SPI_PORT);

    SPI_SET_SS_HIGH(SPI_PORT);

    SPI_PORT->CTL &= (~SPI_CTL_DWIDTH_Msk);
    SPI_PORT->CTL |= (8 << SPI_CTL_DWIDTH_Pos);

    return u32AdcValue;
}

uint32_t SPI_SetChannel_and_ReadADCData(uint8_t u8Channel)
{
    uint32_t u32AdcValue = 0;

    SPI_SET_SS_LOW(SPI_PORT);

    /*
        WRITE_REG  Write nnnnn registers from address aaaaa  0b 011a aaaa     
    */
    SPI_TransferByte(REG_ADDR_PGA_CTRL1 | 0x60);
    SPI_TransferByte(0x00);

    SPI_TransferByte(u8Channel);

    SPI_TransferByte(ADC_START_CONVERSION_CMD);

    /* Read */
    SPI_PORT->CTL &= (~SPI_CTL_DWIDTH_Msk);
    SPI_PORT->CTL |= (24 << SPI_CTL_DWIDTH_Pos);

    /* receive 24-bit */
    SPI_WRITE_TX(SPI_PORT, (uint32_t)0x00000000);

    while (SPI_IS_BUSY(SPI_PORT));

    u32AdcValue = SPI_READ_RX(SPI_PORT);

    SPI_SET_SS_HIGH(SPI_PORT);

    SPI_PORT->CTL &= (~SPI_CTL_DWIDTH_Msk);
    SPI_PORT->CTL |= (8 << SPI_CTL_DWIDTH_Pos);

    return u32AdcValue;
}

void SPI_Send_ADC_Commnad(uint32_t u32Cmd)
{
    SPI_SET_SS_LOW(SPI_PORT);

    SPI_TransferByte((uint8_t)(u32Cmd & 0xFF));

    SPI_SET_SS_HIGH(SPI_PORT);
}
/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

