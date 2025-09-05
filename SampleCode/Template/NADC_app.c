/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "NuMicro.h"

#include "misc_config.h"
#include "NADC_Driver.h"
#include "NADC_app.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/



volatile NADC24_MANAGER_T g_NADC24Manager = 
{
    .VREF = 0.0f,
    .BUF = 0.0f,
    .PGA = 0.0f,
    .VREF_eff = 0.0f,
    .VCM_eff = 0.0f,
    .VCH = {0},
};

uint8_t g_u8nDRDYFlag = 0;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

float __nadc_decode_PGA_GAIN(uint8_t pga_ctrl2)
{
    unsigned g = (pga_ctrl2 >> PGA_CTRL2_PGA_GAIN_Pos) & 0x7u;  // bits [3:1]
    switch (g) 
    {
        case 0:  
            return 1.0f;
        case 1:  
            return 2.0f;
        case 2:  
            return 4.0f;
        case 3:  
            return 8.0f;
        case 4:  
            return 16.0f;
        case 5:  
            return 32.0f;
        default: 
            return 64.0f;   // 110/111 → 64x
    }
}

float __nadc_decode_PGA_BUF_GAIN(uint8_t pga_ctrl2)
{
    if (pga_ctrl2 & (1u << PGA_CTRL2_BUF_BYPASS_Pos))
        return 1.0f;

    // [4]=0 → 1x, =1 → 2x
    return (pga_ctrl2 & (1u << PGA_CTRL2_PGA_BUF_GAIN_Pos)) ? 2.0f : 1.0f;
}


float __nadc_decode_INT_REF(uint8_t vref_ctrl)
{
    // [0]=0 → 1.2f, =1 → 2.4f
    return (vref_ctrl & (1u << VREF_CTRL_SEL_24V_INT_REF_Pos)) ? 2.4f : 1.2f;
}

int32_t __nadc_SignExtend24(uint32_t raw24)
{    
    if (raw24 & 0x800000) 
        raw24 |= 0xFF000000u;
    return (int32_t)raw24;
}

/*
    ADC raw data = Vin * PGA(PGA_CTRL2_PGA_GAIN_32x) /2.4(Vref) * 2^23
*/
float __nadc_code_to_vin(int32_t code24, float vref_volt, float pga_gain)
{
    return  (code24/8388608.0f) * (vref_volt / pga_gain);
}

void __nadc_set_channel(uint8_t ch_shift)
{
    uint8_t u8RegVal = 0xFF;

    while (SPI_ReadReg(REG_ADDR_PGA_CTRL1) != u8RegVal)
    {
        u8RegVal = (SPI_ReadReg(REG_ADDR_PGA_CTRL1) & ~(PGA_CTRL1_SEL_PGA_INP_Msk | PGA_CTRL1_SEL_PGA_INN_Msk)) |
                   (ch_shift);
        SPI_WriteReg(REG_ADDR_PGA_CTRL1, u8RegVal);
    }
}

/*
    // single end    
        NADC_CH0 = (PGA_CTRL1_SEL_PGA_INN_CH_0|PGA_CTRL1_SEL_PGA_INP_VCM),          //INN=CHx, INP=VCM , neg
        NADC_CH1 = (PGA_CTRL1_SEL_PGA_INN_VCM|PGA_CTRL1_SEL_PGA_INP_CH_1),          //INN=VCM, INP=CHx , pos
        NADC_CH2 = (PGA_CTRL1_SEL_PGA_INN_CH_2|PGA_CTRL1_SEL_PGA_INP_VCM),          //INN=CHx, INP=VCM , neg
        NADC_CH3 = (PGA_CTRL1_SEL_PGA_INN_VCM|PGA_CTRL1_SEL_PGA_INP_CH_3),          //INN=VCM, INP=CHx , pos
        NADC_CH4 = (PGA_CTRL1_SEL_PGA_INN_CH_4|PGA_CTRL1_SEL_PGA_INP_VCM),          //INN=CHx, INP=VCM , neg
        NADC_CH5 = (PGA_CTRL1_SEL_PGA_INN_VCM|PGA_CTRL1_SEL_PGA_INP_CH_5),          //INN=VCM, INP=CHx , pos
        NADC_CH6 = (PGA_CTRL1_SEL_PGA_INN_CH_6|PGA_CTRL1_SEL_PGA_INP_VCM),          //INN=CHx, INP=VCM , neg
        NADC_CH7 = (PGA_CTRL1_SEL_PGA_INN_VCM|PGA_CTRL1_SEL_PGA_INP_CH_7),          //INN=VCM, INP=CHx , pos

    // differential
        NADC_CH0_1 = (PGA_CTRL1_SEL_PGA_INN_CH_0|PGA_CTRL1_SEL_PGA_INP_CH_1) ,      // 000 000 00
        NADC_CH2_3 = (PGA_CTRL1_SEL_PGA_INN_CH_2|PGA_CTRL1_SEL_PGA_INP_CH_3) ,      // 001 001 00
        NADC_CH4_5 = (PGA_CTRL1_SEL_PGA_INN_CH_4|PGA_CTRL1_SEL_PGA_INP_CH_5) ,      // 010 010 00
        NADC_CH6_7 = (PGA_CTRL1_SEL_PGA_INN_CH_6|PGA_CTRL1_SEL_PGA_INP_CH_7) ,      // 011 011 00

*/

// *is_pos：1=pos (INP=CHx,INN=VCM → vch = VCM + vdiff)
//          0=neg (INP=VCM,INN=CHx → vch = VCM - vdiff)
int8_t __nadc_decode_single_ended(uint8_t pga_ctrl1, int *is_pos)
{

    uint8_t inn = PGA_CTRL1_INN(pga_ctrl1);
    uint8_t inp = PGA_CTRL1_INP(pga_ctrl1);

    // 1) ODD（CH1/3/5/7）→ INP=CHx, INN=VCM
    if (inn == 0x7u) 
    {
        switch (inp) 
        {
            case 0:  
                if (is_pos) 
                    *is_pos = 1; 
                return 1; // CH1
            case 1:  
                if (is_pos) 
                    *is_pos = 1; 
                return 3; // CH3
            case 2:  
                if (is_pos) 
                    *is_pos = 1; 
                return 5; // CH5
            case 3:  
                if (is_pos) 
                    *is_pos = 1; 
                return 7; // CH7
            default: 
                break;
        }
    }

    // 2) EVEN（CH0/2/4/6）→ INP=VCM, INN=CHx
    if (inp == 0x7u) 
    {
        switch (inn) 
        {
            case 0:  
                if (is_pos) 
                    *is_pos = 0; 
                return 0; // CH0
            case 1:  
                if (is_pos) 
                    *is_pos = 0; 
                return 2; // CH2
            case 2:  
                if (is_pos) 
                    *is_pos = 0;
                return 4; // CH4
            case 3:  
                if (is_pos) 
                    *is_pos = 0; 
                return 6; // CH6
            default: 
                break;
        }
    }

    // NOT single end
    return -1;
}

uint8_t __nadc_encode_single_ended(uint8_t ch)
{
    uint8_t inp_code = 0;
    uint8_t inn_code = 0;
    uint8_t ch_shift = 0;
    
    if (ch & 1u) 
    {
        // ODD：INP=CHx( 1→0,3→1,5→2,7→3)，INN=VCM(7)
        inp_code = (uint8_t)((ch-1u)/2u); // 1->0,3->1,5->2,7->3
        ch_shift |= (0x7u<<PGA_CTRL1_SEL_PGA_INN_Pos) | (inp_code<<PGA_CTRL1_SEL_PGA_INP_Pos);
    } 
    else 
    {
        // EVEN：INN=CHx( 0→0,2→1,4→2,6→3)，INP=VCM(7)
        inn_code = (uint8_t)(ch/2u);      // 0->0,2->1,4->2,6->3
        ch_shift |= (inn_code<<PGA_CTRL1_SEL_PGA_INN_Pos) | (0x7u<<PGA_CTRL1_SEL_PGA_INP_Pos);
    }
    return ch_shift;
}

/*
    Set channel
    Convert ADC

    wait flag / IRQ
    get data
    convert to voltage
*/

void nadc_set_channel(uint8_t is_single_ended , uint8_t ch)
{
    uint8_t ch_shift = 0;

    if (is_single_ended == NADC_single_ended)
    {
        ch_shift = __nadc_encode_single_ended(ch);
    }
    else
    {
        // use NADC_CH0_1 , NADC_CH2_3 , NADC_CH4_5 , NADC_CH6_7
        switch(ch)
        {
            case 0:
                ch_shift = NADC_CH0_1;
                break;
            case 1:
                ch_shift = NADC_CH2_3;
                break;
            case 2:
                ch_shift = NADC_CH4_5;
                break;
            case 3:
                ch_shift = NADC_CH6_7;
                break;
        }
    }
    // printf(",ch(%d):0x%02X",ch,ch_shift);   

    __nadc_set_channel(ch_shift);  

    SPI_Send_ADC_Commnad(ADC_START_CONVERSION_CMD);
}

int32_t nadc_readADCdata(void)
{
    uint32_t raw24 = 0;
    
    while (nDRDY == 1) ;

    // raw24 = SPI_ReadADCData();
    raw24 = SPI_ReadADCDataWithReadCMD();

    SPI_Send_ADC_Commnad(ADC_STOP_CONVERSION_CMD);

    return __nadc_SignExtend24(raw24);
}

float nadc_convert_single_ended_ch_to_voltage(int32_t s32AdcData , uint8_t* ch_from_reg)
{
    volatile uint8_t pga_ctrl1 = 0;
    int is_pos = 0;
    volatile float vch = 0.0f;

    pga_ctrl1 = SPI_ReadReg(REG_ADDR_PGA_CTRL1);
    *ch_from_reg = __nadc_decode_single_ended(pga_ctrl1,&is_pos);

    if (is_pos) // CH1/3/5/7
    {
        vch = g_NADC24Manager.VCM_eff + __nadc_code_to_vin(s32AdcData, g_NADC24Manager.VREF_eff, g_NADC24Manager.PGA);
        vch *= 1000.0f; // change to V
    }
    else        // CH0/2/4/6
    {
        vch = g_NADC24Manager.VCM_eff - __nadc_code_to_vin(s32AdcData, g_NADC24Manager.VREF_eff, g_NADC24Manager.PGA);
        vch *= 1000.0f; // change to V
    }

    // printf(">>[%d]vch=%3.3f",*ch_from_reg,vch);
    // printf("\r\n");

    return vch;
}


float nadc_convert_differential_ch_to_voltage(int32_t s32AdcData)
{
    // float vdiff = 0.0f;
    volatile float vch = 0.0f;

    vch = __nadc_code_to_vin(s32AdcData,g_NADC24Manager.VREF,g_NADC24Manager.PGA);                 //INN_CH_0 : GROUND , measure INP_CH_1
    vch *= 1000.0f; // change to V

    // printf(",[%d]vch=%3.3f",ch_from_reg,vch);
    
    return vch;
}

void nadc_single_ended_process(void)
{
    volatile int32_t s32AdcData1 = 0;
    uint8_t ch_current = 0;
    const uint8_t target_ch_num = TRAGET_ADC_NUM;   // max : 8

    static uint8_t wavie_cnt = TRAGET_WAIVE_CNT;
    static uint8_t ch_cnt = 0;
    volatile float vch = 0.0f;
    // float vdiff = 0.0f;

    /*
        step 0 : At least 3~5 conversions must be waived for output data settled after channel changed
    */ 
    wavie_cnt--;
    if (wavie_cnt != 0)
    {
        return;
    }
    else    // data vaild after first few data 
    {
        wavie_cnt = TRAGET_WAIVE_CNT;
    }

    // step 1 : get previous nadc channel data , when nDRDY active (low)
    s32AdcData1 = nadc_readADCdata();

    // step 2 : conver to voltage
    vch = nadc_convert_single_ended_ch_to_voltage(s32AdcData1,&ch_current);

    // step 3 : store data into array
    g_NADC24Manager.VCH[ch_current] = vch;
    // printf(",[0x%02X]%3.3f",ch_current,g_NADC24Manager.VCH[ch_current]);

    // step 4 : debug message
    // printf(",VREF_eff=%.3fV,VCM_eff=%.3fV",g_NADC24Manager.VREF_eff, g_NADC24Manager.VCM_eff);

    // vdiff = nadc_code_to_vin(s32AdcData, g_NADC24Manager.VREF_eff, g_NADC24Manager.PGA);            
    // printf(",vdiff=%3.3fmv",vdiff*1000.0f);

    // last step : set next channel and start convert
    ch_cnt = (ch_cnt >= (target_ch_num-1)) ? (0) : (ch_cnt+1) ;
    nadc_set_channel(NADC_single_ended,ch_cnt);

    // printf("\r\n\r\n");
}

/*
    connect CH0/2/4/6 or CH1/3/5/7 to target voltage
    and another channel connect to GROUND
    ex : 
    #1
    CH0 to measure taget voltage
    CH1 to GROUND
    #2
    CH1 to measure taget voltage
    CH0 to GROUND

*/
void nadc_differential_process(void)
{
    volatile int32_t s32AdcData1 = 0;
    const uint8_t target_ch_num = 4;    // only group 0 ~ 3

    static uint8_t wavie_cnt = TRAGET_WAIVE_CNT;
    static uint8_t ch_cnt = 0;
    volatile float vch = 0.0f;
    // float vdiff = 0.0f;

    /*
        step 0 : At least 3~5 conversions must be waived for output data settled after channel changed
    */ 
    wavie_cnt--;
    if (wavie_cnt != 0)
    {
        return;
    }
    else    // data vaild after first few data 
    {
        wavie_cnt = TRAGET_WAIVE_CNT;
    }

    // step 1 : get previous nadc channel data , when nDRDY active (low)
    s32AdcData1 = nadc_readADCdata();

    // step 2 : conver to voltage
    vch = nadc_convert_differential_ch_to_voltage(s32AdcData1);

    // step 3 : store data into array
    g_NADC24Manager.VCH[ch_cnt] = vch;
    // printf(",[0x%02X]%3.3f",ch_current,g_NADC24Manager.VCH[ch_current]);

    // step 4 : debug message
    // printf(",VREF_eff=%.3fV,VCM_eff=%.3fV",g_NADC24Manager.VREF_eff, g_NADC24Manager.VCM_eff);

    // vdiff = nadc_code_to_vin(s32AdcData, g_NADC24Manager.VREF_eff, g_NADC24Manager.PGA);            
    // printf(",vdiff=%3.3fmv",vdiff*1000.0f);

    // last step : set next channel and start convert

    ch_cnt = (ch_cnt >= (target_ch_num-1)) ? (0) : (ch_cnt+1) ;
    nadc_set_channel(NADC_differential,ch_cnt);

    // printf("\r\n\r\n");
}

void nadc_dbg_msg_process(void)
{
    uint8_t i = 0;

    printf("[%4dmv",AVdd); 
    printf(",%4dmv]",mv);      
    for(i = 0; i < 4 ; i++)
    {
        printf("[%d]%3.3f,",i,g_NADC24Manager.VCH[i]);
    }

    // printf("\r\n\r\n");
    printf("\r\n");
}

void nadc_read_VREF(void)
{
    uint8_t adc_ctrl = 0;
    uint8_t vref_ctrl = 0;
    uint8_t pga1 = 0;
    uint8_t pga2 = 0;

    adc_ctrl    = SPI_ReadReg(REG_ADDR_ADC_CTRL);
    vref_ctrl   = SPI_ReadReg(REG_ADDR_REF_CTRL);     // 0xA7
    pga1        = SPI_ReadReg(REG_ADDR_PGA_CTRL1);    // 0x00 , will change to CH index after setting 
    pga2        = SPI_ReadReg(REG_ADDR_PGA_CTRL2);    // 0x40

    g_NADC24Manager.VREF = __nadc_decode_INT_REF(vref_ctrl);
    g_NADC24Manager.BUF = __nadc_decode_PGA_BUF_GAIN(pga2);
    g_NADC24Manager.PGA = __nadc_decode_PGA_GAIN(pga2);
    g_NADC24Manager.VREF_eff = g_NADC24Manager.VREF * g_NADC24Manager.BUF;
    g_NADC24Manager.VCM_eff = g_NADC24Manager.VREF_eff * 0.5f;

    printf("adc_ctrl:0x%02X\r\n",adc_ctrl);
    printf("vref_ctrl:0x%02X\r\n",vref_ctrl);
    printf("pga1:0x%02X\r\n",pga1);
    printf("pga2:0x%02X\r\n",pga2);

    printf("VREF:%3.3f\r\n",g_NADC24Manager.VREF);
    printf("BUF:%3.3f\r\n",g_NADC24Manager.BUF);
    printf("PGA:%3.3f\r\n",g_NADC24Manager.PGA);
    printf("VREF_eff:%3.3f\r\n",g_NADC24Manager.VREF_eff);
    printf("VCM_eff:%3.3f\r\n",g_NADC24Manager.VCM_eff);
}


//nDRDY : ADC 24-bit data ready (low active) 
void nadc_Init(void)
{
    uint8_t resp = 0;

    /* Initial NADC24 */
    resp = Reset_NADC24();                                 // RESET
    printf("Reset_NADC24:0x%02X\r\n",resp);
    resp = NADC24_Calibration_and_Initial(CALI_NADC24);    // CALIBRATION
    printf("CALIBRATION:0x%02X\r\n",resp);
    resp = NADC24_Calibration_and_Initial(INIT_NADC24);    // INITIAL
    printf("INITIAL:0x%02X\r\n",resp);

    /* IO Interrupt */
    GPIO_SetMode(PA, BIT4, GPIO_MODE_INPUT);
    GPIO_EnableInt(PA, 4, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPIO_PAPBPGPH_IRQn);
}



