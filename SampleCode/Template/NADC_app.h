#ifndef __NADC_APP_H__
#define __NADC_APP_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "NuMicro.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/
#define TEST_SINGLE_ENDED
// #define TEST_DIFFERENTIAL

/*_____ D E F I N I T I O N S ______________________________________________*/

/*  
	template
	typedef struct _peripheral_manager_t
	{
		uint8_t u8Cmd;
		uint8_t au8Buf[33];
		uint8_t u8RecCnt;
		uint8_t bByPass;
		uint16_t* pu16Far;
	}PERIPHERAL_MANAGER_T;

	volatile PERIPHERAL_MANAGER_T g_PeripheralManager = 
	{
		.u8Cmd = 0,
		.au8Buf = {0},		//.au8Buf = {100U, 200U},
		.u8RecCnt = 0,
		.bByPass = FALSE,
		.pu16Far = NULL,	//.pu16Far = 0	
	};
	extern volatile PERIPHERAL_MANAGER_T g_PeripheralManager;
*/


// SPI master
#define SPI_TARGET_FREQ					                (1000000ul)

#define CALI_NADC24                                     0x00
#define INIT_NADC24                                     0x01
#define VOLTAGE_CHANNEL                                 0xE0    //CH0/1
#define CURRENT_CHANNEL                                 0x24    //CH2/3
#define VCOM_CHANNEL                     		        0xFC

#define PGA_CTRL1_INN(v)                                (((v) >> PGA_CTRL1_SEL_PGA_INN_Pos) & 0x7u)
#define PGA_CTRL1_INP(v)                                (((v) >> PGA_CTRL1_SEL_PGA_INP_Pos) & 0x7u)

#define TRAGET_ADC_NUM                                  (4)
#define TRAGET_WAIVE_CNT                                (6)

enum
{
    NADC_single_ended , 
    NADC_differential ,
};

enum
{
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
};

// static uint32_t u32AdcData = 0;
// static int32_t  s32AdcData = 0;

typedef struct _nadc24_manager_t
{
    float VREF;
    float BUF;
    float PGA;
    float VREF_eff;
    float VCM_eff;
    float VCH[8];
}NADC24_MANAGER_T;


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
extern uint32_t AVdd;
extern volatile uint16_t mv;

extern uint8_t g_u8nDRDYFlag;
extern uint8_t SPI_TransferByte(uint8_t u8data);


void nadc_set_channel(uint8_t is_single_ended , uint8_t ch);
int32_t nadc_readADCdata(void);
float nadc_convert_single_ended_ch_to_voltage(int32_t s32AdcData , uint8_t* ch_from_reg);
float nadc_convert_differential_ch_to_voltage(int32_t s32AdcData);

void nadc_single_ended_process(void);
void nadc_differential_process(void);


void nadc_dbg_msg_process(void);
void nadc_read_VREF(void);
void nadc_Init(void);

#endif //__NADC_APP_H__
