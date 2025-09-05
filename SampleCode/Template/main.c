/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "misc_config.h"
#include "filter.h"

#include "NADC_Driver.h"
#include "NADC_app.h"
// #include "arm_math.h"
/*_____ D E C L A R A T I O N S ____________________________________________*/

volatile struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_TIMER_PERIOD_SPECIFIC        			(flag_PROJ_CTL.bit1)
#define FLAG_PROJ_REVERSE2                 			    (flag_PROJ_CTL.bit2)
#define FLAG_PROJ_REVERSE3                              (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)


/*_____ D E F I N I T I O N S ______________________________________________*/

volatile unsigned int counter_systick = 0;
volatile uint32_t counter_tick = 0;


enum
{
	ADC0_CH0 = 0 ,
	ADC0_CH1 , 	
	ADC0_CH2 , 
	ADC0_CH3 , 
	ADC0_CH4 , 
	
	ADC0_CH5 , 
	ADC0_CH6 , 
	ADC0_CH7 , 
	ADC0_CH8 , 
	ADC0_CH9 , 
	
	ADC0_CH10 , 
	ADC0_CH11 , 
	ADC0_CH12 ,
	ADC0_CH13 , 
	ADC0_CH14 , 
	ADC0_CH15 , 
	
	ADC_CH_DEFAULT 	
}ADC_CH_TypeDef;

#define VBG_VOLTAGE                                     (1230)
// #define ADC_RESOLUTION							        ((uint16_t)(4096u))
// #define ADC_REF_VOLTAGE							        ((uint16_t)(3300u))	//(float)(3.3f)
#define ADC_CH_NUM	 							        (1)

#define ADC_MAX_TARGET							        (4095ul)	//(float)(2.612f)
#define ADC_MIN_TARGET							        (0ul)	//(float)(0.423f)

// #define ADC_SAMPLE_COUNT                                (8)
// #define ADC_SAMPLE_POWER	 				            (3)
// #define ADC_SAMPLE_DROP 						        (4ul)
// #define ADCTotalLength                                  (ADC_SAMPLE_COUNT+ADC_SAMPLE_DROP)   // drop first 4 ADC result 
// #define ADC_DIGITAL_SCALE(void) 		                (0xFFFU >> ((0) >> (3U - 1U)))		//0: 12 BIT 

uint32_t AVdd = 0;
volatile uint16_t ADC_DataArray[ADC_CH_NUM] = {0};
// volatile int16_t g_iConversionData[ADCTotalLength] = {0};

uint32_t g_u32AdcIntFlag = 0;
volatile uint16_t mv = 0;

ema_dual_t g_ema_g[ADC_CH_NUM] = {0};
/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

unsigned int get_systick(void)
{
	return (counter_systick);
}

void set_systick(unsigned int t)
{
	counter_systick = t;
}

void systick_counter(void)
{
	counter_systick++;
}

void SysTick_Handler(void)
{

    systick_counter();

    if (get_systick() >= 0xFFFFFFFF)
    {
        set_systick(0);      
    }

    // if ((get_systick() % 1000) == 0)
    // {
       
    // }

    #if defined (ENABLE_TICK_EVENT)
    TickCheckTickEvent();
    #endif    
}

void SysTick_delay(unsigned int delay)
{  
    
    unsigned int tickstart = get_systick(); 
    unsigned int wait = delay; 

    while((get_systick() - tickstart) < wait) 
    { 
    } 

}

void SysTick_enable(unsigned int ticks_per_second)
{
    set_systick(0);
    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while (1);
    }

    #if defined (ENABLE_TICK_EVENT)
    TickInitTickEvent();
    #endif
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

void delay_ms(uint16_t ms)
{
	#if 1
    uint32_t tickstart = get_tick();
    uint32_t wait = ms;
	uint32_t tmp = 0;
	
    while (1)
    {
		if (get_tick() > tickstart)	// tickstart = 59000 , tick_counter = 60000
		{
			tmp = get_tick() - tickstart;
		}
		else // tickstart = 59000 , tick_counter = 2048
		{
			tmp = 60000 -  tickstart + get_tick();
		}		
		
		if (tmp > wait)
			break;
    }
	
	#else
	TIMER_Delay(TIMER0, 1000*ms);
	#endif
}


__STATIC_INLINE uint32_t FMC_ReadBandGap(void)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;            /* Set ISP Command Code */
    FMC->ISPADDR = 0x70u;                         /* Must keep 0x70 when read Band-Gap */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;           /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                            /* To make sure ISP/CPU be Synchronized */
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) {}  /* Waiting for ISP Done */

    return FMC->ISPDAT & 0xFFF;
}

int32_t getBuiltInBandGap(void)
{
    int32_t res = 0;

    SYS_UnlockReg();
    FMC_Open();
    res = FMC_ReadBandGap();
    FMC_Close();
    SYS_LockReg();

    return res;
}

void ADC_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* Clear the A/D interrupt flag */
}

uint32_t GetAVDDCodeByADC(void)
{
    uint32_t u32Data = 0;

    // /* Power on ADC */
    // SYS_ResetModule(ADC_RST);
    ADC_POWER_ON(ADC);
    // CLK_SysTickDelay(10000);

    /* Set input mode as single-end, Single mode, and select channel 29 (band-gap voltage) */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, BIT29);

    /*
        ADC clock source -> HIRC  = 48 MHz
        ADC clock divider          = 2
        ADC clock                  = 48 MHz / 2 = 24 MHz
        ADC extended sampling time = 71
        ADC conversion time = 17 + ADC extended sampling time = 88
        ADC conversion rate = 1.5 MHz / 88 = 17 ksps
    */

    ADC_SetExtendSampleTime(ADC, 0, 71);

    /* Clear conversion finish flag */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    ADC_ENABLE_INT(ADC, ADC_ADF_INT);   // Enable sample module A/D interrupt.
    NVIC_EnableIRQ(ADC_IRQn);

    g_u32AdcIntFlag = 0;

    ADC_START_CONV(ADC);
    /* Wait ADC conversion done */
    while(g_u32AdcIntFlag == 0);

    /* Disable the A/D interrupt */
    ADC_DISABLE_INT(ADC, ADC_ADF_INT);

    /* Get the conversion result of the channel 29 */
    u32Data = ADC_GET_CONVERSION_DATA(ADC, 29);

    ADC_POWER_DOWN(ADC);   
    
    return (u32Data);
}

uint32_t GetAVDDVoltage(void)
{
    uint32_t  u32ConversionResult = 0;
    uint32_t u32MvAVDD1 = 0;
    uint32_t BuiltInData = 0;

    u32ConversionResult = GetAVDDCodeByADC();
    BuiltInData = getBuiltInBandGap();

    u32MvAVDD1 = (3072*BuiltInData) / u32ConversionResult;

    printf("Conversion result: 0x%04X\r\n", u32ConversionResult);
    printf("AVDD Voltage: %dmV\r\n", u32MvAVDD1);   

    return (uint32_t)u32MvAVDD1;
}


uint16_t ADC_InitChannel(uint8_t ch)
{
    volatile uint16_t tmp = 0;
  	volatile uint32_t sum = 0;
    uint16_t u16adc_convert_target = 0;
	volatile uint16_t adc_value = 0;

	u16adc_convert_target = (ADC_MIN_TARGET*ADC_MAX_TARGET/AVdd);

    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, 0x1 << ch);
    
    NVIC_EnableIRQ(ADC_IRQn);

    /* Clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
    ADC_ENABLE_INT(ADC, ADC_ADF_INT);  // Enable sample module A/D interrupt. //

    g_u32AdcIntFlag = 0;
    ADC_START_CONV(ADC);
    while(g_u32AdcIntFlag == 0);
    ADC_DISABLE_INT(ADC, ADC_ADF_INT);

    tmp = ADC_GET_CONVERSION_DATA(ADC, ch);		
    // adc_value = tmp; // no filter
    adc_value = (uint16_t) ema_dual_update(&g_ema_g[0] , (float)tmp);

	adc_value = (adc_value <= u16adc_convert_target) ? (u16adc_convert_target) : (adc_value); 
	adc_value = (adc_value >= ADC_MAX_TARGET) ? (ADC_MAX_TARGET) : (adc_value); 

    return (uint16_t)(adc_value);
}


uint16_t ADC_To_Voltage(uint16_t adc_value)
{
	uint32_t volt = 0;

	// volt = (uint16_t) (AVdd*adc_value)/ADC_DIGITAL_SCALE();
	// volt = (float) (AVdd*adc_value)/ADC_DIGITAL_SCALE();
	volt = (AVdd*adc_value)/ADC_MAX_TARGET;
	
	#if 0   //debug
    printf(",0x%4X(%5d,%5d),",adc_value,adc_value , volt);

	#endif

	return (uint16_t)volt;	
}

void ADC_Process(void)
{
    uint8_t ch = ADC0_CH2;

    ADC_DataArray[0] = ADC_InitChannel(ch);
    // printf("AVDD:%4dmv,",AVdd);
    mv = ADC_To_Voltage(ADC_DataArray[0]);   
    // printf("[%d:%4dmv,0x%04X],",ch,mv,mv);   
    // printf("\r\n");
}


void ADC_ConvertInit(void)
{
    /* Power on ADC module */
    SYS_ResetModule(ADC_RST);
    ADC_POWER_ON(ADC);
    CLK_SysTickDelay(10000);

    /* Set input mode as single-end, and Single mode*/
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE,(uint32_t) NULL);

    /*
        ADC clock source -> HIRC  = 48 MHz
        ADC clock divider          = 2
        ADC clock                  = 48 MHz / 2 = 24 MHz
        ADC extended sampling time = 0
        ADC conversion time = 17 + ADC extended sampling time = 17
        ADC conversion rate = 24 MHz / 17 = 1.41 Msps
    */

    SYS_UnlockReg();
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PCLK1, CLK_CLKDIV0_ADC(2));
    SYS_LockReg();

    /* Set extend sampling time based on external resistor value.*/
    ADC_SetExtendSampleTime(ADC,(uint32_t) NULL, 17);
}

/*
    M031 TRM : 
    If the internal channel for band-gap voltage is active, the maximum sampling rate will be 300k 
    SPS.
*/

void ADC_Init(void)
{
    AVdd = GetAVDDVoltage(); 
	
    ADC_ConvertInit();    
}

//Exponential Moving Average + Spike Gate
void ADC_filter_Init(void)
{
	volatile uint16_t adc_value = 0;
    uint8_t ch = ADC0_CH2;
   
    // get first adc data
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, 0x1 << ch);  // CH2
    
    NVIC_EnableIRQ(ADC_IRQn);

    /* Clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
    ADC_ENABLE_INT(ADC, ADC_ADF_INT);  // Enable sample module A/D interrupt. //

    g_u32AdcIntFlag = 0;
    ADC_START_CONV(ADC);
    while(g_u32AdcIntFlag == 0);
    ADC_DISABLE_INT(ADC, ADC_ADF_INT);

    adc_value = ADC_GET_CONVERSION_DATA(ADC, ch);

    ema_dual_init(&g_ema_g[0], 0.12f, 0.95f, 10.0f, 600.0f, 4096.0f, adc_value);
}


void GPABGH_IRQHandler(void)
{
    volatile uint32_t temp;

    /* To check if PA.4 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PA, BIT4))
    {
        GPIO_CLR_INT_FLAG(PA, BIT4);

        g_u8nDRDYFlag = 1;  
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        temp = PA->INTSRC;
        PA->INTSRC = temp;
        printf("Un-expected interrupts.\r\n");
    }
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 8-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 1MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 8, 1000000);
}

//
// check_reset_source
//
uint8_t check_reset_source(void)
{
    uint32_t src = SYS_GetResetSrc();

    SYS->RSTSTS |= 0x1FF;
    printf("Reset Source <0x%08X>\r\n", src);

    #if 1   //DEBUG , list reset source
    if (src & BIT0)
    {
        printf("0)POR Reset Flag\r\n");       
    }
    if (src & BIT1)
    {
        printf("1)NRESET Pin Reset Flag\r\n");       
    }
    if (src & BIT2)
    {
        printf("2)WDT Reset Flag\r\n");       
    }
    if (src & BIT3)
    {
        printf("3)LVR Reset Flag\r\n");       
    }
    if (src & BIT4)
    {
        printf("4)BOD Reset Flag\r\n");       
    }
    if (src & BIT5)
    {
        printf("5)System Reset Flag \r\n");       
    }
    if (src & BIT6)
    {
        printf("6)Reserved.\r\n");       
    }
    if (src & BIT7)
    {
        printf("7)CPU Reset Flag\r\n");       
    }
    if (src & BIT8)
    {
        printf("8)CPU Lockup Reset Flag\r\n");       
    }
    #endif
    
    if (src & SYS_RSTSTS_PORF_Msk) {
        SYS_ClearResetSrc(SYS_RSTSTS_PORF_Msk);
        
        printf("power on from POR\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_PINRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_PINRF_Msk);
        
        printf("power on from nRESET pin\r\n");
        return FALSE;
    } 
    else if (src & SYS_RSTSTS_WDTRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_WDTRF_Msk);
        
        printf("power on from WDT Reset\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_LVRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_LVRF_Msk);
        
        printf("power on from LVR Reset\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_BODRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_BODRF_Msk);
        
        printf("power on from BOD Reset\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_SYSRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_SYSRF_Msk);
        
        printf("power on from System Reset\r\n");
        return FALSE;
    } 
    else if (src & SYS_RSTSTS_CPURF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_CPURF_Msk);

        printf("power on from CPU reset\r\n");
        return FALSE;         
    }    
    else if (src & SYS_RSTSTS_CPULKRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_CPULKRF_Msk);
        
        printf("power on from CPU Lockup Reset\r\n");
        return FALSE;
    }   
    
    printf("power on from unhandle reset source\r\n");
    return FALSE;
}

void TMR1_IRQHandler(void)
{
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_1000MS = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}

		if ((get_tick() % 250) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_SPECIFIC = 1;
		}	

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void loop(void)
{
	// static uint32_t LOG1 = 0;
	// static uint32_t LOG2 = 0;

    if ((get_systick() % 1000) == 0)
    {
        // printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    }

    if (FLAG_PROJ_TIMER_PERIOD_1000MS)//(is_flag_set(flag_timer_period_1000ms))
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 0;//set_flag(flag_timer_period_1000ms ,DISABLE);

        // printf("%s(timer) : %4d\r\n",__FUNCTION__,LOG1++);
        PB14 ^= 1;        
    }

    if (FLAG_PROJ_TIMER_PERIOD_SPECIFIC)
    {
        FLAG_PROJ_TIMER_PERIOD_SPECIFIC = 0;
        nadc_dbg_msg_process();
    }

	if (g_u8nDRDYFlag)
	{
        g_u8nDRDYFlag = 0;

        #if defined (TEST_SINGLE_ENDED)
        nadc_single_ended_process();
        #elif defined (TEST_DIFFERENTIAL)
        nadc_differential_process();
        #endif
	}
    ADC_Process();
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		printf("press : %c\r\n" , res);
		switch(res)
		{
			case '1':
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
                SYS_UnlockReg();
				// NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
                // SYS_ResetCPU();     // Not reset I/O and peripherals
                SYS_ResetChip();    // Reset I/O and peripherals ,  BS(FMC_ISPCTL[1]) reload from CONFIG setting (CBS)	
				break;
		}
	}
}

void UART02_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART02_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	dbg_printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	dbg_printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
	dbg_printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	dbg_printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	dbg_printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	dbg_printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif	

    #if 0
    dbg_printf("FLAG_PROJ_TIMER_PERIOD_1000MS : 0x%2X\r\n",FLAG_PROJ_TIMER_PERIOD_1000MS);
    dbg_printf("FLAG_PROJ_REVERSE1 : 0x%2X\r\n",FLAG_PROJ_REVERSE1);
    dbg_printf("FLAG_PROJ_REVERSE2 : 0x%2X\r\n",FLAG_PROJ_REVERSE2);
    dbg_printf("FLAG_PROJ_REVERSE3 : 0x%2X\r\n",FLAG_PROJ_REVERSE3);
    dbg_printf("FLAG_PROJ_REVERSE4 : 0x%2X\r\n",FLAG_PROJ_REVERSE4);
    dbg_printf("FLAG_PROJ_REVERSE5 : 0x%2X\r\n",FLAG_PROJ_REVERSE5);
    dbg_printf("FLAG_PROJ_REVERSE6 : 0x%2X\r\n",FLAG_PROJ_REVERSE6);
    dbg_printf("FLAG_PROJ_REVERSE7 : 0x%2X\r\n",FLAG_PROJ_REVERSE7);
    #endif

}

void GPIO_Init (void)
{
    PB14 = 0;
    PB15 = 1;

    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) | (SYS_GPB_MFPH_PB14MFP_GPIO);
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB15MFP_Msk)) | (SYS_GPB_MFPH_PB15MFP_GPIO);

    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);	

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
//    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);	

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /***********************************/
    // CLK_EnableModuleClock(TMR0_MODULE);
  	// CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
    
	/***********************************/
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
	
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);


	/***********************************/
	/*
		SPI master SPI0 : 
        PA0(MOSI)
        PA1(MISO)
        PA2(CLK)
        PA3(SS)

        PA4 nRdy
	*/
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Setup SPI0 multi-function pins */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA2MFP_SPI0_CLK);	

    // MISO
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA1MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA1MFP_SPI0_MISO);

    // SS
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA3MFP_SPI0_SS);	
    
    /* NADC24 nDRDY Pin  */
    GPIO_SetMode(PA, BIT4, GPIO_MODE_INPUT);

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    // PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

	/***********************************/
    CLK_EnableModuleClock(ADC_MODULE);	
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PCLK1, CLK_CLKDIV0_ADC(2));

    SYS->GPB_MFPL = (SYS->GPB_MFPL &~(SYS_GPB_MFPL_PB2MFP_Msk)) \
                    | (SYS_GPB_MFPL_PB2MFP_ADC0_CH2) ;

    /* Set PB.0 ~ PB.3 to input mode */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);
    /* Disable the PB0 ~ PB3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2);

	/***********************************/
   /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	GPIO_Init();
	UART0_Init();
	TIMER1_Init();
    check_reset_source();

    SysTick_enable(1000);
    #if defined (ENABLE_TICK_EVENT)
    TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    #endif
   
	/*
		SPI master SPI0 : 
        PA0(MOSI)
        PA1(MISO)
        PA2(CLK)
        PA3(SS)
	*/
	SPI_Init();
    nadc_Init();
    nadc_read_VREF();

    #if defined (TEST_SINGLE_ENDED)
    nadc_set_channel(NADC_single_ended,0);
    #elif defined (TEST_DIFFERENTIAL)
    nadc_set_channel(NADC_differential,0);
    #endif
    SPI_Send_ADC_Commnad(ADC_START_CONVERSION_CMD);

    /*
        ADC0_CH2 : PB2
        to compare NADC result
    */
    ADC_Init();
    ADC_filter_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
