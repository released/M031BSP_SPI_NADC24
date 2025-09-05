#ifndef __FILTER_H__
#define __FILTER_H__

/*_____ I N C L U D E S ____________________________________________________*/

/*_____ D E C L A R A T I O N S ____________________________________________*/

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

typedef struct {
    float y;
    float alpha_slow;     // for small noise
    float alpha_fast;     // for real steps
    float th_fast;        // |dx| > th_fast ⇒ use alpha_fast
    float th_spike;       // |dx| > th_spike ⇒ clamp by max_step
    float max_step;       // clamp cap when beyond th_spike
} ema_dual_t;


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void ema_dual_init(ema_dual_t* f,
                                 float a_slow, float a_fast,
                                 float th_fast, float th_spike,
                                 float max_step, float y0);
float ema_dual_update(ema_dual_t* f, float x);


#endif //__FILTER_H__
