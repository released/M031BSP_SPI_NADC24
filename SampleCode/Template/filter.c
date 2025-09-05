/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <math.h>
#include "filter.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
// Ignore or limit updates when a sample jumps too far in one step.

void ema_dual_init(ema_dual_t* f,
                                 float a_slow, float a_fast,
                                 float th_fast, float th_spike,
                                 float max_step, float y0)
{
    f->y=y0; f->alpha_slow=a_slow; f->alpha_fast=a_fast;
    f->th_fast=th_fast; f->th_spike=th_spike; f->max_step=max_step;
}

float ema_dual_update(ema_dual_t* f, float x)
{
    float dx = x - f->y, a = f->alpha_slow;
    if (fabsf(dx) > f->th_fast) a = f->alpha_fast;     // speed up on real edges
    dx *= a;
    // soft clamp only for extreme spikes
    if (dx >  f->max_step) dx =  f->max_step;
    if (dx < -f->max_step) dx = -f->max_step;
    f->y += dx;
    return f->y;
}

