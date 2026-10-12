// Host stand-in for stm32f303/inc/periph.h: TIM8 is a struct the plant reads.
#pragma once
#include <stdint.h>
#include "sim_hw.h"
#define TIM8 (&sim_tim8)
#define LL_TIM_SetAutoReload(t, v) ((t)->ARR = (v))
#define LL_TIM_GetAutoReload(t) ((t)->ARR)
