/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013-2017 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2013-2017 Nico Stute <crinq@crinq.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "f3hw.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim8;

/* TIM8 init function */
void MX_TIM8_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance           = TIM8;
  htim8.Init.Prescaler     = 0;
  htim8.Init.CounterMode   = TIM_COUNTERMODE_CENTERALIGNED3;
  htim8.Init.Period        = PWM_RES;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  // hv0 rewrites ARR every tick (phase lock). Preloaded, it changes at the
  // next UEV together with the compares; without it a smaller ARR written
  // after the counter passed it would miss the overflow (RM0316 20.3.1).
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
#ifdef PWM_INVERT
  htim8.Init.RepetitionCounter = 1;
#else
  htim8.Init.RepetitionCounter = 0;
#endif
  if(HAL_TIM_Base_Init(&htim8) != HAL_OK) {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if(HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }

  if(HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger  = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
  if(HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if(HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }

  if(HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }

  if(HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }

  // RM0316 20.3.16: "BRK2 must only be used with OSSR = OSSI = 1". With OSSI
  // the outputs are driven to their idle level (OISx = OISxN = 0: all
  // switches off) when MOE drops, instead of being handed to the GPIOs.
  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = PWM_DEADTIME;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
  // 0xC: fDTS/16, N = 8, 0.89 us (0xF was 1.78 us). The IM06B50GC1 allows
  // about 3 us from overcurrent to off, 1.43 us of it inside the module
  // after ITRIP. On X (C38 3.3 uF, sense caps) 0xC trips 3-5 A earlier than
  // 0xF at the same hv0.dac from switching ringing, so the dac goes up: 0xC
  // at 255 trips where 0xF did at 215 (19.5-21 A); 0xA tripped at 1 A at 150.
  sBreakDeadTimeConfig.BreakFilter      = 0xc;
  // BRK2 carries COMP1 and COMP2, the W and U overcurrent comparators (main.c).
  // BK2E follows in a second write below: RM0316 20.3.16 forbids setting BK2P
  // and BK2E in one TIMx_BDTR write
  sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter     = 0xc;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  if(HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  TIM8->BDTR |= TIM_BDTR_BK2E;
  __DSB();  // BK2E takes an APB cycle to act
  // LOCK level 1 (RM0316 20.3.16/20.4.21): DTG, BKE/BKP/BKF, BK2E/BK2P/BK2F,
  // AOE and OISx are read-only until reset, so no read-modify-write of BDTR
  // can change the break setup. MOE, which io0 toggles, stays writable.
  TIM8->BDTR |= TIM_BDTR_LOCK_0;
  for(volatile int i = 0; i < 4; i++) {  // "wait 4 timer clocks" before B2IF
  }
  // Clears only what the setup itself raised: the comparators come up later
  // with the DAC at 0 V and set BIF/B2IF again; the enable path handles that.
  TIM8->SR = ~(TIM_SR_BIF | TIM_SR_B2IF);

  HAL_TIM_MspPostInit(&htim8);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle) {
  if(tim_baseHandle->Instance == TIM8) {
    /* USER CODE BEGIN TIM8_MspInit 0 */

    /* USER CODE END TIM8_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();
    // the break interrupt must preempt the rt (TIM8_UP), so it gets the
    // highest priority and the rt moves one level down
    HAL_NVIC_SetPriority(TIM8_BRK_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM8_BRK_IRQn);
    HAL_NVIC_SetPriority(TIM8_UP_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);
    /* USER CODE BEGIN TIM8_MspInit 1 */

    /* USER CODE END TIM8_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if(timHandle->Instance == TIM8) {
    /* USER CODE BEGIN TIM8_MspPostInit 0 */

    /* USER CODE END TIM8_MspPostInit 0 */

    /**TIM8 GPIO Configuration    
    PB3     ------> TIM8_CH1N
    PB4     ------> TIM8_CH2N
    PB5     ------> TIM8_CH3N
    PB6     ------> TIM8_CH1
    PB8     ------> TIM8_CH2
    PB9     ------> TIM8_CH3 
    */
    GPIO_InitStruct.Pin       = GPIO_PIN_3 | GPIO_PIN_4;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_5;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_6;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_TIM8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_TIM8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM8_MspPostInit 1 */

    /* USER CODE END TIM8_MspPostInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle) {
  if(tim_baseHandle->Instance == TIM8) {
    /* USER CODE BEGIN TIM8_MspDeInit 0 */

    /* USER CODE END TIM8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM8_CLK_DISABLE();
  }
  /* USER CODE BEGIN TIM8_MspDeInit 1 */

  /* USER CODE END TIM8_MspDeInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
