# CubeMX reference configurations

`stmbl_f405/stmbl_f405.ioc` (STM32F405VGT, low-voltage side) and
`stmbl_f303/stmbl_f303.ioc` (STM32F303CBT, hv side) describe the pinout,
clock tree, peripheral modes, DMA and interrupt assignments the firmware sets
up, for STM32CubeMX 6.x. They are for viewing and checking (pin conflicts,
clock limits, DMA clashes), not for building: the firmware's init lives in
`src/setup.c`, the HAL components and `stm32f303/src/periph.c`, and none of it
is generated.

Code generation is set to LL drivers and a Makefile project. Generating writes
into this folder (`Src/`, `Inc/`, `Drivers/`, ...), which is ignored by git;
use it to compare a CubeMX init sequence against ours, never copy it over
the firmware.

What the files cannot show:

- F405: pins and peripherals change with the loaded config. The file holds the
  default setup: fb0 on TIM4 (encoder) and ADC1/2 IN6/IN7 (sin/cos), fb1 on
  TIM1, the command encoder on TIM2, the F3 link on USART2, sserial on
  UART4/USART1, fb0 UART (USART6) and SPI (SPI3). SPI3's DMA and the TIM4
  capture DMAs of encs/yaskawa reuse DMA1 S0/S7 and DMA2 S1 and are left out.
  The ADC sequence shows 2 of the 10 ranks per group (9 fb0, 1 fb1).
- F303: TIM8 is shown at 15 kHz (ARR 4800); `PWM_FREQ` in
  `stm32f303/Makefile` sets 10 or 20 kHz. The break/lock order, the opamp
  self calibration and the ADC/DMA start order are done by hand in `periph.c`.

Keep them in step with `inc/hw/hw.h`, `shared/f3hw.h` and the init code when
pins or peripherals change.
