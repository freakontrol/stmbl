#include "encm_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "dma_util.h"
#include "hw/hw.h"

/**
* ## Brief
* `encm` reads Mitsubishi serial absolute encoders (2.5 Mbit/s, byte oriented request/reply protocol) on the FB0 connector of the F4 board. It is loaded by `conf/template/encm_fb0.txt` (used by `conf/mitsu.txt` and `conf/wobl.txt`), which links `encm0.pos` to `fb_switch0.mot_pos` and `mot_abs_pos`, `encm0.state` to `fb_switch0.mot_state` and `encm0.error` to `fault0.mot_fb_error`.
*
* ## Component Explanation
*
* 1. **Hardware (hw_init)**:
* - USART6 at 2.5 Mbaud, 8N1. TX = PC6, line driver enable = PD15.
* - The reply is received by DMA2 stream 1 (channel 5) into a 15 byte buffer.
*
* 2. **Command selection (rt)**:
* - If `cmd > 0` this byte is sent as request. Otherwise it depends on `id`:
* - `id = 0` (unknown): request 0x92 (read encoder id), any length accepted.
* - `id = 32`: request 0x02, 7 reply bytes expected, 13 bit single-turn position.
* - `id = 61`, `65` and all others: request 0x32, 9 reply bytes expected, 17 bit single-turn position.
*
* 3. **Reply check and decoding (rt)**, done on the reply to the request of the previous rt cycle:
* - Checksum: the XOR of all received bytes must be 0, otherwise `crc_error = 1`.
* - Fewer than 3 bytes or not the expected number: `dma_error = 1`.
* - First byte different from the sent command: `cmd_error = 1`.
* - Any of these sets `error = 1`, `state = 0`.
* - Reply to 0x02: `ipos = (buf2 << 11) + ((buf3 & 0x1f) << 19)`; reply to 0x32: `ipos = (buf2 & 0x80) + (buf3 << 8) + (buf4 << 16)`; both give `state = 3`.
* - Reply to 0x92: `id = buf2`, so the next cycle uses the matching position request.
* ```c
* pos = ipos * 2 * pi / 2^24 - pi;   // rad, -pi..pi
* ```
*
* 4. **Request (rt)**:
* - The TX enable is set, the command byte is sent and the rt function busy-waits for the USART transmit-complete flag, then the TX enable is cleared and the RX DMA is restarted.
*
* {{% hint warning %}}
* The protocol was reverse engineered: the byte meanings in the source comments are guesses, multiturn data is not decoded, and only a plain XOR checksum is checked. After the id reply `pos` is set to -pi (ipos = 0) for one cycle but `state` stays 0. The busy-wait in rt ("irq here will cause problems") and the hard coded USART6/PC6/PD15/DMA2 stream 1 make it V4 specific; DMA2 stream 1 is also used by `encs` and `dmm`, so these cannot be loaded together.
* {{% /hint %}}
*/

HAL_COMP(encm);

HAL_PIN(pos);          // *output*, Single-turn position (rad, -pi..pi)
HAL_PIN(error);        // *output*, 1 = any reply error
HAL_PIN(cmd_error);    // *output*, 1 = first reply byte differs from the request
HAL_PIN(crc_error);    // *output*, 1 = XOR checksum of the reply not 0
HAL_PIN(dma_error);    // *output*, 1 = wrong number of reply bytes
HAL_PIN(state);        // *output*, 0 = error or no position, 3 = position valid

HAL_PIN(cmd);          // *parameter*, Request byte override, 0 = select from id
HAL_PIN(req);          // *output*, Request byte sent this cycle
HAL_PIN(full_duplex);  // *parameter*, 0 = half duplex on PC6, > 0 = RX on PC7 (read at init)
HAL_PIN(bytes);        // *output*, Number of reply bytes received
HAL_PIN(id);           // *input/output*, Encoder id, 0 = unknown (read from encoder), 32, 61, 65 known

HAL_PINA(buf, 15);     // *output*, Received reply bytes


struct encm_ctx_t {
  uint32_t error;
  uint8_t rxbuf[15];
};

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct encm_ctx_t *ctx      = (struct encm_ctx_t *)ctx_ptr;
  struct encm_pin_ctx_t *pins = (struct encm_pin_ctx_t *)pin_ptr;
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_GPIO_StructInit(&GPIO_InitStruct);
  LL_USART_InitTypeDef USART_InitStruct;
  LL_USART_StructInit(&USART_InitStruct);
  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);

  //TX enable
  GPIO_InitStruct.Pin   = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);

  //USART TX
  GPIO_InitStruct.Pin   = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  USART_InitStruct.BaudRate            = 2500000;
  USART_InitStruct.DataWidth          = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits            = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity              = LL_USART_PARITY_NONE;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.TransferDirection                = LL_USART_DIRECTION_TX_RX;
  LL_USART_Init(USART6, &USART_InitStruct);

  /* Enable the USART */
  LL_USART_Enable(USART6);

  if(PIN(full_duplex) > 0.0) {
    //USART RX
    GPIO_InitStruct.Pin   = LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode  = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull  = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  } else {
    SET_BIT(USART6->CR3, USART_CR3_HDSEL);  // half duplex
  }

  // DMA-Disable
  dma_stream_stop(DMA2_Stream1);
  LL_DMA_DeInit(DMA2, LL_DMA_STREAM_1);

  // DMA2-Config
  DMA_InitStructure.Channel            = LL_DMA_CHANNEL_5;
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t) & (USART6->DR);
  DMA_InitStructure.MemoryOrM2MDstAddress    = (uint32_t)&ctx->rxbuf;
  DMA_InitStructure.Direction                = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStructure.NbData         = ARRAY_SIZE(ctx->rxbuf);
  DMA_InitStructure.PeriphOrM2MSrcIncMode      = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode          = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStructure.MemoryOrM2MDstDataSize     = LL_DMA_MDATAALIGN_BYTE;
  DMA_InitStructure.Mode               = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority           = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.FIFOMode           = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.FIFOThreshold      = LL_DMA_FIFOTHRESHOLD_1_2;
  DMA_InitStructure.MemBurst        = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst    = LL_DMA_PBURST_SINGLE;
  LL_DMA_Init(DMA2, LL_DMA_STREAM_1, &DMA_InitStructure);

  LL_USART_EnableDMAReq_RX(USART6);
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct encm_ctx_t *ctx      = (struct encm_ctx_t *)ctx_ptr;
  struct encm_pin_ctx_t *pins = (struct encm_pin_ctx_t *)pin_ptr;
  // for(int i = 0; i < ARRAY_SIZE(ctx->rxbuf); i++){
  //    PIN_ARRAY(d,i) = ctx->rxbuf[i];
  // }
  //position request: 0x32
  //reply:
  //0: request
  //1: unknown
  //2: low 1 bit
  //3: mid 8 bit
  //4: hi 8 bit
  //5: 8 bit full turns
  //6: 255 turns... how many bits?
  //7: unknown
  //8: checksum: xor byte 0-8 = 0

  //request: 0x02
  //0: request
  //1: unknown
  //2: low 8 bit
  //3: mid 8 bit
  //4: hi 1 + 7 bit mt
  //5: crc

  //request: 0x92
  //0: request
  //1: unknown
  //2: id
  //3: crc or id ...

  uint8_t cmd            = 0;
  uint8_t expected_bytes = 0;

  if(PIN(cmd) > 0) {  // cmd overwrite
    cmd = PIN(cmd);
  } else {
    switch((int)PIN(id)) {
      case 0:                   // no id
        cmd            = 0x92;  // request id
        expected_bytes = 0;
        break;

      case 32:
        cmd            = 0x2;  // request singel turn data
        expected_bytes = 7;
        break;

      case 61:
      case 65:
        cmd            = 0x32;  // request singel turn data
        expected_bytes = 9;
        break;

      default:
        cmd            = 0x32;
        expected_bytes = 9;
        break;
    }
  }

  PIN(req) = cmd;

  PIN(error)     = 0;
  PIN(crc_error) = 0;
  PIN(dma_error) = 0;
  PIN(cmd_error) = 0;
  PIN(state)     = 0;

  // number of received bytes
  uint8_t bytes = sizeof(ctx->rxbuf) - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_1);
  PIN(bytes)    = bytes;

  // check crc (xor all == 0)
  uint8_t crc = 0;
  for(int i = 0; i < bytes; i++) {
    crc          = crc ^ ctx->rxbuf[i];
    PINA(buf, i) = ctx->rxbuf[i];
  }

  if(crc) {
    PIN(error)     = 1;
    PIN(crc_error) = 1;
  } else {
    // check #bytes
    if(bytes < 3 || ((expected_bytes > 0) & (bytes != expected_bytes))) {
      PIN(error)     = 1;
      PIN(dma_error) = 1;
    } else {
      // check cmd
      if(ctx->rxbuf[0] != cmd) {
        PIN(error)     = 1;
        PIN(cmd_error) = 1;
      } else {
        uint32_t ipos = 0;
        switch(cmd) {
          case 0x2:                                                               // single turn data
            ipos       = (ctx->rxbuf[2] << 11) + ((ctx->rxbuf[3] & 0x1f) << 19);  // 13 bit
            PIN(state) = 3;
            break;

          case 0x32:                                                                             // single turn data
            ipos       = (ctx->rxbuf[2] & 0x80) + (ctx->rxbuf[3] << 8) + (ctx->rxbuf[4] << 16);  // 17 bit
            PIN(state) = 3;
            break;

          case 0x92:  // encoder id
            PIN(id) = ctx->rxbuf[2];
            break;
        }
        PIN(pos) = (ipos * M_PI * 2.0 / 16777216.0) - M_PI;
      }
    }
  }


  //TODO: irq here will cause problems
  LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_15);  //tx enable
  (USART6)->DR = cmd;
  while(LL_USART_IsActiveFlag_TC(USART6) == RESET)
    ;
  LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_15);  //tx disable
  //start rx dma
  dma_stream_stop(DMA2_Stream1);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
}

hal_comp_t encm_comp_struct = {
    .name      = "encm",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct encm_ctx_t),
    .pin_count = sizeof(struct encm_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
