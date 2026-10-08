#include "ls_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "periph.h"
#include "common.h"
#include "f3hw.h"
#include "ringbuf.h"


HAL_COMP(ls);

//process data from LS
HAL_PIN(d_cmd);
HAL_PIN(q_cmd);
HAL_PIN(pos);
HAL_PIN(vel);
// The angle the voltage computed this tick lands at, on average: the
// compares are preloaded, so it is applied over the next period, 1.5
// periods after the current sample that dq0 transforms at pos. idq0 and
// hv0's dead-time reference use it; hv0.adv on the f4 is then the encoder
// to sample latency alone. v_lead in periods, default 1.5, 0 = old
// behaviour (one angle for both).
HAL_PIN(conf_ok);  // every config word received once since boot
HAL_PIN(v_lead);
HAL_PIN(en);

// config data from LS
HAL_PIN(cmd_mode);
HAL_PIN(phase_mode);
HAL_PIN(r);
HAL_PIN(l);
HAL_PIN(lq);  // q axis inductance for curpid0.lq: config lq, or l when that is 0
HAL_PIN(psi);
HAL_PIN(cur_bw);
HAL_PIN(cur_ff);
HAL_PIN(cur_ind);
HAL_PIN(max_y);
HAL_PIN(max_cur);
HAL_PIN(dac);
HAL_PIN(drop_k);
HAL_PIN(emf_run);
HAL_PIN(emf_sel);
HAL_PIN(emf_pp);
HAL_PIN(drop_knee);
HAL_PIN(obs_en);   // obs0 runs, obs_mode 1 or 2
HAL_PIN(obs_src);  // angle0.src, 2 when obs_mode is 2
HAL_PIN(obs_bw);

// process data to LS
HAL_PIN(dc_volt);
HAL_PIN(id_fb);
HAL_PIN(iq_fb);
HAL_PIN(ud_fb);
HAL_PIN(uq_fb);

// state data to LS
HAL_PIN(hv_temp);
HAL_PIN(mot_temp);
HAL_PIN(core_temp);
HAL_PIN(fault_in);  //fault code send to f4

// short-circuit braking request for io0: follows the f4's flag, and on a
// link loss brakes for sbrake_time if the f4 last sent sbrake_arm
HAL_PIN(sbrake);
HAL_PIN(sbrake_arm);
HAL_PIN(sbrake_time);  // [s], default 1
HAL_PIN(ignore_fault_pin);
HAL_PIN(y);
HAL_PIN(u_fb);
HAL_PIN(v_fb);
HAL_PIN(w_fb);
HAL_PIN(emf_val);
HAL_PIN(obs_err);
HAL_PIN(obs_vel);

// misc
HAL_PIN(pwm_volt);
HAL_PIN(duty_max);  // from hv0: what min_on/min_off leave of the link, 0 = unwired
HAL_PIN(crc_error);
HAL_PIN(crc_ok);
HAL_PIN(timeout);
HAL_PIN(dma_pos);
HAL_PIN(idle);
HAL_PIN(fault);  //communication fault output

HAL_PIN(dma_pos2);
HAL_PIN(arr);
HAL_PIN(dma_pos_cmd);
HAL_PIN(inc);
HAL_PIN(window);

struct ls_ctx_t {
  uint32_t timeout;
  uint32_t sbrake_loss;  // brake for this link loss
  uint32_t tx_addr;
  uint32_t conf_seen;  // bit per config word written since boot
  uint8_t send;
  volatile packet_to_hv_t packet_to_hv;
  volatile packet_from_hv_t packet_from_hv;
};

//TODO: move to ctx
// volatile packet_to_hv_t packet_to_hv;
// volatile packet_from_hv_t packet_from_hv;

f3_config_data_t config;
f3_state_data_t state;

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ls_ctx_t *ctx = (struct ls_ctx_t *)ctx_ptr;
  struct ls_pin_ctx_t *pins = (struct ls_pin_ctx_t *)pin_ptr;

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  // 8N1, 8x oversampling, clocked from SYSCLK; DMA requests and overrun
  // disable are set before the init, which keeps them (the order the HAL
  // init used and the link was validated with)
  LL_USART_EnableDMAReq_TX(USART3);
  LL_USART_EnableDMAReq_RX(USART3);
  LL_USART_DisableOverrunDetect(USART3);
  LL_USART_Disable(USART3);
  LL_USART_Init(USART3, &(LL_USART_InitTypeDef){
                            .BaudRate            = DATABAUD,
                            .DataWidth           = LL_USART_DATAWIDTH_8B,
                            .StopBits            = LL_USART_STOPBITS_1,
                            .Parity              = LL_USART_PARITY_NONE,
                            .TransferDirection   = LL_USART_DIRECTION_TX_RX,
                            .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
                            .OverSampling        = LL_USART_OVERSAMPLING_8,
                        });
  LL_USART_DisableOneBitSamp(USART3);
  LL_USART_ConfigAsyncMode(USART3);  // LINEN, CLKEN, SCEN, IREN, HDSEL off
  LL_USART_Enable(USART3);
  while(!LL_USART_IsActiveFlag_TEACK(USART3) || !LL_USART_IsActiveFlag_REACK(USART3)) {
  }

  /**USART3 GPIO Configuration    
   PB10     ------> USART3_TX
   PB11     ------> USART3_RX 
   */
  LL_GPIO_Init(GPIOB, &(LL_GPIO_InitTypeDef){
                          .Pin        = LL_GPIO_PIN_10 | LL_GPIO_PIN_11,
                          .Mode       = LL_GPIO_MODE_ALTERNATE,
                          .Speed      = LL_GPIO_SPEED_FREQ_HIGH,
                          .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                          .Pull       = LL_GPIO_PULL_UP,
                          .Alternate  = LL_GPIO_AF_7,
                      });

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  //TX DMA
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, (uint32_t) & (ctx->packet_from_hv), LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_TRANSMIT), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, sizeof(packet_from_hv_t));
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_2,
                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_MODE_NORMAL | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
                            LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE | LL_DMA_PRIORITY_LOW);
  LL_DMA_ClearFlag_GI2(DMA1);

  //RX DMA
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_RECEIVE), (uint32_t) & (ctx->packet_to_hv), LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, sizeof(packet_to_hv_t));
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_3,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_MODE_NORMAL | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
                            LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE | LL_DMA_PRIORITY_LOW);
  LL_DMA_ClearFlag_GI3(DMA1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

  config.pins.r       = 0.0;
  config.pins.l       = 0.0;
  config.pins.psi     = 0.0;
  config.pins.cur_bw  = 0.0;
  config.pins.cur_ff  = 0.0;
  config.pins.cur_ind = 0.0;
  config.pins.max_y   = 0.0;
  config.pins.max_cur = 0.0;
  config.pins.dac     = 0.0;
  config.pins.drop_k  = 0.0;
  config.pins.lq      = 0.0;
  config.pins.emf_run = 0.0;
  config.pins.emf_sel = 0.0;
  config.pins.emf_pp  = 0.0;
  config.pins.drop_knee = 0.0;
  PIN(v_lead) = 1.5;
  config.pins.obs_mode  = 0.0;
  config.pins.obs_bw    = 200.0;

  LL_USART_SetRxTimeout(USART3, 16);  // 16 bits timeout
  LL_USART_EnableRxTimeout(USART3);
  LL_USART_ClearFlag_RTO(USART3);

  ctx->packet_from_hv.header.len        = (sizeof(packet_from_hv_t) - sizeof(stmbl_talk_header_t)) / 4;
  ctx->packet_from_hv.header.flags.cmd  = WRITE_CONF;
  ctx->packet_from_hv.header.slave_addr = 0;
}

static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ls_ctx_t *ctx      = (struct ls_ctx_t *)ctx_ptr;
  struct ls_pin_ctx_t *pins = (struct ls_pin_ctx_t *)pin_ptr;

  ctx->timeout     = 0;
  ctx->sbrake_loss = 0;
  ctx->tx_addr     = 0;
  ctx->conf_seen   = 0;
  ctx->send        = 0;
  PIN(crc_error)   = 0.0;
  PIN(crc_ok)      = 0.0;
  PIN(timeout)     = 0.0;
  PIN(sbrake)      = 0.0;
  PIN(sbrake_arm)  = 0.0;
  PIN(sbrake_time) = 1.0;
  PIN(idle)        = 0.0;
  PIN(dma_pos_cmd) = 4;
  PIN(inc)         = 5;
  PIN(window)      = 1;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ls_ctx_t *ctx      = (struct ls_ctx_t *)ctx_ptr;
  struct ls_pin_ctx_t *pins = (struct ls_pin_ctx_t *)pin_ptr;

  uint32_t dma_pos = sizeof(packet_to_hv_t) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);

  PIN(dma_pos2) = dma_pos;
  PIN(arr)      = PWM_RES;

  if(dma_pos > PIN(window) && dma_pos < sizeof(packet_to_hv_t) - PIN(window)) {
    if(PIN(dma_pos_cmd) < dma_pos) {
      PIN(arr) = PWM_RES - PIN(inc);
    } else if(PIN(dma_pos_cmd) > dma_pos) {
      PIN(arr) = PWM_RES + PIN(inc);
    }
  }

  uint32_t fault = 0;

  if(dma_pos == sizeof(packet_to_hv_t)) {
    uint32_t crc = crc_calc((uint32_t *)&(ctx->packet_to_hv.header.slave_addr), sizeof(packet_to_hv_t) / 4 - 1);
    if(ctx->packet_to_hv.header.slave_addr == 0 && ctx->packet_to_hv.header.len == (sizeof(packet_to_hv_t) - sizeof(stmbl_talk_header_t)) / 4 && crc == ctx->packet_to_hv.header.crc) {
      //
      // an address past this image's config is a newer f4's word: ignore it
      uint8_t a     = ctx->packet_to_hv.header.conf_addr;
      uint8_t valid = a < sizeof(config) / 4;
      a             = CLAMP(a, 0, sizeof(config) / 4 - 1);

      switch(ctx->packet_to_hv.header.flags.cmd) {
        case NO_CMD:
          break;
        case WRITE_CONF:
          if(valid) {
            config.data[a] = ctx->packet_to_hv.header.config.f32;
            ctx->conf_seen |= 1u << a;
          }
          break;
        case READ_CONF:
          if(valid) {
            ctx->tx_addr = a;
          }
          break;
        case DO_RESET:
          NVIC_SystemReset();
          break;
        case BOOTLOADER:
          LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR0, 0xDEADBEEF);
          NVIC_SystemReset();
          break;
      }

      // no enable before every config word has arrived once since boot: the
      // f4 cycles through them one per packet, and until then r, l, max_cur,
      // dac etc. are still 0
      PIN(conf_ok) = ctx->conf_seen == (1u << (sizeof(config) / 4)) - 1u;
      PIN(en)      = PIN(conf_ok) > 0.0 ? ctx->packet_to_hv.flags.enable : 0;
      PIN(phase_mode)       = ctx->packet_to_hv.flags.phase_type;
      PIN(cmd_mode)         = ctx->packet_to_hv.flags.cmd_type;
      PIN(ignore_fault_pin) = ctx->packet_to_hv.flags.ignore_fault_pin;
      PIN(sbrake)           = ctx->packet_to_hv.flags.sbrake;
      PIN(sbrake_arm)       = ctx->packet_to_hv.flags.sbrake_arm;
      PIN(d_cmd)            = ctx->packet_to_hv.d_cmd;
      PIN(q_cmd)            = ctx->packet_to_hv.q_cmd;
      PIN(pos)              = ctx->packet_to_hv.pos;
      PIN(vel)              = ctx->packet_to_hv.vel;

      if(ctx->packet_to_hv.flags.buf != 0x0) {
        extern struct ringbuf rx_buf;
        rb_write(&rx_buf, (void *)&(ctx->packet_to_hv.flags.buf), 1);
      }

      PIN(r)       = config.pins.r;
      PIN(l)       = config.pins.l;
      PIN(psi)     = config.pins.psi;
      PIN(cur_bw)  = config.pins.cur_bw;
      PIN(cur_ff)  = config.pins.cur_ff;
      PIN(cur_ind) = config.pins.cur_ind;
      PIN(max_y)   = config.pins.max_y;
      PIN(max_cur) = config.pins.max_cur;
      PIN(dac)     = config.pins.dac;
      PIN(drop_k)  = config.pins.drop_k;
      PIN(lq)      = config.pins.lq > 0.0 ? config.pins.lq : config.pins.l;
      PIN(emf_run) = config.pins.emf_run;
      PIN(emf_sel) = config.pins.emf_sel;
      PIN(emf_pp)  = config.pins.emf_pp;
      PIN(drop_knee) = config.pins.drop_knee;
      PIN(obs_en)    = config.pins.obs_mode > 0.5 ? 1.0 : 0.0;
      PIN(obs_src)   = config.pins.obs_mode > 1.5 ? 2.0 : 0.0;
      PIN(obs_bw)    = config.pins.obs_bw;
      ctx->timeout = 0;
      PIN(crc_ok)
      ++;
      if(ctx->send == 0) {
        ctx->send = 1;
      }
    } else {
      PIN(crc_error)
      ++;
      fault = 3;
    }
  } else if(ctx->timeout <= 5) {  // if no packet and no timeout, advance pos by velovity
    PIN(pos) = PIN(pos) + PIN(vel) * period;
  }


  if(LL_USART_IsActiveFlag_RTO(USART3)) {  // idle line
    // timeout, framing and overrun flags in one ICR write; LL clears them one by one
    WRITE_REG(USART3->ICR, USART_ICR_RTOCF | USART_ICR_FECF | USART_ICR_ORECF);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10);

    PIN(idle)
    ++;
    if(dma_pos != sizeof(packet_to_hv_t)) {
      PIN(dma_pos) = dma_pos;
    }

    // reset rx DMA
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, sizeof(packet_to_hv_t));
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    dma_pos = 0;
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);

    //ctx->send = 1;
  }

  if(ctx->send == 2) {
    ctx->send = 0;
  }
  if(ctx->send == 1 && dma_pos != 0) {
    ctx->send = 2;
    //packet_to_hv.d_cmd = -99.0;
    state.pins.u_fb      = PIN(u_fb);
    state.pins.v_fb      = PIN(v_fb);
    state.pins.w_fb      = PIN(w_fb);
    state.pins.hv_temp   = PIN(hv_temp);
    state.pins.mot_temp  = PIN(mot_temp);
    state.pins.core_temp = PIN(core_temp);
    state.pins.y         = PIN(y);
    state.pins.dc_volt   = PIN(dc_volt);
    state.pins.pwm_volt  = PIN(pwm_volt);
    state.pins.emf_val   = PIN(emf_val);
    state.pins.obs_err   = PIN(obs_err);
    state.pins.obs_vel   = PIN(obs_vel);

    // fill tx struct
    ctx->packet_from_hv.fault             = (uint8_t)PIN(fault_in);
    ctx->packet_from_hv.id_fb             = PIN(id_fb);
    ctx->packet_from_hv.iq_fb             = PIN(iq_fb);
    ctx->packet_from_hv.ud_fb             = PIN(ud_fb);
    ctx->packet_from_hv.uq_fb             = PIN(uq_fb);
    ctx->packet_from_hv.header.conf_addr  = ctx->tx_addr;
    ctx->packet_from_hv.header.config.f32 = state.data[ctx->tx_addr++];
    ctx->tx_addr %= sizeof(state) / 4;

    extern struct ringbuf tx_buf;
    uint8_t buf[1];
    if(rb_read(&tx_buf, buf, 1)) {
      ctx->packet_from_hv.buf = buf[0];
    } else {
      ctx->packet_from_hv.buf = 0x0;
    }
    ctx->packet_from_hv.header.crc = crc_calc((uint32_t *)&(ctx->packet_from_hv.header.slave_addr), sizeof(packet_from_hv_t) / 4 - 1);

    // start tx DMA
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, sizeof(packet_from_hv_t));
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
    //ctx->send = 0;
  }

  if(ctx->timeout == 6) {
    // brake on the loss only if the motor was driven or already braking
    ctx->sbrake_loss = PIN(sbrake_arm) > 0.0 && (PIN(en) > 0.0 || PIN(sbrake) > 0.0);
  }
  if(ctx->timeout > 5) {  //disable driver
    PIN(en)     = 0.0;
    PIN(vel)    = 0.0;
    PIN(sbrake) = ctx->sbrake_loss && (float)(ctx->timeout - 5) * period < PIN(sbrake_time);
    PIN(timeout)
    ++;
    fault = 1;
  }
  if(ctx->timeout < 0x7FFFFFFF) {  // saturate: a wrap would re-enable for a few ticks
    ctx->timeout++;
  }

  PIN(fault) = MAX(fault, PIN(fault_in));


  // The ceiling curpid may ask for. hv.c reserves min_on at one end of the
  // period and min_off at the other, so the usable link is duty_max of it (0.91
  // at the 3 us defaults, where the old fixed 0.95 promised 4% more than the
  // clamp would pass, and the loop wound up against the clamp instead of its
  // own limit). Falls back to 0.95 when the pin is not wired.
  float duty = PIN(duty_max) > 0.0 ? PIN(duty_max) : 0.95;

  // TODO: sin = 0.5
  switch((uint16_t)PIN(phase_mode)) {
    case PHASE_90_3PH:  // 90°
      PIN(pwm_volt) = PIN(dc_volt) * M_SQRT1_2 * duty;
      break;

    case PHASE_90_4PH:  // 90°
      PIN(pwm_volt) = PIN(dc_volt) * duty;
      break;

    case PHASE_120_3PH:  // 120°
      PIN(pwm_volt) = PIN(dc_volt) * M_SQRT1_3 * duty;
      break;

    case PHASE_180_2PH:  // 180°
    case PHASE_180_3PH:  // 180°
      PIN(pwm_volt) = PIN(dc_volt) * duty;
      break;

    default:
      PIN(pwm_volt) = 0.0;
  }
}

hal_comp_t ls_comp_struct = {
    .name      = "ls",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .hw_init   = hw_init,
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct ls_ctx_t),
    .pin_count = sizeof(struct ls_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
