#include "hv_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "dma_util.h"
#include "hw/hw.h"
#include "common.h"
#include <stdio.h>
#include "main.h"
#include "ringbuf.h"

HAL_COMP(hv);

//process data from LS
HAL_PIN(d_cmd);
HAL_PIN(q_cmd);
HAL_PIN(pos);
HAL_PIN(vel);
HAL_PIN(adv);  // commutation advance [s], pos is sent as pos + vel * adv
HAL_PIN(en);

// config data from LS
HAL_PIN(phase_mode);
HAL_PIN(cmd_mode);
HAL_PIN(r);
HAL_PIN(l);   // d axis inductance, and q too while lq is 0
HAL_PIN(lq);  // q axis inductance, 0 = same as l
HAL_PIN(psi);
HAL_PIN(cur_bw);
HAL_PIN(cur_ff);
HAL_PIN(cur_ind);
HAL_PIN(max_y);
HAL_PIN(max_cur);
HAL_PIN(dac);
HAL_PIN(drop_k);     // dead time compensation, fraction of the ideal
HAL_PIN(drop_knee);  // dead time compensation curve knee [A], 0 = latched sign
HAL_PIN(emf_run);  // f3 emf0 back emf map: 1 sum, 0 hold, -1 clear
HAL_PIN(emf_sel);  // which emf0 result comes back in emf_val
HAL_PIN(emf_pp);   // pole pairs, for emf0's per pole bins

// process data to LS
HAL_PIN(dc_volt);
HAL_PIN(id_fb);
HAL_PIN(iq_fb);
HAL_PIN(ud_fb);
HAL_PIN(uq_fb);
HAL_PIN(abs_cur);
HAL_PIN(abs_volt);
HAL_PIN(duty);
HAL_PIN(power);   // electrical power into the motor [W], negative when braking
HAL_PIN(dc_cur);  // dc link current [A], estimated from power balance, negative when braking

// state data to LS
HAL_PIN(hv_temp);
HAL_PIN(mot_temp);
HAL_PIN(core_temp);
HAL_PIN(fault);  //fault from hv
HAL_PIN(ignore_fault_pin);
HAL_PIN(sbrake);      // short-circuit braking request, fault0.sbrake
HAL_PIN(sbrake_arm);  // lets the f3 brake on link loss, fault0.sbrake_en
HAL_PIN(y);
HAL_PIN(u_fb);
HAL_PIN(v_fb);
HAL_PIN(w_fb);
HAL_PIN(emf_val);  // emf0 result number emf_sel, from the f3
HAL_PIN(pwm_freq);  // f3 PWM and rt rate [Hz], 15000 from an f3 that doesn't report it
HAL_PIN(link_to);  // f3 rt ticks in link timeout since the f3 booted; any rise is a dropout that took the gates off
HAL_PIN(link_drops);  // f3 dropouts seen while enabled, out; each one faults HV_TIMEOUT_ERROR

// misc
HAL_PIN(rev);
HAL_PIN(pwm_volt);
HAL_PIN(uart_sr);
HAL_PIN(uart_dr);
HAL_PIN(crc_error);  //total number of crc errors, never reset
HAL_PIN(scale);

HAL_PIN(state);
HAL_PIN(value);

struct hv_ctx_t {
  union {
    volatile packet_to_hv_t packet_to_hv;
    volatile packet_bootloader_t packet_to_hv_bootloader;
  } to_hv;
  union {
    volatile packet_from_hv_t packet_from_hv;
    volatile packet_bootloader_t packet_from_hv_bootloader;
  } from_hv;
  f3_config_data_t config;
  f3_state_data_t state;
  uint32_t addr;
  uint16_t timeout;
  float link_to_last;  // link_to at the last state word, -1 before the first
  uint32_t link_err;   // an f3 dropout while enabled, held until en drops
  uint8_t conf_addr;
  uint8_t send_state;
};

typedef enum {
  SLAVE_IN_APP,
  SEND_TO_BOOTLOADER,
  ERASE_FLASH,
  SEND_APP,
  CRC_CHECK,
  SEND_TO_APP,
  FLASH_FAILED,
  VERIFY_FLASH,
} flash_state_t;

flash_state_t flash_state;

uint32_t send_to_bootloader;

// hv_pause <ms>: stop sending to the f3 for that long, to test what the f3
// does on its own when the link is lost (gates off, short brake if armed).
// The f4 sees no replies meanwhile, so it faults HV_TIMEOUT_ERROR too.
static volatile float hv_pause_left;  // [s]

// CRC_CHECK: an f3 bootloader computes the app CRC inside its timer IRQ
// (2-3 ms for 74 KB) and answers once that is done, at an offset that
// depends on the image size. Our rx DMA is re-armed on every tick we send,
// so a check sent every tick can lose that answer every time. Send it once,
// then stay quiet and listen for CRC_LISTEN ticks before sending it again.
#define CRC_LISTEN 100  // 20 ms
static uint32_t crc_wait;
static uint32_t crc_nak;  // the f3 answered NAK: its CRC over the image is not 0

// hv_verify: with the f3 sitting in its bootloader (after a failed
// hv_update), read the app area back word by word with the bootloader's
// READ opcode and compare it with the embedded image, plus VERIFY_EXTRA
// words past its end, which an erased app area holds as 0xFFFFFFFF.
#define VERIFY_EXTRA 8
#define VERIFY_LIST 8
static uint32_t send_verify;
// FLASH_FAILED: a failed update can leave the f3 in its bootloader (it only
// starts the app after a reset with a valid CRC), in its app if it never got
// there, or without power. Sends alternate between a bootloader NOP and an
// app packet with the bridge off; only an app reply leaves FLASH_FAILED, so
// silence (no f3 power) or bootloader answers keep it, and fault 9 stays.
static uint32_t probe_app;  // the next FLASH_FAILED send is an app packet
static volatile struct {
  uint32_t state;  // 0 idle, 1 running, 2 done, 3 no answer
  uint32_t words;  // words read so far
  uint32_t total;
  uint32_t bad;
  uint32_t addr[VERIFY_LIST], got[VERIFY_LIST], want[VERIFY_LIST];
  uint32_t vi_crc, vi_size;  // the f3's version_info image_crc and image_size
} verify;

extern uint8_t _binary_obj_hvf3_hvf3_bin_start;
extern uint8_t _binary_obj_hvf3_hvf3_bin_size;
extern uint8_t _binary_obj_hvf3_hvf3_bin_end;

struct ringbuf hv_rx_buf = RINGBUF(128);
struct ringbuf hv_tx_buf = RINGBUF(128);

void hv_send(char *ptr) {
  if(ptr) {  //TODO: check connection status
    rb_write(&hv_tx_buf, ptr, strlen(ptr));
    rb_write(&hv_tx_buf, "\n", 1);
  }
}
COMMAND("hv", hv_send, "send command to hv board");

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct hv_ctx_t *ctx      = (struct hv_ctx_t *)ctx_ptr;
  struct hv_pin_ctx_t *pins = (struct hv_pin_ctx_t *)pin_ptr;

  //setup uart to the f3. uses DMA to transfer to_hv struct.
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_USART_InitTypeDef USART_InitStruct;
  LL_DMA_InitTypeDef DMA_InitStructure;

  UART_DRV_CLOCK_COMMAND(UART_DRV_RCC);

  //USART TX
  LL_GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.Pin        = UART_DRV_TX_PIN;
  GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate  = UART_DRV_TX_AF_SOURCE;
  LL_GPIO_Init(UART_DRV_TX_PORT, &GPIO_InitStruct);

  //USART RX
  GPIO_InitStruct.Pin       = UART_DRV_RX_PIN;
  GPIO_InitStruct.Alternate = UART_DRV_RX_AF_SOURCE;
  LL_GPIO_Init(UART_DRV_RX_PORT, &GPIO_InitStruct);

  LL_USART_StructInit(&USART_InitStruct);
  USART_InitStruct.BaudRate            = DATABAUD;
  USART_InitStruct.DataWidth           = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits            = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity              = LL_USART_PARITY_NONE;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.TransferDirection   = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.OverSampling        = LL_USART_OVERSAMPLING_8;
  LL_USART_Init(UART_DRV, &USART_InitStruct);

  /* Enable the USART */
  LL_USART_Enable(UART_DRV);

  // DMA-Disable
  dma_stream_stop(UART_DRV_TX_DMA);
  LL_DMA_DeInit(UART_DRV_DMA, UART_DRV_TX_DMA_STREAM);

  // DMA2-Config
  LL_DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.Channel                = UART_DRV_TX_DMA_CHAN;
  DMA_InitStructure.PeriphOrM2MSrcAddress  = (uint32_t) & (UART_DRV->DR);
  DMA_InitStructure.MemoryOrM2MDstAddress  = (uint32_t) & (ctx->to_hv.packet_to_hv);
  DMA_InitStructure.Direction              = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStructure.NbData                 = MAX(sizeof(packet_to_hv_t), sizeof(packet_bootloader_t));
  DMA_InitStructure.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  DMA_InitStructure.Mode                   = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority               = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.FIFOMode               = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.FIFOThreshold          = LL_DMA_FIFOTHRESHOLD_1_2;
  DMA_InitStructure.MemBurst               = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst            = LL_DMA_PBURST_SINGLE;
  LL_DMA_Init(UART_DRV_DMA, UART_DRV_TX_DMA_STREAM, &DMA_InitStructure);

  //DMA_Cmd(UART_DRV_TX_DMA, ENABLE);

  LL_USART_EnableDMAReq_TX(UART_DRV);


  // DMA-Disable
  dma_stream_stop(UART_DRV_RX_DMA);
  LL_DMA_DeInit(UART_DRV_DMA, UART_DRV_RX_DMA_STREAM);

  // DMA2-Config
  DMA_InitStructure.Channel               = UART_DRV_RX_DMA_CHAN;
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t) & (UART_DRV->DR);
  DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t) & (ctx->from_hv.packet_from_hv);
  DMA_InitStructure.Direction             = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStructure.NbData                = MAX(sizeof(packet_from_hv_t), sizeof(packet_bootloader_t));
  DMA_InitStructure.Priority              = LL_DMA_PRIORITY_MEDIUM;
  LL_DMA_Init(UART_DRV_DMA, UART_DRV_RX_DMA_STREAM, &DMA_InitStructure);


  LL_USART_EnableDMAReq_RX(UART_DRV);
  dma_stream_stop(UART_DRV_RX_DMA);
  LL_DMA_EnableStream(UART_DRV_DMA, UART_DRV_RX_DMA_STREAM);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);
  ctx->timeout          = 0;
  PIN(dac)              = 2500;
  PIN(drop_k)           = 0;
  PIN(lq)               = 0;
  PIN(adv)              = 0;
  send_to_bootloader    = 0;
  hv_pause_left         = 0.0;
  ctx->link_to_last     = -1.0;
  ctx->link_err         = 0;
  flash_state           = SLAVE_IN_APP;
  ctx->send_state       = 0;
  PIN(ignore_fault_pin) = 1;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct hv_ctx_t *ctx      = (struct hv_ctx_t *)ctx_ptr;
  struct hv_pin_ctx_t *pins = (struct hv_pin_ctx_t *)pin_ptr;
  float e                   = PIN(en);
  float pos                 = PIN(pos);
  float vel                 = PIN(vel);
  pos                       = mod(pos + vel * PIN(adv));

  ctx->config.pins.r       = PIN(r);
  ctx->config.pins.l       = PIN(l);
  ctx->config.pins.psi     = PIN(psi);
  ctx->config.pins.cur_bw  = PIN(cur_bw);
  ctx->config.pins.cur_ff  = PIN(cur_ff);
  ctx->config.pins.cur_ind = PIN(cur_ind);
  ctx->config.pins.max_y   = PIN(max_y);
  ctx->config.pins.max_cur = PIN(max_cur) * PIN(scale);
  ctx->config.pins.dac     = PIN(dac);
  ctx->config.pins.drop_k  = PIN(drop_k);
  ctx->config.pins.lq      = PIN(lq);
  ctx->config.pins.emf_run = PIN(emf_run);
  ctx->config.pins.emf_sel = PIN(emf_sel);
  ctx->config.pins.emf_pp  = PIN(emf_pp);
  ctx->config.pins.drop_knee = PIN(drop_knee);
  ctx->config.pins.unused0   = 0.0;  // an older f3 reads its observer off here
  ctx->config.pins.unused1   = 0.0;

  uint32_t dma_count = MAX(sizeof(packet_from_hv_t), sizeof(packet_bootloader_t)) - LL_DMA_GetDataLength(UART_DRV_DMA, UART_DRV_RX_DMA_STREAM);

  PIN(value) = 0.0;

  if(dma_count >= sizeof(stmbl_talk_header_t)) {
    // PIN(value) = 0.5;
    if(dma_count >= sizeof(stmbl_talk_header_t) + ctx->from_hv.packet_from_hv.header.len * 4) {
      PIN(value) = 0.75;

      CRC->CR = CRC_CR_RESET;
      uint32_t crc = crc_calc_block((uint32_t *)&(ctx->from_hv.packet_from_hv.header.slave_addr), sizeof(stmbl_talk_header_t) / 4 + ctx->from_hv.packet_from_hv.header.len - 1);
      if(ctx->from_hv.packet_from_hv.header.crc == crc) {
        switch(flash_state) {
          case SLAVE_IN_APP:
            if(ctx->from_hv.packet_from_hv.header.slave_addr == 0 && ctx->from_hv.packet_from_hv.header.len == (sizeof(packet_from_hv_t) - sizeof(stmbl_talk_header_t)) / 4) {
              // from f3 app
              PIN(id_fb) = ctx->from_hv.packet_from_hv.id_fb;
              PIN(iq_fb) = ctx->from_hv.packet_from_hv.iq_fb;
              PIN(ud_fb) = ctx->from_hv.packet_from_hv.ud_fb;
              PIN(uq_fb) = ctx->from_hv.packet_from_hv.uq_fb;
              if(PIN(rev) > 0.0) {
                PIN(uq_fb) *= -1.0;
                PIN(iq_fb) *= -1.0;
              }
              PIN(fault)    = ctx->from_hv.packet_from_hv.fault;
              PIN(abs_cur)  = sqrtf(PIN(id_fb) * PIN(id_fb) + PIN(iq_fb) * PIN(iq_fb));
              PIN(abs_volt) = sqrtf(PIN(ud_fb) * PIN(ud_fb) + PIN(uq_fb) * PIN(uq_fb));
              if(PIN(pwm_volt) > 0.0) {
                PIN(duty) = PIN(abs_volt) / PIN(pwm_volt);
              }

              // an address past this image's state is a newer f3's word: ignore it
              uint16_t a = ctx->from_hv.packet_from_hv.header.conf_addr;
              if(a < sizeof(f3_state_data_t) / 4) {
                ctx->state.data[a] = ctx->from_hv.packet_from_hv.header.config.f32;
              }

              PIN(dc_volt)   = ctx->state.pins.dc_volt;
              PIN(pwm_volt)  = ctx->state.pins.pwm_volt;
              PIN(u_fb)      = ctx->state.pins.u_fb;
              PIN(v_fb)      = ctx->state.pins.v_fb;
              PIN(w_fb)      = ctx->state.pins.w_fb;
              PIN(hv_temp)   = ctx->state.pins.hv_temp;
              PIN(mot_temp)  = ctx->state.pins.mot_temp;
              PIN(core_temp) = ctx->state.pins.core_temp;
              PIN(y)         = ctx->state.pins.y;
              PIN(emf_val)   = ctx->state.pins.emf_val;
              PIN(pwm_freq)  = ctx->state.pins.pwm_freq > 0.0 ? ctx->state.pins.pwm_freq : 15000.0;
              PIN(link_to)   = ctx->state.pins.link_to;

              // The f3 times out after 2 packets (400 us at 5 kHz) and takes
              // the gates off, the f4 only after 3 ticks without a reply, so
              // a short dropout can stop the bridge without this side ever
              // faulting, and the f3 then comes back on its own. A rise in
              // the f3's count while enabled turns that into a fault here.
              // The count only falls when the f3 reboots: take it as the new
              // baseline.
              if(ctx->link_to_last >= 0.0 && PIN(link_to) > ctx->link_to_last && e > 0.0) {
                ctx->link_err = 1;
                PIN(link_drops)++;
              }
              ctx->link_to_last = PIN(link_to);
              if(ctx->link_err) {
                PIN(fault) = HV_TIMEOUT_ERROR;
              }

              // not measured: P = 3/2 (ud id + uq iq) from the commanded
              // voltages, so inverter losses are left out
              PIN(power) = 1.5 * (PIN(ud_fb) * PIN(id_fb) + PIN(uq_fb) * PIN(iq_fb)) * 0.5 + PIN(power) * 0.5;
              if(PIN(dc_volt) > 1.0) {
                PIN(dc_cur) = PIN(power) / PIN(dc_volt);
              }

              PIN(value) = 1.0;

              ctx->timeout = 0;

              if(ctx->from_hv.packet_from_hv.buf != 0x0) {
                rb_write(&hv_rx_buf, (void *)&(ctx->from_hv.packet_from_hv.buf), 1);
              }
            } else {
              // wrong packet len or slave addr
            }
            break;
          case SEND_TO_BOOTLOADER:

            break;
          case ERASE_FLASH:
            if(ctx->from_hv.packet_from_hv.header.slave_addr == 255 && ctx->from_hv.packet_from_hv.header.len == (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4) {
              // from f3 bootloader
              if(ctx->from_hv.packet_from_hv_bootloader.state == BOOTLOADER_STATE_OK && ctx->from_hv.packet_from_hv_bootloader.cmd == BOOTLOADER_OPCODE_PAGEERASE) {
                ctx->timeout = 0;
                flash_state  = SEND_APP;
              }
            } else {
              // wrong packet len or slave addr
              PIN(value) = 3.0;
            }
            break;
          case SEND_APP:
            if(ctx->from_hv.packet_from_hv.header.slave_addr == 255 && ctx->from_hv.packet_from_hv.header.len == (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4) {
              // from f3 bootloader
              if(ctx->from_hv.packet_from_hv_bootloader.state == BOOTLOADER_STATE_OK && ctx->from_hv.packet_from_hv_bootloader.cmd == BOOTLOADER_OPCODE_WRITE && ctx->from_hv.packet_from_hv_bootloader.addr == 0x08004000 + ctx->addr * 4 && ctx->from_hv.packet_from_hv_bootloader.value == ((uint32_t *)&(_binary_obj_hvf3_hvf3_bin_start))[ctx->addr]) {
                ctx->timeout = 0;
                ctx->addr++;
              }
              if(ctx->addr >= (((uint32_t) & (_binary_obj_hvf3_hvf3_bin_size)) + 3) / 4) {
                flash_state = CRC_CHECK;
                crc_wait    = 0;
                crc_nak     = 0;
                // flash_state = SEND_TO_APP;
              }
            } else {
              // wrong packet len or slave addr
              PIN(value) = 3.0;
            }
            break;
          case CRC_CHECK:
            if(ctx->from_hv.packet_from_hv.header.slave_addr == 255 && ctx->from_hv.packet_from_hv.header.len == (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4) {
              // from f3 bootloader
              if(ctx->from_hv.packet_from_hv_bootloader.state == BOOTLOADER_STATE_OK && ctx->from_hv.packet_from_hv_bootloader.cmd == BOOTLOADER_OPCODE_CRCCHECK) {
                ctx->timeout = 0;
                flash_state  = SEND_TO_APP;
              } else if(ctx->from_hv.packet_from_hv_bootloader.state == BOOTLOADER_STATE_NAK && ctx->from_hv.packet_from_hv_bootloader.cmd == BOOTLOADER_OPCODE_CRCCHECK) {
                ctx->timeout = 0;
                crc_nak      = 1;
                flash_state  = FLASH_FAILED;
              }
            } else {
              // wrong packet len or slave addr
              PIN(value) = 3.0;
            }
            break;
          case SEND_TO_APP:

            break;
          case FLASH_FAILED:
            if(ctx->from_hv.packet_from_hv.header.slave_addr == 0 && ctx->from_hv.packet_from_hv.header.len == (sizeof(packet_from_hv_t) - sizeof(stmbl_talk_header_t)) / 4) {
              ctx->timeout = 0;  // the f3 runs its app
              flash_state  = SLAVE_IN_APP;
            }
            // a bootloader answer or none: stay, ctx->timeout keeps running so fault 9 stays
            break;
          case VERIFY_FLASH:
            if(ctx->from_hv.packet_from_hv.header.slave_addr == 255 && ctx->from_hv.packet_from_hv.header.len == (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4) {
              uint32_t a = 0x08004000 + ctx->addr * 4;
              if(ctx->from_hv.packet_from_hv_bootloader.state == BOOTLOADER_STATE_OK && ctx->from_hv.packet_from_hv_bootloader.cmd == BOOTLOADER_OPCODE_READ && ctx->from_hv.packet_from_hv_bootloader.addr == a) {
                uint32_t n    = ((uint32_t) & (_binary_obj_hvf3_hvf3_bin_size)) / 4;
                uint32_t got  = ctx->from_hv.packet_from_hv_bootloader.value;
                uint32_t want = ctx->addr < n ? ((uint32_t *)&(_binary_obj_hvf3_hvf3_bin_start))[ctx->addr] : 0xFFFFFFFFu;
                if(ctx->addr == 0x188 / 4) {
                  verify.vi_crc = got;
                }
                if(ctx->addr == 0x18C / 4) {
                  verify.vi_size = got;
                }
                if(got != want) {
                  if(verify.bad < VERIFY_LIST) {
                    verify.addr[verify.bad] = a;
                    verify.got[verify.bad]  = got;
                    verify.want[verify.bad] = want;
                  }
                  verify.bad++;
                }
                ctx->timeout = 0;
                ctx->addr++;
                verify.words = ctx->addr;
                if(ctx->addr >= verify.total) {
                  verify.state = 2;
                  flash_state  = FLASH_FAILED;  // the f3 answered from its bootloader, so it is still there
                }
              }
            }
            break;
        }


      } else {
        // CRC fault
        PIN(crc_error)
        ++;
        // PIN(fault) = HV_CRC_ERROR;
        // PIN(value) = 4.0;
      }
    }
  }

  if(ctx->timeout > 2) {
    PIN(fault) = HV_TIMEOUT_ERROR;
  }
  ctx->timeout++;
  if(e <= 0.0) {  // fault0 has latched it by now; a new enable starts clean
    ctx->link_err = 0;
  }

  float d_cmd = PIN(d_cmd);
  float q_cmd = PIN(q_cmd);

  if(PIN(rev) > 0.0) {
    q_cmd *= -1.0;
    pos = minus(0, pos);
    vel *= -1.0;  // the f3 extrapolates pos and decouples with vel
  }


  uint32_t tx_size = 0;

  switch(flash_state) {
    case SLAVE_IN_APP:
      if(e > 0.0) {
        ctx->to_hv.packet_to_hv.d_cmd        = d_cmd;
        ctx->to_hv.packet_to_hv.q_cmd        = q_cmd;
        ctx->to_hv.packet_to_hv.flags.enable = 1;
      } else {
        ctx->to_hv.packet_to_hv.d_cmd        = 0.0;
        ctx->to_hv.packet_to_hv.q_cmd        = 0.0;
        ctx->to_hv.packet_to_hv.flags.enable = 0;
      }
      ctx->to_hv.packet_to_hv.flags.ignore_fault_pin = PIN(ignore_fault_pin) > 0.0;
      ctx->to_hv.packet_to_hv.flags.sbrake           = e <= 0.0 && PIN(sbrake) > 0.0;
      ctx->to_hv.packet_to_hv.flags.sbrake_arm       = PIN(sbrake_arm) > 0.0;
      ctx->to_hv.packet_to_hv.flags.cmd_type         = PIN(cmd_mode);
      ctx->to_hv.packet_to_hv.flags.phase_type       = PIN(phase_mode);
      ctx->to_hv.packet_to_hv.pos                    = pos;
      ctx->to_hv.packet_to_hv.vel                    = vel;

      ctx->to_hv.packet_to_hv.header.slave_addr = 0;
      ctx->to_hv.packet_to_hv.header.flags.cmd  = WRITE_CONF;
      ctx->to_hv.packet_to_hv.header.flags.counter++;
      ctx->to_hv.packet_to_hv.header.len        = (sizeof(packet_to_hv_t) - sizeof(stmbl_talk_header_t)) / 4;
      ctx->to_hv.packet_to_hv.header.conf_addr  = ctx->conf_addr;
      ctx->to_hv.packet_to_hv.header.config.f32 = ctx->config.data[ctx->conf_addr++];

      uint8_t buf[1];
      if(rb_read(&hv_tx_buf, buf, 1)) {
        ctx->to_hv.packet_to_hv.flags.buf = buf[0];
      } else {
        ctx->to_hv.packet_to_hv.flags.buf = 0x0;
      }

      tx_size = sizeof(packet_to_hv_t);

      ctx->conf_addr %= sizeof(f3_config_data_t) / 4;

      if(send_verify) {
        send_verify    = 0;
        ctx->addr      = 0;
        ctx->timeout   = 0;
        verify.words   = 0;
        verify.bad     = 0;
        verify.vi_crc  = 0;
        verify.vi_size = 0;
        verify.total   = ((uint32_t) & (_binary_obj_hvf3_hvf3_bin_size)) / 4 + VERIFY_EXTRA;
        verify.state   = 1;
        flash_state    = VERIFY_FLASH;
      }
      if(send_to_bootloader) {
        send_to_bootloader = 0;
        flash_state        = SEND_TO_BOOTLOADER;
        ctx->timeout       = 0;
        // TODO: check f3 crc, size, ...
      }
      break;

    case SEND_TO_BOOTLOADER:  // fix
      ctx->to_hv.packet_to_hv.header.slave_addr = 0;  // the app; FLASH_FAILED left 255 and the bootloader len here
      ctx->to_hv.packet_to_hv.header.len        = (sizeof(packet_to_hv_t) - sizeof(stmbl_talk_header_t)) / 4;
      ctx->to_hv.packet_to_hv.header.flags.cmd = BOOTLOADER;
      ctx->to_hv.packet_to_hv.flags.buf        = 0x0;
      ctx->to_hv.packet_to_hv.header.flags.counter++;
      ctx->to_hv.packet_to_hv.d_cmd        = 0.0;
      ctx->to_hv.packet_to_hv.q_cmd        = 0.0;
      ctx->to_hv.packet_to_hv.flags.enable = 0;
      ctx->to_hv.packet_to_hv.flags.sbrake     = 0;
      ctx->to_hv.packet_to_hv.flags.sbrake_arm = 0;

      tx_size = sizeof(packet_to_hv_t);

      if(ctx->timeout > 10) {
        ctx->timeout = 0;
        flash_state  = ERASE_FLASH;
      }
      break;

    case ERASE_FLASH:
      ctx->to_hv.packet_to_hv.header.slave_addr = 255;
      ctx->to_hv.packet_to_hv.header.flags.cmd  = NO_CMD;
      ctx->to_hv.packet_to_hv.header.flags.counter++;
      ctx->to_hv.packet_to_hv.header.len        = (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4;
      ctx->to_hv.packet_to_hv.header.conf_addr  = 0;
      ctx->to_hv.packet_to_hv.header.config.f32 = 0;
      ctx->to_hv.packet_to_hv_bootloader.addr   = 0;
      ctx->to_hv.packet_to_hv_bootloader.value  = 0;
      ctx->to_hv.packet_to_hv_bootloader.cmd    = BOOTLOADER_OPCODE_PAGEERASE;

      tx_size = sizeof(packet_bootloader_t);

      ctx->addr = 0;

      // flash_state = SLAVE_IN_APP;

      // 12 s: a full erase takes 1-2 s and a lost reply costs another round
      // (the f3 bootloader skips pages that are already blank on a repeat)
      if(ctx->timeout > 60000) {
        ctx->timeout = 0;
        flash_state  = FLASH_FAILED;
      }
      break;

    case SEND_APP:
      ctx->to_hv.packet_to_hv.header.flags.counter++;
      ctx->to_hv.packet_to_hv.header.len       = (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4;
      ctx->to_hv.packet_to_hv_bootloader.addr  = 0x08004000 + ctx->addr * 4;
      ctx->to_hv.packet_to_hv_bootloader.value = ((uint32_t *)&_binary_obj_hvf3_hvf3_bin_start)[ctx->addr];
      ctx->to_hv.packet_to_hv_bootloader.cmd   = BOOTLOADER_OPCODE_WRITE;

      tx_size = sizeof(packet_bootloader_t);

      if(ctx->timeout > 10) {
        ctx->timeout = 0;
        flash_state  = FLASH_FAILED;
      }
      break;

    case CRC_CHECK:
      if(crc_wait) {  // listening for the answer to the last check
        crc_wait--;
      } else {
        ctx->to_hv.packet_to_hv.header.flags.counter++;
        ctx->to_hv.packet_to_hv.header.len     = (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4;
        ctx->to_hv.packet_to_hv_bootloader.cmd = BOOTLOADER_OPCODE_CRCCHECK;

        tx_size = sizeof(packet_bootloader_t);
      }

      if(ctx->timeout > 2000) {
        ctx->timeout = 0;
        flash_state  = FLASH_FAILED;
      }
      break;
    case SEND_TO_APP:
      ctx->to_hv.packet_to_hv.header.flags.cmd = DO_RESET;
      ctx->to_hv.packet_to_hv.header.flags.counter++;
      ctx->to_hv.packet_to_hv.header.len     = (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4;
      ctx->to_hv.packet_to_hv_bootloader.cmd = BOOTLOADER_OPCODE_NOP;

      tx_size = sizeof(packet_bootloader_t);

      if(ctx->timeout > 2000) {
        ctx->timeout = 0;
        flash_state  = SLAVE_IN_APP;
      }
      break;
    case VERIFY_FLASH:
      ctx->to_hv.packet_to_hv.header.slave_addr = 255;
      ctx->to_hv.packet_to_hv.header.flags.cmd  = NO_CMD;
      ctx->to_hv.packet_to_hv.header.flags.counter++;
      ctx->to_hv.packet_to_hv.header.len        = (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4;
      ctx->to_hv.packet_to_hv.header.conf_addr  = 0;
      ctx->to_hv.packet_to_hv.header.config.f32 = 0;
      ctx->to_hv.packet_to_hv_bootloader.addr   = 0x08004000 + ctx->addr * 4;
      ctx->to_hv.packet_to_hv_bootloader.value  = 0;
      ctx->to_hv.packet_to_hv_bootloader.cmd    = BOOTLOADER_OPCODE_READ;

      tx_size = sizeof(packet_bootloader_t);

      if(ctx->timeout > 200) {  // 40 ms without an answer: the f3 is not in its bootloader
        ctx->timeout = 0;
        verify.state = 3;
        flash_state  = SLAVE_IN_APP;
      }
      break;

    case FLASH_FAILED:
      // hv_update and hv_verify work from here
      if(probe_app) {  // an app packet with the bridge off; only an app answers it
        ctx->to_hv.packet_to_hv.d_cmd            = 0.0;
        ctx->to_hv.packet_to_hv.q_cmd            = 0.0;
        ctx->to_hv.packet_to_hv.pos              = pos;
        ctx->to_hv.packet_to_hv.vel              = vel;
        ctx->to_hv.packet_to_hv.flags.enable     = 0;
        ctx->to_hv.packet_to_hv.flags.sbrake     = 0;
        ctx->to_hv.packet_to_hv.flags.sbrake_arm = 0;
        ctx->to_hv.packet_to_hv.flags.buf        = 0x0;
        ctx->to_hv.packet_to_hv.flags.ignore_fault_pin = PIN(ignore_fault_pin) > 0.0;
        ctx->to_hv.packet_to_hv.flags.cmd_type   = PIN(cmd_mode);
        ctx->to_hv.packet_to_hv.flags.phase_type = PIN(phase_mode);

        ctx->to_hv.packet_to_hv.header.slave_addr = 0;
        ctx->to_hv.packet_to_hv.header.flags.cmd  = WRITE_CONF;
        ctx->to_hv.packet_to_hv.header.flags.counter++;
        ctx->to_hv.packet_to_hv.header.len        = (sizeof(packet_to_hv_t) - sizeof(stmbl_talk_header_t)) / 4;
        ctx->to_hv.packet_to_hv.header.conf_addr  = ctx->conf_addr;
        ctx->to_hv.packet_to_hv.header.config.f32 = ctx->config.data[ctx->conf_addr++];
        ctx->conf_addr %= sizeof(f3_config_data_t) / 4;

        tx_size = sizeof(packet_to_hv_t);
      } else {  // a bootloader NOP
        ctx->to_hv.packet_to_hv.header.slave_addr = 255;
        ctx->to_hv.packet_to_hv.header.flags.cmd  = NO_CMD;
        ctx->to_hv.packet_to_hv.header.flags.counter++;
        ctx->to_hv.packet_to_hv.header.len        = (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4;
        ctx->to_hv.packet_to_hv.header.conf_addr  = 0;
        ctx->to_hv.packet_to_hv.header.config.f32 = 0;
        ctx->to_hv.packet_to_hv_bootloader.addr   = 0;
        ctx->to_hv.packet_to_hv_bootloader.value  = 0;
        ctx->to_hv.packet_to_hv_bootloader.cmd    = BOOTLOADER_OPCODE_NOP;

        tx_size = sizeof(packet_bootloader_t);
      }

      if(send_verify) {
        send_verify    = 0;
        ctx->addr      = 0;
        ctx->timeout   = 0;
        verify.words   = 0;
        verify.bad     = 0;
        verify.vi_crc  = 0;
        verify.vi_size = 0;
        verify.total   = ((uint32_t) & (_binary_obj_hvf3_hvf3_bin_size)) / 4 + VERIFY_EXTRA;
        verify.state   = 1;
        flash_state    = VERIFY_FLASH;
      } else if(send_to_bootloader) {
        send_to_bootloader = 0;
        flash_state        = SEND_TO_BOOTLOADER;
        ctx->timeout       = 0;
      }
      break;
  }

  if(ctx->send_state > 1) {
    if(flash_state != SLAVE_IN_APP) {
      tx_size = 0;
    }
    ctx->send_state = 0;
  }
  ctx->send_state++;

  if(flash_state == FLASH_FAILED && tx_size) {
    probe_app = !probe_app;
  }

  // rx is re-armed below only together with a send, and the last good
  // reply stays in the buffer until then: a paused tick that skipped the
  // re-arm parsed that stale reply again on every tick, kept ctx->timeout at
  // 0, and the f4 never saw the loss (bench, X, 8 Oct: fault 6 instead of 9).
  uint32_t rx_rearm = tx_size;
  if(hv_pause_left > 0.0) {
    hv_pause_left -= period;
    if(flash_state == SLAVE_IN_APP) {
      tx_size  = 0;
      rx_rearm = 1;
    }
  }

  if(flash_state == CRC_CHECK && tx_size) {
    crc_wait = CRC_LISTEN;
  }

  if(tx_size) {
    CRC->CR = CRC_CR_RESET;
    ctx->to_hv.packet_to_hv.header.crc = crc_calc_block((uint32_t *)&(ctx->to_hv.packet_to_hv.header.slave_addr), tx_size / 4 - 1);

    //start DMA TX transfer
    dma_stream_stop(UART_DRV_TX_DMA);
    LL_DMA_SetDataLength(UART_DRV_DMA, UART_DRV_TX_DMA_STREAM, tx_size);
    LL_DMA_EnableStream(UART_DRV_DMA, UART_DRV_TX_DMA_STREAM);
  }

  if(rx_rearm) {
    // clear uart faults
    PIN(uart_sr) = UART_DRV->SR;
    PIN(uart_dr) = UART_DRV->DR;

    //start DMA RX transfer. The stream is usually stopped mid-transfer
    //(sized for the larger bootloader packet), so set NDTR again rather than
    //rely on the reload (RM0090 10.3.17 step 5)
    dma_stream_stop(UART_DRV_RX_DMA);
    LL_DMA_SetDataLength(UART_DRV_DMA, UART_DRV_RX_DMA_STREAM, MAX(sizeof(packet_from_hv_t), sizeof(packet_bootloader_t)));
    LL_DMA_EnableStream(UART_DRV_DMA, UART_DRV_RX_DMA_STREAM);
  }


  PIN(state) = flash_state;
}

void hv_pause(char *ptr) {
  int ms = 0;
  if(!ptr || sscanf(ptr, "%i", &ms) != 1 || ms <= 0 || ms > 2000) {
    printf("usage: hv_pause <ms>, 1 to 2000\n");
    return;
  }
  hv_pause_left = ms * 0.001;
  printf("not sending to the f3 for %i ms\n", ms);
}
COMMAND("hv_pause", hv_pause, "stop sending to the f3 for <ms> (link loss test)");

void send_boot(char *ptr) {
  send_to_bootloader = 1;
}
COMMAND("hv_update", send_boot, "try hv update");

void hv_verify(char *ptr) {
  if(flash_state == SLAVE_IN_APP || flash_state == FLASH_FAILED) {
    send_verify = 1;
  } else {
    printf("hv_verify: busy\n");
  }
}
COMMAND("hv_verify", hv_verify, "read the f3 app area back from its bootloader and compare it with the embedded image");

static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct hv_ctx_t *ctx = (struct hv_ctx_t *)ctx_ptr;
  // struct hv_pin_ctx_t *pins = (struct hv_pin_ctx_t *)pin_ptr;
  char c;
  while(rb_getc(&hv_rx_buf, &c)) {
    printf("%c", c);
  }

  static uint32_t verify_shown = 0, verify_tenth = 0;
  if(verify.state == 1 && verify.total) {
    uint32_t tenth = verify.words * 10 / verify.total;
    if(tenth != verify_tenth) {
      verify_tenth = tenth;
      printf("hv_verify: %lu%%, %lu bad so far\n", tenth * 10, verify.bad);
    }
    verify_shown = 0;
  } else if(verify.state >= 2 && !verify_shown) {
    verify_shown = 1;
    verify_tenth = 0;
    if(verify.state == 3) {
      printf("hv_verify: no answer at word %lu: the f3 is not in its bootloader\n", verify.words);
    }
    printf("hv_verify: %lu of %lu words read (image %lu + %u past its end), %lu differ\n", verify.words, verify.total, verify.total - VERIFY_EXTRA, VERIFY_EXTRA, verify.bad);
    printf("hv_verify: f3 version_info image_crc 0x%08lx image_size %lu, embedded image_crc 0x%08lx image_size %lu\n", verify.vi_crc, verify.vi_size,
           ((uint32_t *)&(_binary_obj_hvf3_hvf3_bin_start))[0x188 / 4], ((uint32_t *)&(_binary_obj_hvf3_hvf3_bin_start))[0x18C / 4]);
    for(uint32_t i = 0; i < verify.bad && i < VERIFY_LIST; i++) {
      printf("hv_verify: 0x%08lx f3 0x%08lx image 0x%08lx\n", verify.addr[i], verify.got[i], verify.want[i]);
    }
  }

  static flash_state_t last_flash_state = SLAVE_IN_APP;
  static uint32_t last_addr             = 0;

  if(flash_state == SEND_APP && ctx->addr >= last_addr + 1024) {
    printf("hv_update: status: %i%%\n", (int)(100.0 * ctx->addr * 4. / (float)((uint32_t) & (_binary_obj_hvf3_hvf3_bin_size))));
    last_addr = ctx->addr;
  }

  if(last_flash_state != flash_state) {
    switch(flash_state) {
      case SLAVE_IN_APP:
        printf("hv_update: SLAVE_IN_APP\n");
        break;
      case VERIFY_FLASH:
        printf("hv_verify: reading the f3 app area\n");
        last_addr = 0;
        break;
      case SEND_TO_BOOTLOADER:
        printf("hv_update: SEND_TO_BOOTLOADER\n");
        last_addr = 0;
        break;
      case ERASE_FLASH:
        printf("hv_update: ERASE_FLASH\n");
        last_addr = 0;
        break;
      case SEND_APP:
        printf("hv_update: SEND_APP\n");
        break;
      case CRC_CHECK:
        printf("hv_update: CRC_CHECK\n");
        last_addr = 0;
        break;
      case SEND_TO_APP:
        printf("hv_update: SEND_TO_APP\n");
        last_addr = 0;
        break;
      case FLASH_FAILED:
        if(last_flash_state == VERIFY_FLASH) {
          printf("hv_update: the f3 is in its bootloader; hv_update flashes it\n");
          break;
        }
        if(crc_nak) {
          printf("hv_update: the f3 bootloader says the app CRC is wrong\n");
        } else if(last_flash_state == CRC_CHECK) {
          printf("hv_update: no answer to CRC_CHECK\n");
        }
        printf("hv_update: FLASH_FAILED\n");
        last_addr = 0;
        break;
    }
    last_flash_state = flash_state;
  }
}

hal_comp_t hv_comp_struct = {
    .name      = "hv",
    .nrt       = nrt_func,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct hv_ctx_t),
    .pin_count = sizeof(struct hv_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
