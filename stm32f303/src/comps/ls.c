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

/**
* ## Brief
* `ls` is the F3 (HV board) end of the serial link to the F4 (3 Mbaud UART, USART3 with DMA, CRC checked packets). It receives the current or voltage command, rotor angle, velocity, enable, mode and braking flags, plus one word of configuration per packet, and puts them on pins for the current loop, ramping the d/q command between packets. It answers with the measured d/q currents, output voltages, the fault code and one word of state per packet. It also locks the F3's PWM period to the F4's packet timing (trimming one period per packet), carries the F3 terminal (one byte each way per packet), holds the enable off until the whole configuration has arrived, disables the bridge when packets stop and can then ask `io0` for a short-circuit brake. It is loaded by `stm32f303/src/main.c` as `ls0` (rt_prio 0.6, the first in the chain). The F4 end of the link is the `hv` component.
*
* The F4 sends a packet every 200 us (5 kHz). The F3 rt runs once per PWM period at `PWM_FREQ`, a build option (`stm32f303/Makefile`, 10, 15 or 20 kHz, default 15 kHz), so there are `PWM_TICKS_PER_PACKET` = `PWM_FREQ / 5000` rt ticks per packet (3 at 15 kHz). The timings below are given in packets and hold at every PWM frequency; tick counts are for 15 kHz.
*
* ## Component Explanation
*
* 1. **Wiring on the F3** (fixed in main.c):
* - Commands out: `d_cmd`/`q_cmd` to `curpid0.id_cmd/iq_cmd` (and to `hv0` for the dead time compensation), `pos` to `dq0` and `emf0`, `pos_v` (see below) to `idq0`, `vel` to `curpid0`, `en` to `curpid0.en`, `io0.hv_en` and `emf0.en`, `cmd_mode` to `curpid0` and `hv0`, `phase_mode` to `dq0`, `idq0` and `hv0`, `sbrake` to `io0.sbrake`.
* - Configuration out: `r`, `l` (as `ld`), `lq`, `psi`, `cur_bw`, `cur_ff`, `cur_ind`, `max_cur` and `pwm_volt` to `curpid0`, `oc_cur` (as `max_cur`), `dac` and `ignore_fault_pin` to `io0`, `drop_k`, `drop_knee` and `arr` to `hv0`, `emf_run`/`emf_sel`/`emf_pp` to `emf0.run/sel/pp`, `fault` to `io0.led`.
* - Feedback in: `id_fb`/`iq_fb`/`y` from `dq0`, `ud_fb`/`uq_fb` from `curpid0`, `dc_volt` (`io0.udc`), `udc_duty` (`io0.udc_duty`), `hv_temp`, `hv_temp_ok`, `mot_temp`, `u_fb`/`v_fb`/`w_fb` and `fault_in` from `io0`, `emf_val` from `emf0.val`, `duty_max` from `hv0.duty_max`.
*
* 2. **Receiving** (rt):
* - The RX DMA writes each 32 byte `packet_to_hv_t` into the context. When all 32 bytes have arrived, the packet is accepted if `slave_addr` is 0, `len` matches and the CRC is right.
* - The header command is handled: `WRITE_CONF` stores the float into `f3_config_data_t` at `conf_addr`, `READ_CONF` sets the address of the next state word to send, `DO_RESET` resets the F3, `BOOTLOADER` sets a backup register flag and resets into the bootloader. Addresses past this image's config are ignored (the F4 may be newer than the F3 image; the layout is append only).
* - The process data goes to `phase_mode`, `cmd_mode`, `ignore_fault_pin`, `sbrake`, `sbrake_arm`, `pos` and `vel`, the d/q command becomes the new ramp target (item 4), and all configuration words go to their pins (`r`, `l`, `psi`, `cur_bw`, `cur_ff`, `cur_ind`, `max_y`, `max_cur`, `dac`, `drop_k`, `lq`, `emf_run`, `emf_sel`, `emf_pp`, `drop_knee`, `oc_cur`). A nonzero terminal byte is pushed into the F3's terminal RX buffer (read by `term0`).
* - Only one tick per packet sees a new one. In the others `pos` is extrapolated with `pos += vel * period` (not wrapped), until the timeout (`2 * PWM_TICKS_PER_PACKET - 1` ticks, 5 at 15 kHz).
* - After a whole packet has been taken (good or bad), the RX DMA is re-armed in the same tick instead of waiting for the idle line: at 10 and 20 kHz the tick can come before the idle flag, and the old packet would have been taken a second time.
*
* 3. **Enable only after a full configuration** (rt):
* - The F4 sends the 17 configuration words one per packet, so right after an F3 reset `r`, `l`, `max_cur`, `dac` etc. are still 0 for a while. `ls` keeps one bit per word written, and `conf_ok` becomes 1 once every word has been written at least once (the bits are cleared in rt_start, which also clears the ramp targets). Until then `en` stays 0 whatever the F4 asks; after that `en` follows the F4's enable flag.
* - A link gap of 10 ms (`PWM_FREQ / 100` ticks) clears the bits and `conf_ok` again: a restarted F4 sends its configuration from word 0, and the bridge stays off until the whole set has come round once more.
* - This assumes the F4 and F3 images come from the same build (same config layout): an older F4 that sends fewer words never gets `conf_ok`.
*
* 4. **Command ramp** (rt):
* - Stepped straight in, the d/q command would reach the current loop as a 5 kHz staircase. With `ramp` = 1 (the default, set in hw_init) each new packet's command is reached in `PWM_TICKS_PER_PACKET` equal steps, starting in the packet's own tick, so the command is a linear ramp that lags the F4 by about half a packet:
* ```c
* d_step = (d_tgt - d_cmd) / PWM_TICKS_PER_PACKET;  // on each packet, q alike
* d_cmd += d_step;                                  // in the first PWM_TICKS_PER_PACKET ticks after it
* ```
* - In any later tick (a late or missing packet), and always with `ramp` = 0, `d_cmd`/`q_cmd` are set to the last packet's command (which also cleans up the float sum).
*
* 5. **Voltage angle**:
* - `dq0` transforms the currents at the angle of the current sample. The voltage computed from them is preloaded and applied over the next PWM period, so on average it lands 1.5 periods later. `ls` therefore outputs a second angle, `pos_v = pos + vel * v_lead * period` (wrapped by `sincos_fast`), for `idq0` (and, through `idq0.si_out/co_out`, for `hv0`'s dead time reference).
* - With `v_lead`, the F4's `hv0.adv` only has to cover the encoder to sample latency.
*
* 6. **Answering** (rt):
* - After each good packet, one `packet_from_hv_t` is sent by DMA: `fault_in`, `id_fb`, `iq_fb`, `ud_fb`, `uq_fb`, one byte from the terminal TX buffer, and one word of `f3_state_data_t` (in order `u_fb`, `v_fb`, `w_fb`, `hv_temp`, `mot_temp`, `core_temp`, `dc_volt`, `pwm_volt`, `y`, `emf_val`, two unused words (0), `pwm_freq` (the `PWM_FREQ` the image was built for, read by the F4 as `hv0.pwm_freq`), `link_to` (the `timeout` counter), `hv_temp_ok`), cycling through the 15 words one per packet.
* - A UART receive timeout (16 bit times of idle line) after a partial packet or noise resets the RX DMA so the next packet starts at byte 0. The idle line that follows a whole packet is ignored, as the DMA was already re-armed (item 2) and the next packet may have started.
*
* 7. **PWM phase lock** (rt):
* - On the first tick that finds a packet being received (more than `window` bytes from either end), `arr` is set to `PWM_RES - inc` if more than `dma_pos_cmd` bytes have arrived and to `PWM_RES + inc` if fewer. In every other tick it is `PWM_RES` (7200, 4800 or 3600 at 10, 15 or 20 kHz). So the period is trimmed at most once per packet (at 20 kHz several ticks fall inside one packet, and their votes would not cancel). `hv0` writes it to the PWM timer, so the F3's PWM and rt tick slide until they are in a fixed phase to the F4's packets.
*
* 8. **Timeout and braking on link loss** (rt):
* - After two missed packets (0.4 ms, more than 5 rt ticks at 15 kHz without a good packet), `en` and `vel` are set to 0, `timeout` is counted up every tick and the communication fault is 1. The bridge is thus switched off by `io0` and `curpid0`. The internal tick counter saturates, so a long loss cannot wrap it and re-enable the bridge.
* - On the first timeout tick `ls` decides whether to brake: only if the F4's last packet had `sbrake_arm` set and the motor was driven (`en`) or already braking (`sbrake`). If so, `sbrake` goes to 1 once four packets are missed (0.8 ms, so one or two packets lost to noise only take the gates off until the next one) and is held for `sbrake_time` from then, and `io0` short-circuit brakes the motor on its own; otherwise `sbrake` is 0.
* - While packets arrive, `sbrake` simply follows the F4's flag (the F4 only sets it while `en` is 0).
*
* 9. **Faults and voltage limit** (rt):
* - `fault` only drives the LED blink count (`io0.led`). The F4 gets `fault_in` (the `io0` fault code) and detects CRC errors and timeouts on its own side.
* - `pwm_volt`, the largest voltage vector `curpid0` may output, depends on `phase_mode`: `udc_duty / sqrt(3) * duty` for 120 deg 3 phase, `udc_duty / sqrt(2) * duty` for 90 deg 3 phase, `udc_duty * duty` for 90 deg 4 phase and the 180 deg modes, else 0. `udc_duty` is the fast DC link value `hv0` divides by, so the ceiling follows link sag and regen. `duty` is `duty_max` from `hv0` (0.91 with the default 3 us minimum on and off times at 15 kHz).
*
* {{% hint warning %}}
* - The communication fault codes 1 (timeout) and 3 (CRC) are not `fault_t` codes (they would read as `CMD_ERROR` and `COM_FB_ERROR`). They are only used for the LED blink count and never reach the F4.
* - A config word still takes effect as soon as it arrives, one per packet; `conf_ok` only guards the first enable after a reset or a 10 ms link gap, not later changes while enabled.
* - `sbrake_time` is reset to 1 s by rt_start and is not an F4 config word, so the F4's own braking time does not apply to a link loss.
* {{% /hint %}}
*/


HAL_COMP(ls);

//process data from LS
HAL_PIN(d_cmd);  // *output*, D-axis command from the F4, current (A) or voltage (V) depending on cmd_mode, ramped between packets
HAL_PIN(q_cmd);  // *output*, Q-axis command from the F4, current (A) or voltage (V) depending on cmd_mode, ramped between packets
// The f4 sends d/q every PWM_TICKS_PER_PACKET ticks (3 at 15 kHz); stepped straight in, the command carries
// a 5 kHz staircase into the current loop. ramp = 1 spreads each step over
// the LS_RAMP_TICKS ticks to the next packet, a linear ramp that lags the
// step by half an f4 period. 0 = step.
HAL_PIN(ramp);  // *parameter*, 1 = ramp d_cmd/q_cmd to each new command over PWM_TICKS_PER_PACKET ticks, 0 = step, default 1
#define LS_RAMP_TICKS PWM_TICKS_PER_PACKET
// link loss after two missed packets (0.4 ms): 5 ticks at 15 kHz
#define LS_TIMEOUT_TICKS (2 * PWM_TICKS_PER_PACKET - 1)
// short-circuit braking on a link loss only after four missed packets (0.8
// ms): one or two lost to noise just take the gates off until the next packet
#define LS_SBRAKE_TICKS (4 * PWM_TICKS_PER_PACKET - 1)
// a restarted f4 is silent far longer than 10 ms (boot, config load); a gap
// that long must deliver the whole config again before the next enable
#define LS_CONF_TICKS (PWM_FREQ / 100)
HAL_PIN(pos);  // *output*, Electrical rotor angle at the current sample (rad), from the F4, extrapolated between packets, to angle0.pos_fb
HAL_PIN(vel);  // *output*, Electrical velocity from the F4 (rad/s), 0 on timeout, to angle0.vel_fb
// The angle the voltage computed this tick lands at, on average: the
// compares are preloaded, so it is applied over the next period, 1.5
// periods after the current sample that dq0 transforms at pos. idq0 and
// hv0's dead-time reference use it; hv0.adv on the f4 is then the encoder
// to sample latency alone. v_lead in periods, default 1.5, 0 = one
// angle for both.
HAL_PIN(pos_v);  // *output*, Voltage angle (rad), pos + v_lead periods of motion, for idq0 and hv0
HAL_PIN(conf_ok);  // *output*, 1 once every config word has been received since rt_start or a 10 ms link gap, en stays 0 before
HAL_PIN(v_lead);  // *parameter*, Voltage angle lead in PWM periods for angle0.v_lead, default 1.5, 0 = same angle as pos
HAL_PIN(en);  // *output*, Enable from the F4, 0 until conf_ok and on timeout

// config data from LS
HAL_PIN(cmd_mode);  // *output*, Command mode from the F4, 0 = voltage, 1 = current
HAL_PIN(phase_mode);  // *output*, Phase mode from the F4, 0 = 90 deg 3ph, 1 = 90 deg 4ph, 2 = 120 deg 3ph, 3 = 180 deg 2ph, 4 = 180 deg 3ph
HAL_PIN(r);  // *output*, Motor resistance from the F4 config (Ohm)
HAL_PIN(l);  // *output*, Motor inductance, d-axis, from the F4 config (H)
HAL_PIN(lq);  // *output*, Q-axis inductance for curpid0.lq (H), config lq, or l when that is 0
HAL_PIN(psi);  // *output*, Magnet flux linkage from the F4 config (Vs/rad)
HAL_PIN(cur_bw);  // *output*, Current loop bandwidth from the F4 config (rad/s)
HAL_PIN(cur_ff);  // *output*, Resistance feed forward factor from the F4 config
HAL_PIN(cur_ind);  // *output*, Back EMF and cross coupling feed forward factor from the F4 config
HAL_PIN(max_y);  // *output*, From the F4 config, not used on the F3
HAL_PIN(max_cur);  // *output*, Maximum current from the F4 config (A), for curpid0 and io0
HAL_PIN(oc_cur);  // *output*, The F4's max_cur before fault0.scale (A), for io0's software trip, 0 from an older F4 (io0 then trips at ABS_MAX_CURRENT only)
HAL_PIN(dac);  // *output*, Overcurrent comparator DAC value from the F4 config, for io0
HAL_PIN(drop_k);  // *output*, Dead time compensation factor from the F4 config, for hv0
HAL_PIN(emf_run);  // *output*, emf0 control from the F4 config, 1 = sum, 0 = hold, -1 = clear
HAL_PIN(emf_sel);  // *output*, emf0 result number to return in emf_val, from the F4 config
HAL_PIN(emf_pp);  // *output*, Pole pairs for emf0's per pole bins, from the F4 config
HAL_PIN(drop_knee);  // *output*, Dead time compensation curve knee (A) from the F4 config, 0 = latched sign, for hv0

// process data to LS
HAL_PIN(dc_volt);  // *input*, DC link voltage (V), from io0.udc, sent to the F4
HAL_PIN(udc_duty);  // *input*, io0.udc_duty, the fast DC link value hv0 divides by, used for pwm_volt (V)
HAL_PIN(id_fb);  // *input*, Measured d-axis current (A), from dq0.d, sent to the F4
HAL_PIN(iq_fb);  // *input*, Measured q-axis current (A), from dq0.q, sent to the F4
HAL_PIN(ud_fb);  // *input*, D-axis output voltage (V), from curpid0.ud, sent to the F4
HAL_PIN(uq_fb);  // *input*, Q-axis output voltage (V), from curpid0.uq, sent to the F4

// state data to LS
HAL_PIN(hv_temp);  // *input*, Power stage temperature (degC), from io0, sent to the F4
HAL_PIN(hv_temp_ok);  // *input*, io0.hv_temp_ok, 0 = never read, 1 = live, 2 = held, sent to the F4
HAL_PIN(mot_temp);  // *input*, Motor temperature (degC), from io0, sent to the F4
HAL_PIN(core_temp);  // *input*, Core temperature, not measured, sent to the F4 as 0
HAL_PIN(fault_in);  // *input*, HV fault code from io0.fault, sent to the F4

// short-circuit braking request for io0: follows the f4's flag, and on a
// link loss brakes for sbrake_time if the f4 last sent sbrake_arm
HAL_PIN(sbrake);  // *output*, Short-circuit braking request for io0.sbrake, the F4's flag, or 1 for sbrake_time after a link loss
HAL_PIN(sbrake_arm);  // *output*, Flag from the F4, > 0 allows braking on a link loss
HAL_PIN(sbrake_time);  // *parameter*, Braking time after a link loss (s), set to 1 in rt_start
HAL_PIN(ignore_fault_pin);  // *output*, Ignore the driver fault pin, flag from the F4, for io0
HAL_PIN(y);  // *input*, Zero sequence current, mean of the phase currents (A), from dq0.y, sent to the F4
HAL_PIN(u_fb);  // *input*, U phase voltage (V), from io0.u, sent to the F4
HAL_PIN(v_fb);  // *input*, V phase voltage (V), from io0.v, sent to the F4
HAL_PIN(w_fb);  // *input*, W phase voltage (V), from io0.w, sent to the F4
HAL_PIN(emf_val);  // *input*, emf0 result number emf_sel, from emf0.val, sent to the F4

// misc
HAL_PIN(pwm_volt);  // *output*, Maximum output voltage vector for curpid0 (V), from dc_volt, phase_mode and duty_max
HAL_PIN(duty_max);  // *input*, Usable duty from hv0.duty_max, 0 = not wired (0.95 is used)
HAL_PIN(crc_error);  // *output*, Counter of packets with a bad CRC, address or length
HAL_PIN(crc_ok);  // *output*, Counter of good packets
HAL_PIN(timeout);  // *output*, Counter of rt ticks spent in timeout, sent to the F4 as link_to
HAL_PIN(dma_pos);  // *output*, Bytes received of the last partial packet ended by an idle line (debug)
HAL_PIN(idle);  // *output*, Counter of UART idle line events
HAL_PIN(fault);  // *output*, Max of the communication fault (1 = timeout, 3 = CRC) and fault_in, drives the LED

HAL_PIN(dma_pos2);  // *output*, Bytes of the current packet received at this rt tick (debug)
HAL_PIN(arr);  // *output*, PWM timer reload value for hv0.arr, PWM_RES, or PWM_RES +- inc on the first tick inside a packet
HAL_PIN(dma_pos_cmd);  // *parameter*, Phase lock target, bytes received at the rt tick, default 4
HAL_PIN(inc);  // *parameter*, Phase lock step on arr (timer ticks), default PWM_RES * 5 / 4800 (5 at 15 kHz)
HAL_PIN(window);  // *parameter*, Phase lock only acts more than this many bytes from either end of a packet, default 1

_Static_assert(sizeof(f3_config_data_t) / 4 < 32, "conf_seen has a bit per config word");

struct ls_ctx_t {
  uint32_t timeout;
  uint32_t lock_voted;  // the first tick in this packet has trimmed ARR
  float d_tgt, q_tgt;    // the last packet's command
  float d_step, q_step;  // per tick towards it while ramping
  uint32_t sbrake_loss;  // brake for this link loss
  uint32_t tx_addr;
  uint32_t conf_seen;  // bit per config word written since boot
  uint32_t rx_done;     // rx DMA re-armed after a whole packet, before its idle flag
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

  /*USART3 GPIO Configuration    
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
  config.pins.oc_cur  = 0.0;
  config.pins.dac     = 0.0;
  config.pins.drop_k  = 0.0;
  config.pins.lq      = 0.0;
  config.pins.emf_run = 0.0;
  config.pins.emf_sel = 0.0;
  config.pins.emf_pp  = 0.0;
  config.pins.drop_knee = 0.0;
  PIN(v_lead) = 1.5;
  PIN(ramp)   = 1.0;
  PIN(sbrake_time) = 1.0;

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
  ctx->lock_voted  = 0;
  ctx->sbrake_loss = 0;
  ctx->tx_addr     = 0;
  ctx->conf_seen   = 0;
  ctx->d_tgt = ctx->q_tgt = 0.0;
  ctx->d_step = ctx->q_step = 0.0;
  ctx->rx_done     = 0;
  ctx->send        = 0;
  PIN(crc_error)   = 0.0;
  PIN(crc_ok)      = 0.0;
  PIN(timeout)     = 0.0;
  PIN(sbrake)      = 0.0;
  PIN(sbrake_arm)  = 0.0;
  PIN(idle)        = 0.0;
  PIN(dma_pos_cmd) = 4;
  PIN(inc)         = PWM_RES * 5 / 4800;  // ARR step, 5 at 15 kHz, about 0.1 %
  PIN(window)      = 1;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ls_ctx_t *ctx      = (struct ls_ctx_t *)ctx_ptr;
  struct ls_pin_ctx_t *pins = (struct ls_pin_ctx_t *)pin_ptr;

  uint32_t dma_pos = sizeof(packet_to_hv_t) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);

  PIN(dma_pos2) = dma_pos;
  PIN(arr)      = PWM_RES;

  // Phase lock to the f4 packets: the first tick inside a packet compares
  // how far it has come with dma_pos_cmd and trims this period. Only the
  // first: at 20 kHz three ticks land inside a packet and their votes never
  // cancel. Between packets dma_pos is 0 or the whole packet.
  if(dma_pos > PIN(window) && dma_pos < sizeof(packet_to_hv_t) - PIN(window)) {
    if(!ctx->lock_voted) {
      ctx->lock_voted = 1;
      if(PIN(dma_pos_cmd) < dma_pos) {
        PIN(arr) = PWM_RES - PIN(inc);
      } else if(PIN(dma_pos_cmd) > dma_pos) {
        PIN(arr) = PWM_RES + PIN(inc);
      }
    }
  } else {
    ctx->lock_voted = 0;
  }

  uint32_t fault = 0;
  uint32_t rearm = 0;  // restart rx DMA this tick

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
      ctx->d_tgt            = ctx->packet_to_hv.d_cmd;
      ctx->q_tgt            = ctx->packet_to_hv.q_cmd;
      ctx->d_step           = (ctx->d_tgt - PIN(d_cmd)) * (1.0 / LS_RAMP_TICKS);
      ctx->q_step           = (ctx->q_tgt - PIN(q_cmd)) * (1.0 / LS_RAMP_TICKS);
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
      PIN(oc_cur)  = config.pins.oc_cur;
      PIN(dac)     = config.pins.dac;
      PIN(drop_k)  = config.pins.drop_k;
      PIN(lq)      = config.pins.lq > 0.0 ? config.pins.lq : config.pins.l;
      PIN(emf_run) = config.pins.emf_run;
      PIN(emf_sel) = config.pins.emf_sel;
      PIN(emf_pp)  = config.pins.emf_pp;
      PIN(drop_knee) = config.pins.drop_knee;
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

    // Re-arm rx now rather than on the idle flag. At 10 and 20 kHz this tick
    // can come less than the 16 bit idle time (5.3 us) after the last byte;
    // waiting for the flag would leave dma_pos at 32 for the next tick, which
    // would take the packet a second time and start the reply early enough to
    // run into the f4's next rx re-arm.
    rearm        = 1;
    ctx->rx_done = 1;
  } else if(ctx->timeout <= LS_TIMEOUT_TICKS) {  // if no packet and no timeout, advance pos by velovity
    PIN(pos) = PIN(pos) + PIN(vel) * period;
  }
  PIN(pos_v) = PIN(pos) + PIN(vel) * PIN(v_lead) * period;  // sincos_fast wraps

  // timeout counts the ticks since the packet: 0, 1, 2 are the ramp, a late
  // or missing packet lands on the target (also cleans up the float sum)
  if(PIN(ramp) > 0.0 && ctx->timeout < LS_RAMP_TICKS) {
    PIN(d_cmd) += ctx->d_step;
    PIN(q_cmd) += ctx->q_step;
  } else {
    PIN(d_cmd) = ctx->d_tgt;
    PIN(q_cmd) = ctx->q_tgt;
  }


  if(LL_USART_IsActiveFlag_RTO(USART3)) {  // idle line
    // timeout, framing and overrun flags in one ICR write; LL clears them one by one
    WRITE_REG(USART3->ICR, USART_ICR_RTOCF | USART_ICR_FECF | USART_ICR_ORECF);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10);

    PIN(idle)
    ++;
    if(ctx->rx_done) {
      // this idle ended a packet already taken and re-armed above; the next
      // packet may have started since, so leave its bytes alone
      ctx->rx_done = 0;
    } else {
      // a partial packet or noise: resync on the idle line
      PIN(dma_pos) = dma_pos;
      rearm        = 1;
    }
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);

    //ctx->send = 1;
  }

  if(rearm) {
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, sizeof(packet_to_hv_t));
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    dma_pos = 0;
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
    state.pins.unused0   = 0.0;
    state.pins.unused1   = 0.0;
    state.pins.hv_temp_ok = PIN(hv_temp_ok);
    state.pins.pwm_freq  = PWM_FREQ;
    state.pins.link_to   = PIN(timeout);

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

  if(ctx->timeout == LS_TIMEOUT_TICKS + 1) {
    // brake on the loss only if the motor was driven or already braking
    ctx->sbrake_loss = PIN(sbrake_arm) > 0.0 && (PIN(en) > 0.0 || PIN(sbrake) > 0.0);
  }
  if(ctx->timeout == LS_CONF_TICKS) {
    // a restarted f4 sends its config from word 0 again: no enable until the
    // whole set has come round once more, as after an f3 boot
    ctx->conf_seen = 0;
    PIN(conf_ok)   = 0.0;
  }
  if(ctx->timeout > LS_TIMEOUT_TICKS) {  //disable driver
    PIN(en)     = 0.0;
    PIN(vel)    = 0.0;
    PIN(sbrake) = ctx->sbrake_loss && ctx->timeout > LS_SBRAKE_TICKS && (float)(ctx->timeout - LS_SBRAKE_TICKS) * period < PIN(sbrake_time);
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
  // at the 3 us defaults; a fixed 0.95 would promise 4% more than the clamp
  // passes, and the loop would wind up against the clamp instead of its own
  // limit). Falls back to 0.95 when the pin is not wired.
  float duty = PIN(duty_max) > 0.0 ? PIN(duty_max) : 0.95;

  // TODO: sin = 0.5
  switch((uint16_t)PIN(phase_mode)) {
    case PHASE_90_3PH:  // 90°
      PIN(pwm_volt) = PIN(udc_duty) * M_SQRT1_2 * duty;
      break;

    case PHASE_90_4PH:  // 90°
      PIN(pwm_volt) = PIN(udc_duty) * duty;
      break;

    case PHASE_120_3PH:  // 120°
      PIN(pwm_volt) = PIN(udc_duty) * M_SQRT1_3 * duty;
      break;

    case PHASE_180_2PH:  // 180°
    case PHASE_180_3PH:  // 180°
      PIN(pwm_volt) = PIN(udc_duty) * duty;
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
