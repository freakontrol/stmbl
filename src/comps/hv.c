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

/**
* ## Brief
* `hv` runs on the F4 logic board and is its end of the UART link to the F3 power (HV) board. Every rt period it sends the current/voltage command, the commutation angle and velocity, the short brake flags and one word of current loop configuration to the F3, and unpacks the F3 reply (dq feedback, fault code and one word of state such as DC link voltage and temperatures). It also carries the `hv` terminal pass-through, the `hv_update` F3 firmware update and the `hv_verify` read back of the F3 flash. The F3 side of the link is the F3 `ls` component (stm32f303/src/comps/ls.c); the F3 bridge driver that shares this name is stm32f303/src/comps/hv.c, documented as page hv_f3.
*
* Typical wiring (conf/template/pid.txt + pmsm.txt): `hv0.en = fault0.en_out`, `hv0.sbrake = fault0.sbrake`, `hv0.sbrake_arm = fault0.sbrake_en`, `hv0.pos = angle0.pos`, `hv0.vel = angle0.vel`, `hv0.q_cmd = pmsm_ttc0.cur`, `hv0.d_cmd = pmsm_ttc0.id`, `hv0.scale = fault0.scale`, the motor constants from `conf0`, and `fault0.hv_error = hv0.fault`, `fault0.dc_volt = hv0.dc_volt`, `fault0.dc_cur = hv0.dc_cur`, `iit0.cur = hv0.abs_cur`. conf/template/id_pmsm.txt links the `emf_*` pins and `pwm_freq` to `idpmsm0`, the IPM templates (e.g. conf/template/ipm_im06b50gc1.txt) read `ipm0.f_sw = hv0.pwm_freq`, conf/template/id_tune.txt drives `drop_k` from `idtune0`.
*
* ## Component Explanation
*
* 1. **Link and packets**:
* - USART `UART_DRV` (USART2) at `DATABAUD` = 3 Mbaud, 8N1, 8x oversampling, TX and RX both by DMA (set up with the STM32CubeF4 LL drivers); packets use the stmbl_talk header with an STM32 hardware CRC32 (`crc_calc_block()`, inc/f4_ll_util.h) over everything after the CRC field.
* - F4 to F3 (`packet_to_hv_t`, 32 bytes): `d_cmd`, `q_cmd`, `pos`, `vel`, the flags `enable`, `cmd_type` (from `cmd_mode`), `phase_type` (from `phase_mode`), `ignore_fault_pin`, `sbrake`, `sbrake_arm`, one terminal byte, plus one config word (`conf_addr` + float).
* - F3 to F4 (`packet_from_hv_t`, 32 bytes): `id_fb`, `iq_fb`, `ud_fb`, `uq_fb`, `fault`, one terminal byte, plus one state word.
* - The exchange is one request/answer per rt period: the answer to the packet sent in period n is read at the start of period n+1.
* - Each period both DMA streams are stopped with `dma_stream_stop()` (inc/dma_util.h: waits until EN reads 0 and clears all five stream flags) before they are re-armed. The RX stream is sized for the larger bootloader packet, so it is usually stopped mid-transfer; its NDTR is set again before every re-arm.
*
* 2. **Commands sent (rt)**:
* - `pos` is advanced by `vel * adv` and wrapped with `mod()` before sending: `pos_sent = mod(pos + vel * adv)`. `adv` is meant to cover only the latency from the encoder reading to the F3 current sample: the F3 applies its output voltage at its own angle `ls0.pos_v = pos + vel * v_lead * period` (`v_lead` default 1.5 periods), so re-run id_tune after updating from older firmware, `adv` drops by about 100 us. `vel` is sent too; the F3 uses it to extrapolate `pos` between packets, for that voltage angle and for the BEMF decoupling of its current loop.
* - If `en > 0`, `d_cmd`/`q_cmd` are sent and the enable flag is set; otherwise both are sent as 0 and the F3 is disabled.
* - The F3 ignores the enable flag until it has received every config word once since its boot (`ls0.conf_ok`), so after an F3 reset the power stage stays off for at least one full config round (17 periods). The F3 also forgets the received words after 10 ms without packets (`LS_CONF_TICKS`), since a restarted F4 starts its round at word 0 again, so after a link loss the enable waits for a full round as well. This assumes the F4 and F3 images come from the same build (same config layout), which `hv_update` ensures.
* - In current mode (`cmd_mode` = 1) the command is closed by the F3 current loop.
*
* 3. **Short-circuit braking flags**:
* - The `sbrake` flag is sent only while `en <= 0` and `sbrake > 0`: the F3 (io.c) then turns all three low sides on and chops the phase current at its `io0.sbrake_cur` (default `max_cur`, at most 0.8 x the overcurrent limit). It refuses after any trip since the last enable and stops for good on a trip while braking; that trip is reported in `fault` until the request goes away.
* - `sbrake_arm > 0` lets the F3 brake on its own (for its `ls0.sbrake_time`, default 1 s) when packets from the F4 stop while the motor was driven or already braking.
* - Both are wired from `fault0` (see the fault page for when it asks for braking). During an F3 update both flags are sent as 0.
*
* 4. **Config words (round robin)**:
* - The F3 current loop parameters are sent one word per packet, cycling through `r`, `l`, `psi`, `cur_bw`, `cur_ff`, `cur_ind`, `max_y`, `max_cur * scale`, `dac`, `drop_k`, `lq`, `emf_run`, `emf_sel`, `emf_pp`, `drop_knee`, an unused word (always 0, so an older F3 reads its removed observer as off) and `oc_cur` = `max_cur` (the layout of `f3_config_data_t` in shared/common.h, append only). A full update takes 17 rt periods.
* - `max_cur` is multiplied by `scale` (the fault component's derating, 0..1) before sending; on the F3 it is the current loop command limit. The software overcurrent trip follows the unscaled `max_cur` (`oc_cur`), so a derated limit cannot pull the trip under the current and cause fault 15.
* - Dead time compensation (F3 hv, 120 deg 3 phase only): `drop_k` scales the ideal dead time loss (dead time / pwm period x DC link voltage), 0 = off. The F3 adds it to the phase voltages before it takes the SVM offset. In current mode the sign per phase comes from the commanded phase currents; in voltage mode it is off unless the F3's own `hv0.drop_volt > 0` (not sent over the link, off by default), which takes the sign from the measured dq current low passed with `hv0.drop_vlp` (3 ms). With `drop_knee = 0` the sign latches outside a current band; `drop_knee > 0` (A) instead scales each phase by `k(i) = 1 - 1/(1 + |i|/drop_knee)^2`, which fades out at low current and cannot stick.
* - `emf_run` (1 sum, 0 hold, -1 clear), `emf_sel` and `emf_pp` (pole pairs) drive the F3 `emf0` back EMF map, which sums the unfiltered phase voltages while the bridge is off and the rotor coasts; the result selected by `emf_sel` comes back in `emf_val`. Used by `idpmsm0`.
*
* 5. **Feedback (rt)**:
* - A received packet is only used if it is complete, its CRC matches, `slave_addr` is 0 and its length is that of `packet_from_hv_t`. Then `id_fb`, `iq_fb`, `ud_fb`, `uq_fb`, `fault` are copied, `abs_cur = sqrt(id_fb^2 + iq_fb^2)`, `abs_volt = sqrt(ud_fb^2 + uq_fb^2)` and `duty = abs_volt / pwm_volt` (only while `pwm_volt > 0`). `ud_fb`/`uq_fb` are the F3 `curpid0` output, which limits the voltage as one vector (d first, q gets what is left of the circle), so `abs_volt` stays at or below `pwm_volt`. The F3 computes `pwm_volt` from the DC link voltage and its usable duty `hv0.duty_max` (what the min on/off times leave, about 0.91), or 0.95 if that is not wired.
* - One state word per packet fills `u_fb`, `v_fb`, `w_fb`, `hv_temp`, `mot_temp`, `core_temp`, `dc_volt`, `pwm_volt`, `y`, `emf_val`, two unused words, `pwm_freq`, `link_to`, `hv_temp_ok` (`f3_state_data_t`, 15 words, so each is refreshed every 15 periods). A word address past this layout (from a newer F3 image) is ignored.
* - `pwm_freq` is the PWM and rt rate the F3 image was built for; an older F3 that does not report it reads as 15000 Hz. `link_to` counts the F3 rt ticks spent in link timeout since the F3 booted (`ls0.timeout`); every rise is a dropout that disabled the gates. A rise while `en` > 0 counts in `link_drops` and holds `fault` at `HV_TIMEOUT_ERROR` (9) until `en` drops (a fault code the F3 reports itself wins over it), because the F3 times out after 2 packets (400 us) but the F4 only after 3 ticks, so a short dropout could otherwise stop the bridge without the F4 ever faulting. A fall in `link_to` (F3 reboot) is taken as the new baseline.
* - `power` and `dc_cur` are not measured, they are estimated from the commanded dq voltages, so they leave out inverter losses and read low:
* ```c
* power  = 0.5 * power + 0.5 * 1.5 * (ud_fb * id_fb + uq_fb * iq_fb);
* dc_cur = power / dc_volt;   // only updated while dc_volt > 1 V
* ```
* - Every failed CRC increments `crc_error` (never reset); it does not raise a fault.
*
* 6. **Timeout**:
* - A counter is reset by every valid packet; after more than 2 rt periods without one, `fault` is forced to `HV_TIMEOUT_ERROR` (9). All other codes in `fault` come from the F3 (`fault_t` in shared/common.h, e.g. `HV_TEMP_ERROR` 10, `HV_VOLT_ERROR` 11, `HV_FAULT_ERROR` 12, `HV_OVERCURRENT_*` 14..16).
*
* 7. **Terminal pass-through**:
* - The terminal command `hv_pause <ms>` (1 to 2000) stops sending to the F3 for that long, to test what the F3 does on its own on link loss (gates off, short brake if armed); the F4 faults `HV_TIMEOUT_ERROR` meanwhile. It does nothing during an `hv_update`.
* - The terminal command `hv <text>` queues the text plus newline into a 128 byte ring buffer; one byte is sent per packet to the F3 terminal. Bytes coming back from the F3 are printed by `nrt`.
*
* 8. **F3 firmware update (`hv_update`)**:
* - The F4 image contains the F3 application (obj_hvf3/hvf3.bin). The terminal command `hv_update` starts a state machine in `rt`, reported on `state`:
* - 0 `SLAVE_IN_APP`: normal operation.
* - 1 `SEND_TO_BOOTLOADER`: sends the BOOTLOADER command with the output disabled for 10 periods, the F3 resets into its bootloader.
* - 2 `ERASE_FLASH`: page erase request to slave address 255, repeated until an OK comes back (the F3 bootloader skips pages that are already blank on a repeat), gives up after 60000 periods (12 s; a full erase takes 1-2 s).
* - 3 `SEND_APP`: writes the image word by word to 0x08004000, each word must be acknowledged with the same address and value (10 period timeout per word). `nrt` prints the progress in percent.
* - 4 `CRC_CHECK`: asks the bootloader to verify the app CRC. The bootloader computes it in its timer interrupt (2-3 ms) and answers only then, so the request is sent once and then `CRC_LISTEN` (100) periods are listened before it is sent again. An OK goes on to 5, a NAK (CRC over the image not 0) to 6 with the message `the f3 bootloader says the app CRC is wrong`; no answer for 2000 periods also ends in 6 (`no answer to CRC_CHECK`).
* - 5 `SEND_TO_APP`: sends DO_RESET for 2000 periods, then back to 0.
* - 6 `FLASH_FAILED`: the sends alternate between a bootloader NOP and an app packet with the output disabled (which carries a config word, as in state 0). Only an answer from the app returns the state to 0. While the bootloader answers (the F3 only starts its app after a reset with a valid CRC), or nothing answers at all (for example the F3 has no power), it stays here and `fault` shows the timeout. It accepts `hv_update` and `hv_verify` directly. A finished `hv_verify` also ends here, since the F3 answered from its bootloader.
* - 7 `VERIFY_FLASH`: see `hv_verify` below.
* - While updating, every second rt period sends nothing, to give the bootloader time to answer.
*
* 9. **F3 flash read back (`hv_verify`)**:
* - Accepted in state 0 or 6 (otherwise it prints `hv_verify: busy`). In state 0 only with the drive disabled: a running F3 app ignores the READ packets, so it would see a link loss, gate off and fault 9. While enabled it prints `hv_verify: refused, the drive is enabled`. Meant for an F3 that sits in its bootloader after a failed `hv_update`: it reads the app area back word by word from 0x08004000 with the bootloader READ opcode and compares it with the embedded image, plus 8 words past its end, which must read as erased (0xFFFFFFFF).
* - `nrt` prints the progress every 10 %, then the number of differing words, the first 8 differences (address, F3 value, image value) and the `version_info` image CRC and size of the F3 flash next to those of the embedded image.
* - Without an answer for 200 periods (40 ms) it stops with `the f3 is not in its bootloader` and goes back to state 0.
*
* {{% hint info %}}
* `flash_state`, the update/verify state and the terminal buffers are global, so only one `hv` instance works.
* {{% /hint %}}
*/

HAL_COMP(hv);

//process data from LS
HAL_PIN(d_cmd);             // *input*, d axis command: current (A peak) in current mode, voltage (V) in voltage mode
HAL_PIN(q_cmd);             // *input*, q axis command: current (A peak) in current mode, voltage (V) in voltage mode
HAL_PIN(pos);               // *input*, commutation angle (rad)
HAL_PIN(vel);               // *input*, velocity of pos (rad/s), sent to the F3 and used for the advance
HAL_PIN(adv);               // *parameter*, commutation advance (s), encoder to F3 current sample latency, pos is sent as pos + vel * adv, default 0
HAL_PIN(en);                // *input*, enable the F3 power stage (> 0), commands are sent as 0 while disabled

// config data from LS
HAL_PIN(phase_mode);        // *parameter*, 0 = 90 deg 3ph, 1 = 90 deg 4ph, 2 = 120 deg 3ph, 3 = 180 deg 2ph (DC), 4 = 180 deg 3ph
HAL_PIN(cmd_mode);          // *parameter*, 0 = voltage mode, 1 = current mode
HAL_PIN(r);                 // *parameter*, phase resistance (Ohm), for the F3 current loop
HAL_PIN(l);                 // *parameter*, d axis inductance (H), and q too while lq is 0
HAL_PIN(lq);                // *parameter*, q axis inductance (H), 0 = same as l, default 0
HAL_PIN(psi);               // *parameter*, flux linkage / torque constant (V*s/rad), for BEMF decoupling
HAL_PIN(cur_bw);            // *parameter*, current loop bandwidth (rad/s)
HAL_PIN(cur_ff);            // *parameter*, current loop resistance feed forward gain
HAL_PIN(cur_ind);           // *parameter*, current loop BEMF / cross coupling feed forward gain
HAL_PIN(max_y);             // *parameter*, sent to the F3 but unused there
HAL_PIN(max_cur);           // *parameter*, max phase current (A peak), sent multiplied by scale
HAL_PIN(dac);               // *parameter*, F3 overcurrent comparator DAC value (0..4095), default 2500 = comparator off (about 350 A); set it with id_dac or ocdac only on a board with the 1 nF sense caps
HAL_PIN(drop_k);            // *parameter*, dead time compensation, fraction of the ideal loss, 0 = off, default 0
HAL_PIN(drop_knee);         // *parameter*, dead time compensation curve knee (A peak), 0 = latched sign
HAL_PIN(emf_run);           // *input*, F3 emf0 back EMF map: 1 sum, 0 hold, -1 clear
HAL_PIN(emf_sel);           // *input*, which emf0 result comes back in emf_val
HAL_PIN(emf_pp);            // *parameter*, pole pairs, for emf0's per pole bins

// process data to LS
HAL_PIN(dc_volt);           // *output*, DC link voltage (V)
HAL_PIN(id_fb);             // *output*, measured d axis current (A peak)
HAL_PIN(iq_fb);             // *output*, measured q axis current (A peak), negated when rev > 0
HAL_PIN(ud_fb);             // *output*, d axis output voltage (V)
HAL_PIN(uq_fb);             // *output*, q axis output voltage (V), negated when rev > 0
HAL_PIN(abs_cur);           // *output*, current magnitude sqrt(id^2 + iq^2) (A peak)
HAL_PIN(abs_volt);          // *output*, voltage magnitude sqrt(ud^2 + uq^2) (V)
HAL_PIN(duty);              // *output*, abs_volt / pwm_volt
HAL_PIN(power);             // *output*, electrical power into the motor (W), estimated, negative when braking
HAL_PIN(dc_cur);            // *output*, DC link current (A), estimated from power balance, negative when braking

// state data to LS
HAL_PIN(hv_temp);           // *output*, power stage temperature (deg C)
HAL_PIN(hv_temp_ok);        // *output*, 0 = never read (or an F3 that does not send it), 1 = live, 2 = held while the bridge is off
HAL_PIN(mot_temp);          // *output*, motor temperature input of the F3 board
HAL_PIN(core_temp);         // *output*, F3 chip temperature (deg C)
HAL_PIN(fault);             // *output*, fault code from the F3, or HV_TIMEOUT_ERROR (9) when the link is lost
HAL_PIN(ignore_fault_pin);  // *parameter*, > 0 makes the F3 ignore the gate driver fault pin, default 1
HAL_PIN(sbrake);            // *input*, short-circuit braking request, sent only while en <= 0, usually fault0.sbrake
HAL_PIN(sbrake_arm);        // *input*, > 0 lets the F3 brake on its own on link loss, usually fault0.sbrake_en
HAL_PIN(y);                 // *output*, zero sequence component of the phase currents (A)
HAL_PIN(u_fb);              // *output*, phase U voltage (V)
HAL_PIN(v_fb);              // *output*, phase V voltage (V)
HAL_PIN(w_fb);              // *output*, phase W voltage (V)
HAL_PIN(emf_val);           // *output*, emf0 result number emf_sel, from the F3
HAL_PIN(pwm_freq);          // *output*, F3 PWM and rt rate (Hz), 15000 from an F3 that does not report it
HAL_PIN(link_to);           // *output*, F3 rt ticks spent in link timeout since the F3 booted, any rise is a dropout that took the gates off
HAL_PIN(link_drops);        // *output*, F3 dropouts seen while enabled, each one faults HV_TIMEOUT_ERROR (9)

// misc
HAL_PIN(rev);               // *parameter*, > 0 reverses the output direction (q_cmd, pos, vel, iq_fb, uq_fb)
HAL_PIN(pwm_volt);          // *output*, max output voltage the F3 can produce (V)
HAL_PIN(uart_sr);           // *output*, debug, USART status register, read to clear errors
HAL_PIN(uart_dr);           // *output*, debug, USART data register, read to clear errors
HAL_PIN(crc_error);         // *output*, total number of crc errors, never reset
HAL_PIN(scale);             // *input*, current limit scale (0..1), usually fault0.scale

HAL_PIN(state);             // *output*, F3 update state, 0 = normal operation, 1..7 see above
HAL_PIN(value);             // *output*, debug, 0 = no packet, 0.75 = packet not accepted, 1 = valid packet, 3 = bad bootloader reply

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
static volatile uint32_t verify_refused;
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
  // the trip limit stays at the rated current: following scale, an overshoot
  // past max_cur cut oc_lim below the current and tripped fault 15
  ctx->config.pins.oc_cur    = PIN(max_cur);

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
              PIN(hv_temp_ok) = ctx->state.pins.hv_temp_ok;
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
              if(ctx->link_err && PIN(fault) == 0.0) {  // a real F3 fault code wins
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

      if(send_verify && e > 0.0) {
        // bootloader READ packets are a link loss to a running F3 app: it
        // gates off or short-brakes and fault 9 latches. Only from disabled
        // (the F3 may be sitting in its bootloader after an F4 reset).
        send_verify    = 0;
        verify_refused = 1;
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

  if(verify_refused) {
    verify_refused = 0;
    printf("hv_verify: refused, the drive is enabled\n");
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
