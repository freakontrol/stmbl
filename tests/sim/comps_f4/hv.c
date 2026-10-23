// Host stand-in for src/comps/hv.c: the F4 end of the F3 link, with the
// same pins. The packet logic of the app state (SLAVE_IN_APP) is the
// firmware's; the USART, DMA and CRC are replaced by runner/link.c, which
// delivers each packet after its time on the wire. No F3 update/verify.
#include "hv_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "common.h"
#include <stdio.h>
#include <string.h>
#include "ringbuf.h"
#include "sim_hw.h"

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
HAL_PIN(hv_temp_ok);  // 0 never read (or an f3 that does not send it), 1 live, 2 held while the bridge is off
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
  packet_to_hv_t to_hv;
  packet_from_hv_t from_hv;
  f3_config_data_t config;
  f3_state_data_t state;
  uint16_t timeout;
  float link_to_last;  // link_to at the last state word, -1 before the first
  uint32_t link_err;   // an f3 dropout while enabled, held until en drops
  uint8_t conf_addr;
};

struct ringbuf hv_rx_buf = RINGBUF(128);
struct ringbuf hv_tx_buf = RINGBUF(128);

void hv_send(char *ptr) {
  if(ptr) {
    rb_write(&hv_tx_buf, ptr, strlen(ptr));
    rb_write(&hv_tx_buf, "\n", 1);
  }
}
COMMAND("hv", hv_send, "send command to hv board");

static volatile float hv_pause_left;  // [s]

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

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct hv_ctx_t *ctx      = (struct hv_ctx_t *)ctx_ptr;
  struct hv_pin_ctx_t *pins = (struct hv_pin_ctx_t *)pin_ptr;

  ctx->timeout          = 0;
  PIN(dac)              = 2500;
  PIN(drop_k)           = 0;
  PIN(lq)               = 0;
  PIN(adv)              = 0;
  hv_pause_left         = 0.0;
  ctx->link_to_last     = -1.0;
  ctx->link_err         = 0;
  PIN(ignore_fault_pin) = 1;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct hv_ctx_t *ctx      = (struct hv_ctx_t *)ctx_ptr;
  struct hv_pin_ctx_t *pins = (struct hv_pin_ctx_t *)pin_ptr;
  float e                   = PIN(en);
  float pos                 = PIN(pos);
  float vel                 = PIN(vel);
  pos                       = mod(pos + vel * PIN(adv));

  ctx->config.pins.r         = PIN(r);
  ctx->config.pins.l         = PIN(l);
  ctx->config.pins.psi       = PIN(psi);
  ctx->config.pins.cur_bw    = PIN(cur_bw);
  ctx->config.pins.cur_ff    = PIN(cur_ff);
  ctx->config.pins.cur_ind   = PIN(cur_ind);
  ctx->config.pins.max_y     = PIN(max_y);
  ctx->config.pins.max_cur   = PIN(max_cur) * PIN(scale);
  ctx->config.pins.dac       = PIN(dac);
  ctx->config.pins.drop_k    = PIN(drop_k);
  ctx->config.pins.lq        = PIN(lq);
  ctx->config.pins.emf_run   = PIN(emf_run);
  ctx->config.pins.emf_sel   = PIN(emf_sel);
  ctx->config.pins.emf_pp    = PIN(emf_pp);
  ctx->config.pins.drop_knee = PIN(drop_knee);
  ctx->config.pins.unused0   = 0.0;
  ctx->config.pins.oc_cur    = PIN(max_cur);

  PIN(value) = 0.0;

  if(simlink_f4_recv(&ctx->from_hv)) {
    PIN(value) = 0.75;
    if(ctx->from_hv.header.slave_addr == 0 && ctx->from_hv.header.len == (sizeof(packet_from_hv_t) - sizeof(stmbl_talk_header_t)) / 4) {
      PIN(id_fb) = ctx->from_hv.id_fb;
      PIN(iq_fb) = ctx->from_hv.iq_fb;
      PIN(ud_fb) = ctx->from_hv.ud_fb;
      PIN(uq_fb) = ctx->from_hv.uq_fb;
      if(PIN(rev) > 0.0) {
        PIN(uq_fb) *= -1.0;
        PIN(iq_fb) *= -1.0;
      }
      PIN(fault)    = ctx->from_hv.fault;
      PIN(abs_cur)  = sqrtf(PIN(id_fb) * PIN(id_fb) + PIN(iq_fb) * PIN(iq_fb));
      PIN(abs_volt) = sqrtf(PIN(ud_fb) * PIN(ud_fb) + PIN(uq_fb) * PIN(uq_fb));
      if(PIN(pwm_volt) > 0.0) {
        PIN(duty) = PIN(abs_volt) / PIN(pwm_volt);
      }

      uint16_t a = ctx->from_hv.header.conf_addr;
      if(a < sizeof(f3_state_data_t) / 4) {
        ctx->state.data[a] = ctx->from_hv.header.config.f32;
      }

      PIN(dc_volt)    = ctx->state.pins.dc_volt;
      PIN(pwm_volt)   = ctx->state.pins.pwm_volt;
      PIN(u_fb)       = ctx->state.pins.u_fb;
      PIN(v_fb)       = ctx->state.pins.v_fb;
      PIN(w_fb)       = ctx->state.pins.w_fb;
      PIN(hv_temp)    = ctx->state.pins.hv_temp;
      PIN(hv_temp_ok) = ctx->state.pins.hv_temp_ok;
      PIN(mot_temp)   = ctx->state.pins.mot_temp;
      PIN(core_temp)  = ctx->state.pins.core_temp;
      PIN(y)          = ctx->state.pins.y;
      PIN(emf_val)    = ctx->state.pins.emf_val;
      PIN(pwm_freq)   = ctx->state.pins.pwm_freq > 0.0 ? ctx->state.pins.pwm_freq : 15000.0;
      PIN(link_to)    = ctx->state.pins.link_to;

      if(ctx->link_to_last >= 0.0 && PIN(link_to) > ctx->link_to_last && e > 0.0) {
        ctx->link_err = 1;
        PIN(link_drops)++;
      }
      ctx->link_to_last = PIN(link_to);
      if(ctx->link_err) {
        PIN(fault) = HV_TIMEOUT_ERROR;
      }

      PIN(power) = 1.5 * (PIN(ud_fb) * PIN(id_fb) + PIN(uq_fb) * PIN(iq_fb)) * 0.5 + PIN(power) * 0.5;
      if(PIN(dc_volt) > 1.0) {
        PIN(dc_cur) = PIN(power) / PIN(dc_volt);
      }

      PIN(value)   = 1.0;
      ctx->timeout = 0;

      if(ctx->from_hv.buf != 0x0) {
        rb_write(&hv_rx_buf, (void *)&(ctx->from_hv.buf), 1);
      }
    }
  }

  if(ctx->timeout > 2) {
    PIN(fault) = HV_TIMEOUT_ERROR;
  }
  ctx->timeout++;
  if(e <= 0.0) {
    ctx->link_err = 0;
  }

  float d_cmd = PIN(d_cmd);
  float q_cmd = PIN(q_cmd);

  if(PIN(rev) > 0.0) {
    q_cmd *= -1.0;
    pos = minus(0, pos);
    vel *= -1.0;
  }

  if(e > 0.0) {
    ctx->to_hv.d_cmd        = d_cmd;
    ctx->to_hv.q_cmd        = q_cmd;
    ctx->to_hv.flags.enable = 1;
  } else {
    ctx->to_hv.d_cmd        = 0.0;
    ctx->to_hv.q_cmd        = 0.0;
    ctx->to_hv.flags.enable = 0;
  }
  ctx->to_hv.flags.ignore_fault_pin = PIN(ignore_fault_pin) > 0.0;
  ctx->to_hv.flags.sbrake           = e <= 0.0 && PIN(sbrake) > 0.0;
  ctx->to_hv.flags.sbrake_arm       = PIN(sbrake_arm) > 0.0;
  ctx->to_hv.flags.cmd_type         = PIN(cmd_mode);
  ctx->to_hv.flags.phase_type       = PIN(phase_mode);
  ctx->to_hv.pos                    = pos;
  ctx->to_hv.vel                    = vel;

  ctx->to_hv.header.slave_addr = 0;
  ctx->to_hv.header.flags.cmd  = WRITE_CONF;
  ctx->to_hv.header.flags.counter++;
  ctx->to_hv.header.len        = (sizeof(packet_to_hv_t) - sizeof(stmbl_talk_header_t)) / 4;
  ctx->to_hv.header.conf_addr  = ctx->conf_addr;
  ctx->to_hv.header.config.f32 = ctx->config.data[ctx->conf_addr++];
  ctx->conf_addr %= sizeof(f3_config_data_t) / 4;

  uint8_t buf[1];
  if(rb_read(&hv_tx_buf, buf, 1)) {
    ctx->to_hv.flags.buf = buf[0];
  } else {
    ctx->to_hv.flags.buf = 0x0;
  }

  if(hv_pause_left > 0.0) {
    hv_pause_left -= period;
  } else {
    simlink_f4_send(&ctx->to_hv);
  }

  PIN(state) = 0;  // SLAVE_IN_APP
}

static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  char c;
  while(rb_getc(&hv_rx_buf, &c)) {
    printf("%c", c);
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
