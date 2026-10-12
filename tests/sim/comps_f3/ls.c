// Host stand-in for stm32f303/src/comps/ls.c: the F3 end of the F4 link,
// with the same pins. The packet handling, ramp, timeouts, short-brake arming
// and pwm_volt are the firmware's; the USART and DMA are replaced by
// runner/link.c. Not modelled: the PWM phase lock (the runner places the F3
// ticks where the lock settles, see runner/main.c), CRC errors.
#include "ls_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "common.h"
#include "f3hw.h"
#include "ringbuf.h"
#include "sim_hw.h"



HAL_COMP(ls);

//process data from LS
HAL_PIN(d_cmd);
HAL_PIN(q_cmd);
// The f4 sends d/q every PWM_TICKS_PER_PACKET ticks (3 at 15 kHz); stepped straight in, the command carries
// a 5 kHz staircase into the current loop. ramp = 1 spreads each step over
// the LS_RAMP_TICKS ticks to the next packet, a linear ramp that lags the
// step by half an f4 period. 0 = step.
HAL_PIN(ramp);
#define LS_RAMP_TICKS PWM_TICKS_PER_PACKET
// link loss after two missed packets (0.4 ms): 5 ticks at 15 kHz
#define LS_TIMEOUT_TICKS (2 * PWM_TICKS_PER_PACKET - 1)
// short-circuit braking on a link loss only after four missed packets (0.8
// ms): one or two lost to noise just take the gates off until the next packet
#define LS_SBRAKE_TICKS (4 * PWM_TICKS_PER_PACKET - 1)
// a restarted f4 is silent far longer than 10 ms (boot, config load); a gap
// that long must deliver the whole config again before the next enable
#define LS_CONF_TICKS (PWM_FREQ / 100)
HAL_PIN(pos);
HAL_PIN(vel);
// The angle the voltage computed this tick lands at, on average: the
// compares are preloaded, so it is applied over the next period, 1.5
// periods after the current sample that dq0 transforms at pos. idq0 and
// hv0's dead-time reference use it; hv0.adv on the f4 is then the encoder
// to sample latency alone. v_lead in periods, default 1.5, 0 = one
// angle for both.
HAL_PIN(pos_v);
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
HAL_PIN(max_y);  // the f4's hv0.max_y, not used on the f3
HAL_PIN(max_cur);
HAL_PIN(oc_cur);  // the f4's max_cur before fault0.scale, for io0's trip (0 from an older f4: io0 then trips at ABS_MAX_CURRENT only)
HAL_PIN(dac);
HAL_PIN(drop_k);
HAL_PIN(emf_run);
HAL_PIN(emf_sel);
HAL_PIN(emf_pp);
HAL_PIN(drop_knee);

// process data to LS
HAL_PIN(dc_volt);
HAL_PIN(udc_duty);  // io0.udc_duty, the link hv0 divides by, for pwm_volt
HAL_PIN(id_fb);
HAL_PIN(iq_fb);
HAL_PIN(ud_fb);
HAL_PIN(uq_fb);

// state data to LS
HAL_PIN(hv_temp);
HAL_PIN(hv_temp_ok);  // io0.hv_temp_ok: 0 never read, 1 live, 2 held
HAL_PIN(mot_temp);
HAL_PIN(core_temp);  // not measured, sends 0
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
  float d_tgt, q_tgt;    // the last packet's command
  float d_step, q_step;  // per tick towards it while ramping
  uint32_t sbrake_loss;  // brake for this link loss
  uint32_t tx_addr;
  uint32_t conf_seen;  // bit per config word written since boot
  uint8_t send;
  packet_to_hv_t packet_to_hv;
  packet_from_hv_t packet_from_hv;
};

static f3_config_data_t config;
static f3_state_data_t state;

extern struct ringbuf rx_buf;  // host/side.c: term0 reads its command lines here
extern struct ringbuf tx_buf;

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ls_ctx_t *ctx      = (struct ls_ctx_t *)ctx_ptr;
  struct ls_pin_ctx_t *pins = (struct ls_pin_ctx_t *)pin_ptr;

  PIN(v_lead)      = 1.5;
  PIN(ramp)        = 1.0;
  PIN(sbrake_time) = 1.0;

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
  ctx->d_tgt = ctx->q_tgt = 0.0;
  ctx->d_step = ctx->q_step = 0.0;
  ctx->send        = 0;
  PIN(crc_error)   = 0.0;
  PIN(crc_ok)      = 0.0;
  PIN(timeout)     = 0.0;
  PIN(sbrake)      = 0.0;
  PIN(sbrake_arm)  = 0.0;
  PIN(idle)        = 0.0;
  PIN(dma_pos_cmd) = 4;
  PIN(inc)         = PWM_RES * 5 / 4800;
  PIN(window)      = 1;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ls_ctx_t *ctx      = (struct ls_ctx_t *)ctx_ptr;
  struct ls_pin_ctx_t *pins = (struct ls_pin_ctx_t *)pin_ptr;

  int partial = 0;
  int got     = simlink_f3_recv(&ctx->packet_to_hv, &partial);

  PIN(dma_pos2) = got ? sizeof(packet_to_hv_t) : (partial ? 1 : 0);
  PIN(arr)      = PWM_RES;  // phase lock not modelled

  uint32_t fault = 0;

  if(got) {
    if(ctx->packet_to_hv.header.slave_addr == 0 && ctx->packet_to_hv.header.len == (sizeof(packet_to_hv_t) - sizeof(stmbl_talk_header_t)) / 4) {
      uint8_t a     = ctx->packet_to_hv.header.conf_addr;
      uint8_t valid = a < sizeof(config) / 4;
      a             = CLAMP(a, 0, sizeof(config) / 4 - 1);

      if(ctx->packet_to_hv.header.flags.cmd == WRITE_CONF && valid) {
        config.data[a] = ctx->packet_to_hv.header.config.f32;
        ctx->conf_seen |= 1u << a;
      } else if(ctx->packet_to_hv.header.flags.cmd == READ_CONF && valid) {
        ctx->tx_addr = a;
      }

      PIN(conf_ok)          = ctx->conf_seen == (1u << (sizeof(config) / 4)) - 1u;
      PIN(en)               = PIN(conf_ok) > 0.0 ? ctx->packet_to_hv.flags.enable : 0;
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
        rb_write(&rx_buf, (void *)&(ctx->packet_to_hv.flags.buf), 1);
      }

      PIN(r)         = config.pins.r;
      PIN(l)         = config.pins.l;
      PIN(psi)       = config.pins.psi;
      PIN(cur_bw)    = config.pins.cur_bw;
      PIN(cur_ff)    = config.pins.cur_ff;
      PIN(cur_ind)   = config.pins.cur_ind;
      PIN(max_y)     = config.pins.max_y;
      PIN(max_cur)   = config.pins.max_cur;
      PIN(oc_cur)    = config.pins.oc_cur;
      PIN(dac)       = config.pins.dac;
      PIN(drop_k)    = config.pins.drop_k;
      PIN(lq)        = config.pins.lq > 0.0 ? config.pins.lq : config.pins.l;
      PIN(emf_run)   = config.pins.emf_run;
      PIN(emf_sel)   = config.pins.emf_sel;
      PIN(emf_pp)    = config.pins.emf_pp;
      PIN(drop_knee) = config.pins.drop_knee;
      ctx->timeout   = 0;
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
    partial = 0;  // rx re-armed this tick, as the firmware does
  } else if(ctx->timeout <= LS_TIMEOUT_TICKS) {
    PIN(pos) = PIN(pos) + PIN(vel) * period;
  }
  PIN(pos_v) = PIN(pos) + PIN(vel) * PIN(v_lead) * period;

  if(PIN(ramp) > 0.0 && ctx->timeout < LS_RAMP_TICKS) {
    PIN(d_cmd) += ctx->d_step;
    PIN(q_cmd) += ctx->q_step;
  } else {
    PIN(d_cmd) = ctx->d_tgt;
    PIN(q_cmd) = ctx->q_tgt;
  }

  if(ctx->send == 2) {
    ctx->send = 0;
  }
  // the reply goes out once the next packet has started to arrive, as on the
  // board (ls.c: send == 1 && dma_pos != 0)
  if(ctx->send == 1 && partial) {
    ctx->send = 2;
    state.pins.u_fb       = PIN(u_fb);
    state.pins.v_fb       = PIN(v_fb);
    state.pins.w_fb       = PIN(w_fb);
    state.pins.hv_temp    = PIN(hv_temp);
    state.pins.mot_temp   = PIN(mot_temp);
    state.pins.core_temp  = PIN(core_temp);
    state.pins.y          = PIN(y);
    state.pins.dc_volt    = PIN(dc_volt);
    state.pins.pwm_volt   = PIN(pwm_volt);
    state.pins.emf_val    = PIN(emf_val);
    state.pins.unused0    = 0.0;
    state.pins.unused1    = 0.0;
    state.pins.hv_temp_ok = PIN(hv_temp_ok);
    state.pins.pwm_freq   = PWM_FREQ;
    state.pins.link_to    = PIN(timeout);

    ctx->packet_from_hv.fault             = (uint8_t)PIN(fault_in);
    ctx->packet_from_hv.id_fb             = PIN(id_fb);
    ctx->packet_from_hv.iq_fb             = PIN(iq_fb);
    ctx->packet_from_hv.ud_fb             = PIN(ud_fb);
    ctx->packet_from_hv.uq_fb             = PIN(uq_fb);
    ctx->packet_from_hv.header.conf_addr  = ctx->tx_addr;
    ctx->packet_from_hv.header.config.f32 = state.data[ctx->tx_addr++];
    ctx->tx_addr %= sizeof(state) / 4;

    uint8_t buf[1];
    if(rb_read(&tx_buf, buf, 1)) {
      ctx->packet_from_hv.buf = buf[0];
    } else {
      ctx->packet_from_hv.buf = 0x0;
    }
    simlink_f3_send(&ctx->packet_from_hv);
  }

  if(ctx->timeout == LS_TIMEOUT_TICKS + 1) {
    ctx->sbrake_loss = PIN(sbrake_arm) > 0.0 && (PIN(en) > 0.0 || PIN(sbrake) > 0.0);
  }
  if(ctx->timeout == LS_CONF_TICKS) {
    ctx->conf_seen = 0;
    PIN(conf_ok)   = 0.0;
  }
  if(ctx->timeout > LS_TIMEOUT_TICKS) {
    PIN(en)     = 0.0;
    PIN(vel)    = 0.0;
    PIN(sbrake) = ctx->sbrake_loss && ctx->timeout > LS_SBRAKE_TICKS && (float)(ctx->timeout - LS_SBRAKE_TICKS) * period < PIN(sbrake_time);
    PIN(timeout)
    ++;
    fault = 1;
  }
  if(ctx->timeout < 0x7FFFFFFF) {
    ctx->timeout++;
  }

  PIN(fault) = MAX(fault, PIN(fault_in));

  float duty = PIN(duty_max) > 0.0 ? PIN(duty_max) : 0.95;

  switch((uint16_t)PIN(phase_mode)) {
    case PHASE_90_3PH:
      PIN(pwm_volt) = PIN(udc_duty) * M_SQRT1_2 * duty;
      break;
    case PHASE_90_4PH:
      PIN(pwm_volt) = PIN(udc_duty) * duty;
      break;
    case PHASE_120_3PH:
      PIN(pwm_volt) = PIN(udc_duty) * M_SQRT1_3 * duty;
      break;
    case PHASE_180_2PH:
    case PHASE_180_3PH:
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
