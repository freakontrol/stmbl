#include "sl_seq_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"

/**
* ## Brief
* The `sl_seq` component starts a motor without an encoder and hands it to
* the observer (obs), for a PMSM and an induction motor alike.
*
* ## States
* 0. off: `en` = 0. Everything reset, obs tracks.
* 1. align: current `i_f` on d at a standing frame for `align_time`. PMSM:
*    the rotor turns onto the d axis. Induction motor: the flux builds (use
*    about 3 tr).
* 2. I/f: the frame turns at `f`, which ramps toward `vel_cmd` at `acc`, with
*    `i_f` on d. The rotor follows the rotating current. obs tracks the frame
*    below `w_hand` / 2, so it starts with the right direction, then runs free.
* 3. observer: angle0 from obs (src 2), d and q from the drive (`d_in`, `q_in`,
*    pmsm_ttc0 or acim_foc0), `pid_en` = 1, pid closes the speed loop on
*    obs0.vel_m, while d goes from i_f to d_in over `fade_time` so the current
*    does not step. Entered once obs is ok, |f| >= w_hand and obs ran free
*    `lock_time`. Back to 2 when obs drops ok or |obs0.vel_m| < w_hand - hyst,
*    with f seeded from obs0.vel_m.
*
* `vel_ref` and `acc_ref` replace the command into pid: they follow f in I/f
* and, from the handover, ramp from the observed speed to vel_cmd at `acc`,
* so the handover is bumpless and pid gets acceleration feedforward.
*
* Encoder plausibility guard (optional, `enc_tol` > 0, needs `vel_enc` from
* an encoder with the same sign and scale as obs0.vel_m, e.g. a spindle
* orientation encoder). The tolerance is enc_tol times the slip model's own
* limit, `slip_max` (acim_flux0.slip_max, electrical) / polecount, so about 1
* fits any motor: in I/f the frequency stays within it of the encoder speed,
* so a rotor that does not follow is not left behind. In state 3 an observer
* speed further than it from the encoder for `enc_time`
* falls back to I/f, seeded from the encoder, and sets `enc_err` (held until
* `en` = 0; link it to a fault input to trip instead). `slip_err` is
* obs0.vel_m - vel_enc in state 3, low passed at 1 Hz: steady, it is the
* slip model's error, a check on acim_flux0.tr without the MRAS. enc_tol 0
* = off.
*
* Defaults from the motor (each 0 = derived, a value set in the config wins):
* - `align_time` 0 = 3 * `tr` (acim_flux0.tr: the flux has built), 0.3 s
*   without tr (PMSM: the rotor has turned onto d).
* - `w_hand` 0 = the speed where the emf is 5 * `e_min` (obs0.e_min, the
*   observer's own trust level): 5 * e_min / (polecount * flux), flux =
*   `psi` (conf0.psi, PMSM) or `lmr` * i_f (acim_flux0.lmr, induction
*   motor after align). 50 rad/s without either.
* - `hyst` 0 = w_hand / 3.
* `align` and `hand` show the values in use.
*
* Speeds are mechanical rad/s. `vel_e` is f * polecount for angle0.vel_cmd,
* `src` goes to angle0.src.
*/

HAL_COMP(sl_seq);

HAL_PIN(en);          // *input*, fault0.en_pid
HAL_PIN(vel_cmd);     // *input*, speed command [rad/s mech], vel0.vel
HAL_PIN(polecount);   // *parameter*, pole pairs
HAL_PIN(i_f);         // *parameter*, align and I/f current on d [A peak]
HAL_PIN(align_time);  // *parameter*, [s], 0 = 3 * tr
HAL_PIN(acc);         // *parameter*, I/f acceleration [rad/s^2 mech]
HAL_PIN(w_hand);      // *parameter*, handover speed [rad/s mech], 0 = from e_min and the flux
HAL_PIN(hyst);        // *parameter*, fall back below w_hand - hyst [rad/s mech], 0 = w_hand / 3
HAL_PIN(lock_time);   // *parameter*, obs free running before handover [s]
HAL_PIN(fade_time);   // *parameter*, i_f fades out over this after the handover [s]

HAL_PIN(obs_ok);      // *input*, obs0.ok
HAL_PIN(obs_vel);     // *input*, obs0.vel_m
HAL_PIN(d_in);        // *input*, d current from the drive in state 3
HAL_PIN(q_in);        // *input*, q current from the drive in state 3

HAL_PIN(state);       // *output*
HAL_PIN(f);           // *output*, I/f frequency [rad/s mech]
HAL_PIN(vel_e);       // *output*, to angle0.vel_cmd [rad/s electrical]
HAL_PIN(src);         // *output*, to angle0.src
HAL_PIN(track);       // *output*, to obs0.track
HAL_PIN(pid_en);      // *output*, to pid0.en
HAL_PIN(d_cmd);       // *output*, to hv0.d_cmd
HAL_PIN(q_cmd);       // *output*, to hv0.q_cmd
HAL_PIN(vel_ref);     // *output*, speed command for pid [rad/s mech], to pid0.vel_ext_cmd
HAL_PIN(acc_ref);     // *output*, its slope [rad/s^2], to pid0.acc_ext_cmd
HAL_PIN(vel_enc);     // *input*, encoder rotor speed [rad/s mech], for the guard
HAL_PIN(enc_tol);     // *parameter*, allowed |speed - vel_enc| as a multiple of slip_max, 0 = no guard
HAL_PIN(slip_max);    // *parameter*, slip limit [rad/s electrical], acim_flux0.slip_max
HAL_PIN(enc_time);    // *parameter*, observer error longer than this falls back [s]
HAL_PIN(enc_err);     // *output*, 1 = observer disagreed with the encoder, held until en 0
HAL_PIN(slip_err);    // *output*, obs0.vel_m - vel_enc in state 3, 1 Hz low pass [rad/s mech]
HAL_PIN(tr);          // *input*, rotor time constant [s], acim_flux0.tr, for align_time 0
HAL_PIN(psi);         // *input*, magnet flux [V s], conf0.psi, for w_hand 0 (PMSM)
HAL_PIN(lmr);         // *input*, magnetizing inductance [H], acim_flux0.lmr, for w_hand 0 (induction motor)
HAL_PIN(e_min);       // *input*, obs0.e_min [V], for w_hand 0
HAL_PIN(align);       // *output*, align time in use [s]
HAL_PIN(hand);        // *output*, handover speed in use [rad/s mech]

struct sl_seq_ctx_t {
  float time;
  float free_time;
  float fade;
  float ref;
  float enc_time;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct sl_seq_pin_ctx_t *pins = (struct sl_seq_pin_ctx_t *)pin_ptr;

  PIN(polecount)  = 4.0;
  PIN(i_f)        = 3.0;
  PIN(align_time) = 0.0;
  PIN(acc)        = 100.0;
  PIN(w_hand)     = 0.0;
  PIN(hyst)       = 0.0;
  PIN(e_min)      = 3.0;
  PIN(lock_time)  = 0.1;
  PIN(fade_time)  = 0.1;
  PIN(enc_time)   = 0.05;
  PIN(src)        = 3.0;
  PIN(track)      = 1.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct sl_seq_ctx_t *ctx     = (struct sl_seq_ctx_t *)ctx_ptr;
  struct sl_seq_pin_ctx_t *pins = (struct sl_seq_pin_ctx_t *)pin_ptr;

  int state    = (int)PIN(state);
  float f      = PIN(f);
  float pp     = MAX(PIN(polecount), 1.0);
  float flux   = PIN(psi) > 0.0 ? PIN(psi) : MAX(PIN(lmr), 0.0) * ABS(PIN(i_f));
  float w_hand = PIN(w_hand) > 0.0 ? PIN(w_hand) : (flux > 0.0 ? 5.0 * MAX(PIN(e_min), 0.1) / (pp * flux) : 50.0);
  w_hand       = MAX(w_hand, 1.0);
  float hyst   = PIN(hyst) > 0.0 ? PIN(hyst) : w_hand / 3.0;
  float align  = PIN(align_time) > 0.0 ? PIN(align_time) : (PIN(tr) > 0.0 ? 3.0 * PIN(tr) : 0.3);
  PIN(align)   = align;
  PIN(hand)    = w_hand;
  float ov     = PIN(obs_vel);
  float tol    = PIN(enc_tol) > 0.0 ? PIN(enc_tol) * MAX(PIN(slip_max), 1.0) / MAX(PIN(polecount), 1.0) : 0.0;
  float ve     = PIN(vel_enc);

  if(PIN(en) <= 0.0) {
    state          = 0;
    f              = 0.0;
    ctx->time      = 0.0;
    ctx->free_time = 0.0;
    ctx->enc_time  = 0.0;
    PIN(enc_err)   = 0.0;
    PIN(slip_err)  = 0.0;
  }

  switch(state) {
    case 0:
      if(PIN(en) > 0.0) {
        state     = 1;
        ctx->time = 0.0;
      }
      break;

    case 1:
      f = 0.0;
      ctx->time += period;
      if(ctx->time >= align) {
        state          = 2;
        ctx->free_time = 0.0;
      }
      break;

    case 2:
      f += LIMIT(PIN(vel_cmd) - f, MAX(PIN(acc), 0.0) * period);
      if(tol > 0.0) {  // the field does not run away from the rotor
        f = CLAMP(f, ve - tol, ve + tol);
      }
      if(ABS(f) >= 0.5 * w_hand) {
        ctx->free_time += period;
      } else {
        ctx->free_time = 0.0;
      }
      if(PIN(obs_ok) > 0.0 && ABS(f) >= w_hand && ABS(ov) >= w_hand - hyst &&
         ctx->free_time >= PIN(lock_time)) {
        state     = 3;
        ctx->fade = 1.0;
        ctx->ref  = ov;  // pid starts from the speed it has, not from the command
      }
      break;

    case 3:
      f = ov;
      ctx->fade -= period / MAX(PIN(fade_time), period);
      ctx->fade = MAX(ctx->fade, 0.0);
      if(tol > 0.0) {
        PIN(slip_err) += (ov - ve - PIN(slip_err)) * CLAMP(2.0 * M_PI * period, 0.0, 1.0);
        ctx->enc_time = ABS(ov - ve) > tol ? ctx->enc_time + period : 0.0;
      }
      if(tol > 0.0 && ctx->enc_time > MAX(PIN(enc_time), period)) {
        state          = 2;  // observer implausible: back to I/f from the encoder speed
        f              = ve;
        ctx->free_time = 0.0;
        ctx->enc_time  = 0.0;
        PIN(enc_err)   = 1.0;
      } else if(PIN(obs_ok) <= 0.0 || ABS(ov) < w_hand - hyst) {
        state          = 2;
        ctx->free_time = 0.0;
      }
      break;
  }

  if(state != 3) {  // no observer speed to compare: tr_ks must not adapt on the last value
    PIN(slip_err) = 0.0;
  }

  // speed command for pid: f until the handover, then from the observed speed
  // toward vel_cmd at acc, with the slope as acceleration feedforward, so pid
  // neither sees a step nor has to build the inertia torque on its integrator
  float ref = state == 3 ? ctx->ref + LIMIT(PIN(vel_cmd) - ctx->ref, MAX(PIN(acc), 0.0) * period) : f;
  PIN(acc_ref) = state == 3 || state == 2 ? (ref - (state == 3 ? ctx->ref : PIN(vel_ref))) / period : 0.0;
  ctx->ref     = ref;
  PIN(vel_ref) = ref;

  PIN(state)  = state;
  PIN(f)      = f;
  PIN(vel_e)  = f * MAX(PIN(polecount), 1.0);
  PIN(src)    = state == 3 ? 2.0 : 3.0;
  PIN(track)  = state < 2 || (state == 2 && ctx->free_time <= 0.0) ? 1.0 : 0.0;
  PIN(pid_en) = state == 3 ? 1.0 : 0.0;
  // state 3: from i_f to the drive's d, so an induction motor's d_in (id_n) is not added on top
  PIN(d_cmd)  = state == 3 ? PIN(d_in) + (PIN(i_f) - PIN(d_in)) * ctx->fade : (state > 0 ? PIN(i_f) : 0.0);
  PIN(q_cmd)  = state == 3 ? PIN(q_in) : 0.0;
}

hal_comp_t sl_seq_comp_struct = {
    .name      = "sl_seq",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct sl_seq_ctx_t),
    .pin_count = sizeof(struct sl_seq_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
