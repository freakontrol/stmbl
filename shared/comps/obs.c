#include "obs_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* The `obs` component estimates the flux angle and speed from voltage and
* current, for a PMSM and an induction motor alike. It generalises
* `sensorless` (which stays for old configs) and fixes its undamped loop.
*
* ## Model
* In a frame at angle pos turning at vel, the back emf is
*
*     ed = ud - r id - ld d(id)/dt + vel lq iq
*     eq = uq - r iq - lq d(iq)/dt - vel ld id
*
* PMSM: ld, lq, and the emf is vel * psi on q. Induction motor: ld = lq =
* sigma*Ls, and the emf is the rotor flux's, vel * lmr * i_mr on q. Either
* way a frame lagging the flux by d reads ed = -e sin d, eq = e cos d times
* the sign of vel, so the angle error is -sign(vel) ed / |e|. ed/eq alone
* would also lock half a turn off.
*
* ## Frames
* The feedback (hv0.id_fb ...) is in the frame hv0 used, `pos_ref` (angle0.pos)
* plus `vel_ref * adv` (hv0.adv). obs rotates it into its own frame first, so
* it can run as a shadow while angle0 follows the encoder (src 0 or 1), and
* `pos_err` = pos - (pos_ref + vel_ref adv) shows how far it is from the
* frame hv0 used, the encoder's angle in shadow. Run
* obs before angle0 (lower rt_prio): it then reads the angle the feedback was
* taken with. A constant pos_err growing with speed is timing, not model, and
* `adv` trims it. hv0 adds vel * adv again, so commutate from `pos_c`, which
* takes it off: in shadow pos_c is the encoder's angle.
*
* ## Loop
* A PI phase locked loop on the angle error: pos += (vel + kp err) T,
* vel += ki err T, kp = 1.4 bw, ki = bw^2. Below e_min volts of emf the error
* is not trusted: the loop holds its speed (ok = 0). Start it from a known
* angle and direction (track, or I/f): at standstill the sign of vel is a
* guess. `track` = 1 copies pos_ref and vel_ref, for a sequencer that hands
* over to it.
*/

HAL_COMP(obs);

HAL_PIN(r);          // *parameter*, winding resistance [ohm]
HAL_PIN(r_w);        // *parameter*, d loss growing with speed [ohm per rad/s electrical], r + r_w |vel|, 0 = none
HAL_PIN(ld);         // *parameter*, d inductance [H] (induction motor: sigma*Ls)
HAL_PIN(lq);         // *parameter*, q inductance [H], 0 = ld
HAL_PIN(polecount);  // *parameter*, pole pairs
HAL_PIN(bw);         // *parameter*, loop bandwidth [rad/s]
HAL_PIN(kl);         // *parameter*, current derivative low pass, 0..0.99
HAL_PIN(e_min);      // *parameter*, emf below which the angle is not trusted [V]
HAL_PIN(max_vel);    // *parameter*, speed clamp [rad/s electrical]
HAL_PIN(adv);        // *parameter*, feedback frame lead over pos_ref [s], hv0.adv

HAL_PIN(id);         // *input*, hv0.id_fb
HAL_PIN(iq);         // *input*, hv0.iq_fb
HAL_PIN(ud);         // *input*, hv0.ud_fb
HAL_PIN(uq);         // *input*, hv0.uq_fb
HAL_PIN(pos_ref);    // *input*, angle the feedback frame was at, angle0.pos
HAL_PIN(vel_ref);    // *input*, angle0.vel
HAL_PIN(slip);       // *input*, induction motor slip [rad/s electrical], 0 on a PMSM
HAL_PIN(track);      // *input*, 1 = follow pos_ref / vel_ref
HAL_PIN(pos_chk);    // *input*, encoder angle to compare with while commutating from obs

HAL_PIN(pos);        // *output*, flux angle in the feedback frame [rad electrical]
HAL_PIN(pos_c);      // *output*, pos - vel * adv, the angle to commutate with, angle0.pos_obs
HAL_PIN(vel);        // *output*, synchronous speed [rad/s electrical]
HAL_PIN(vel_m);      // *output*, rotor speed [rad/s mechanical], (vel - slip) / polecount, slip with the loop's lag
HAL_PIN(ed);         // *output*, d emf in the observer's frame [V]
HAL_PIN(eq);         // *output*, q emf [V]
HAL_PIN(err);        // *output*, angle error the loop sees [rad]
HAL_PIN(pos_err);    // *output*, pos - (pos_ref + vel_ref * adv) [rad]
HAL_PIN(chk_err);    // *output*, pos - (pos_chk + vel_ref * adv) [rad]
HAL_PIN(ok);         // *output*, 1 = emf above e_min for 50 ms

struct obs_ctx_t {
  float id_old, iq_old;
  float did, diq;
  float ok_time;
  float s_e, s_v;  // slip through a copy of the tracking loop
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct obs_pin_ctx_t *pins = (struct obs_pin_ctx_t *)pin_ptr;

  PIN(r)         = 0.5;
  PIN(ld)        = 0.003;
  PIN(lq)        = 0.0;
  PIN(polecount) = 4.0;
  PIN(bw)        = 100.0;
  PIN(kl)        = 0.75;
  PIN(e_min)     = 3.0;
  PIN(max_vel)   = 3000.0;
  PIN(adv)       = 0.0;
}

static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct obs_ctx_t *ctx = (struct obs_ctx_t *)ctx_ptr;

  ctx->id_old = ctx->iq_old = 0.0;
  ctx->did = ctx->diq = 0.0;
  ctx->ok_time        = 0.0;
  ctx->s_e = ctx->s_v = 0.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct obs_ctx_t *ctx     = (struct obs_ctx_t *)ctx_ptr;
  struct obs_pin_ctx_t *pins = (struct obs_pin_ctx_t *)pin_ptr;

  // induction motor iron loss shows as a resistance that grows with speed;
  // left out it puts a sign(vel) angle lag into the observer
  float r   = MAX(PIN(r) + PIN(r_w) * ABS(PIN(vel)), 0.0);
  float ld  = MAX(PIN(ld), 0.00001);
  float lq  = PIN(lq) > 0.0 ? PIN(lq) : ld;
  float kl  = CLAMP(PIN(kl), 0.0, 0.99);
  float pos = PIN(pos);
  float vel = PIN(vel);

  // feedback frame into the observer's frame
  float frame = mod(PIN(pos_ref) + PIN(vel_ref) * PIN(adv));
  float delta = minus(pos, frame);
  float sn, cs;
  sincos_fast(delta, &sn, &cs);
  float id = PIN(id) * cs + PIN(iq) * sn;
  float iq = -PIN(id) * sn + PIN(iq) * cs;
  float ud = PIN(ud) * cs + PIN(uq) * sn;
  float uq = -PIN(ud) * sn + PIN(uq) * cs;

  ctx->did    = ctx->did * kl + (id - ctx->id_old) * (1.0 - kl);
  ctx->diq    = ctx->diq * kl + (iq - ctx->iq_old) * (1.0 - kl);
  ctx->id_old = id;
  ctx->iq_old = iq;

  float ed = ud - r * id - ld * ctx->did / period + vel * lq * iq;
  float eq = uq - r * iq - lq * ctx->diq / period - vel * ld * id;

  float e_min = MAX(PIN(e_min), 0.01);
  float e     = sqrtf(ed * ed + eq * eq);
  float err   = 0.0;
  if(e > e_min) {
    // the emf leads on q in the direction of rotation: s * eq > 0 near lock.
    // -ed / eq alone also locks half a turn off, so use -s ed / e (sin of the
    // error) and push at full rate while eq points the wrong way
    float s = vel > 0.0 ? 1.0 : (vel < 0.0 ? -1.0 : (eq >= 0.0 ? 1.0 : -1.0));
    if(s * eq > 0.0) {
      err = CLAMP(-s * ed / e, -0.5, 0.5);
    } else {
      err = -s * ed >= 0.0 ? 0.5 : -0.5;
    }
    ctx->ok_time += period;
  } else {
    ctx->ok_time = 0.0;
  }
  float ok = ctx->ok_time > 0.05 ? 1.0 : 0.0;

  // vel picks up a slip step only through the tracking loop, so vel_m takes
  // the slip through a copy of that loop. Taken straight, a step in iq dips
  // vel_m for about 1 / bw, pid asks for more iq, and with little flux (deep
  // field weakening) that positive feedback rings at about the observer bw.
  if(PIN(track) > 0.0) {
    pos       = frame;  // the feedback frame, like the free running estimate
    vel       = PIN(vel_ref);
    ctx->s_e  = 0.0;
    ctx->s_v  = PIN(slip);
  } else {
    float bw = MAX(PIN(bw), 0.0);
    vel += bw * bw * err * period;
    vel = LIMIT(vel, MAX(PIN(max_vel), 1.0));
    pos = mod(pos + (vel + 1.4 * bw * err) * period);
    ctx->s_e += (PIN(slip) - ctx->s_v - 1.4 * bw * ctx->s_e) * period;
    ctx->s_v += bw * bw * ctx->s_e * period;
  }

  PIN(ed)      = ed;
  PIN(eq)      = eq;
  PIN(err)     = err;
  PIN(ok)      = ok;
  PIN(vel)     = vel;
  PIN(vel_m)   = (vel - ctx->s_v) / MAX(PIN(polecount), 1.0);
  // compare this tick's estimate, before the step to the next tick
  PIN(pos_err) = delta;
  PIN(chk_err) = minus(PIN(pos), mod(PIN(pos_chk) + PIN(vel_ref) * PIN(adv)));
  PIN(pos)     = pos;
  PIN(pos_c)   = mod(pos - vel * PIN(adv));
}

hal_comp_t obs_comp_struct = {
    .name      = "obs",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct obs_ctx_t),
    .pin_count = sizeof(struct obs_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
