#include "vf_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"

/**
* ## Brief
* The `vf` component runs an induction motor open loop, V/f, and replaces uf
* (uf2.c) for new configs. Its frequency goes to angle0 (src 3) and its
* voltage to hv0.d_cmd in volt mode.
*
* ## Behaviour
* - Frequency `vel` ramps toward `vel_cmd` (mechanical rad/s) at `acc` and
*   `dec`; vf is its own ramp, so no ramp0 is needed in front. Accel stall prevention scales the ramp with the active current (low
*   passed at damp_hz): full `acc` up to `i_stall`, down to 0 at 1.1 *
*   i_stall, then back toward zero, reaching `stall_dec` at 1.3 * i_stall.
*   Proportional, so the frequency never steps. The magnetizing current doesn't count, so
*   i_stall is a load limit and works near base speed too. It stops
*   falling while the dc link is above `dc_hold` (decel stall prevention, the
*   motor regenerating into the link). `scale` (fault0.scale) lowers i_stall,
*   so a derate slows the motor instead of cutting its flux.
* - Voltage: u_n * |vel| / vel_n plus a boost u_boost that fades linearly to 0
*   at boost_vel. Phase peak volts, like every volt mode d_cmd. Capped at
*   duty * pwm_volt (hv0.pwm_volt, live from the dc link): above that speed
*   the flux falls as 1/f, field weakening, and `u_lim` is 1. pwm_volt 0 =
*   no cap.
* - Slip compensation from the ACTIVE current, signed: the current along the
*   voltage vector, which is torque producing and goes negative when the motor
*   brakes, minus nothing for magnetizing. slip_n is the rated slip in
*   mechanical rad/s at active current cur_n. It uses the active current low
*   passed at damp_hz, so it follows the load but not the rotor swing:
*   unfiltered, every current swing would move the field and feed the
*   hunting.
* - Encoder path (optional, `enc` 1, needs `vel_fb`): slip frequency control.
*   The field turns at vel_fb + slip, and slip comes from a PI on the speed
*   error vel - vel_fb (`enc_kp` [1], `enc_ki` [1/s]), clamped to `slip_max`
*   (mechanical rad/s, 0 = 2 * slip_n). The ramp, stall prevention and
*   dc_hold still shape `vel`, which becomes the speed reference. The slip
*   clamp bounds the torque, so the rotor cannot pull out, and the integrator
*   holds while clamped, and vel stays within slip_max of vel_fb, so a ramp
*   faster than the motor can follow just slows down. It replaces the slip compensation and the damping
*   (k_damp, k_vel): the field is tied to the rotor. vel_fb must have the
*   field's sign and mechanical scale; a reversed encoder runs away against
*   the clamp, so check vel_fb in open loop first. enc 0 = off; with
*   slip_n and slip_max both 0 it runs open loop.
* - `vel_e` = (vel + slip) * polecount, synchronous electrical speed for
*   angle0.vel_cmd, so hv0.vel is right and the f3 extrapolates between packets.
* - Damping (optional): open loop V/f on a lightly loaded motor hunts, the
*   rotor swinging against the field. `damp` = k_damp * the active current
*   band passed between damp_hz and damp_lp_hz moves the field frequency with
*   the swing and damps it. The high pass keeps steady slip and the ramp's
*   acceleration current out, the low pass the pwm and current loop ripple.
*   Frequency only, the voltage stays on vel + slip. Clamped to 5 % of vel_n.
*   k_damp 0 = off, positive damps.
* - Speed damping (optional, needs a speed feedback): `k_vel` times the
*   rotor speed swing (vel_fb - vel, same band pass) adds to `damp`. With an
*   encoder this measures the swing directly, to find the damping law that
*   an estimate can then replace. k_vel 0 = off.
*   vel_src 1 uses an estimate instead, no encoder: the air gap torque
*   1.5 * (u * i_act - r * |i|^2) / signed synchronous speed over `j` is the rotor
*   acceleration; less the field's own acceleration (the ramp, stall
*   prevention, slip, damp), high passed at damp_hz and integrated (leaky at
*   damp_hz), it gives the swing of the rotor against the field, `w_est`. Needs `j` (conf0.j) and `r` (conf0.r).
*/

HAL_COMP(vf);

HAL_PIN(en);         // *input*, 0 = frequency back to 0 at once
HAL_PIN(vel_cmd);    // *input*, speed command [rad/s mech]
HAL_PIN(polecount);  // *parameter*, pole pairs
HAL_PIN(u_n);        // *parameter*, phase peak volts at vel_n
HAL_PIN(vel_n);      // *parameter*, base speed [rad/s mech]
HAL_PIN(u_boost);    // *parameter*, low speed boost at standstill [V]
HAL_PIN(boost_vel);  // *parameter*, speed where the boost has faded out [rad/s mech]
HAL_PIN(slip_n);     // *parameter*, rated slip [rad/s mech] at active current cur_n, 0 = off
HAL_PIN(cur_n);      // *parameter*, rated active current [A peak]
HAL_PIN(acc);        // *parameter*, acceleration [rad/s^2 mech]
HAL_PIN(dec);        // *parameter*, deceleration [rad/s^2 mech]
HAL_PIN(i_stall);    // *parameter*, accel stall active current [A peak], 0 = off
HAL_PIN(stall_dec);  // *parameter*, deceleration while stalled [rad/s^2 mech]
HAL_PIN(dc_hold);    // *parameter*, dc link volts that hold deceleration, 0 = off
HAL_PIN(scale);      // *input*, derate, 1 = none
HAL_PIN(k_damp);     // *parameter*, damping gain [rad/s mech per A], 0 = off
HAL_PIN(damp_hz);    // *parameter*, damping high pass corner [Hz]
HAL_PIN(damp_lp_hz); // *parameter*, damping low pass corner [Hz]
HAL_PIN(k_vel);      // *parameter*, speed damping gain [1], 0 = off
HAL_PIN(vel_fb);     // *input*, rotor speed [rad/s mech], for k_vel
HAL_PIN(vel_src);    // *parameter*, k_vel input: 0 vel_fb, 1 estimate
HAL_PIN(j);          // *parameter*, inertia [kg m^2], conf0.j, for the estimate
HAL_PIN(r);          // *parameter*, phase resistance [ohm], conf0.r, for the estimate

HAL_PIN(id);         // *input*, hv0.id_fb
HAL_PIN(iq);         // *input*, hv0.iq_fb
HAL_PIN(ud);         // *input*, hv0.ud_fb
HAL_PIN(uq);         // *input*, hv0.uq_fb
HAL_PIN(dc_volt);    // *input*, hv0.dc_volt
HAL_PIN(pwm_volt);   // *input*, hv0.pwm_volt [V peak], 0 = no voltage cap
HAL_PIN(duty);       // *parameter*, voltage cap as a fraction of pwm_volt
HAL_PIN(enc);        // *parameter*, 1 = slip frequency control on vel_fb, 0 = open loop
HAL_PIN(enc_kp);     // *parameter*, slip per speed error [1]
HAL_PIN(enc_ki);     // *parameter*, slip integral gain [1/s]
HAL_PIN(slip_max);   // *parameter*, slip clamp [rad/s mech], 0 = 2 * slip_n

HAL_PIN(vel);        // *output*, stator frequency, less slip [rad/s mech]
HAL_PIN(vel_e);      // *output*, synchronous electrical speed [rad/s], to angle0.vel_cmd
HAL_PIN(u_cmd);      // *output*, phase peak volts, to hv0.d_cmd
HAL_PIN(i_act);      // *output*, active current [A], negative when braking
HAL_PIN(slip);       // *output*, slip compensation [rad/s mech]
HAL_PIN(stall);      // *output*, 1 accel held, 2 accel reversed, -1 decel held
HAL_PIN(damp);       // *output*, damping frequency offset [rad/s mech]
HAL_PIN(torque);     // *output*, air gap torque estimate [Nm]
HAL_PIN(w_est);      // *output*, estimated rotor speed swing [rad/s mech]
HAL_PIN(u_lim);      // *output*, 1 = u_cmd capped at duty * pwm_volt

struct vf_ctx_t {
  float i_lp;  // active current low pass at damp_hz, damping high pass
  float d_lp;  // high passed current, low passed at damp_lp_hz
  float v_lp;  // speed swing low pass at damp_hz, high pass
  float w_lp;  // high passed speed swing, low passed at damp_lp_hz
  float t_lp;  // relative acceleration low pass at damp_hz, high pass
  float w_est; // speed swing estimate, leaky integral of the torque swing
  float vel_f; // last field speed vel + slip + damp, for the field acceleration
  float s_i;   // encoder path: slip integral [rad/s mech]
  int enc_on;  // encoder path ran last tick
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct vf_pin_ctx_t *pins = (struct vf_pin_ctx_t *)pin_ptr;

  PIN(polecount) = 2.0;
  PIN(u_n)       = 90.0;
  PIN(vel_n)     = 246.3;
  PIN(u_boost)   = 0.0;
  PIN(boost_vel) = 25.0;
  PIN(slip_n)    = 0.0;
  PIN(cur_n)     = 10.0;
  PIN(acc)       = 50.0;
  PIN(dec)       = 50.0;
  PIN(i_stall)   = 0.0;
  PIN(stall_dec) = 50.0;
  PIN(dc_hold)   = 0.0;
  PIN(scale)     = 1.0;
  PIN(k_damp)    = 0.0;
  PIN(k_vel)     = 0.0;
  PIN(vel_src)   = 0.0;
  PIN(j)         = 0.0;
  PIN(r)         = 0.0;
  PIN(duty)      = 0.9;
  PIN(enc)       = 0.0;
  PIN(enc_kp)    = 0.25;
  PIN(enc_ki)    = 5.0;
  PIN(slip_max)  = 0.0;
  PIN(damp_hz)    = 3.0;
  PIN(damp_lp_hz) = 20.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct vf_ctx_t *ctx      = (struct vf_ctx_t *)ctx_ptr;
  struct vf_pin_ctx_t *pins = (struct vf_pin_ctx_t *)pin_ptr;

  float vel = PIN(vel);
  float cmd = PIN(vel_cmd);
  float id  = PIN(id);
  float iq  = PIN(iq);
  float ud  = PIN(ud);
  float uq  = PIN(uq);

  float u     = sqrtf(ud * ud + uq * uq);
  float i_act = u > 0.1 ? (ud * id + uq * iq) / u : 0.0;

  // active current low pass: stall prevention, and the damping high pass
  if(PIN(en) > 0.0) {
    ctx->i_lp += (i_act - ctx->i_lp) * CLAMP(2.0 * M_PI * PIN(damp_hz) * period, 0.0, 1.0);
  } else {
    ctx->i_lp = 0.0;
  }

  float stall = 0.0;
  float i_st  = PIN(i_stall) * CLAMP(PIN(scale), 0.0, 1.0);
  int accel   = ABS(cmd) > ABS(vel) && cmd * vel >= 0.0;
  int decel   = !accel && cmd != vel;

  if(PIN(en) <= 0.0) {
    vel = 0.0;
  } else if(accel) {
    // over: active current above i_stall, as a fraction of i_stall
    float over = PIN(i_stall) > 0.0 ? (ctx->i_lp - i_st) / MAX(i_st, 0.1) : -1.0;
    if(over > 0.1) {  // back off toward zero
      stall = 2.0;
      float dec = MAX(PIN(stall_dec), 0.0) * MIN((over - 0.1) / 0.2, 1.0);
      vel -= SIGN(vel) * MIN(dec * period, ABS(vel));
    } else {
      float acc = MAX(PIN(acc), 0.0);
      if(over > 0.0) {  // slow down toward a hold
        stall = 1.0;
        acc *= 1.0 - over / 0.1;
      }
      vel += LIMIT(cmd - vel, acc * period);
    }
  } else if(decel) {
    if(PIN(dc_hold) > 0.0 && PIN(dc_volt) > PIN(dc_hold)) {
      stall = -1.0;
    } else {
      vel += LIMIT(cmd - vel, MAX(PIN(dec), 0.0) * period);
    }
  }

  float slip = 0.0;
  if(PIN(slip_n) > 0.0 && PIN(cur_n) > 0.0) {
    // i_act follows the power: positive motoring in either direction, so the
    // slip adds in the direction of rotation when motoring and takes away
    // when braking
    slip = LIMIT(PIN(slip_n) * ctx->i_lp / PIN(cur_n), 2.0 * PIN(slip_n)) * SIGN(vel);
  }

  // encoder path: the field runs at the rotor speed plus a slip from a speed
  // PI, so the speed follows vel without the slip model and the rotor
  // cannot pull out of the field
  // without slip_n or slip_max the slip clamp is 0: the field would sit on
  // the rotor with no torque, so that runs open loop instead
  int enc    = PIN(enc) > 0.0 && (PIN(slip_max) > 0.0 || PIN(slip_n) > 0.0);
  float base = vel;  // field speed less slip: the reference open loop, the rotor with the encoder
  if(enc && PIN(en) > 0.0) {
    float s_max = PIN(slip_max) > 0.0 ? PIN(slip_max) : 2.0 * PIN(slip_n);
    if(!ctx->enc_on) {  // switched on while running: start from the open loop slip
      ctx->s_i = LIMIT(slip, s_max);
    }
    float e     = vel - PIN(vel_fb);
    float s_p   = PIN(enc_kp) * e;
    float s     = s_p + ctx->s_i + PIN(enc_ki) * e * period;
    if(ABS(s) < s_max || s * e < 0.0) {  // integrate unless that winds up against the clamp
      ctx->s_i += PIN(enc_ki) * e * period;
    }
    ctx->s_i = LIMIT(ctx->s_i, s_max);
    slip     = LIMIT(s_p + ctx->s_i, s_max);
    base     = PIN(vel_fb);
    // the reference stays within slip_max of the rotor: on the clamp the ramp
    // follows the rotor's acceleration instead of running away from it
    vel = CLAMP(vel, base - s_max, base + s_max);
  } else {
    ctx->s_i = 0.0;
  }
  ctx->enc_on = enc && PIN(en) > 0.0;

  float boost = 0.0;
  if(PIN(boost_vel) > 0.0) {
    // on the rotor speed with the encoder: it starts from standstill with the
    // field at slip only, so the boost must not fade with the reference
    boost = PIN(u_boost) * MAX(1.0 - ABS(base) / PIN(boost_vel), 0.0);
  }
  float u_cmd = PIN(u_n) * ABS(base + slip) / MAX(PIN(vel_n), 0.1) + boost;
  float u_max = CLAMP(PIN(duty), 0.0, 1.0) * PIN(pwm_volt);
  float u_lim = 0.0;
  if(PIN(pwm_volt) > 0.0 && u_cmd > u_max) {
    u_cmd = u_max;
    u_lim = 1.0;
  }

  float damp = 0.0;
  if(PIN(en) > 0.0) {
    float k_lp = CLAMP(2.0 * M_PI * PIN(damp_lp_hz) * period, 0.0, 1.0);
    ctx->d_lp += (i_act - ctx->i_lp - ctx->d_lp) * k_lp;
    float k_hp = CLAMP(2.0 * M_PI * PIN(damp_hz) * period, 0.0, 1.0);
    float dv   = PIN(vel_fb) - vel;
    ctx->v_lp += (dv - ctx->v_lp) * k_hp;
    float w = dv - ctx->v_lp;  // measured swing

    // signed synchronous speed: the air gap power over it is the torque with
    // its sign, so the estimate is the same in reverse
    float w_f    = base + slip;
    float w_s    = (w_f < 0.0 ? -1.0 : 1.0) * MAX(ABS(w_f), 5.0);  // [rad/s mech]
    float torque = 1.5 * (u * i_act - PIN(r) * (id * id + iq * iq)) / w_s;
    // rotor acceleration less field acceleration, both [rad/s^2 mech]
    float acc_f = (base + slip + PIN(damp) - ctx->vel_f) / period;
    ctx->vel_f  = base + slip + PIN(damp);
    float a_rel = PIN(j) > 0.0 ? torque / PIN(j) - acc_f : 0.0;
    ctx->t_lp += (a_rel - ctx->t_lp) * k_hp;
    if(PIN(j) > 0.0) {
      ctx->w_est += (a_rel - ctx->t_lp - ctx->w_est * 2.0 * M_PI * PIN(damp_hz)) * period;
    } else {
      ctx->w_est = 0.0;
    }
    PIN(torque) = torque;
    if(PIN(vel_src) > 0.0) {
      w = ctx->w_est;
    }
    ctx->w_lp += (w - ctx->w_lp) * k_lp;
    // i_act is positive motoring either way, so its swing moves the field in
    // the direction of rotation: times the sign of vel, like the slip
    float dir = vel < 0.0 ? -1.0 : 1.0;
    damp = enc ? 0.0 : LIMIT(PIN(k_damp) * ctx->d_lp * dir + PIN(k_vel) * ctx->w_lp, 0.05 * MAX(PIN(vel_n), 0.1));
  } else {
    ctx->d_lp = 0.0;
    ctx->v_lp  = 0.0;
    ctx->w_lp  = 0.0;
    ctx->t_lp  = 0.0;
    ctx->w_est = 0.0;
    ctx->vel_f = 0.0;
  }

  PIN(vel)   = vel;
  PIN(slip)  = slip;
  PIN(i_act) = i_act;
  PIN(stall) = stall;
  PIN(damp)  = damp;
  PIN(w_est) = ctx->w_est;
  PIN(vel_e) = (base + slip + damp) * MAX(PIN(polecount), 1.0);
  PIN(u_cmd) = u_cmd;
  PIN(u_lim) = u_lim;
}

hal_comp_t vf_comp_struct = {
    .name      = "vf",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct vf_ctx_t),
    .pin_count = sizeof(struct vf_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
