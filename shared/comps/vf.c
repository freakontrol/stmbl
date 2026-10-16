#include "vf_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"

/**
* ## Brief
* `vf` runs an induction motor open loop, V/f, and replaces uf (uf2.c) for new configs. It is loaded by `conf/template/vf.txt`: its frequency `vel_e` goes to `angle0.vel_cmd` (`angle0.src` 3) and its voltage `u_cmd` to `hv0.d_cmd` in volt mode (`hv0.cmd_mode` 0, `hv0.q_cmd` 0). `en` and `scale` come from fault0, the currents and voltages from hv0, `dc_hold` from `conf0.high_dc_volt`. `conf/template/vf_enc.txt` links vf, adds fb_switch and vel and sets `vf0.vel_fb = vel0.vel`, `vf0.enc = 1` for slip frequency control on an encoder, and `vel0.torque = vf0.torque`. The speed command is `vf0.vel_cmd` (rad/s mech).
*
* ## Component Explanation
* All work is done in `rt`. The active current `i_act` = (ud * id + uq * iq) / abs(u) is the current along the voltage vector: it is torque producing, positive when motoring in either direction and negative when the motor brakes; the magnetizing current does not count. Its low pass at `damp_hz` drives stall prevention and slip compensation, so they follow the load but not the rotor swing.
*
* 1. **Ramp and stall prevention**:
* - `vel` ramps toward `vel_cmd` at `acc` and `dec`; vf is its own ramp, so no ramp0 is needed in front. `en` 0 sets `vel` to 0 at once.
* - Accel stall prevention scales the ramp with the low passed active current: full `acc` up to `i_stall`, down to 0 at 1.1 * i_stall (`stall` 1), then back toward zero, reaching `stall_dec` at 1.3 * i_stall (`stall` 2). Proportional, so the frequency never steps. As the magnetizing current does not count, i_stall is a load limit and works near base speed too.
* - Decel stall prevention: `vel` stops falling while the dc link is above `dc_hold`, the motor regenerating into the link (`stall` -1).
* - `scale` (fault0.scale) lowers i_stall, so a derate slows the motor instead of cutting its flux.
*
* 2. **Voltage**:
* - `u_cmd` = u_n * abs(vel + slip) / vel_n plus a boost u_boost that fades linearly to 0 at boost_vel. Phase peak volts, like every volt mode d_cmd.
* - Capped at duty * pwm_volt (hv0.pwm_volt, live from the dc link): above that speed the flux falls as 1/f, field weakening, and `u_lim` is 1.
* - The vf template sets u_boost 12 V: it covers the dead time drop (no compensation in volt mode) plus r * the magnetizing current, so the encoder path starts from rest on the slip frequency alone (spindle IM06B50GC1, drop about 8.7 V).
*
* 3. **Stop at 0**:
* - A `vel_cmd` below 0.5 rad/s (either sign) counts as 0, so noise on a 0 command does not keep the boost and the speed PI running. Once the command is 0 and the ramp has reached `vel` 0, `u_cmd` is 0 and, with the encoder, the slip and its integrator are 0 too, so the field stops and the rotor runs down. The damping states reset there as well, so `damp` and `vel_e` are exactly 0 and the next start begins clean. Without this the boost drove a dc current into the limit at standstill (21 A on the spindle), which only heats a cage motor, and the encoder path's speed PI on 0 hunted with the field near 0 Hz (spindle, 12-13 Oct: +-17 rad/s, 29 A peaks).
* - A new command restarts the encoder path's PI from the open loop slip.
*
* 4. **Slip compensation** (open loop, `slip_n` > 0):
* - `slip` = slip_n * i_act / cur_n from the low passed active current, clamped to 2 * slip_n, times the sign of vel: it adds in the direction of rotation when motoring and takes away when braking. Unfiltered, every current swing would move the field and feed the hunting.
*
* 5. **Encoder path** (`enc` 1, needs `vel_fb`): slip frequency control.
* - The field turns at vel_fb + slip, and slip comes from a PI on the speed error vel - vel_fb (`enc_kp`, `enc_ki`), clamped to `slip_max` (2 * slip_n when 0). With slip_n and slip_max both 0 the clamp would be 0 and the field would sit on the rotor with no torque, so vf runs open loop instead.
* - The ramp, stall prevention and dc_hold still shape `vel`, which becomes the speed reference. The slip clamp bounds the torque, so the rotor cannot pull out; the integrator holds while clamped, and vel stays within slip_max of vel_fb, so a ramp faster than the motor can follow just slows down.
* - It replaces the slip compensation and the damping (k_damp, k_vel): the field is tied to the rotor. Switched on while running, the integrator starts from the open loop slip.
* - With the encoder the voltage (u_n * abs(vel_fb + slip) / vel_n) and the boost fade (on abs(vel_fb)) use the rotor speed vel_fb in place of vel, so the boost does not fade with the reference at standstill.
*
* 6. **Damping** (optional, open loop only):
* - Open loop V/f on a lightly loaded motor hunts, the rotor swinging against the field. `damp` = k_damp * the active current band passed between damp_hz and damp_lp_hz moves the field frequency with the swing and damps it. The term is multiplied by the sign of vel, like the slip, so it damps the same in reverse. The high pass keeps steady slip and the ramp's acceleration current out, the low pass the pwm and current loop ripple. Frequency only, the voltage stays on vel + slip. Clamped to 5 % of vel_n. Positive k_damp damps.
* - Speed damping (needs a speed feedback): `k_vel` times the rotor speed swing (vel_fb - vel, same band pass) adds to `damp`. With an encoder this measures the swing directly, to find the damping law that an estimate can then replace.
* - `vel_src` 1 uses an estimate instead, no encoder: the air gap torque `torque` = 1.5 * (u * i_act - r * (id^2 + iq^2)) / signed synchronous speed (at least 5 rad/s) over `j` is the rotor acceleration; less the field's own acceleration (the ramp, stall prevention, slip, damp), high passed at damp_hz and integrated (leaky at damp_hz), it gives the swing of the rotor against the field, `w_est`. Needs `j` > 0.
*
* 7. **Output**:
* - `vel_e` = (vel + slip + damp) * polecount (vel_fb in place of vel with the encoder), the synchronous electrical speed for angle0.vel_cmd, so hv0.vel is right and the f3 extrapolates between packets.
*
* {{% hint warning %}}
* - vf has no speed clamp and the vf template does not link conf0.max_vel, so limit vel_cmd at its source. Under load it is usable from about boost_vel (below it r and the dead time drop eat most of the small voltage) to about vel_n; above the voltage cap the flux falls as 1/f and the breakdown torque as 1/f^2.
* - vel_fb must have the field's sign and mechanical scale; a reversed encoder runs away against the slip clamp, so check vel_fb (vel0.vel against vf0.vel, conf0.mot_fb_rev) with enc 0 before the first run.
* - Speed commands below 0.5 rad/s mech (about 5 rpm) are not possible: they count as 0 and stop the field.
* {{% /hint %}}
*/

HAL_COMP(vf);

HAL_PIN(en);         // *input*, Enable, 0 = frequency back to 0 at once (fault0.en_out)
HAL_PIN(vel_cmd);    // *input*, Speed command (rad/s mech)
HAL_PIN(polecount);  // *parameter*, Pole pairs (conf0.polecount, default 2)
HAL_PIN(u_n);        // *parameter*, Phase peak voltage at vel_n (V, default 90)
HAL_PIN(vel_n);      // *parameter*, Base speed (rad/s mech, default 246.3)
HAL_PIN(u_boost);    // *parameter*, Low speed voltage boost at standstill (V, default 0, vf template 12)
HAL_PIN(boost_vel);  // *parameter*, Speed where the boost has faded out (rad/s mech, default 25)
HAL_PIN(slip_n);     // *parameter*, Rated slip at active current cur_n, 0 = off (rad/s mech, default 0)
HAL_PIN(cur_n);      // *parameter*, Rated active current (A peak, default 10)
HAL_PIN(acc);        // *parameter*, Acceleration (rad/s^2 mech, default 50)
HAL_PIN(dec);        // *parameter*, Deceleration (rad/s^2 mech, default 50)
HAL_PIN(i_stall);    // *parameter*, Accel stall prevention active current, 0 = off (A peak, default 0)
HAL_PIN(stall_dec);  // *parameter*, Deceleration while stalled (rad/s^2 mech, default 50)
HAL_PIN(dc_hold);    // *parameter*, DC link voltage above which deceleration holds, 0 = off (V, default 0, vf template conf0.high_dc_volt)
HAL_PIN(scale);      // *input*, Derate of i_stall, 1 = none (fault0.scale, default 1)
HAL_PIN(k_damp);     // *parameter*, Current damping gain, 0 = off (rad/s mech per A, default 0)
HAL_PIN(damp_hz);    // *parameter*, Damping high pass and active current low pass corner (Hz, default 3)
HAL_PIN(damp_lp_hz); // *parameter*, Damping low pass corner (Hz, default 20)
HAL_PIN(k_vel);      // *parameter*, Speed damping gain, 0 = off (1, default 0)
HAL_PIN(vel_fb);     // *input*, Rotor speed for k_vel and the encoder path (rad/s mech, vf_enc template vel0.vel)
HAL_PIN(vel_src);    // *parameter*, k_vel input, 0 = vel_fb, 1 = estimate (default 0)
HAL_PIN(j);          // *parameter*, Inertia for the estimate, e.g. conf0.j (kg m^2, default 0)
HAL_PIN(r);          // *parameter*, Phase resistance for the estimate, e.g. conf0.r (ohm, default 0)

HAL_PIN(id);         // *input*, D current feedback (A, hv0.id_fb)
HAL_PIN(iq);         // *input*, Q current feedback (A, hv0.iq_fb)
HAL_PIN(ud);         // *input*, D voltage feedback (V, hv0.ud_fb)
HAL_PIN(uq);         // *input*, Q voltage feedback (V, hv0.uq_fb)
HAL_PIN(dc_volt);    // *input*, DC link voltage (V, hv0.dc_volt)
HAL_PIN(pwm_volt);   // *input*, Usable phase voltage, 0 = no voltage cap (V peak, hv0.pwm_volt)
HAL_PIN(duty);       // *parameter*, Voltage cap as a fraction of pwm_volt (default 0.9)
HAL_PIN(enc);        // *parameter*, 1 = slip frequency control on vel_fb, 0 = open loop (default 0, vf_enc template 1)
HAL_PIN(enc_kp);     // *parameter*, Encoder path slip per speed error (1, default 0.25)
HAL_PIN(enc_ki);     // *parameter*, Encoder path slip integral gain (1/s, default 5)
HAL_PIN(slip_max);   // *parameter*, Encoder path slip clamp, 0 = 2 * slip_n (rad/s mech, default 0)

HAL_PIN(vel);        // *output*, Ramped stator frequency less slip, the speed reference (rad/s mech)
HAL_PIN(vel_e);      // *output*, Synchronous electrical speed, to angle0.vel_cmd (rad/s)
HAL_PIN(u_cmd);      // *output*, Phase peak voltage, to hv0.d_cmd (V)
HAL_PIN(i_act);      // *output*, Active current, negative when braking (A)
HAL_PIN(slip);       // *output*, Slip compensation or encoder path slip (rad/s mech)
HAL_PIN(stall);      // *output*, Stall prevention, 1 = accel held, 2 = accel reversed, -1 = decel held, 0 = none
HAL_PIN(damp);       // *output*, Damping frequency offset (rad/s mech)
HAL_PIN(torque);     // *output*, Air gap torque estimate (Nm)
HAL_PIN(w_est);      // *output*, Estimated rotor speed swing (rad/s mech)
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
  if(ABS(cmd) < 0.5) {  // [rad/s] noise on a 0 command must not keep the boost and the PI on
    cmd = 0.0;
  }
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
  // at a 0 command once the ramp is there, the field stops as open loop
  // does: a speed PI on 0 hunts with the field near 0 Hz, where the boost
  // drives a dc current into the limit and the slip's torque gain swings
  // (spindle, 12-13 Oct: +-17 rad/s, 29 A peaks); the rotor runs down
  int stop0 = cmd == 0.0 && vel == 0.0;
  if(enc && PIN(en) > 0.0 && stop0) {
    ctx->s_i = 0.0;
    slip     = 0.0;
  } else if(enc && PIN(en) > 0.0) {
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
  ctx->enc_on = enc && PIN(en) > 0.0 && !stop0;

  float boost = 0.0;
  if(PIN(boost_vel) > 0.0) {
    // on the rotor speed with the encoder: it starts from standstill with the
    // field at slip only, so the boost must not fade with the reference
    boost = PIN(u_boost) * MAX(1.0 - ABS(base) / PIN(boost_vel), 0.0);
  }
  float u_cmd = PIN(u_n) * ABS(base + slip) / MAX(PIN(vel_n), 0.1) + boost;
  if(stop0) {
    // standing at 0: the boost would only drive a dc current (21 A on the
    // spindle) that heats the motor, a cage takes no torque from it
    u_cmd = 0.0;
  }
  float u_max = CLAMP(PIN(duty), 0.0, 1.0) * PIN(pwm_volt);
  float u_lim = 0.0;
  if(PIN(pwm_volt) > 0.0 && u_cmd > u_max) {
    u_cmd = u_max;
    u_lim = 1.0;
  }

  float damp = 0.0;
  if(PIN(en) > 0.0 && !stop0) {  // at stop0 the states reset, so vel_e is 0 and the start is clean
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
