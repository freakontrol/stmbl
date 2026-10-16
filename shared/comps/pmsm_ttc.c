#include "pmsm_ttc_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `pmsm_ttc` (torque to current) converts a torque command (Nm) into a q-axis current command (peak A) for a permanent magnet synchronous motor, with optional cogging / torque ripple compensation and an experimental block commutation mode. It runs on the F4 board and is loaded by `conf/template/pmsm.txt`: `pmsm_ttc0.torque = pid0.torque_cmd`, `pmsm_ttc0.pos_in = vel2.pos_out`, `hv0.q_cmd = pmsm_ttc0.cur`, `hv0.d_cmd = pmsm_ttc0.id`, `angle0.pos_fb = pmsm_ttc0.pos_out`, with `psi` and `polecount` from `conf0`.
*
* ## Component Explanation
* All work is done in `rt`.
*
* 1. **Torque to current**:
* ```c
* cur = (torque + g * (tc + te)) * 2 / 3 / polecount / psi;
* ```
* - This inverts `torque = 3/2 * polecount * psi * iq`.
* - `id = id_in + id_mtpa`: the phasing current from `fb_switch0.id` passes through, plus the MTPA d current (0 unless `mtpa` is on).
*
* 2. **MTPA** (`mtpa` 1, needs `lq` > 0 and `lq` != `ld`):
* - With ld != lq the reluctance torque `1.5 p (ld - lq) id iq` adds to the magnet's, so a torque costs the least current at an id of its own. The command is split on that curve: `T = 1.5 p iq (psi - (lq - ld) id)` and `id = (psi - sqrt(psi^2 + 4 (lq - ld)^2 iq^2)) / (2 (lq - ld))`, written so it holds for either sign of lq - ld.
* - Two fixed point passes per tick, starting from last tick's id (the command moves little per tick). The flux term is floored at half of psi.
* - `mtpa` 0, or `lq` 0: id 0 and `iq = torque / (1.5 p psi)` as above. A later field weakening id goes on the same `id` path.
*
* 3. **Torque ripple model** (all amplitudes 0 by default, so nothing is added):
* - Cogging torque, independent of load: `tc = ac * sin(pc + pos_in * nc)`.
* - Electrical ripple, proportional to the torque command: `te = torque * ae * sin(pe + pos_in * ne)`.
* - With `ac` or `ae` at 0 the matching term is 0 without computing the sine (the normal case, saves rt time).
* - `t = tc + te` is always output; it is only added to the current command when the gain `g` is non zero.
*
* 4. **Block commutation** (experimental):
* - `pos_out` is `pos_in` quantized to six 60 degree sectors, blended with the unmodified position by `block_gain`:
* ```c
* pos_out = pos_in * (1 - block_gain) + sector_pos(pos_in) * block_gain;
* ```
* - `block_gain` defaults to 0, so `pos_out = pos_in`.
*/

HAL_COMP(pmsm_ttc);

// motor values
HAL_PIN(psi);        // *parameter*, Flux linkage (Vs), min 0.01
HAL_PIN(polecount);  // *parameter*, Pole pairs, min 1

// cogging torque ripple, constant
HAL_PIN(ac);  // *parameter*, Cogging torque amplitude (Nm)
HAL_PIN(pc);  // *parameter*, Cogging torque phase (rad)
HAL_PIN(nc);  // *parameter*, Cogging periods per turn of pos_in (default 1)

// electrical torque ripple, linear with current
HAL_PIN(ae);  // *parameter*, Electrical ripple amplitude, relative to torque
HAL_PIN(pe);  // *parameter*, Electrical ripple phase (rad)
HAL_PIN(ne);  // *parameter*, Electrical ripple periods per turn of pos_in (default 1)

HAL_PIN(pos_in);      // *input*, Rotor position (rad)
HAL_PIN(pos_out);     // *output*, Commutation position, pos_in blended with block commutation (rad)
HAL_PIN(t);           // *output*, Modelled ripple torque tc + te (Nm)
HAL_PIN(g);           // *parameter*, Ripple compensation gain, 0 = off
HAL_PIN(block_gain);  // *parameter*, Block commutation blend 0..1 (default 0, experimental)


// torque cmd in
HAL_PIN(torque);  // *input*, Torque command (Nm)

// cur cmd out
HAL_PIN(cur);  // *output*, q-axis current command (A)

// MTPA: with ld != lq the reluctance torque 1.5 p (ld - lq) id iq adds to the
// magnet's, so a torque costs the least current at an id of its own. mtpa 1
// splits the torque into id and iq on that curve, 0 (default) keeps id 0 and
// iq = torque / (1.5 p psi). id = id_in + the MTPA id; a later field weakening
// id goes on the same path.
HAL_PIN(mtpa);   // *parameter*, 1 = split the torque into id and iq on the MTPA curve, 0 (default) = id 0
HAL_PIN(ld);     // *input*, d axis inductance (H), from conf0.l
HAL_PIN(lq);     // *input*, q axis inductance (H), from conf0.lq, 0 = same as ld (no reluctance torque, id 0)
HAL_PIN(id_in);  // *input*, d current from fb_switch0.id, the phasing current (A)
HAL_PIN(id);     // *output*, d current command id_in + MTPA id (A), to hv0.d_cmd

struct pmsm_ttc_ctx_t {
  float id_mtpa;  // last tick's, the start for this tick's iteration
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct pmsm_ttc_ctx_t *ctx      = (struct pmsm_ttc_ctx_t *)ctx_ptr;
  struct pmsm_ttc_pin_ctx_t *pins = (struct pmsm_ttc_pin_ctx_t *)pin_ptr;

  ctx->id_mtpa = 0.0;
  PIN(mtpa)    = 0.0;

  PIN(nc)         = 1.0;
  PIN(ne)         = 1.0;
  PIN(block_gain) = 0.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct pmsm_ttc_ctx_t *ctx      = (struct pmsm_ttc_ctx_t *)ctx_ptr;
  struct pmsm_ttc_pin_ctx_t *pins = (struct pmsm_ttc_pin_ctx_t *)pin_ptr;

  float p      = MAX(PIN(polecount), 1.0);
  float psi_m  = MAX(PIN(psi), 0.01);
  float torque = PIN(torque);
  float pos    = PIN(pos_in);

  // ttc is normally loaded with ac = ae = 0, skip the sinf then
  float tc = PIN(ac) != 0.0 ? PIN(ac) * sinf(PIN(pc) + pos * PIN(nc)) : 0.0;
  float te = PIN(ae) != 0.0 ? torque * PIN(ae) * sinf(PIN(pe) + pos * PIN(ne)) : 0.0;

  PIN(pos_out) = pos * (1.0 - PIN(block_gain)) + (((int)((pos / 2.0 * M_1_PI + 0.5) * 6 + 0.5)) / 6.0 * 2.0 * M_PI - M_PI) * PIN(block_gain);
  PIN(t)       = tc + te;
  float t_cmd  = torque + PIN(g) * (tc + te);

  float id = 0.0;
  float iq = t_cmd / 3.0 * 2.0 / p / psi_m;
  float dl = PIN(lq) > 0.0 ? PIN(lq) - PIN(ld) : 0.0;  // lq - ld, > 0 on an IPM
  if(PIN(mtpa) > 0.0 && dl != 0.0) {
    // T = 1.5 p iq (psi - dl id), and on the MTPA curve
    // id = (psi - sqrt(psi^2 + 4 dl^2 iq^2)) / (2 dl), written so it stays
    // exact as dl goes to 0 and holds for either sign of dl. Two fixed point
    // passes from last tick's id: dl id is a small part of psi, so it
    // converges in a few, and the torque command moves little per tick.
    id = ctx->id_mtpa;
    for(int n = 0; n < 2; n++) {
      iq = t_cmd / (1.5 * p * MAX(psi_m - dl * id, 0.5 * psi_m));
      id = -2.0 * dl * iq * iq / (psi_m + sqrtf(psi_m * psi_m + 4.0 * dl * dl * iq * iq));
    }
  }
  ctx->id_mtpa = id;

  PIN(cur) = iq;
  PIN(id)  = PIN(id_in) + id;
}

hal_comp_t pmsm_ttc_comp_struct = {
    .name      = "pmsm_ttc",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct pmsm_ttc_ctx_t),
    .pin_count = sizeof(struct pmsm_ttc_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
