#include "pmsm_ttc_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

HAL_COMP(pmsm_ttc);

// motor values
HAL_PIN(psi);
HAL_PIN(polecount);

// cogging torque ripple, constant
HAL_PIN(ac);  // amplitude
HAL_PIN(pc);  // phase
HAL_PIN(nc);  // frequency

// electrical torque ripple, linear with current
HAL_PIN(ae);  // amplitude
HAL_PIN(pe);  // phase
HAL_PIN(ne);  // frequency

HAL_PIN(pos_in);
HAL_PIN(pos_out);
HAL_PIN(t);           // compensation torque
HAL_PIN(g);           // compensation gain
HAL_PIN(block_gain);  // block commutation gain


// torque cmd in
HAL_PIN(torque);

// cur cmd out
HAL_PIN(cur);  // q current

// MTPA: with ld != lq the reluctance torque 1.5 p (ld - lq) id iq adds to the
// magnet's, so a torque costs the least current at an id of its own. mtpa 1
// splits the torque into id and iq on that curve, 0 (default) keeps id 0 and
// iq = torque / (1.5 p psi). id = id_in + the MTPA id; a later field weakening
// id goes on the same path.
HAL_PIN(mtpa);
HAL_PIN(ld);     // conf0.l
HAL_PIN(lq);     // conf0.lq, 0 = same as ld (no reluctance torque, id 0)
HAL_PIN(id_in);  // fb_switch0.id, the phasing current
HAL_PIN(id);     // d current, to hv0.d_cmd

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
