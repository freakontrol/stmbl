// The motor, inverter and load the firmware drives.
//
// Electrical: a PMSM in its own dq frame (r, ld, lq, psi, pp). The inverter
// turns the F3's compares into phase voltages averaged over each PWM period
// (switching ripple is not modelled), less the dead-time loss: per phase
// udc * dt_eff / T_pwm * tanh(i / dt_i0), the sign of the phase current with
// the ripple's softening at zero, plus an optional device drop. With the
// gates off (MOE clear) the phases freewheel through the diodes, or float
// once the currents have died and the back emf is below the link.
//
// Mechanical: motor inertia jm and load inertia jl, joined by a spring k and
// damper c (k = 0: one rigid body jm + jl). Coulomb f and viscous d act on
// the load, fm and dm on the motor; load is an external torque on the load.
//
// The firmware's electrical angle is sigma * pp * (encoder + offset), with
// sigma = +-1 from out_rev and mot_fb_rev and the offset from
// conf0.mot_fb_offset (runner sets both with plant_frame), so a correctly
// configured drive is correctly phased; off_err [rad el] mis-phases it.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim_hw.h"
#include "f3hw.h"
#include "runner.h"

sim_f3_adc_t sim_f3_adc;
sim_tim_t sim_tim8;

typedef struct {
  const char *key;
  double v;
  int set;
  const char *doc;
} param_t;

static param_t P[] = {
    {"r", 0.75, 0, "phase resistance [ohm]"},
    {"ld", 0.0015, 0, "d inductance [H]"},
    {"lq", 0.0015, 0, "q inductance [H]"},
    {"psi", 0.05, 0, "flux linkage [Vs]"},
    {"pp", 4, 0, "pole pairs"},
    {"udc", 295, 0, "link voltage [V]"},
    {"udc_r", 0, 0, "link source resistance [ohm]"},
    {"dt_eff", 2.0e-6 * 0.94, 0, "effective dead time [s]"},
    {"dt_i0", 0.3, 0, "phase current where the dead-time loss is 76% of full [A]"},
    {"dt_shape", 0, 0, "dead-time loss over current: 0 tanh(i/dt_i0), 1 hv.c's curve 1-1/(1+|i|/dt_i0)^2"},
    {"vdrop", 0, 0, "device drop [V]"},
    {"adc_lsb", 0.0176, 0, "current ADC step [A] (3 mOhm shunt, gain 16)"},
    {"adc_noise", 0, 0, "current ADC noise per sample [A rms]"},
    {"adc_off_u", 0, 0, "current offset u [A] (the F3 learns it at boot)"},
    {"adc_off_v", 0, 0, "current offset v [A]"},
    {"adc_off_w", 0, 0, "current offset w [A]"},
    {"oc_dac0", 144, 0, "comparator dac word at 0 A"},
    {"oc_k", 6.53, 0, "comparator dac words per A, 0 = no comparator"},
    {"jm", 0.000025, 0, "motor inertia [kg m2]"},
    {"jl", 0, 0, "load inertia [kg m2]"},
    {"k", 0, 0, "coupling stiffness [Nm/rad], 0 = rigid"},
    {"c", 0, 0, "coupling damping [Nm s/rad]"},
    {"ring_hz", 0, 0, "sets k for this two-mass ring frequency [Hz]"},
    {"ring_zeta", 0.2, 0, "sets c for this ring damping, with ring_hz"},
    {"f", 0, 0, "coulomb friction on the load [Nm]"},
    {"d", 0, 0, "viscous friction on the load [Nm s/rad]"},
    {"fm", 0, 0, "coulomb friction on the motor [Nm]"},
    {"dm", 0, 0, "viscous friction on the motor [Nm s/rad]"},
    {"w_s", 0.01, 0, "below this speed coulomb friction sticks [rad/s]"},
    {"load", 0, 0, "external torque on the load [Nm]"},
    {"lead", 10, 0, "screw lead for pos_mm [mm/rev]"},
    {"off_err", 0, 0, "commutation error [rad el]"},
    {"enc_delay", 0, 0, "encoder sample age at the F4 rt [s]"},
    {"enc_bits", 22, 0, "encoder bits per turn"},
    {"hv_temp", 40, 0, "IPM case temperature [C]"},
    {"substeps", 16, 0, "integration steps per PWM period"},
    {"motor", 0, 0, "0 = PMSM, 1 = induction motor (ACIM)"},
    {"lmr", 0.0124, 0, "ACIM: rotor side magnetizing inductance Lm^2/Lr at i_n [H]"},
    {"tr", 0.09, 0, "ACIM: rotor time constant at i_n [s]"},
    {"i_n", 19.5, 0, "ACIM: magnetizing current where lmr and tr hold [A]"},
    {"i_knee", 0, 0, "ACIM: saturation stops growing below this [A]"},
    {"lmr_sat", 0, 0, "ACIM: lmr growth at zero flux, as acim_flux0"},
    {"tr_sat", 0, 0, "ACIM: tr growth at zero flux"},
    {"i_dip", 0, 0, "ACIM: low induction dip below this [A]"},
    {"lmr_dip", 0, 0, "ACIM: fraction lmr loses at zero flux"},
    {"tr_dip", 0, 0, "ACIM: fraction tr loses at zero flux"},
};
#define NP (sizeof(P) / sizeof(P[0]))

static double *pp_(const char *k) {
  for(unsigned i = 0; i < NP; i++) {
    if(!strcmp(P[i].key, k)) {
      return &P[i].v;
    }
  }
  fprintf(stderr, "plant: no parameter %s\n", k);
  exit(2);
}
// each use looks its parameter up once
#define PV(k) (*({              \
  static double *p_;            \
  if(!p_) p_ = pp_(#k);         \
  p_;                           \
}))

double plant_enc_delay;
double plant_enc_bits;

// state
static double id, iq;      // plant dq frame [A]
static double thm, wm;     // motor [rad, rad/s], multi-turn
static double thl, wl;     // load
static double tm, ts;      // motor torque, shaft torque [Nm]
static double vu, vv, vw;  // phase voltages to ground, last step [V]
static double ud, uq;      // applied dq voltage [V]
static double udc_now;
static double sigma = 1.0, mech_off = 0.0;
static uint32_t cu, cv, cw, carr = PWM_RES;  // compares in force
static uint32_t nu, nv, nw, narr = PWM_RES;  // compares for the next period
static double t_plant;
static double psa, psb;    // ACIM rotor flux, stationary frame [Vs]
static double im_now, lm_now, tr_now;

#define HIST 4096
static double hist_t[HIST], hist_th[HIST];
static int hist_n;

void plant_init(void) {
  id = iq = thm = wm = thl = wl = tm = ts = 0.0;
  psa = psb = im_now = 0.0;
  vu = vv = vw = ud = uq = 0.0;
  cu = cv = cw = nu = nv = nw = 0;
  carr = narr = PWM_RES;
  hist_n      = 0;
  t_plant     = 0.0;
  memset(&sim_tim8, 0, sizeof(sim_tim8));
  sim_tim8.ARR = PWM_RES;
}

int plant_set(const char *key, double v) {
  for(unsigned i = 0; i < NP; i++) {
    if(!strcmp(P[i].key, key)) {
      P[i].v   = v;
      P[i].set = 1;
      return 1;
    }
  }
  if(!strcmp(key, "pos")) {  // place the shaft
    thm = thl = v;
    return 1;
  }
  return 0;
}

int plant_was_set(const char *key) {
  for(unsigned i = 0; i < NP; i++) {
    if(!strcmp(P[i].key, key)) {
      return P[i].set;
    }
  }
  return 0;
}

void plant_print_params(void) {
  for(unsigned i = 0; i < NP; i++) {
    printf("  plant.%-10s %-12g %s%s\n", P[i].key, P[i].v, P[i].doc, P[i].set ? "" : " (default)");
  }
}

void plant_frame(double s, double off) {
  sigma    = s;
  mech_off = off;
}

void plant_pwm(uint32_t u, uint32_t v, uint32_t w, uint32_t arr) {
  nu   = u;
  nv   = v;
  nw   = w;
  narr = arr;
}

static double th_el(void) {
  return sigma * PV(pp) * (thm + mech_off) + PV(off_err);
}

// The frame id/iq and ud/uq live in: the rotor for a PMSM, the stator
// (alpha/beta) for an induction motor
static int acim(void) {
  return PV(motor) > 0.5;
}
static double th_frame(void) {
  return acim() ? 0.0 : th_el();
}

// ACIM saturation, the shape acim_flux uses, as a static curve:
// psi = lm(im) * im, rotor resistance lm / tr. Returns lm, sets *tr.
static double acim_lm(double im, double *tr) {
  double i_n = PV(i_n), i_k = fmin(fmax(PV(i_knee), 0.0), 0.9 * i_n);
  double x = i_n > 0.0 ? fmin(fmax((i_n - fabs(im)) / (i_n - i_k), 0.0), 1.0) : 0.0;
  double y = PV(i_dip) > 0.0 ? fmin(fmax((PV(i_dip) - fabs(im)) / PV(i_dip), 0.0), 1.0) : 0.0;
  double kd = fmax(1.0 - PV(lmr_dip) * y, 0.1);
  *tr = PV(tr) * fmax(1.0 + PV(tr_sat) * x, 0.1) * fmax(1.0 - PV(tr_dip) * y, 0.1);
  return PV(lmr) * fmax(1.0 + PV(lmr_sat) * x, 0.1) * kd;
}
// magnetizing current behind a rotor flux magnitude (psi grows with im)
static double acim_im(double psi, double *lm, double *tr) {
  // fixed point from the last step's answer: lm moves little per step
  static double last = 1.0;
  double im = last;
  for(int k = 0; k < 6; k++) {
    im = psi / acim_lm(im, tr);
  }
  *lm = acim_lm(im, tr);
  if(fabs(*lm * im - psi) <= 1e-7 * fmax(psi, 1e-6)) {
    last = im;
    return im;
  }
  double lo = 0.0, hi = 1.0;
  while(acim_lm(hi, tr) * hi < psi && hi < 1e4) hi *= 2.0;
  for(int k = 0; k < 40; k++) {
    double m = 0.5 * (lo + hi);
    if(acim_lm(m, tr) * m < psi) lo = m; else hi = m;
  }
  im   = 0.5 * (lo + hi);
  *lm  = acim_lm(im, tr);
  last = im;
  return im;
}

static void abc(double th, double *iu, double *iv, double *iw) {
  double s = sin(th), c = cos(th);
  double ia = id * c - iq * s;
  double ib = id * s + iq * c;
  *iu       = ia;
  *iv       = -0.5 * ia + 0.5 * sqrt(3.0) * ib;
  *iw       = -0.5 * ia - 0.5 * sqrt(3.0) * ib;
}

static double gauss(void) {
  double a = (rand() + 1.0) / (RAND_MAX + 2.0), b = (rand() + 1.0) / (RAND_MAX + 2.0);
  return sqrt(-2.0 * log(a)) * cos(2.0 * M_PI * b);
}

// one phase leg's average voltage to ground over a PWM period
static double leg(uint32_t cmp, uint32_t arr, double i, double udc) {
  double i0   = fmax(PV(dt_i0), 1e-6);
  double soft = tanh(i / i0);
  if(PV(dt_shape) > 0.5) {
    double q = 1.0 + fabs(i) / i0;
    soft     = (i > 0.0 ? 1.0 : -1.0) * (1.0 - 1.0 / (q * q));
  }
  double v;
  if(cmp == 0) {
    v = 0.0;
  } else if(cmp >= arr) {
    v = udc;
  } else {
    double t_pwm = 2.0 * arr / PWM_TIM_CLK;
    v            = udc * (double)cmp / arr - udc * PV(dt_eff) / t_pwm * soft;
  }
  return v - PV(vdrop) * soft;
}

// Coulomb friction f on a body at speed v with the other torques tn on it.
// Moving (|v| > w_s): f against v. Inside w_s it is stiction (Karnopp): it
// cancels tn up to f and takes out the remaining speed within one step h, so
// a body held by friction stays put instead of creeping.
static double coulomb(double f, double v, double tn, double j, double h) {
  if(f <= 0.0) {
    return 0.0;
  }
  if(fabs(v) > PV(w_s)) {
    return v > 0.0 ? f : -f;
  }
  double need = tn + j * v / h;
  return need > f ? f : need < -f ? -f : need;
}

static void deriv(const double *x, double *dx, double h) {
  // x: id iq thm wm thl wl [psa psb], in the plant's dq frame
  double _id = x[0], _iq = x[1];
  double r = PV(r), ld = PV(ld), lq = PV(lq), psi = PV(psi), pp = PV(pp);
  double we = sigma * pp * x[3];
  double te;
  if(acim()) {
    // inverse-gamma model in the stator frame: ld is the leakage sigma*Ls
    double pa = x[6], pb = x[7], ps = hypot(pa, pb), lm, tr;
    double im = acim_im(ps, &lm, &tr);
    double rr = lm / tr;
    double ima = ps > 1e-9 ? pa / ps * im : 0.0, imb = ps > 1e-9 ? pb / ps * im : 0.0;
    double dpa = rr * (_id - ima) - we * pb;
    double dpb = rr * (_iq - imb) + we * pa;
    if(x[8] > 0.5) {  // phases open: no stator current
      dx[0] = dx[1] = 0.0;
    } else {
      dx[0] = (ud - r * _id - dpa) / ld;
      dx[1] = (uq - r * _iq - dpb) / ld;
    }
    dx[6]  = dpa;
    dx[7]  = dpb;
    te     = 1.5 * pp * (pa * _iq - pb * _id);
    im_now = im;
    lm_now = lm;
    tr_now = tr;
  } else {
    dx[0] = (ud - r * _id + we * lq * _iq) / ld;
    dx[1] = (uq - r * _iq - we * (ld * _id + psi)) / lq;
    te    = 1.5 * pp * (psi * _iq + (ld - lq) * _id * _iq);
    dx[6] = dx[7] = 0.0;
  }
  double k = PV(k), c = PV(c), jm = PV(jm), jl = PV(jl);
  double tq = sigma * te;
  if(k > 0.0 && jl > 0.0) {
    double sh = k * (x[2] - x[4]) + c * (x[3] - x[5]);
    double tm_n = tq - sh - PV(dm) * x[3];
    double tl_n = sh + PV(load) - PV(d) * x[5];
    dx[2]     = x[3];
    dx[3]     = (tm_n - coulomb(PV(fm), x[3], tm_n, jm, h)) / jm;
    dx[4]     = x[5];
    dx[5]     = (tl_n - coulomb(PV(f), x[5], tl_n, jl, h)) / jl;
    ts        = sh;
  } else {
    double j  = jm + jl;
    double tn = tq + PV(load) - (PV(dm) + PV(d)) * x[3];
    dx[2]     = x[3];
    dx[3]     = (tn - coulomb(PV(fm) + PV(f), x[3], tn, j, h)) / j;
    dx[4]     = dx[2];
    dx[5]     = dx[3];
    ts        = tq - jm * dx[3];
  }
  tm = tq;
}

static void step(double dt) {
  double th = th_frame();
  int open  = 0;
  double we = sigma * PV(pp) * wm;
  double iu, iv, iw;
  abc(th, &iu, &iv, &iw);

  // link sag from the drive's own draw
  double udc = PV(udc);
  double p   = vu * iu + vv * iv + vw * iw;
  if(PV(udc_r) > 0.0 && udc > 1.0) {
    udc -= PV(udc_r) * p / udc;
  }
  udc_now = udc;

  // comparators: instantaneous, on any phase
  if(sim_tim8.moe && PV(oc_k) > 0.0) {
    double thr  = (sim_tim8.dac - PV(oc_dac0)) / PV(oc_k);
    uint32_t cm = (fabs(iu) > thr) | (fabs(iv) > thr) << 1 | (fabs(iw) > thr) << 2;
    sim_tim8.cmp = cm;
    if(cm) {
      sim_tim8.moe = 0;
      sim_tim8.brk = 1;
    }
  }

  double s = sin(th), c = cos(th);
  if(sim_tim8.moe) {
    vu = leg(cu, carr, iu, udc);
    vv = leg(cv, carr, iv, udc);
    vw = leg(cw, carr, iw, udc);
  } else {
    // back emf per phase, to see whether the diodes conduct
    double ea = -we * PV(psi) * s, eb = we * PV(psi) * c;
    if(acim()) {  // rotating rotor flux
      ea = -we * psb;
      eb = we * psa;
    }
    double eu = ea, ev = -0.5 * ea + 0.5 * sqrt(3.0) * eb, ew = -0.5 * ea - 0.5 * sqrt(3.0) * eb;
    double ell = fmax(fabs(eu - ev), fmax(fabs(ev - ew), fabs(ew - eu)));
    if(fabs(iu) < 0.01 && fabs(iv) < 0.01 && fabs(iw) < 0.01 && ell < udc) {
      id = iq = 0.0;  // open: the phases float on the back emf
      vu      = eu + udc / 2.0;
      vv      = ev + udc / 2.0;
      vw      = ew + udc / 2.0;
      ud = uq = 0.0;
      open    = 1;
      goto mech;
    }
    vu = iu > 0.0 ? 0.0 : udc;
    vv = iv > 0.0 ? 0.0 : udc;
    vw = iw > 0.0 ? 0.0 : udc;
  }
  {
    double va = (2.0 / 3.0) * (vu - 0.5 * (vv + vw));
    double vb = (vv - vw) / sqrt(3.0);
    ud        = va * c + vb * s;
    uq        = -va * s + vb * c;
  }
mech:;
  // RK2 (midpoint), voltages held
  double x[9] = {id, iq, thm, wm, thl, wl, psa, psb, open}, k1[9], k2[9], xm[9];
  deriv(x, k1, dt);
  xm[8] = open;
  for(int i = 0; i < 8; i++) {
    xm[i] = x[i] + 0.5 * dt * k1[i];
  }
  deriv(xm, k2, dt);
  if(sim_tim8.moe || ud != 0.0 || uq != 0.0 || id != 0.0 || iq != 0.0) {
    id += dt * k2[0];
    iq += dt * k2[1];
  }
  thm += dt * k2[2];
  wm += dt * k2[3];
  thl += dt * k2[4];
  wl += dt * k2[5];
  psa += dt * k2[6];
  psb += dt * k2[7];
  // a rigid pair moves as one
  if(!(PV(k) > 0.0 && PV(jl) > 0.0)) {
    thl = thm;
    wl  = wm;
  }
}

static void apply_ring(void) {
  // two-mass resonance f = sqrt(k (jm + jl) / (jm jl)) / 2 pi
  if(PV(ring_hz) > 0.0 && PV(jl) > 0.0) {
    double jr = PV(jm) * PV(jl) / (PV(jm) + PV(jl));
    double w  = 2.0 * M_PI * PV(ring_hz);
    PV(k)     = w * w * jr;
    PV(c)     = 2.0 * PV(ring_zeta) * w * jr;
  }
}

void plant_advance(double dt) {
  if(dt <= 0.0) {
    return;
  }
  apply_ring();
  double h = 1.0 / PWM_FREQ / fmax(PV(substeps), 1.0);
  int n    = (int)ceil(dt / h - 1e-9);
  double d = dt / n;
  for(int i = 0; i < n; i++) {
    step(d);
    t_plant += d;
    int j      = hist_n++ % HIST;
    hist_t[j]  = t_plant;
    hist_th[j] = thm;
  }
}

void plant_sample_adc(void) {
  // the compares the F3 wrote last tick take over at this update event
  cu   = nu;
  cv   = nv;
  cw   = nw;
  carr = narr;

  double iu, iv, iw;
  abc(th_frame(), &iu, &iv, &iw);
  double lsb = PV(adc_lsb), nz = PV(adc_noise);
  double in[3]  = {iu + PV(adc_off_u), iv + PV(adc_off_v), iw + PV(adc_off_w)};
  double out[3] = {0, 0, 0};
  for(int p = 0; p < 3; p++) {
    for(int k = 0; k < ADC_CUR_SAMPLES; k++) {  // averaged like io.c's sum
      double x = in[p] + (nz > 0.0 ? nz * gauss() : 0.0);
      out[p] += (lsb > 0.0 ? lsb * floor(x / lsb + 0.5) : x) / ADC_CUR_SAMPLES;
    }
  }
  sim_f3_adc.iu      = out[0];
  sim_f3_adc.iv      = out[1];
  sim_f3_adc.iw      = out[2];
  sim_f3_adc.ur      = vu;
  sim_f3_adc.vr      = vv;
  sim_f3_adc.wr      = vw;
  sim_f3_adc.udc     = udc_now > 0.0 ? udc_now : PV(udc);
  sim_f3_adc.hv_temp = PV(hv_temp);
}

void plant_encoder(double delay, double *mech) {
  if(hist_n == 0 || delay <= 0.0) {
    *mech = thm;
    return;
  }
  double t = t_plant - delay;
  int last = hist_n - 1, first = hist_n > HIST ? hist_n - HIST : 0;
  for(int i = last; i > first; i--) {
    int a = (i - 1) % HIST, b = i % HIST;
    if(hist_t[a] <= t) {
      double f = (t - hist_t[a]) / fmax(hist_t[b] - hist_t[a], 1e-12);
      *mech    = hist_th[a] + f * (hist_th[b] - hist_th[a]);
      return;
    }
  }
  *mech = hist_th[first % HIST];
}

double plant_get(const char *key, int *ok) {
  *ok = 1;
  double iu, iv, iw;
  if(!strcmp(key, "id")) return id;
  if(!strcmp(key, "iq")) return iq;
  if(!strcmp(key, "ud")) return ud;
  if(!strcmp(key, "uq")) return uq;
  if(!strcmp(key, "iu") || !strcmp(key, "iv") || !strcmp(key, "iw")) {
    abc(th_frame(), &iu, &iv, &iw);
    return key[1] == 'u' ? iu : key[1] == 'v' ? iv : iw;
  }
  if(!strcmp(key, "pos")) return thm;
  if(!strcmp(key, "vel")) return wm;
  if(!strcmp(key, "load_pos")) return thl;
  if(!strcmp(key, "load_vel")) return wl;
  if(!strcmp(key, "twist")) return thm - thl;
  if(!strcmp(key, "pos_mm")) return thl / (2.0 * M_PI) * PV(lead);
  if(!strcmp(key, "vel_mm")) return wl / (2.0 * M_PI) * PV(lead);
  if(!strcmp(key, "torque")) return tm;
  if(!strcmp(key, "shaft")) return ts;
  if(!strcmp(key, "udc")) return udc_now;
  if(!strcmp(key, "moe")) return sim_tim8.moe;
  if(!strcmp(key, "brk")) return sim_tim8.brk;
  if(!strcmp(key, "th_el")) return remainder(th_el(), 2.0 * M_PI);
  if(!strcmp(key, "psi_r")) return hypot(psa, psb);
  if(!strcmp(key, "i_mr")) return im_now;
  if(!strcmp(key, "lm")) return lm_now;
  if(!strcmp(key, "tr_act")) return tr_now;
  if(!strcmp(key, "th_flux")) return atan2(psb, psa);
  if(!strcmp(key, "slip")) {  // rotor flux speed minus rotor speed [rad/s el]
    double rr = tr_now > 0.0 ? lm_now / tr_now : 0.0, ps = hypot(psa, psb);
    return ps > 1e-6 ? rr * (psa * iq - psb * id) / (ps * ps) : 0.0;
  }
  for(unsigned i = 0; i < NP; i++) {
    if(!strcmp(P[i].key, key)) {
      return P[i].v;
    }
  }
  *ok = 0;
  return 0.0;
}
