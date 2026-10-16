#include "idacim_comp.h"
#include "hal.h"
#include "string.h"
#include "defines.h"
#include "angle.h"

HAL_COMP(idacim);

HAL_PIN(d_cmd);
HAL_PIN(q_cmd);
HAL_PIN(com_pos);
HAL_PIN(cmd_mode);
HAL_PIN(en);
HAL_PIN(en_out);

HAL_PIN(id_fb);
HAL_PIN(ud_fb);

HAL_PIN(state);
HAL_PIN(timer);

HAL_PIN(r);
HAL_PIN(l);
HAL_PIN(l_ok);    // 1 = l is a measurement, 0 = it is not
HAL_PIN(l_freq_a);  // *parameter*, leakage test, lower injection frequency [Hz], 0 = 0.75 loop_bw / 2 pi
HAL_PIN(l_freq_b);  // *parameter*, leakage test, upper injection frequency [Hz], 0 = 1.5 loop_bw / 2 pi
HAL_PIN(loop_bw);   // the run's current loop bandwidth, conf0.cur_bw [rad/s], 0 = 1000
HAL_PIN(l_ripple);  // *parameter*, injected current, fraction of test_cur
HAL_PIN(l_za);      // |Z| at l_freq_a [ohm]
HAL_PIN(l_zb);      // |Z| at l_freq_b [ohm]
HAL_PIN(l_ia);      // injected current amplitude reached at l_freq_a [A]
HAL_PIN(l_ib);      // injected current amplitude reached at l_freq_b [A]
HAL_PIN(l_res);     // resistance the two frequencies imply, stator plus cage [ohm]
HAL_PIN(l_fa);      // lower injection frequency used [Hz]
HAL_PIN(l_fb);      // upper injection frequency used [Hz]

HAL_PIN(rot_half);    // *parameter*, rotor test, time at each current level [s]
HAL_PIN(rot_cycles);  // *parameter*, rotor test, measured cycles (two edges each)
HAL_PIN(rot_bw);      // *parameter*, rotor test, current loop bandwidth [rad/s]
HAL_PIN(rot_t0);      // *parameter*, rotor test, skip this long after each edge [s]
HAL_PIN(tr);          // rotor time constant Lr/Rr, mean of tr_rise and tr_fall [s]
HAL_PIN(slip_n);      // 1/tr, acim_ttc's slip constant [rad/s electrical]
HAL_PIN(lmr);         // rotor side magnetizing inductance Lm^2/Lr, median over edges [H]
HAL_PIN(ls);          // stator inductance l + lmr [H]
HAL_PIN(rot_n);       // edges that went into tr and lmr
HAL_PIN(rot_dip);     // largest current error at rot_t0, fraction of the step
HAL_PIN(tr_ok);       // 1 = tr, slip_n and lmr are measurements
HAL_PIN(tr_spread);   // (max - min) / median of tr, the worse of the two directions
HAL_PIN(tr_rise);     // median tr of the edges up to test_cur [s]
HAL_PIN(tr_fall);     // median tr of the edges down to test_cur/2 [s]
HAL_PIN(tr_min);      // shortest edge [s]
HAL_PIN(tr_max);      // longest edge [s]

HAL_PIN(lad_top);     // *parameter*, offset ladder, top current [A], 0 = no ladder
HAL_PIN(lad_n);       // *parameter*, offset ladder, rungs (at most 4), rung k steps between top k/n and half of it
HAL_PIN(lad_bot);     // *parameter*, offset ladder, lowest rung's top [A]: rungs evenly from here to lad_top, 0 = top k/n
HAL_PIN(lad_ratio);   // *parameter*, offset ladder, rung's lower current over its top, 0 = 0.5
HAL_PINA(lad_i, 4);   // rung's upper current [A]
HAL_PINA(lad_lm, 4);  // rung's lmr, the slope of rotor flux over the rung [H]
HAL_PINA(lad_tr, 4);  // rung's tr [s]
HAL_PINA(lad_sp, 4);  // rung's tr spread
HAL_PIN(i_min);       // *parameter*, lowest current any test level may use; under it the dead time distorts the voltage [A]
HAL_PIN(knee);        // *parameter*, 1 = run the ladder for i_knee/tr_sat, from 0.4 test_cur to test_cur (unless lad_top is set)
HAL_PIN(lad_auto);    // lad_top the knee wrote, cleared on the next run, 0 = none
HAL_PIN(knee_i);      // acim_flux0.i_knee from the ladder's tr, 0 = not fitted [A]
HAL_PIN(knee_tr_sat); // acim_flux0.tr_sat from the ladder's tr
HAL_PIN(knee_tr);     // tr at id_n from the same fit [s]
HAL_PIN(drop);          // dead time volts per phase at the top dwell [V]
HAL_PIN(r_known);       // *parameter*, measured winding resistance, 0 = fit it
HAL_PIN(fit_di);        // dwell current separation, 0 = r/drop fit did not run
HAL_PIN(r_2p);          // two dwell chord slope, 0 = the fit did not run
HAL_PIN(drop_slope);    // *parameter*, the chord's dead time bias, ohm A per volt
HAL_PIN(r_bias);        // what was subtracted from the chord to get r
HAL_PIN(r_ok);          // this run produced a resistance

HAL_PIN(pp);
HAL_PIN(out_rev);
HAL_PIN(spin);          // *parameter*, 1 = go on to the rotating test after the standstill test (turns the rotor)

// Plate values for the V/f set the standstill test computes; 0 = not given
HAL_PIN(n_volt);        // *parameter*, plate voltage, line to line rms [V]
HAL_PIN(n_freq);        // *parameter*, plate frequency at that voltage [Hz]
HAL_PIN(n_cur);         // *parameter*, plate current, rms [A]
HAL_PIN(n_pp);          // *parameter*, pole pairs (poles / 2)
HAL_PIN(id_n);          // magnetizing current at plate voltage and frequency [A peak]
HAL_PIN(iq_n);          // active current at plate current [A peak]
HAL_PIN(vf_u_n);        // vf0.u_n, phase peak volts at vf_vel_n [V]
HAL_PIN(vf_vel_n);      // vf0.vel_n [rad/s mech]
HAL_PIN(vf_boost);      // vf0.u_boost [V]
HAL_PIN(vf_boost_vel);  // vf0.boost_vel [rad/s mech]
HAL_PIN(vf_slip_n);     // vf0.slip_n, slip at vf_cur_n [rad/s mech]
HAL_PIN(vf_cur_n);      // vf0.cur_n, active current at plate current [A peak]

HAL_PIN(test_cur);
HAL_PIN(test_vel);

HAL_PIN(vel_fb);
HAL_PIN(uq_fb);
HAL_PIN(iq_fb);

// Rotating test (state 4): spins the rotor open loop at plate flux, needs the
// standstill test's r, l, tr, lmr and the plate pins from the same session
HAL_PIN(rot_vel);       // *parameter*, rotating test speed, fraction of the plate speed
HAL_PIN(rot_acc);       // *parameter*, rotating test ramp [rad/s^2 electrical], 0 = plate speed in 5 s
HAL_PIN(sw_n);          // flux sweep points taken
HAL_PINA(sw_i, 8);      // flux sweep: d current [A]
HAL_PINA(sw_psi, 8);    // flux sweep: rotor flux [Vs peak]
HAL_PIN(rot_id_n);      // d current for plate flux at plate voltage and frequency [A]
HAL_PIN(rot_lmr);       // secant lmr at rot_id_n [H]
HAL_PIN(rot_lmr_sat);   // acim_flux0.lmr_sat (and tr_sat) from the sweep
HAL_PIN(rot_i_knee);    // acim_flux0.i_knee from the sweep, 0 = not fitted [A]
HAL_PIN(rot_i_dip);     // acim_flux0.i_dip from the sweep, 0 = no dip [A]
HAL_PIN(rot_lmr_dip);   // acim_flux0.lmr_dip from the sweep
HAL_PIN(rot_j);         // inertia from the ramps (stator voltages), 0 = no encoder [kg m^2]
HAL_PIN(rot_j_s);       // inertia from the ramps (slip, scales with tr) [kg m^2]
HAL_PIN(rot_tf);        // friction torque in the ramps' speed range [Nm]
HAL_PIN(rot_enc);       // 1 = vel_fb followed the field, so the ramps could read slip

HAL_PIN(pwm_volt);
HAL_PIN(dc_volt);

HAL_PIN(cur_bw);

HAL_PIN(tmp0);
HAL_PIN(tmp1);
HAL_PIN(tmp2);
HAL_PIN(tmp3);
HAL_PIN(avg_test_volt);

#define ROT_BLK 8     // rotor test: the settled tail is read as the median of this many blocks
#define ROT_EDGES 32  // rotor test: edges kept per rung, so rot_cycles is at most 16
#define ROT_DEC 8     // rotor test: the fit sums take every this many ticks

// State of the leakage and rotor tests. They integrate over thousands of
// ticks, so none of this can be pins without making the state machine's
// scratch pins mean two different things at once.
#define SW_N 8             // rotating test: flux sweep levels

struct idacim_ctx_t {
  // leakage injection
  uint8_t l_fi;     // 0 = l_freq_a, 1 = l_freq_b
  uint8_t l_up;     // the pair runs at l_freq_a/b times 2^l_up
  uint8_t l_stage;  // 0 settle, 1 size the amplitude, 2 measure
  uint16_t l_block; // sizing blocks done
  uint32_t l_n;     // samples in this block or window
  float l_t;        // time in this stage or block
  float l_th;       // injection phase
  float l_amp;      // injection voltage amplitude
  float v_re, v_im, i_re, i_im;
  float v_dc, i_dc, s_dc, c_dc;  // leakage window: sums for the dc removal
  float pp_raw;     // pole pair test: the ratio before rounding
  uint32_t pp_n;    // pole pair test: samples, and those the rotor turned in
  uint32_t pp_turn;
  // rotor step
  uint16_t r_edge;      // edges seen in this rung, the first comes from another level
  uint16_t r_n;         // edges that produced a fit (nrt)
  uint16_t r_seen;      // edges handed to nrt, fitted or not
  float half_e;         // this edge's length [s]
  float t0_e;           // this edge's rot_t0 [s]
  uint16_t dec;         // ticks since the fit sums were last fed
  uint8_t rung;         // 0 = the test at test_cur, 1.. = ladder rungs
  uint8_t act;          // this edge is being fitted
  uint8_t have_prev;    // the previous edge's settled values are known
  uint8_t have_t0;      // i_t0 captured
  float r_t;            // time since the last commanded edge
  float prev_u, prev_i; // where the previous edge settled
  float c0;             // offset taken out while integrating, ud - R id where the last edge settled
  float i0;             // settled current before the step
  float i_t0;           // id_fb at rot_t0
  float p;              // integral of ud - R id - c0 since the edge
  float q;              // integral of id - i0 since the edge
  float jp;             // integral of p
  float u1, u2, i1, i2; // the two samples before this one, for a median of three
  float s[14];          // fit sums, kept as a polynomial in this edge's own offset
  float blk_u[ROT_BLK], blk_i[ROT_BLK];  // settled tail, sums per block
  uint16_t blk_n[ROT_BLK];
  // rt hands each finished edge and rung to nrt, which does the solving and
  // the medians: in rt they cost the f4 more than its slack (bench, 5 Oct)
  float ps[14];         // the edge's sums
  float pd;             // its offset correction c - c0, from the tail
  float pdip;           // its |current error at rot_t0| / step
  uint8_t pup;          // 1 = it went up to the upper level
  float pdi;            // its step, settled to settled [A]
  float ptail;          // max - min of its tail's block means of ud [V]
  volatile uint8_t pend;   // an edge waits for nrt
  volatile uint8_t rdone;  // a rung waits for nrt
  uint8_t rd_rung;      // which rung
  float rd_hi;          // its upper current
  float rd_t1;          // where its settled tail starts, longest tr it can see
  float rd_per;         // rt period
  // nrt's
  float tr_e[ROT_EDGES], lm_e[ROT_EDGES];  // per edge fits
  uint8_t up_e[ROT_EDGES];                 // 1 = the edge went up to the upper level
  float dip;            // largest |current error at rot_t0| / step
  // rotating test
  float w_e;            // field speed [rad/s electrical]
  float w_t;            // test speed [rad/s electrical]
  float sw_t;           // time at this sweep level or ramp phase
  uint8_t sw_k;         // sweep level, or ramp phase in the inertia test
  uint8_t sw_k0;        // lowest sweep level used: lower ones would repeat i_min
  uint8_t sw_top;       // highest sweep level used: higher ones would repeat imax
  float sw_slip;        // flux sweep: slip, low passed, to stop before the rotor pulls out
  uint8_t sw_pass;      // flux sweep: 0 at the test speed, 1 at half of it, 2 back up
  uint8_t sw_np;        // flux sweep: points taken in pass 0
  float sw_hold;        // flux sweep: time at id_n before a speed change
  uint8_t sw_fi[SW_N];  // flux sweep: level of each point
  float sw_i1[SW_N], sw_p1[SW_N];  // pass 0: magnetizing current [A], rotor flux [Vs]
  float sw_i2[SW_N], sw_p2[SW_N];  // pass 1, 0 = not taken
  float sw_x2[SW_N];    // pass 1: slip x tr
  float sw_du[SW_N];    // q voltage offset the two speeds put on each point [V]
  uint8_t stall;        // the rotor fell behind the field
  uint32_t sw_cnt;      // samples summed
  float s_uq, s_iq, s_id, s_slip;  // sums
  float s_tv;                      // ramp window: air gap power / field speed, summed [Nm / (1.5 pp)]
  float v_a, t_a, v_b, t_b;        // the ramp window's first and last speed and time
  float j_slip[2], j_alpha[2];                      // up and down ramps: mean slip [rad/s el], acceleration [rad/s^2 mech]
  float j_tv[2];                   // up and down ramps: mean air gap torque / (1.5 pp) from the stator voltages
  float sw_x[SW_N];     // flux sweep: slip x tr per level
};

#define SW_MEASURE 0.5     // rotating test: averaging window per level [s]
#define SW_X_MAX 0.35      // flux sweep: largest slip x tr a point may have
#define SW_X_STOP 1.0      // flux sweep: slip x tr where the drag nears pull out
// of the d current at plate flux; the low ones reach into field weakening,
// where acim_flux's knee sits, and stop at i_min
static const float sw_frac[SW_N] = {0.25, 0.35, 0.45, 0.6, 0.75, 0.9, 1.0, 1.15};

#define L_BIAS_SETTLE 1.0  // leakage test: first settle, the dc bias rides the slow rotor pole [s]
#define L_SETTLE 0.2       // leakage test: settle after changing frequency [s]
#define L_BLOCK 0.05       // leakage test: one amplitude sizing block [s]
#define L_BLOCKS 4         // leakage test: sizing blocks per frequency
#define L_MEASURE 0.4      // leakage test: demodulation window per frequency [s]
#define ROT_TAIL 0.75      // rotor test: the settled tail starts here, fraction of rot_half

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idacim_ctx_t * ctx = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;
  PIN(r_known)                  = 0.0;
  PIN(drop_slope)               = 0.0039;
  // The r chord's dead time correction needs every phase above about 2 A in
  // its lower dwell (test_cur/2 on d is -test_cur/4 on v and w), and the
  // rotor test reuses the same two levels.
  PIN(test_cur)                 = 8.0;
  PIN(i_min)                    = 4.0;
  PIN(test_vel)                 = 50.0;
  // 0: a pair either side of the current loop's crossover f_c = loop_bw /
  // 2 pi, at 0.75 and 1.5 f_c (120/240 Hz at cur_bw 1000, 360/720 at 3000).
  // The leakage has no plateau on a cage rotor (spindle, 6 Oct: 0.93 mH at
  // 120/240 Hz, 0.75 at 360/720), so this is the band conf0.l has to
  // describe, not the kHz an LCR meter defaults to.
  PIN(l_freq_a)                 = 0.0;
  PIN(l_freq_b)                 = 0.0;
  PIN(l_ripple)                 = 0.15;
  // tr is 50 to 150 ms on a few kW motor. Each level's settled value is
  // read over its last quarter, so 1.5 s puts that 7.6 tr out even at
  // 150 ms; at 1.0 s the leftover tail reads tr 2% short there (simulated).
  PIN(rot_half)                 = 1.5;
  PIN(rot_cycles)               = 4.0;
  PIN(rot_bw)                   = 1500.0;
  PIN(rot_t0)                   = 0.015;
  PIN(lad_top)                  = 0.0;
  PIN(lad_n)                    = 4.0;
  PIN(rot_vel)                  = 0.4;
  PIN(rot_acc)                  = 0.0;
  PIN(cur_bw)                   = 1.0;
}

static float median(float *v, int n) {  // sorts v
  for(int i = 1; i < n; i++) {
    float x = v[i];
    int j   = i - 1;
    while(j >= 0 && v[j] > x) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = x;
  }
  if(n <= 0) {
    return 0.0;
  }
  return n & 1 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

static float med3(float a, float b, float c) {
  return MAX(MIN(a, b), MIN(MAX(a, b), c));
}

// ctx survives a stop
static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idacim_ctx_t *ctx = (struct idacim_ctx_t *)ctx_ptr;
  memset(ctx, 0, sizeof(struct idacim_ctx_t));
}

// x' G y over the basis (a, bk, t, sg, yk) the fit sums are kept in
static double rot_dot(const double g[5][5], const double *x, const double *y) {
  double r = 0.0;
  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      r += x[i] * g[i][j] * y[j];
    }
  }
  return r;
}

// The rotor test's arithmetic, out of rt. Each edge's fit is solved here in
// double, from the sums rt collected; at the end of a rung the edges are
// reduced to medians.
//
// The edge's offset c is fitted with Lmr and tr, not read off the tail: an
// error e in c grows as e t in lam, and at 16 A one mV of it moved lmr 2% and
// tr with it (bench, 5 Oct; the tail's median carried a few mV). With
// y = yk + d sg and b = bk + d t the model y = Lmr a + tr b is linear in Lmr
// and tr but not in d, so Gauss-Newton from the tail's d, which is close.
// The settled part pins d down through the curvature d t^2/2 it leaves in the
// integral, over the whole edge instead of its last quarter.
static void rot_nrt(struct idacim_ctx_t *ctx, struct idacim_pin_ctx_t *pins) {
  if(ctx->pend) {
    float tr = 0.0, lm = 0.0, dfit = 0.0;
    int ok   = 0;
    if(ctx->r_n < ROT_EDGES) {
      float *s       = ctx->ps;
      double g[5][5] = {
          {s[0], s[1], s[2], s[7], s[6]},
          {s[1], s[3], s[4], s[9], s[8]},
          {s[2], s[4], s[5], s[11], s[10]},
          {s[7], s[9], s[11], s[12], s[13]},
          {s[6], s[8], s[10], s[13], 0.0},  // yk yk is not needed
      };
      // start: Lmr and tr at the tail's d
      double d   = ctx->pd;
      double ea[5] = {1, 0, 0, 0, 0};
      double eb[5] = {0, 1, d, 0, 0};
      double ey[5] = {0, 0, 0, d, 1};
      double aa  = rot_dot(g, ea, ea);
      double ab  = rot_dot(g, ea, eb);
      double bb  = rot_dot(g, eb, eb);
      double ay  = rot_dot(g, ea, ey);
      double by  = rot_dot(g, eb, ey);
      double det = aa * bb - ab * ab;
      ok         = det > 0.0;
      double x0  = ok ? (ay * bb - by * ab) / det : 0.0;
      double x1  = ok ? (by * aa - ay * ab) / det : 0.0;
      for(int it = 0; it < 6 && ok; it++) {
        // residual y - model and the model's derivatives in (Lmr, tr, d)
        double r[5]    = {-x0, -x1, -x1 * d, d, 1};
        double j[3][5] = {
            {1, 0, 0, 0, 0},
            {0, 1, d, 0, 0},
            {0, 0, x1, -1, 0},
        };
        double m[3][3], v[3];
        for(int k = 0; k < 3; k++) {
          v[k] = rot_dot(g, j[k], r);
          for(int l = 0; l < 3; l++) {
            m[k][l] = rot_dot(g, j[k], j[l]);
          }
        }
        double dm = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        if(!(dm > 0.0)) {
          ok = 0;
          break;
        }
        double s0 = (v[0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (v[1] * m[2][2] - m[1][2] * v[2]) + m[0][2] * (v[1] * m[2][1] - m[1][1] * v[2])) / dm;
        double s1 = (m[0][0] * (v[1] * m[2][2] - m[1][2] * v[2]) - v[0] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * v[2] - v[1] * m[2][0])) / dm;
        double s2 = (m[0][0] * (m[1][1] * v[2] - v[1] * m[2][1]) - m[0][1] * (m[1][0] * v[2] - v[1] * m[2][0]) + v[0] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) / dm;
        x0 += s0;
        x1 += s1;
        d += s2;
      }
      if(ok) {
        lm   = (float)x0;
        tr   = (float)x1;
        dfit = (float)d;
        ok = tr > 0.0 && lm > 0.0;
        if(ok) {
          ctx->tr_e[ctx->r_n] = tr;
          ctx->lm_e[ctx->r_n] = lm;
          ctx->up_e[ctx->r_n] = ctx->pup;
          ctx->dip            = MAX(ctx->dip, ctx->pdip);
          ctx->r_n++;
        }
      }
    }
    // one line per edge, to see whether the scatter between edges is noise
    // or follows something: direction, step, offset correction, tail noise
    ctx->r_seen++;
    printf("# edge %f %s tr %f ms lmr %f mH d %f V (tail %f) di %f A dip %f tail %f V%s\n", (float)ctx->r_seen, ctx->pup ? "up" : "dn", tr * 1000.0, lm * 1000.0, dfit, ctx->pd, ctx->pdi, ctx->pdip, ctx->ptail, ok ? "" : " rejected");
    ctx->pend = 0;
  }
  if(ctx->rdone && !ctx->pend) {
    int n    = ctx->r_n;
    float tr = 0.0, lm = 0.0, sp = 0.0, tmin = 0.0, tmax = 0.0, trise = 0.0, tfall = 0.0;
    int ok   = 0;
    if(n >= 2) {
      // Rising and falling edges are taken apart: over a step that
      // reaches into saturation the flux is not linear in i_mr, and the
      // fit then reads one direction long and the other short by about
      // the same amount (simulated, 8 <-> 16 A on a knee at 12 A: 82 and
      // 123 ms on 100). The mean of the two medians cancels that, and the
      // spread is the worse of the two directions' own, so it says how
      // repeatable the edges are, not how saturated the step is.
      float v[ROT_EDGES];
      float med[2] = {0.0, 0.0};
      int cnt[2]   = {0, 0};
      for(int up = 0; up < 2; up++) {
        int m = 0;
        for(int k = 0; k < n; k++) {
          if(ctx->up_e[k] == up) {
            v[m++] = ctx->tr_e[k];
          }
        }
        med[up] = median(v, m);  // sorted now
        cnt[up] = m;
        if(m > 0) {
          tmin = (cnt[0] + cnt[1] == m) ? v[0] : MIN(tmin, v[0]);
          tmax = (cnt[0] + cnt[1] == m) ? v[m - 1] : MAX(tmax, v[m - 1]);
          if(med[up] > 0.0) {
            sp = MAX(sp, (v[m - 1] - v[0]) / med[up]);
          }
        }
      }
      tfall = med[0];
      trise = med[1];
      tr    = cnt[0] && cnt[1] ? 0.5 * (trise + tfall) : (cnt[1] ? trise : tfall);
      // lmr from the falling edges. Rising ones read it 20% under them at
      // 12 and 16 A, and with 3 s edges instead of 1.5 their lmr spread
      // 7.5-11.6 mH and two of four fits failed, while falling ones held
      // 13.2-13.8, the at-speed 13.2 (bench, 5 Oct): something slow at the
      // upper level (heating, or flux creeping near saturation) bends the
      // edges that end there. A falling edge ends at the lower level, where
      // both are smallest. Rising edges only when no falling one fitted.
      int lup = cnt[0] ? 0 : 1;
      int m   = 0;
      for(int k = 0; k < n; k++) {
        if(ctx->up_e[k] == lup) {
          v[m++] = ctx->lm_e[k];
        }
      }
      lm = median(v, m);
      // a fit that lands outside the window it was taken over is not an
      // exponential this test can see
      ok = tr > 5.0 * ctx->rd_per && tr < ctx->rd_t1;
    }
    if(ctx->rd_rung == 0) {
      PIN(rot_n)     = n;
      PIN(rot_dip)   = ctx->dip;
      PIN(tr)        = tr;  // reported either way, so a rejected fit still says what it was
      PIN(tr_rise)   = trise;
      PIN(tr_fall)   = tfall;
      PIN(tr_min)    = tmin;
      PIN(tr_max)    = tmax;
      PIN(tr_spread) = sp;
      PIN(slip_n)    = ok ? 1.0 / tr : 0.0;
      PIN(lmr)       = ok ? lm : 0.0;
      PIN(ls)        = ok ? PIN(l) + lm : 0.0;
      PIN(tr_ok)     = ok ? 1.0 : 0.0;
      PIN(knee_i)      = 0.0;
      PIN(knee_tr_sat) = 0.0;
      PIN(knee_tr)     = 0.0;
      for(int k = 0; k < 4; k++) {
        PINA(lad_i, k)  = 0.0;
        PINA(lad_lm, k) = 0.0;
        PINA(lad_tr, k) = 0.0;
        PINA(lad_sp, k) = 0.0;
      }
    } else {
      PINA(lad_i, ctx->rd_rung - 1)  = ctx->rd_hi;
      PINA(lad_lm, ctx->rd_rung - 1) = ok ? lm : 0.0;
      PINA(lad_tr, ctx->rd_rung - 1) = tr;
      PINA(lad_sp, ctx->rd_rung - 1) = sp;
    }

    ctx->r_n    = 0;
    ctx->r_seen = 0;
    ctx->dip    = 0.0;
    ctx->rdone = 0;
  }
}

// ladder rung k (1..n): its top, and its lower current as a fraction of it
static float lad_hi(struct idacim_pin_ctx_t *pins, int k, int n) {
  if(PIN(lad_bot) > 0.0 && n > 1) {
    return PIN(lad_bot) + (PIN(lad_top) - PIN(lad_bot)) * (float)(k - 1) / (float)(n - 1);
  }
  return PIN(lad_top) * (float)k / (float)MAX(n, 1);
}
static float lad_lo(struct idacim_pin_ctx_t *pins) {
  return PIN(lad_ratio) > 0.0 ? CLAMP(PIN(lad_ratio), 0.3, 0.9) : 0.5;
}

// acim_flux's tr model, tr (1 + tr_sat x) with x = (i_n - i) / (i_n - i_knee)
// clamped 0..1, fitted to the ladder rungs' tr and the main test's, each at
// its range's middle (0.75 of its top). For every knee on a grid a weighted
// straight line in x gives tr at i_n and tr_sat; the knee with the smallest
// residual wins. Rungs are weighted by their spread.
#define KNEE_SP_MAX 0.3   // largest edge spread a level may have to enter the fit
#define KNEE_SAT_MAX 1.0  // largest tr_sat accepted
#define KNEE_TR_DEV 0.3   // tr at id_n within this fraction of the main test's

static void knee_fit(struct idacim_pin_ctx_t *pins) {
  float pi[5], pt[5], pw[5];
  int n    = 0;
  float in = PIN(id_n);
  PIN(knee_i)      = 0.0;
  PIN(knee_tr_sat) = 0.0;
  PIN(knee_tr)     = 0.0;
  for(int k = 0; k < (int)MIN(PIN(lad_n), 4.0); k++) {
    // under i_min the r chord no longer describes the dead time, and the
    // step's flux reads off (simulated: tr +14% at 2.4..4.8 A): such rungs
    // are listed, not fitted
    // a rung whose edges spread over KNEE_SP_MAX says less than the effect
    // looked for (spindle, 6 Oct: 0.32-1.03 on 1.9-4.8 A steps, where the
    // main test's 8 A step held 0.15), so it is listed, not fitted
    float lo = lad_lo(pins) * PINA(lad_i, k);
    if(PINA(lad_lm, k) > 0.0 && PINA(lad_tr, k) > 0.0 && lo >= 0.999 * PIN(i_min) && PINA(lad_sp, k) <= KNEE_SP_MAX) {
      float sp = MAX(PINA(lad_sp, k), 0.05);
      pi[n]    = 0.5 * (1.0 + lad_lo(pins)) * PINA(lad_i, k);
      pt[n]    = PINA(lad_tr, k);
      pw[n++]  = 1.0 / (sp * sp);
    }
  }
  if(PIN(tr_ok) > 0.0) {
    float sp = MAX(PIN(tr_spread), 0.05);
    pi[n]    = 0.75 * PIN(test_cur);
    pt[n]    = PIN(tr);
    pw[n++]  = 1.0 / (sp * sp);
  }
  if(n < 3 || in <= 0.0) {
    printf("<font color='red'># tr knee: needs id_n (plate pins) and three levels with spread under %f (ladder rungs or the main test), have %f: keep the config's i_knee and tr_sat</font>\n", KNEE_SP_MAX, (float)n);
    return;
  }
  float best = -1.0, ba = 0.0, bb = 0.0, bk = 0.0;
  for(int g = 0; g <= 90; g++) {
    float ik = in * 0.01 * (float)g;
    float sw = 0.0, sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for(int j = 0; j < n; j++) {
      float x = CLAMP((in - pi[j]) / (in - ik), 0.0, 1.0);
      sw += pw[j];
      sx += pw[j] * x;
      sy += pw[j] * pt[j];
      sxx += pw[j] * x * x;
      sxy += pw[j] * x * pt[j];
    }
    float det = sw * sxx - sx * sx;
    if(det <= 1e-9 * sw * sw) {  // every point on the same x: no slope to fit
      continue;
    }
    float b = (sw * sxy - sx * sy) / det;
    float a = (sy - b * sx) / sw;
    float e = 0.0;
    for(int j = 0; j < n; j++) {
      float x = CLAMP((in - pi[j]) / (in - ik), 0.0, 1.0);
      float d = pt[j] - a - b * x;
      e += pw[j] * d * d;
    }
    if(a > 0.0 && (best < 0.0 || e < best)) {
      best = e;
      ba   = a;
      bb   = b;
      bk   = ik;
    }
  }
  if(best < 0.0) {
    printf("<font color='red'># tr knee: no fit</font>\n");
    return;
  }
  float sat = bb / ba;
  printf("<font color='green'># tr knee fit over %f levels: id [A], tr [ms], model [ms]\n", (float)n);
  for(int j = 0; j < n; j++) {
    float x = CLAMP((in - pi[j]) / (in - bk), 0.0, 1.0);
    printf("# %f %f %f\n", pi[j], pt[j] * 1000.0, ba * (1.0 + sat * x) * 1000.0);
  }
  printf("</font>");
  // A fit that bends tr by more than its own size, or puts tr at id_n far
  // from the main test's, is extrapolating noise (spindle, 6 Oct: tr_sat 7
  // through one rung, tr 12 ms at id_n against 97 measured), not a knee.
  // tr falling at low flux is not what saturation does either.
  float trm = PIN(tr_ok) > 0.0 ? PIN(tr) : 0.0;
  if(sat < 0.0 || sat > KNEE_SAT_MAX || (trm > 0.0 && (ba < (1.0 - KNEE_TR_DEV) * trm || ba > (1.0 + KNEE_TR_DEV) * trm))) {
    printf("<font color='red'># tr knee: fit rejected (i_knee %f, tr_sat %f, tr at id_n %f ms against %f measured): keep the config's i_knee and tr_sat</font>\n", bk, sat, ba * 1000.0, trm * 1000.0);
    return;
  }
  PIN(knee_i)      = bk;
  PIN(knee_tr_sat) = sat;
  PIN(knee_tr)     = ba;
  printf("acim_flux0.tr = %f <font color='green'># append to config, tr at id_n %f A (replaces the tr above, taken at %f A)</font>\n", ba, in, 0.75 * PIN(test_cur));
  printf("acim_flux0.i_knee = %f <font color='green'># append to config</font>\n", bk);
  printf("acim_flux0.tr_sat = %f <font color='green'># append to config</font>\n", PIN(knee_tr_sat));
}

// The V/f set from the standstill measurements and the plate: what a VFD's
// standstill autotune gives, enough to run a spindle without feedback.
// The plate voltage at the plate frequency sets the flux; at no load the
// stator sees r id_n + j w ls id_n, so that voltage gives id_n with the ls
// this test read. ls is lmr's chord over test_cur/2..test_cur, so id_n is
// best when test_cur is near it. Slip at active current iq is iq / (tr i_mr)
// electrical, and the boost covers r id_n where the emf is still small,
// plus the dead time loss: V/f runs in volt mode with no compensation, and a
// rotating vector loses the square wave's fundamental, 4/pi of the per phase drop
// (spindle, 9 Oct: r id_n alone, 3.7 V, stalled; 12 V started).
static void vf_set(struct idacim_pin_ctx_t *pins) {
  float u  = PIN(n_volt) * 0.8164966;  // line to line rms to phase peak
  float we = 2.0 * M_PI * PIN(n_freq);
  float pp = PIN(n_pp);
  float r  = PIN(r);
  float ls = PIN(ls);
  float in = PIN(n_cur) * 1.4142136;
  float id = u / sqrtf(r * r + we * we * ls * ls);
  float iq = in > id ? sqrtf(in * in - id * id) : 0.0;

  PIN(id_n)         = id;
  PIN(iq_n)         = iq;
  PIN(vf_u_n)       = u;
  PIN(vf_vel_n)     = we / pp;
  PIN(vf_boost)     = r * id + 4.0 / M_PI * PIN(drop);  // square wave loss, fundamental
  PIN(vf_boost_vel) = CLAMP(3.0 * r / ls, 0.02 * we, 0.2 * we) / pp;  // where w ls is 3 r
  PIN(vf_slip_n)    = iq / (PIN(tr) * id) / pp;
  PIN(vf_cur_n)     = iq;

  printf("<font color='green'># V/f from the standstill test and the plate (%f V, %f Hz, %f A, %f pole pairs):</font>\n", PIN(n_volt), PIN(n_freq), PIN(n_cur), pp);
  printf("conf0.polecount = %f <font color='green'># append to config</font>\n", pp);
  printf("vf0.u_n = %f <font color='green'># append to config</font>\n", PIN(vf_u_n));
  printf("vf0.vel_n = %f <font color='green'># append to config</font>\n", PIN(vf_vel_n));
  printf("vf0.u_boost = %f <font color='green'># append to config</font>\n", PIN(vf_boost));
  printf("vf0.boost_vel = %f <font color='green'># append to config</font>\n", PIN(vf_boost_vel));
  printf("vf0.slip_n = %f <font color='green'># append to config</font>\n", PIN(vf_slip_n));
  printf("vf0.cur_n = %f <font color='green'># append to config</font>\n", PIN(vf_cur_n));
  printf("acim_foc0.id_n = %f <font color='green'># FOC start value, the rotating test refines it</font>\n", id);
  printf("<font color='green'># id_n %f A and iq_n %f A peak at the plate current.\n", id, iq);
  if(in <= id) {
    printf("</font><font color='red'># id_n is over the plate current: check n_volt, n_freq and n_cur.</font>\n");
  } else if(id > 1.2 * PIN(test_cur) || id < 0.6 * PIN(test_cur)) {
    printf("# lmr was read at %f..%f A: for an id_n that saturates the iron the\n", PIN(test_cur) * 0.5, PIN(test_cur));
    printf("# way it will run, rerun with idacim0.test_cur near %f.</font>\n", id);
  } else {
    printf("# lmr was read at %f..%f A, close enough to id_n.</font>\n", PIN(test_cur) * 0.5, PIN(test_cur));
  }
}

// The rotating test's results. Flux sweep: at no load the rotor runs at the
// field's speed, so in the field's frame uq = r iq + w psi_s,d and the rotor
// flux is psi_s,d - l id. Its secant over id is acim_flux's lmr; the point
// where the stator flux reaches plate voltage over plate frequency is the
// rated magnetizing current. The uq read is clean of the dead time: that sits
// in phase with the current, on d.
//
// Inertia, with an encoder: on a ramp the rotor lags the field by the slip
// ws, and a current fed cage then gives T = 1.5 pp lmr i^2 ws tr / (1 +
// (ws tr)^2). The same ramp up and down cancels friction:
// J = (T_up - T_down) / (alpha_up - alpha_down), friction = (T_up + T_down) / 2.
static void rot_report(struct idacim_ctx_t *ctx, struct idacim_pin_ctx_t *pins) {
  int n    = (int)PIN(sw_n);
  float pp = PIN(n_pp);
  float wn = 2.0 * M_PI * PIN(n_freq);
  float idr = PIN(id_n) > 0.0 ? PIN(id_n) : PIN(test_cur);
  printf("<font color='green'># flux sweep at %f and %f rad/s mech: magnetizing current [A], rotor flux [Vs], secant lmr [mH]; lmr [mH] and slip x tr at each speed, q voltage offset [V]\n", ctx->w_t / pp, 0.5 * ctx->w_t / pp);
  for(int k = 0; k < n; k++) {
    printf("# %f %f %f; %f %f %f %f %f\n", PINA(sw_i, k), PINA(sw_psi, k), PINA(sw_i, k) > 0.0 ? PINA(sw_psi, k) / PINA(sw_i, k) * 1000.0 : 0.0,
           ctx->sw_i1[k] > 0.0 ? ctx->sw_p1[k] / ctx->sw_i1[k] * 1000.0 : 0.0, ctx->sw_x[k], ctx->sw_i2[k] > 0.0 ? ctx->sw_p2[k] / ctx->sw_i2[k] * 1000.0 : 0.0, ctx->sw_x2[k], ctx->sw_du[k]);
  }
  printf("</font>");
  if(ctx->stall && n == 0) {
    printf("<font color='red'>the rotor did not follow the field up to speed: lower idacim0.rot_acc, or unload the shaft</font>\n");
    return;
  }
  if(n < 2) {
    printf("<font color='red'>flux sweep incomplete</font>\n");
    return;
  }
  // d current at plate flux: stator flux psi_r + l id against u_n / w_n, by
  // linear interpolation (or extension from the nearest two)
  float id_n = idr;
  if(PIN(n_volt) > 0.0) {
    float psn = PIN(n_volt) * 0.8164966 / wn;
    int k     = 1;
    while(k < n - 1 && PINA(sw_psi, k) + PIN(l) * PINA(sw_i, k) < psn) {
      k++;
    }
    float ia = PINA(sw_i, k - 1), ib = PINA(sw_i, k);
    float pa = PINA(sw_psi, k - 1) + PIN(l) * ia, pb = PINA(sw_psi, k) + PIN(l) * ib;
    if(pb > pa) {
      id_n = ia + (psn - pa) * (ib - ia) / (pb - pa);
    }
  }
  // secant lmr there, interpolated the same way
  int k = 1;
  while(k < n - 1 && PINA(sw_i, k) < id_n) {
    k++;
  }
  float ia = PINA(sw_i, k - 1), ib = PINA(sw_i, k);
  float la = PINA(sw_psi, k - 1) / ia, lb = PINA(sw_psi, k) / ib;
  float lmr = ib > ia ? la + (id_n - ia) * (lb - la) / (ib - ia) : lb;
  // acim_flux's shape of the secant lmr under id_n: lmr (1 + lmr_sat x)
  // (1 - lmr_dip y), x = (id_n - i) / (id_n - i_knee) and y = (i_dip - i) /
  // i_dip clamped 0..1. Growth from id_n down to the knee as the iron
  // desaturates, flat, then a fall under i_dip at low induction (spindle, 6
  // Oct: flat 9-19 A, 11% down at 6.7 A). For every i_knee and i_dip <=
  // i_knee on a grid, lmr, lmr_sat and lmr_dip by least squares (a term that
  // comes out negative is dropped and the rest refitted); the smallest
  // residual wins. The points repeat to about 0.5%, against 10-25% for
  // standstill steps at these currents. Rotor resistance does not saturate,
  // so tr = Lr / Rr follows the same curve: tr_sat = lmr_sat, and acim_flux
  // puts the dip on tr itself.
  float sat = 0.0, ik = 0.0, fa = 0.0, dip = 0.0, idp = 0.0;
  int nlow  = 0;
  for(int j = 0; j < n; j++) {
    nlow += PINA(sw_i, j) < 0.95 * id_n;
  }
  if(nlow >= 3 && lmr > 0.0) {
    float best = -1.0;
    // a knee or a dip edge under the lowest point is a plateau nobody saw
    // (simulated: tr 10% off and 3 ms of speed lag put a knee at 0.7 A with
    // tr_sat 0.27 on a 8.5 A, 0.14 motor), so the grids start there
    float i_lo = PINA(sw_i, 0);
    float g0   = CLAMP(i_lo / id_n, 0.0, 0.9);
    for(int g = 0; g <= 30; g++) {
      float kg = id_n * (g0 + (0.9 - g0) * 0.0333333 * (float)g);
      for(int h = 0; h <= 30; h++) {
        float dg = i_lo + (kg - i_lo) * 0.0333333 * (float)h;
        // columns 1, x, -y; a fit that turns a term negative is redone
        // without it: both, growth only, dip only, flat
        for(int vr = 0; vr < 4; vr++) {
          int use_x = vr == 0 || vr == 1, use_y = vr == 0 || vr == 2;
          // one point under the dip's edge fits any depth with some edge:
          // a dip needs two
          int under = 0;
          for(int j = 0; j < n; j++) {
            under += PINA(sw_i, j) < dg;
          }
          if(use_y && under < 2) {
            continue;
          }
          int m     = 1 + use_x + use_y;
          float A[3][3] = {{0.0}}, B[3] = {0.0};
          int cnt = 0;
          for(int j = 0; j < n; j++) {
            // acim_flux holds lmr above i_n, where the iron still
            // saturates: points over id_n would only pull the line off
            // the ones under it
            float ij = PINA(sw_i, j);
            if(ij > 1.02 * id_n) {
              continue;
            }
            float vx   = CLAMP((id_n - ij) / (id_n - kg), 0.0, 1.0);
            float vy   = dg > 0.0 ? -CLAMP((dg - ij) / dg, 0.0, 1.0) : 0.0;
            float v[3] = {1.0, use_x ? vx : vy, vy};
            float yj = PINA(sw_psi, j) / ij;
            for(int r = 0; r < m; r++) {
              for(int c = 0; c < m; c++) {
                A[r][c] += v[r] * v[c];
              }
              B[r] += v[r] * yj;
            }
            cnt++;
          }
          if(cnt < m + 1) {
            continue;
          }
          // Gauss elimination, m <= 3
          int sing = 0;
          for(int c = 0; c < m && !sing; c++) {
            if(ABS(A[c][c]) < 1e-9 * (A[0][0] + 1e-9)) {
              sing = 1;
              break;
            }
            for(int r = c + 1; r < m; r++) {
              float f = A[r][c] / A[c][c];
              for(int k = c; k < m; k++) {
                A[r][k] -= f * A[c][k];
              }
              B[r] -= f * B[c];
            }
          }
          if(sing) {
            continue;
          }
          float X[3] = {0.0, 0.0, 0.0};
          for(int r = m - 1; r >= 0; r--) {
            float t = B[r];
            for(int k = r + 1; k < m; k++) {
              t -= A[r][k] * X[k];
            }
            X[r] = t / A[r][r];
          }
          float a0 = X[0];
          float s0 = use_x ? X[1] : 0.0;
          float d0 = use_y ? X[m - 1] : 0.0;
          if(a0 <= 0.0 || s0 < 0.0 || d0 < 0.0) {
            continue;  // try with fewer terms
          }
          float e = 0.0;
          for(int j = 0; j < n; j++) {
            float ij = PINA(sw_i, j);
            if(ij > 1.02 * id_n) {
              continue;
            }
            float x = CLAMP((id_n - ij) / (id_n - kg), 0.0, 1.0);
            float y = dg > 0.0 ? CLAMP((dg - ij) / dg, 0.0, 1.0) : 0.0;
            float d = PINA(sw_psi, j) / ij - (a0 + s0 * x - d0 * y);
            e += d * d;
          }
          if(best < 0.0 || e < best * 0.999) {
            best = e;
            fa   = a0;
            sat  = s0 / a0;
            ik   = kg;
            dip  = d0 / a0;
            idp  = dg;
          }
          break;
        }
      }
    }
  }
  if(sat < 0.005) {  // no growth: no knee to set
    sat = 0.0;
    ik  = 0.0;
  }
  if(dip < 0.005) {
    dip = 0.0;
    idp = 0.0;
  }
  // terms past double or a dip near total, or an lmr at id_n off the
  // interpolated one, are noise or a rotor that slipped
  int knee_ok = PIN(rot_enc) > 0.0 && fa > 0.0 && sat <= KNEE_SAT_MAX && dip <= 0.5 && ABS(fa - lmr) < 0.05 * lmr;
  float sat_f = sat, dip_f = dip, fa_f = fa;
  if(!knee_ok) {
    // fallback: a straight line through the lowest point at or over 0.4
    // id_n (lower ones sit in the dip), no knee
    int j0 = 0;
    while(j0 < n - 1 && PINA(sw_i, j0) < 0.4 * id_n) {
      j0++;
    }
    float i0 = PINA(sw_i, j0);
    sat      = (i0 < id_n && lmr > 0.0) ? (PINA(sw_psi, j0) / i0 / lmr - 1.0) / (1.0 - i0 / id_n) : 0.0;
    ik       = 0.0;
    dip      = 0.0;
    idp      = 0.0;
  }
  // without an encoder a pulled out rotor shows as flux far under what the
  // standstill test read: slip shorts it
  if(lmr < 0.7 * PIN(lmr)) {
    printf("<font color='red'># lmr %f mH is far under the standstill %f mH: the rotor may not have followed the field (load, or rot_acc too high)</font>\n", lmr * 1000.0, PIN(lmr) * 1000.0);
  }
  if(PIN(rot_enc) <= 0.0) {
    printf("<font color='green'># no encoder: the sweep is not corrected for slip, drag reads as saturation at the low points (lmr_sat low)</font>\n");
  }
  PIN(rot_id_n)    = id_n;
  PIN(rot_lmr)     = lmr;
  PIN(rot_lmr_sat) = sat;
  PIN(rot_i_knee)  = ik;
  PIN(rot_i_dip)   = idp;
  PIN(rot_lmr_dip) = dip;
  printf("acim_foc0.id_n = %f <font color='green'># append to config</font>\n", id_n);
  printf("acim_flux0.i_n = %f <font color='green'># append to config</font>\n", id_n);
  printf("acim_flux0.lmr = %f <font color='green'># append to config, secant at id_n</font>\n", lmr);
  if(knee_ok || PIN(rot_enc) <= 0.0) {
    printf("acim_flux0.lmr_sat = %f <font color='green'># append to config</font>\n", MAX(sat, 0.0));
  } else {
    // the fallback line runs through the dip and does not fit the config's
    // knee and dip, which stay
    printf("<font color='green'># acim_flux0.lmr_sat %f from the fallback line, not to append</font>\n", sat);
  }
  if(knee_ok) {
    printf("acim_flux0.i_knee = %f <font color='green'># append to config, lmr and tr flat below it</font>\n", ik);
    printf("acim_flux0.tr_sat = %f <font color='green'># append to config, tr follows the secant lmr</font>\n", sat);
    printf("acim_flux0.i_dip = %f <font color='green'># append to config, lmr and tr fall below it</font>\n", idp);
    printf("acim_flux0.lmr_dip = %f <font color='green'># append to config</font>\n", dip);
    printf("<font color='green'># fitted lmr at id_n %f mH; model against the sweep:", fa * 1000.0);
    for(int j = 0; j < n; j++) {
      float ij = PINA(sw_i, j);
      float x  = ik > 0.0 || sat > 0.0 ? CLAMP((id_n - ij) / (id_n - ik), 0.0, 1.0) : 0.0;
      float y  = idp > 0.0 ? CLAMP((idp - ij) / idp, 0.0, 1.0) : 0.0;
      printf(" %f %f/%f", ij, PINA(sw_psi, j) / ij * 1000.0, ij > 1.02 * id_n ? fa * 1000.0 : fa * (1.0 + sat * x) * (1.0 - dip * y) * 1000.0);
    }
    printf("</font>\n");
  } else if(PIN(rot_enc) > 0.0 && nlow < 3) {
    printf("<font color='red'># saturation knee: %f sweep points under id_n with slip x tr up to %f, 3 needed (the drag stopped the sweep); keep the config's i_knee, tr_sat, i_dip and lmr_dip</font>\n", (float)nlow, SW_X_MAX);
  } else if(PIN(rot_enc) > 0.0) {
    printf("<font color='red'># saturation knee: fit rejected (lmr_sat %f, lmr_dip %f, lmr at id_n %f mH against %f), lmr_sat from the lowest point over 0.4 id_n; keep the config's i_knee, tr_sat, i_dip and lmr_dip</font>\n", sat_f, dip_f, fa_f * 1000.0, lmr * 1000.0);
  } else {
    printf("<font color='green'># no encoder: no saturation knee, keep the config's i_knee and tr_sat</font>\n");
  }
  if(PIN(n_volt) <= 0.0) {
    printf("<font color='green'># no idacim0.n_volt: id_n is the current the sweep was centred on</font>\n");
  } else if(id_n > PINA(sw_i, n - 1) || id_n < PINA(sw_i, 0)) {
    printf("<font color='red'># id_n is outside the sweep: extended, rerun with id_n there</font>\n");
  }

  PIN(rot_j)   = 0.0;
  PIN(rot_j_s) = 0.0;
  PIN(rot_tf) = 0.0;
  if(PIN(rot_enc) <= 0.0) {
    printf("<font color='green'># no encoder (vel_fb did not follow the field): no inertia</font>\n");
  } else if(ctx->stall) {
    printf("<font color='red'># the rotor fell behind the field on a ramp: lower idacim0.rot_acc for inertia</font>\n");
  } else {
    // the ramps run at idr: lmr there, from the sweep
    int m = 1;
    while(m < n - 1 && PINA(sw_i, m) < idr) {
      m++;
    }
    float ja = PINA(sw_i, m - 1), jb = PINA(sw_i, m);
    float lma = PINA(sw_psi, m - 1) / ja, lmb = PINA(sw_psi, m) / jb;
    float lmj = jb > ja ? lma + (idr - ja) * (lmb - lma) / (jb - ja) : lmb;
    float tr  = PIN(tr);
    float t[2];
    for(int u = 0; u < 2; u++) {
      float ws = ctx->j_slip[u];
      t[u]     = 1.5 * pp * lmj * idr * idr * ws * tr / (1.0 + ws * ws * tr * tr);
    }
    float w_hi = wn * MIN(2.0 * PIN(rot_vel), 0.9);
    float da = ctx->j_alpha[0] - ctx->j_alpha[1];
    if(da > 0.0) {
      // J from the stator voltages (air gap power, no tr). The slip would
      // give J too, but any lag of the speed feedback behind the field
      // adds slip in proportion to the acceleration, a fixed offset in J
      // (the spindle read +0.007): the slip only gives friction here,
      // where the up and down ramps' lag cancels, and the lag itself.
      float tv0 = 1.5 * pp * ctx->j_tv[0], tv1 = 1.5 * pp * ctx->j_tv[1];
      float j_s = (t[0] - t[1]) / da;
      float j_v = (tv0 - tv1) / da;
      PIN(rot_j)   = j_v;
      PIN(rot_tf)  = 0.5 * (t[0] + t[1]);
      PIN(rot_j_s) = j_s;
      printf("conf0.j = %f <font color='green'># append to config, kg m^2</font>\n", j_v);
      printf("<font color='green'># friction %f Nm at %f-%f rad/s mech; ramps %f / %f rad/s^2 mech, slip %f / %f rad/s el</font>\n", PIN(rot_tf), (ctx->w_t + 0.25 * (w_hi - ctx->w_t)) / pp, (ctx->w_t + 0.75 * (w_hi - ctx->w_t)) / pp, ctx->j_alpha[0], ctx->j_alpha[1], ctx->j_slip[0], ctx->j_slip[1]);
      // slip per torque near zero slip: 1.5 pp lmr i^2 tr
      float kt = 1.5 * pp * lmj * idr * idr * tr;
      if(kt > 0.0) {
        printf("<font color='green'># speed feedback lags the field by ~%f ms (slip J %f)</font>\n", (j_s - j_v) / (kt * pp) * 1000.0, j_s);
      }
      if(j_v <= 0.0) {
        printf("<font color='red'># no inertia: the air gap torque did not rise with acceleration; check the ud_fb/uq_fb links, rerun</font>\n");
      }
    }
  }
  printf("rotating test done\n");
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idacim_ctx_t *ctx       = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;

  rot_nrt(ctx, pins);

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      // hv0.r and l go back to safe defaults: nothing after this may use
      // them as measured
      PIN(r)       = 0.1;
      PIN(l)       = 0.001;
      PIN(drop)    = 0.0;
      PIN(r_ok)    = 0.0;
      PIN(l_ok)    = 0.0;
      PIN(tr_ok)   = 0.0;
      PIN(out_rev) = 0.0;
      PIN(cur_bw)  = 1.0;
      break;

    case 10:  // r, l
      // test_cur is the highest current of the standstill test, as in
      // idpmsm. The knee ladder runs under it, from 0.4 test_cur, in 0.7
      // steps; no level goes under i_min, where the dead time distorts the
      // voltage (the rotor test's lower level is half of test_cur, a rung's
      // is lad_ratio of its top)
      if(PIN(test_cur) < 2.0 * PIN(i_min)) {
        printf("<font color='red'># idacim0.test_cur %f A puts the rotor test's lower step under idacim0.i_min %f A</font>\n", PIN(test_cur), PIN(i_min));
      }
      // a ladder the knee wrote is redone from this run's test_cur, or
      // dropped with knee 0; one set by hand (lad_top changed) stays
      if(PIN(lad_auto) > 0.0 && PIN(lad_top) == PIN(lad_auto)) {
        PIN(lad_top) = 0.0;
        PIN(lad_bot) = 0.0;
      }
      PIN(lad_auto) = 0.0;
      if(PIN(knee) > 0.0 && PIN(lad_top) <= 0.0) {
        PIN(lad_ratio) = 0.7;
        PIN(lad_bot)   = MAX(0.4 * PIN(test_cur), PIN(i_min) / 0.7);
        PIN(lad_top)   = PIN(test_cur);
        PIN(lad_n)     = 4.0;
        if(PIN(lad_top) < 1.5 * PIN(lad_bot)) {  // too little range above i_min for a knee
          printf("<font color='red'># knee: idacim0.test_cur %f A leaves no range above idacim0.i_min %f A, no ladder</font>\n", PIN(test_cur), PIN(i_min));
          PIN(lad_top) = 0.0;
          PIN(lad_bot) = 0.0;
        }
        PIN(lad_auto) = PIN(lad_top);
      }
      PIN(state)    = 1.1;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(tmp0)     = 0.0;
      PIN(tmp1)     = 0.0;
      PIN(tmp2)     = 0.0;
      PIN(tmp3)     = 0.0;

      printf("Measure r, leakage l, rotor time constant\n");
      printf("<font color='green'>the rotor may stay free: a dc field makes no torque on a cage</font>\n");
      printf("idacim0.state = 1.2 <font color='green'>to start</font>\n");
      break;

    case 15:
      if(ctx->pend || ctx->rdone) {  // the last rung is still being reduced
        break;
      }
      if(PIN(r_ok) > 0.0) {
        printf("conf0.r = %f <font color='green'># append to config</font>\n", PIN(r));
        if(PIN(l_ok) > 0.0) {
          printf("conf0.l = %f <font color='green'># leakage, sigma*Ls</font>\n", PIN(l));
          printf("<font color='green'># from |Z| %f / %f ohm at %f / %f Hz, which\n", PIN(l_za), PIN(l_zb), PIN(l_fa), PIN(l_fb));
          printf("# also imply %f ohm of stator plus cage resistance there.\n", PIN(l_res));
          printf("# right for acim_foc (hv0.psi carries the rotor flux); acim_ttc\n");
          printf("# has no flux term, so there it makes iq fall short at speed.</font>\n");
        } else {
          printf("<font color='red'>l not measured</font>: injected %f / %f A of %f A asked\n", PIN(l_ia), PIN(l_ib), PIN(l_ripple) * PIN(test_cur));
          printf("use an LCR meter: line to line near 150 Hz, halved\n");
        }
        if(PIN(tr_ok) > 0.0) {
          printf("acim_flux0.tr = %f <font color='green'># append to config</font>\n", PIN(tr));
          printf("acim_flux0.lmr = %f <font color='green'># append to config</font>\n", PIN(lmr));
          printf("<font color='green'># rotor time constant tr = %f ms from %f edges\n", PIN(tr) * 1000.0, PIN(rot_n));
          printf("# slip_n = 1/tr = %f rad/s. for acim_ttc, which derives it from vel_n:\n", PIN(slip_n));
          printf("# acim_ttc0.vel_n = (2 pi acim_ttc0.freq_n - %f) / conf0.polecount\n", PIN(slip_n));
          printf("# from %f edges: up %f ms, down %f ms (median each), min %f max %f ms,\n", PIN(rot_n), PIN(tr_rise) * 1000.0, PIN(tr_fall) * 1000.0, PIN(tr_min) * 1000.0, PIN(tr_max) * 1000.0);
          printf("# spread %f of tr within a direction. up and down apart means\n", PIN(tr_spread));
          printf("# the step reaches into saturation; tr is their mean.</font>\n");
          if(PIN(tr_spread) > 0.25) {
            printf("<font color='red'># edges disagree: check ud_fb noise, or rerun with more rot_cycles.</font>\n");
          }
          printf("<font color='green'>");
          printf("# lmr = Lm^2/Lr = %f mH, ls = %f mH, at %f..%f A on d.\n", PIN(lmr) * 1000.0, PIN(ls) * 1000.0, PIN(test_cur) * 0.5, PIN(test_cur));
          printf("# tr is the rotor's at this temperature and flux: a hot cage\n");
          printf("# reads shorter, and rated flux saturates it a little shorter.</font>\n");
          if(PIN(n_volt) > 0.0 && PIN(n_freq) > 0.0 && PIN(n_cur) > 0.0 && PIN(n_pp) >= 1.0) {
            vf_set(pins);
          } else {
            printf("<font color='green'># for the V/f set give the plate: idacim0.n_volt, n_freq, n_cur, n_pp</font>\n");
          }
          if(PIN(rot_dip) > 0.1) {
            printf("<font color='red'># the current was %f of the step off at rot_t0:\n", PIN(rot_dip));
            printf("# the loop is slow. the fit uses the current that flowed, but\n");
            printf("# raise idacim0.rot_bw and rerun to confirm.</font>\n");
          }
        } else if(PIN(rot_n) >= 2.0) {
          printf("<font color='red'>tr not measured</font>: the fit gave %f ms, outside what this test can see\n", PIN(tr) * 1000.0);
        } else {
          printf("<font color='red'>tr not measured</font>: fewer than two edges fitted\n");
        }
        if(PIN(lad_top) > 0.0 && PIN(lad_n) >= 1.0) {
          // each rung's lmr is the slope of psi_r over its range; taken at the
          // range's middle and integrated from zero, it gives psi_r and the
          // secant psi_r / id that acim_flux's lmr is. Below the first rung
          // the slope is extended along the line through the first two.
          printf("<font color='green'># offset ladder: rung, id range [A], lmr slope [mH], tr [ms], spread, psi_r [Vs], secant lmr [mH]\n");
          int nr   = (int)MIN(PIN(lad_n), 4.0);
          float l0 = 0.0;
          for(int k = 0, j = -1; k < nr; k++) {
            if(PINA(lad_lm, k) > 0.0 && PINA(lad_i, k) > 0.0) {
              if(j < 0) {
                j  = k;
                l0 = PINA(lad_lm, k);
              } else {
                float ma = 0.75 * PINA(lad_i, j), mb = 0.75 * PINA(lad_i, k);
                l0       = PINA(lad_lm, j) + (PINA(lad_lm, j) - PINA(lad_lm, k)) * ma / (mb - ma);
                l0       = MAX(l0, PINA(lad_lm, j));
                break;
              }
            }
          }
          float psi = 0.0, mp = 0.0, lp = l0;
          for(int k = 0; k < nr; k++) {
            float lm = PINA(lad_lm, k);
            float m  = 0.75 * PINA(lad_i, k);
            float lo = lad_lo(pins) * PINA(lad_i, k);
            if(lm <= 0.0 || m <= 0.0) {
              printf("# %f %f..%f no fit (tr %f ms, spread %f)\n", (float)(k + 1), lo, PINA(lad_i, k), PINA(lad_tr, k) * 1000.0, PINA(lad_sp, k));
              continue;
            }
            if(PIN(lad_bot) > 0.0 || lad_lo(pins) != 0.5) {
              // rungs that do not tile 0..top: no psi_r from integrating them
              printf("# %f %f..%f %f %f %f\n", (float)(k + 1), lo, PINA(lad_i, k), lm * 1000.0, PINA(lad_tr, k) * 1000.0, PINA(lad_sp, k));
              continue;
            }
            psi += 0.5 * (lp + lm) * (m - mp);
            mp = m;
            lp = lm;
            printf("# %f %f..%f %f %f %f %f %f\n", (float)(k + 1), PINA(lad_i, k) * 0.5, PINA(lad_i, k), lm * 1000.0, PINA(lad_tr, k) * 1000.0, PINA(lad_sp, k), psi, psi / m * 1000.0);
          }
          printf("# psi_r and secant are at each range's middle, 0.75 of its top.</font>\n");
          knee_fit(pins);
        }
        printf("<font color='green'># dead time %f V per phase at the %f A dwell, %f V link\n", PIN(drop), PIN(test_cur), PIN(dc_volt));
        if(PIN(r_known) > 0.0) {
          printf("# read against the r you gave.</font>\n");
        } else {
          printf("# r is the chord %f less its dead time bias %f (idacim0.drop_slope).\n", PIN(r_2p), PIN(r_bias));
          printf("# below about 5 A the bias is understated; a four wire r in\n");
          printf("# idacim0.r_known is better.</font>\n");
        }
      } else {
        if(PIN(r_known) > 0.0) {
          printf("<font color='red'>r read failed</font>: the top dwell did not reach half of %f A\n", PIN(test_cur));
        } else {
          printf("<font color='red'>r fit failed</font>: the two dwells differ by %f A\n", PIN(fit_di));
        }
        printf("nothing below is measured, do not append it\n");
        printf("check that idacim0.test_cur (%f) is under conf0.max_ac_cur\n", PIN(test_cur));
      }
      // the standstill test ends here; spin goes on to the rotating test
      // (4.0), the pole pair test (2.0) runs only when set by hand. walk on
      // only if r worked: hv0.r is this pin
      if(PIN(r_ok) <= 0.0) {
        PIN(state) = 0.0;
      } else if(PIN(spin) > 0.0) {
        PIN(state) = 4.0;
      } else {
        printf("standstill test done\n");
        PIN(state) = 3.0;
      }
      break;

    case 20:  // pp
      PIN(state)    = 2.1;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(pp)       = 0.0;  // the filter starts from this run's first reading
      ctx->pp_n     = 0;
      ctx->pp_turn  = 0;

      printf("Measure polepairs\n");
      printf("<font color='green'>unblock the rotor, it will move</font>\n");
      printf("idacim0.state = 2.2 <font color='green'>to start</font>\n");
      break;

    case 40:  // rotating test
      if(PIN(tr_ok) <= 0.0 || PIN(n_freq) <= 0.0 || PIN(n_pp) < 1.0) {
        printf("<font color='red'>the rotating test needs this session's standstill test (tr, lmr) and idacim0.n_freq, n_pp</font>\n");
        PIN(state) = 3.0;
        break;
      }
      memset(ctx, 0, sizeof(struct idacim_ctx_t));
      PIN(sw_n)  = 0.0;  // a stall before the sweep reports no points, not the last run's
      PIN(state) = 4.1;
      printf("Rotating test: open loop field at %f A d, up to %f rad/s mech\n", PIN(id_n) > 0.0 ? PIN(id_n) : PIN(test_cur), 2.0 * M_PI * PIN(n_freq) * MIN(2.0 * PIN(rot_vel), 0.9) / PIN(n_pp));
      printf("<font color='green'>the rotor turns: unblock it, no load on the shaft</font>\n");
      printf("idacim0.state = 4.2 <font color='green'>to start</font>\n");
      break;

    case 47:
      rot_report(ctx, pins);
      PIN(state) = 5.0;
      break;

    case 24:
      // a slipping rotor, or no vel_fb, reads a ratio off any integer
      if(PIN(pp) >= 1.0 && ABS(ctx->pp_raw - PIN(pp)) < 0.15 && ctx->pp_turn > 0.8 * ctx->pp_n) {
        printf("conf0.polecount = %f <font color='green'># append to config</font>\n", PIN(pp));
        if(PIN(out_rev) > 0.0) {
          printf("conf0.out_rev = 1 <font color='green'># append to config</font>\n");
        }
      } else {
        printf("<font color='red'>pole pair read failed</font>: ratio %f, rotor turned for %f of the time (check vel_fb, or a load on the shaft)\n", ctx->pp_raw, ctx->pp_n > 0 ? (float)ctx->pp_turn / (float)ctx->pp_n : 0.0);
      }
      printf("done\n");
      PIN(state) = 3.0;
      break;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idacim_ctx_t *ctx        = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;

  if(PIN(en) <= 0.0) {
    PIN(state) = 0.0;
  }

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(en_out)   = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(cur_bw)   = 1.0;

      if(PIN(en) > 0.0) {
        PIN(state) = 1.0;
      }
      break;

    case 12:  // r -- rotor blocked, current control: dwell at two currents and
              // fit the voltage the current loop needs to sustain them.
              // com_pos is held at 0 throughout -- an induction motor has no rotor
              // flux to align to, so the injection axis is arbitrary.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = 1.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;

      // ud = r * id + 4/3 * drop: at angle 0 the phase currents are id,
      // -id/2, -id/2, and the three per phase dead time drops land 4/3 on d.
      // Two dwells split r (the slope) from the drop. Full current first, so
      // the first dwell aligns what the second holds.
      if(PIN(timer) < 2.0) {
        PIN(d_cmd) = PIN(test_cur);
        PIN(tmp2)  = PIN(tmp2) * 0.999 + PIN(id_fb) * 0.001;
        PIN(tmp3)  = PIN(tmp3) * 0.999 + PIN(ud_fb) * 0.001;
      } else {
        PIN(d_cmd) = PIN(test_cur) * 0.5;
        PIN(tmp0)  = PIN(tmp0) * 0.999 + PIN(id_fb) * 0.001;
        PIN(tmp1)  = PIN(tmp1) * 0.999 + PIN(ud_fb) * 0.001;
      }

      // hv0.r is this pin, the loop's plant model: the ud/id ratio bootstraps
      // it so current flows at cur_bw 1; the fit replaces it
      PIN(r) = PIN(r) * 0.99 + PIN(ud_fb) / MAX(PIN(id_fb), 0.01) * 0.01;

      PIN(timer) += period;
      if(PIN(timer) >= 4.0) {
        // the filters are linear and alike, so the fit holds on the filtered
        // pair even where neither dwell reaches its command
        float di    = PIN(tmp2) - PIN(tmp0);
        PIN(fit_di) = di;
        PIN(r_2p) = di > 0.01 ? MAX((PIN(tmp3) - PIN(tmp1)) / di, 0.001) : 0.0;

        float r_ok = 0.0;

        if(PIN(r_known) > 0.0) {
          // the drop is read at the top dwell, which has to be near its command
          if(PIN(tmp2) > PIN(test_cur) * 0.5) {
            PIN(r)      = PIN(r_known);
            PIN(r_bias) = 0.0;
            PIN(drop) = MAX(0.75 * (PIN(tmp3) - PIN(r) * PIN(tmp2)), 0.0);
            r_ok      = 1.0;
          }
        } else if(di > 0.01) {
          // The dead time drop rises as K ln(i), which biases the chord by
          // 8/3 K ln2 / test_cur. drop_slope = 8/3 ln2 K / Vdc, 0.0039 ohm A
          // per volt on the bridge it was fitted on. Skipped without dc_volt.
          PIN(r_bias) = PIN(dc_volt) > 1.0 ? PIN(drop_slope) * PIN(dc_volt) / MAX(PIN(test_cur), 0.1) : 0.0;
          PIN(r)      = MAX(PIN(r_2p) - PIN(r_bias), 0.001);
          PIN(drop)   = MAX(0.75 * (PIN(tmp3) - PIN(r) * PIN(tmp2)), 0.0);
          r_ok        = 1.0;
        }

        PIN(r_ok) = r_ok;

        // the l test needs the voltage that holds test_cur, dead time
        // included, and never one built from a failed measurement
        if(r_ok > 0.0) {
          PIN(avg_test_volt) = PIN(r) * PIN(test_cur) + 4.0 / 3.0 * PIN(drop);
          PIN(avg_test_volt) = LIMIT(PIN(avg_test_volt), PIN(pwm_volt) / 2.0);
        } else {
          PIN(avg_test_volt) = 0.0;
        }

        memset(ctx, 0, sizeof(struct idacim_ctx_t));
        ctx->l_amp  = 1.0;
        PIN(timer)  = 0.0;
        PIN(state)  = r_ok > 0.0 ? 1.3 : 1.5;
        PIN(d_cmd)  = 0.0;
        PIN(en_out) = 0.0;
        PIN(tmp0)   = 0.0;
        PIN(tmp1)   = 0.0;
        PIN(tmp2)   = 0.0;
        PIN(tmp3)   = 0.0;
      }
      break;

    case 13: {  // leakage l, by injection at two frequencies
      // A time constant test (l = tau * r) does not work here: on an induction
      // motor a voltage step's current has two poles, about 2 ms and 100-250 ms,
      // and the area method returns the slow one's Ls / r: the full stator
      // inductance, some twenty times the leakage the current loop wants. So
      // inject instead, as idpmsm does, but on d only (a cage rotor is round)
      // and at two frequencies:
      //
      //   |Z|^2 = R^2 + w^2 l^2   at both  =>  l^2 = (|Zb|^2 - |Za|^2) / (wb^2 - wa^2)
      //
      // so no resistance goes in. That matters here: the cage adds a
      // frequency dependent resistance on top of the stator's (0.2 ohm at
      // 100 Hz on the spindle, 0.8 at 1 kHz), and subtracting the dc r from
      // one |Z| gets l 10-13% wrong near the loop's crossover. The pair
      // assumes R and l are equal at both frequencies; the cage's rise between
      // 120 and 240 Hz costs a few percent of l, not more.
      //
      // The dc bias holds test_cur on d, so every phase current stays on one
      // side of zero and the dead time is an offset, not a nonlinearity. In
      // volt mode that bias settles on the slow rotor pole, hence the longer
      // first settle. A single pulsating axis makes no torque at standstill.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 0.0;  // volt cmd
      PIN(cur_bw)   = 1.0;
      PIN(com_pos)  = 0.0;
      PIN(q_cmd)    = 0.0;

      float f_c = (PIN(loop_bw) > 0.0 ? PIN(loop_bw) : 1000.0) / (2.0 * M_PI);
      float fa  = PIN(l_freq_a) > 0.0 ? PIN(l_freq_a) : 0.75 * f_c;
      float fb  = PIN(l_freq_b) > 0.0 ? PIN(l_freq_b) : 1.5 * f_c;
      // every window (L_BLOCK and the settles are multiples of it) has to
      // hold whole cycles, or the dc bias leaks into the demodulation
      // (simulated: 119.4/238.7 Hz read l 10% high): multiples of 1/L_BLOCK
      fa        = MAX(roundf(fa * L_BLOCK), 1.0) / L_BLOCK;
      fb        = MAX(roundf(fb * L_BLOCK), 1.0) / L_BLOCK;
      fa        = CLAMP(fa * (float)(1 << ctx->l_up), 20.0, 0.2 / period);
      fb        = CLAMP(fb * (float)(1 << ctx->l_up), 20.0, 0.2 / period);
      float f  = ctx->l_fi == 0 ? fa : fb;
      float w  = 2.0 * M_PI * f;
      ctx->l_th += w * period;
      if(ctx->l_th > 2.0 * M_PI) {
        ctx->l_th -= 2.0 * M_PI;
      }
      float sn, cs;
      sincos_fast(ctx->l_th, &sn, &cs);
      PIN(d_cmd) = PIN(avg_test_volt) + ctx->l_amp * sn;

      float v = PIN(ud_fb);
      float i = PIN(id_fb);
      ctx->l_t += period;
      float target = PIN(l_ripple) * PIN(test_cur);

      if(ctx->l_stage == 0) {
        if(ctx->l_t >= (ctx->l_fi == 0 ? L_BIAS_SETTLE : L_SETTLE)) {
          ctx->l_stage = 1;
          ctx->l_t     = 0.0;
          ctx->l_n     = 0;
          ctx->v_re = ctx->v_im = ctx->i_re = ctx->i_im = 0.0;
          ctx->v_dc = ctx->i_dc = ctx->s_dc = ctx->c_dc = 0.0;
        }
      } else {
        ctx->v_re += v * sn;
        ctx->v_im += v * cs;
        ctx->i_re += i * sn;
        ctx->i_im += i * cs;
        ctx->v_dc += v;
        ctx->i_dc += i;
        ctx->s_dc += sn;
        ctx->c_dc += cs;
        ctx->l_n++;
        float n = MAX((float)ctx->l_n, 1.0);
        // take the dc bias out with the mean times the reference sums (as
        // idpmsm), and end the window on a sample count, not float time
        float vm = ctx->v_dc / n, im = ctx->i_dc / n;
        float vr = ctx->v_re - vm * ctx->s_dc, vi = ctx->v_im - vm * ctx->c_dc;
        float ir = ctx->i_re - im * ctx->s_dc, ii = ctx->i_im - im * ctx->c_dc;
        if(ctx->l_stage == 1 && ctx->l_t >= L_BLOCK) {
          // size the amplitude to the ripple asked for: enough signal, and
          // never so much that a phase current crosses zero
          float i1 = 2.0 / n * sqrtf(ir * ir + ii * ii);
          float k  = i1 > 0.001 ? target / i1 : 4.0;
          ctx->l_amp *= CLAMP(k, 0.25, 4.0);
          ctx->l_amp = CLAMP(ctx->l_amp, 0.2, PIN(pwm_volt) / 4.0);
          ctx->l_block++;
          ctx->l_t = 0.0;
          ctx->l_n = 0;
          ctx->v_re = ctx->v_im = ctx->i_re = ctx->i_im = 0.0;
          ctx->v_dc = ctx->i_dc = ctx->s_dc = ctx->c_dc = 0.0;
          if(ctx->l_block >= L_BLOCKS) {
            ctx->l_stage = 2;
          }
        } else if(ctx->l_stage == 2 && ctx->l_n >= (uint32_t)(MAX(roundf(L_MEASURE * f), 1.0) / f / period + 0.5)) {
          // the f3 holds each 5 kHz command for a whole tick, which scales the
          // applied fundamental by sin(pi f T) / (pi f T); take it back out
          float v1  = 2.0 / n * sqrtf(vr * vr + vi * vi);
          float i1  = 2.0 / n * sqrtf(ir * ir + ii * ii);
          float x   = M_PI * f * period;
          float zoh = sinf(x) / x;
          float z   = i1 > 0.001 ? v1 * zoh / i1 : 0.0;
          if(ctx->l_fi == 0) {  // on to the upper frequency, from this amplitude
            PIN(l_za)    = z;
            PIN(l_ia)    = i1;
            ctx->l_fi    = 1;
            ctx->l_stage = 0;
            ctx->l_block = 0;
            ctx->l_t     = 0.0;
            ctx->l_n     = 0;
          } else {
            PIN(l_zb)  = z;
            PIN(l_ib)  = i1;
            float wa   = 2.0 * M_PI * fa;
            float wb   = 2.0 * M_PI * fb;
            float za   = PIN(l_za);
            float den  = wb * wb - wa * wa;
            float l2   = den > 0.0 ? (z * z - za * za) / den : 0.0;
            float r2   = den > 0.0 ? (wb * wb * za * za - wa * wa * z * z) / den : 0.0;
            int ok     = l2 > 0.0 && PIN(l_ia) > target * 0.5 && PIN(l_ia) < target * 2.0 && i1 > target * 0.5 && i1 < target * 2.0;
            PIN(l_ok)  = ok ? 1.0 : 0.0;
            PIN(l_res) = r2 > 0.0 ? sqrtf(r2) : 0.0;
            PIN(l_fa)  = fa;
            PIN(l_fb)  = fb;
            // a small leakage, as on high speed spindles, leaves |Z| mostly
            // resistance at these frequencies, and the cage's rise between
            // them then weighs on l^2. Go up an octave until the reactance at
            // the upper one is twice the resistance, as far as rt can draw it.
            if(ok && sqrtf(l2) * wb < 2.0 * PIN(l_res) && fb * 2.0 <= 0.2 / period && ctx->l_up < 3) {
              ctx->l_up++;
              ctx->l_fi    = 0;
              ctx->l_stage = 0;
              ctx->l_block = 0;
              ctx->l_t     = 0.0;
              ctx->l_n     = 0;
              break;
            }
            // hv0.l = idacim0.l, so the rotor test's current loop runs on this.
            // A failed read leaves the 1 mH init, a sane leakage for a few kW.
            PIN(l) = ok ? sqrtf(l2) : 0.001;

            memset(ctx, 0, sizeof(struct idacim_ctx_t));
            PIN(timer)  = 0.0;
            PIN(state)  = 1.4;
            PIN(d_cmd)  = 0.0;
            PIN(en_out) = 0.0;
          }
        }
      }
      break;
    }

    case 14: {  // rotor time constant and magnetizing inductance, by d current steps
      // Hold com_pos and step the d current between test_cur and test_cur/2
      // with a fast current loop. The rotor flux follows the stator current
      // with tr = Lr/Rr, and while it moves the stator sees its emf
      //
      //   e = Lmr * d(i_mr)/dt,   tr * d(i_mr)/dt = id - i_mr,   Lmr = Lm^2/Lr
      //
      // on top of r*id, the dead time and l*d(id)/dt. The first two settle with
      // the current; the chord from the r test is their dc slope over exactly
      // these two levels, so it takes them out of the part that has not.
      //
      // The current is not a clean step: the tail pushes against the loop and
      // id sags for as long as the integrator takes to catch it, and that sag
      // moves the flux too. Fitting an exponential to ud alone came out 3% short
      // on tr with a 1500 rad/s loop and 10% at 300 (simulated). So fit the
      // model to the current that actually flowed. With everything integrated
      // from the edge, where i_mr still sits at the old level i0, and c the
      // offset ud - R id where this edge settles:
      //
      //   lam(t) = integral(ud - R id - c) - l (id - i0) = Lmr (i_mr - i0)
      //   integral(lam) = Lmr * integral(id - i0) - tr * lam
      //
      // which is linear in Lmr and tr, fitted by least squares from rot_t0 to
      // the end of the edge.
      //
      // c is this edge's own, fitted with them (rot_nrt). Taken from an
      // earlier visit to the level, one bad sample or a cage warming between
      // visits put an error in c that grows as t in lam and
      // t^2 in its integral, and edges spread 100-300% on the spindle (2 Oct);
      // read off the edge's last quarter, a few mV of it still spread them
      // 30% (5 Oct). The integration takes out c0, the previous edge's offset
      // (the chord makes it the same at both levels), and the sums are kept as
      // a polynomial in the correction d = c - c0. The median of ROT_BLK block
      // means over the last quarter starts the fit and settles the level for
      // the next edge. A median of three
      // past rot_t0 keeps single bad samples out of the integrals. Each edge
      // is fitted on its own and the median is reported.
      //
      // Ladder rungs (lad_n > 0, lad_top > 0) repeat the same steps between
      // lad_top k/n and half of it after the main test: each rung's lmr is the
      // slope of rotor flux over its range, so together they give psi_r(id).
      // slip_n in acim_ttc is 1/tr in electrical rad/s.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = MAX(PIN(rot_bw), 1.0);
      PIN(com_pos)  = 0.0;
      PIN(q_cmd)    = 0.0;

      int nrung  = (int)CLAMP(PIN(lad_n), 0.0, 4.0);
      float hi   = ctx->rung ? lad_hi(pins, ctx->rung, nrung) : PIN(test_cur);
      float lo   = ctx->rung ? lad_lo(pins) * hi : 0.5 * hi;
      if(ctx->r_t == 0.0) {
        // rot_half is the longest edge. Once an edge has given a tr, the
        // next ones last 16 tr: past that the flux has settled and a longer
        // edge only lets an offset error grow (as e t in lam). On a short tr,
        // as on high speed spindles, 1.5 s edges spread 40-55% and 16 tr
        // ones 7-9% (simulated, tr 20 ms). rot_t0 skips the current loop's
        // settling; there it would skip most of the flux's move too, so it
        // stays under a fifth of tr.
        float tr_last = ctx->r_n > 0 ? ctx->tr_e[ctx->r_n - 1] : 0.0;
        ctx->half_e   = MAX(PIN(rot_half), 0.1);
        ctx->t0_e     = PIN(rot_t0);
        if(tr_last > 0.0) {
          ctx->half_e = CLAMP(16.0 * tr_last, 0.1, ctx->half_e);
          ctx->t0_e   = MIN(ctx->t0_e, 0.2 * tr_last);
        }
      }
      float half = ctx->half_e;
      float t0   = CLAMP(ctx->t0_e, period, half * 0.25);
      float t1   = half * ROT_TAIL;
      int lv     = ctx->r_edge & 1;  // 0 at hi, 1 at half of it
      float rin  = PIN(r_2p) > 0.0 ? PIN(r_2p) : PIN(r);

      PIN(d_cmd) = lv ? lo : hi;

      float ud = PIN(ud_fb);
      float id = PIN(id_fb);

      if(ctx->r_t == 0.0) {  // first tick of this level
        // the first edge of a rung comes from somewhere else: magnetizes only
        ctx->act = ctx->r_edge >= 1 && ctx->have_prev;
        if(ctx->act) {
          ctx->i0      = ctx->prev_i;
          ctx->c0      = ctx->prev_u - rin * ctx->prev_i;
          ctx->have_t0 = 0;
          ctx->p = ctx->q = ctx->jp = 0.0;
          ctx->dec = 0;
          for(int k = 0; k < 14; k++) {
            ctx->s[k] = 0.0;
          }
        }
        for(int k = 0; k < ROT_BLK; k++) {
          ctx->blk_u[k] = ctx->blk_i[k] = 0.0;
          ctx->blk_n[k] = 0;
        }
      }
      ctx->r_t += period;
      float t = ctx->r_t;

      // single bad samples: a median of three once the edge's fast part is over
      float um = ud;
      float im = id;
      if(t > t0) {
        um = med3(ud, ctx->u1, ctx->u2);
        im = med3(id, ctx->i1, ctx->i2);
      }
      ctx->u2 = ctx->u1;
      ctx->u1 = ud;
      ctx->i2 = ctx->i1;
      ctx->i1 = id;

      if(ctx->act) {
        ctx->p += (um - rin * im - ctx->c0) * period;
        ctx->q += (im - ctx->i0) * period;
        ctx->jp += ctx->p * period;
        if(t >= t0) {
          if(!ctx->have_t0) {
            ctx->i_t0    = im;
            ctx->have_t0 = 1;
          }
          if(++ctx->dec >= ROT_DEC) {
            ctx->dec = 0;
            // y = Lmr a + tr b, with b = bk + d t and y = yk - d t^2/2
            float a  = ctx->q;
            float bk = -(ctx->p - PIN(l) * (im - ctx->i0));
            float yk = ctx->jp - PIN(l) * ctx->q;
            float tt = t;
            float sg = -0.5 * t * t;
            ctx->s[0] += a * a;
            ctx->s[1] += a * bk;
            ctx->s[2] += a * tt;
            ctx->s[3] += bk * bk;
            ctx->s[4] += bk * tt;
            ctx->s[5] += tt * tt;
            ctx->s[6] += a * yk;
            ctx->s[7] += a * sg;
            ctx->s[8] += bk * yk;
            ctx->s[9] += bk * sg;
            ctx->s[10] += tt * yk;
            ctx->s[11] += tt * sg;
            ctx->s[12] += sg * sg;
            ctx->s[13] += sg * yk;
          }
        }
      }
      if(t >= t1) {
        int k = (int)((t - t1) / (half - t1) * ROT_BLK);
        k     = CLAMP(k, 0, ROT_BLK - 1);
        ctx->blk_u[k] += um;
        ctx->blk_i[k] += im;
        ctx->blk_n[k]++;
      }

      if(t >= half) {  // end of this level
        float bu[ROT_BLK], bi[ROT_BLK];
        int nb = 0;
        for(int k = 0; k < ROT_BLK; k++) {
          if(ctx->blk_n[k] > 0) {
            bu[nb] = ctx->blk_u[k] / (float)ctx->blk_n[k];
            bi[nb] = ctx->blk_i[k] / (float)ctx->blk_n[k];
            nb++;
          }
        }
        if(nb > 0) {
          float uinf = median(bu, nb);
          float iinf = median(bi, nb);
          float di   = iinf - ctx->i0;
          if(ctx->act && ABS(di) > 0.1 * hi && !ctx->pend) {
            for(int k = 0; k < 14; k++) {
              ctx->ps[k] = ctx->s[k];
            }
            ctx->pd   = uinf - rin * iinf - ctx->c0;
            ctx->pdip = ABS((ctx->i_t0 - iinf) / di);
            ctx->pup  = lv == 0;
            ctx->pdi  = di;
            ctx->ptail = bu[nb - 1] - bu[0];  // median() sorted bu
            ctx->pend = 1;
          }
          ctx->prev_u    = uinf;
          ctx->prev_i    = iinf;
          ctx->have_prev = 1;
        }
        ctx->r_edge++;
        ctx->r_t = 0.0;
      }

      if(ctx->r_edge >= 1 + 2 * (int)CLAMP(PIN(rot_cycles), 1.0, ROT_EDGES / 2)) {
        ctx->rd_rung = ctx->rung;
        ctx->rd_hi   = hi;
        ctx->rd_t1   = t1;
        ctx->rd_per  = period;
        ctx->rdone   = 1;

        if(PIN(lad_top) > 0.0 && ctx->rung < nrung) {  // next rung
          ctx->rung++;
          ctx->r_edge = 0;
          ctx->r_t    = 0.0;
        } else {
          PIN(timer)  = 0.0;
          PIN(state)  = 1.5;
          PIN(d_cmd)  = 0.0;
          PIN(en_out) = 0.0;
        }
      }
      break;
    }

    case 22:  // pp -- rotor free to turn: hold q_cmd at 0 and ramp com_pos open-loop
              // at test_vel while injecting d_cmd, forcing the rotor to follow the
              // rotating stator field (same technique as a sensorless PMSM I/F
              // startup). Comparing the commanded electrical rate against the
              // measured mechanical rate gives pole pairs directly.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = 100.0;
      PIN(q_cmd)    = 0.0;

      PIN(d_cmd) = PIN(test_cur);

      PIN(com_pos) += PIN(test_vel) * period;
      PIN(com_pos) = mod(PIN(com_pos));

      ctx->pp_n++;
      if(ABS(PIN(vel_fb)) > 0.1) {
        ctx->pp_turn++;
        float pp_m = PIN(test_vel) / PIN(vel_fb);
        PIN(pp)    = PIN(pp) != 0.0 ? PIN(pp) * 0.995 + pp_m * 0.005 : pp_m;
      }

      PIN(timer) += period;
      if(PIN(timer) >= 3.0) {
        PIN(timer) = 0.0;

        if(PIN(pp) < 0.0) {
          PIN(out_rev) = 1.0;
          PIN(pp) *= -1.0;
        }
        ctx->pp_raw = PIN(pp);
        PIN(pp)     = (int)(PIN(pp) + 0.5);

        PIN(en_out)   = 0.0;
        PIN(d_cmd)    = 0.0;
        PIN(cmd_mode) = 0.0;

        PIN(state) = 2.4;
      }
      break;

    case 42:  // rotating test: field up to speed, open loop at the plate flux current
    case 43:  // flux sweep
    case 44:  // inertia ramps
    case 46: {  // back down
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = MAX(PIN(rot_bw), 1.0);
      PIN(q_cmd)    = 0.0;
      int st        = (int)(PIN(state) * 10.0 + 0.5);
      float pp      = PIN(n_pp);
      float wn      = 2.0 * M_PI * PIN(n_freq);
      float acc     = PIN(rot_acc) > 0.0 ? PIN(rot_acc) : 0.2 * wn;
      float idr     = PIN(id_n) > 0.0 ? PIN(id_n) : PIN(test_cur);
      float imax    = PIN(n_cur) > 0.0 ? 0.85 * 1.4142136 * PIN(n_cur) : 1.15 * idr;
      float w_hi    = wn * MIN(2.0 * PIN(rot_vel), 0.9);
      ctx->w_t      = wn * CLAMP(PIN(rot_vel), 0.05, 0.45);
      float vel     = PIN(vel_fb);
      // electrical slip, with the field's sign; only meaningful with an encoder
      float slip = ctx->w_e - pp * ABS(vel);
      ctx->sw_t += period;
      PIN(d_cmd) = idr;

      if(st == 42) {
        ctx->w_e = MIN(ctx->w_e + acc * period, ctx->w_t);
        if(ctx->w_e >= ctx->w_t && ctx->sw_t > ctx->w_t / acc + 1.0) {
          // the encoder follows the field within a fifth: inertia can be
          // read. Turning but well behind it: the rotor pulled out on the
          // ramp, and nothing after this would mean anything.
          float lag    = ABS(pp * ABS(vel) - ctx->w_e);
          PIN(rot_enc) = lag < 0.2 * ctx->w_e ? 1.0 : 0.0;
          if(lag >= 0.2 * ctx->w_e && pp * ABS(vel) > 0.2 * ctx->w_e) {
            ctx->stall = 1;
            PIN(state) = 4.6;
            break;
          }
          PIN(state)   = 4.3;
          PIN(sw_n)    = 0.0;
          ctx->sw_k    = 0;
          // The sweep runs down from the top: at low flux the drag needs
          // more slip (pull out at slip x tr = 1, where 0.8 Nm of drag on the
          // spindle sits near 4.6 A), so the sweep stops at the first level
          // that slips too far, while the rotor is still at speed. Without an
          // encoder the slip is unseen: no level under 0.45 of id_n.
          ctx->sw_k0   = PIN(rot_enc) > 0.0 ? 0 : 2;
          while(ctx->sw_k0 < SW_N - 2 && sw_frac[ctx->sw_k0 + 1] * idr <= PIN(i_min)) {
            ctx->sw_k0++;
          }
          ctx->sw_top = SW_N - 1;
          while(ctx->sw_top > ctx->sw_k0 && sw_frac[ctx->sw_top - 1] * idr >= imax) {
            ctx->sw_top--;
          }
          ctx->sw_slip = 0.0;
          ctx->sw_pass = 0;
          ctx->sw_np   = 0;
          ctx->sw_hold = 0.0;
          ctx->sw_t    = 0.0;
          ctx->sw_cnt  = 0;
        }
      } else if(st == 43) {
        // Two passes over the same levels, at the test speed and at half of
        // it. The rotor flux read from uq carries any offset du on uq as
        // du / w (on the spindle the low levels read 4% higher at half speed
        // with the same slip, 6 Oct): with psi_k = lmr i_k + du / w_k at both
        // speeds, lmr = (w1 psi1 - w2 psi2) / (w1 i1 - w2 i2). Pass 2 brings
        // the field back to the test speed for the inertia ramps.
        float w_s = ctx->sw_pass == 1 ? 0.5 * ctx->w_t : ctx->w_t;
        if(ctx->w_e != w_s) {
          // the last level was the lowest flux: back to id_n and let the
          // flux build before the field changes speed, or the rotor falls
          // behind and pulls out (simulated)
          PIN(d_cmd) = idr;
          ctx->sw_hold += period;
          if(ctx->sw_hold > MAX(6.0 * PIN(tr), 0.3)) {
            ctx->w_e = ctx->w_e < w_s ? MIN(ctx->w_e + acc * period, w_s) : MAX(ctx->w_e - acc * period, w_s);
          }
          ctx->sw_t    = 0.0;
          ctx->sw_slip = 0.0;
        } else if(ctx->sw_pass == 2) {
          PIN(d_cmd) = idr;
          {
            int n = ctx->sw_np;
            for(int k = 0; k < n; k++) {
              float lm = ctx->sw_i1[k] > 0.0 ? ctx->sw_p1[k] / ctx->sw_i1[k] : 0.0;
              float du = 0.0;
              float den = ctx->w_t * ctx->sw_i1[k] - 0.5 * ctx->w_t * ctx->sw_i2[k];
              if(ctx->sw_i2[k] > 0.0 && den > 0.0) {
                lm = (ctx->w_t * ctx->sw_p1[k] - 0.5 * ctx->w_t * ctx->sw_p2[k]) / den;
                du = ctx->w_t * (ctx->sw_p1[k] - lm * ctx->sw_i1[k]);
              }
              PINA(sw_i, k)   = ctx->sw_i1[k];
              PINA(sw_psi, k) = lm * ctx->sw_i1[k];
              ctx->sw_du[k]   = du;
            }
            // taken from the top: put them in rising order, as the report
            // and the inertia test read them
            for(int a = 0, b = n - 1; a < b; a++, b--) {
              float t;
              t = PINA(sw_i, a), PINA(sw_i, a) = PINA(sw_i, b), PINA(sw_i, b) = t;
              t = PINA(sw_psi, a), PINA(sw_psi, a) = PINA(sw_psi, b), PINA(sw_psi, b) = t;
              t = ctx->sw_x[a], ctx->sw_x[a] = ctx->sw_x[b], ctx->sw_x[b] = t;
              t = ctx->sw_x2[a], ctx->sw_x2[a] = ctx->sw_x2[b], ctx->sw_x2[b] = t;
              t = ctx->sw_du[a], ctx->sw_du[a] = ctx->sw_du[b], ctx->sw_du[b] = t;
              t = ctx->sw_i1[a], ctx->sw_i1[a] = ctx->sw_i1[b], ctx->sw_i1[b] = t;
              t = ctx->sw_p1[a], ctx->sw_p1[a] = ctx->sw_p1[b], ctx->sw_p1[b] = t;
              t = ctx->sw_i2[a], ctx->sw_i2[a] = ctx->sw_i2[b], ctx->sw_i2[b] = t;
              t = ctx->sw_p2[a], ctx->sw_p2[a] = ctx->sw_p2[b], ctx->sw_p2[b] = t;
            }
            PIN(sw_n)  = n;
            ctx->sw_k  = 0;
            ctx->sw_t  = 0.0;
            PIN(state) = PIN(rot_enc) > 0.0 ? 4.4 : 4.6;
          }
        } else {
          ctx->sw_hold = 0.0;
          int fi     = ctx->sw_pass == 0 ? ctx->sw_top - ctx->sw_k : ctx->sw_fi[ctx->sw_k];
          float id   = CLAMP(sw_frac[fi] * idr, PIN(i_min), imax);
          PIN(d_cmd) = id;
          ctx->sw_slip += (slip - ctx->sw_slip) * MIN(period / 0.05, 1.0);
          int last = ctx->sw_pass == 0 ? fi <= ctx->sw_k0 : ctx->sw_k + 1 >= ctx->sw_np;
          // the rotor nears pull out: back to id_n before it falls behind
          if(PIN(rot_enc) > 0.0 && ctx->sw_t > 0.1 && ABS(ctx->sw_slip) * PIN(tr) > SW_X_STOP) {
            ctx->s_uq = ctx->s_iq = ctx->s_id = ctx->s_slip = 0.0;
            ctx->sw_cnt = 0;
            last        = 1;
            ctx->sw_t   = MAX(6.0 * PIN(tr), 0.3) + SW_MEASURE + 1.0;
          }
          if(ctx->sw_t > MAX(6.0 * PIN(tr), 0.3)) {
            ctx->s_uq += PIN(uq_fb);
            ctx->s_iq += PIN(iq_fb);
            ctx->s_id += PIN(id_fb);
            ctx->s_slip += slip;
            ctx->sw_cnt++;
          }
          if(ctx->sw_t > MAX(6.0 * PIN(tr), 0.3) + SW_MEASURE) {
            if(ctx->sw_cnt > 0) {
              float n   = (float)ctx->sw_cnt;
              float idm = ctx->s_id / n;
              float psd = (ctx->s_uq / n - PIN(r) * ctx->s_iq / n) / ctx->w_e;
              // with load (drag) the rotor slips and its flux turns off the
              // current axis by atan(slip tr): uq only sees the projection.
              // Back to the flux magnitude and the magnetizing current, from
              // the encoder's slip; without an encoder the slip is unknown.
              // tr is Lr / Rr and grows with lmr at low flux: x with the
              // standstill tr alone read lmr 6% low at x 0.6 (simulated, 14%
              // saturation), so tr follows this point's own lmr against the
              // standstill one, a few rounds
              float ws  = PIN(rot_enc) > 0.0 ? ctx->s_slip / n : 0.0;
              float pr  = psd - PIN(l) * idm;
              float trp = PIN(tr);
              float x   = 0.0, c = 1.0;
              for(int it = 0; it < 4; it++) {
                x = ws * trp;
                c = sqrtf(1.0 + x * x);
                if(PIN(lmr) > 0.0 && idm > 0.0) {
                  trp = PIN(tr) * CLAMP(pr * c * c / idm / PIN(lmr), 0.5, 2.0);
                }
              }
              // past SW_X_MAX the slip correction leans on tr more than the
              // point can carry (spindle, 6 Oct: 5.4 and 8.5 mH at x 0.57 and
              // 0.50 against 11-12 mH above): dropped, and lower levels only
              // slip more
              int k = ctx->sw_k;
              if(ABS(x) <= SW_X_MAX) {
                if(ctx->sw_pass == 0) {
                  ctx->sw_fi[k] = fi;
                  ctx->sw_x[k]  = x;
                  ctx->sw_i1[k] = idm / c;
                  ctx->sw_p1[k] = pr * c;
                  ctx->sw_i2[k] = 0.0;
                  ctx->sw_p2[k] = 0.0;
                  ctx->sw_x2[k] = 0.0;
                  ctx->sw_np    = k + 1;
                } else {
                  ctx->sw_x2[k] = x;
                  ctx->sw_i2[k] = idm / c;
                  ctx->sw_p2[k] = pr * c;
                }
              } else {
                last = 1;
              }
            }
            ctx->s_uq = ctx->s_iq = ctx->s_id = ctx->s_slip = 0.0;
            ctx->sw_cnt = 0;
            ctx->sw_t   = 0.0;
            ctx->sw_k++;
            if(last) {
              ctx->sw_k    = 0;
              ctx->sw_slip = 0.0;
              // the second speed needs the slip, so an encoder
              ctx->sw_pass = ctx->sw_pass == 0 && PIN(rot_enc) > 0.0 && ctx->sw_np > 0 ? 1 : 2;
            }
          }
        }
      } else if(st == 44) {
        // phase 0 hold until the rotor flux has settled back to id_n from
        // the sweep's last level, 1 up to w_hi, 2 hold, 3 down to w_t; the
        // middle half of each ramp is averaged
        float wa = ctx->w_t, wb = w_hi;
        if(ctx->sw_k == 1) {
          ctx->w_e = MIN(ctx->w_e + acc * period, wb);
        } else if(ctx->sw_k == 3) {
          ctx->w_e = MAX(ctx->w_e - acc * period, wa);
        }
        if(ctx->sw_k == 1 || ctx->sw_k == 3) {
          float x = (ctx->w_e - wa) / (wb - wa);
          if(x > 0.25 && x < 0.75) {
            if(ctx->sw_cnt == 0) {
              ctx->v_a = ABS(vel);
              ctx->t_a = ctx->sw_t;
            }
            ctx->s_slip += slip;
            // air gap power over the field speed: copper loss off the
            // stator power. Independent of tr; the r and dead time errors
            // are the same on both ramps and drop out of the difference.
            float id_f = PIN(id_fb), iq_f = PIN(iq_fb);
            ctx->s_tv += (PIN(ud_fb) * id_f + PIN(uq_fb) * iq_f - PIN(r) * (id_f * id_f + iq_f * iq_f)) / MAX(ctx->w_e, 1.0);
            ctx->v_b = ABS(vel);
            ctx->t_b = ctx->sw_t;
            ctx->sw_cnt++;
          }
          if(ABS(slip) * PIN(tr) > 1.0) {  // past the torque peak: pulled out
            ctx->stall = 1;
            PIN(state) = 4.6;
          }
        }
        int done = (ctx->sw_k == 0 && ctx->sw_t > MAX(6.0 * PIN(tr), 0.5)) || (ctx->sw_k == 1 && ctx->w_e >= wb) || (ctx->sw_k == 2 && ctx->sw_t > 0.5) || (ctx->sw_k == 3 && ctx->w_e <= wa);
        if(done && PIN(state) < 4.55) {
          if(ctx->sw_k == 1 || ctx->sw_k == 3) {
            int u            = ctx->sw_k == 1 ? 0 : 1;
            float dt         = ctx->t_b - ctx->t_a;
            ctx->j_slip[u]   = ctx->s_slip / (float)MAX(ctx->sw_cnt, 1);
            ctx->j_tv[u]     = ctx->s_tv / (float)MAX(ctx->sw_cnt, 1);
            ctx->s_tv        = 0.0;
            ctx->j_alpha[u]  = dt > 0.0 ? (ctx->v_b - ctx->v_a) / dt : 0.0;
            ctx->s_slip      = 0.0;
            ctx->sw_cnt      = 0;
          }
          ctx->sw_k++;
          ctx->sw_t = 0.0;
          if(ctx->sw_k > 3) {
            PIN(state) = 4.6;
          }
        }
      } else {
        ctx->w_e = MAX(ctx->w_e - acc * period, 0.0);
        if(ctx->w_e <= 0.0) {
          PIN(en_out)   = 0.0;
          PIN(d_cmd)    = 0.0;
          PIN(cmd_mode) = 0.0;
          PIN(state)    = 4.7;
        }
      }
      PIN(com_pos) = mod(PIN(com_pos) + ctx->w_e * period);
      break;
    }
  }
}

hal_comp_t idacim_comp_struct = {
    .name      = "idacim",
    .nrt       = nrt,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct idacim_ctx_t),
    .pin_count = sizeof(struct idacim_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
