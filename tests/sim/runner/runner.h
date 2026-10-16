// The harness runner: link, plant, LinuxCNC command source, scenario.
#pragma once
#include <stdint.h>
#include "side.h"

// ---- link.c ----
typedef struct {
  double tx_delay;  // F4 rt start to the first byte [s]: the rt before hv0 plus hv0
  double f3_delay;  // F3 tick to the reply's first byte [s]
  double baud;
  double bits;      // per byte on the wire, 8N1 = 10
  double jitter;    // uniform 0..jitter added to tx_delay [s]
  double drop;      // probability an F4 packet is lost
} link_param_t;
extern link_param_t link_param;

typedef struct {
  long f4_sent, f4_dropped, f3_taken, f3_overrun, f3_sent, f4_taken;
} link_stats_t;
extern link_stats_t link_stats;
void simlink_reset(void);

// ---- plant.c ----
void plant_init(void);
int plant_set(const char *key, double v);  // 0: unknown key
int plant_was_set(const char *key);
void plant_frame(double sigma, double mech_offset);  // see plant.c
void plant_advance(double dt);
void plant_pwm(uint32_t u, uint32_t v, uint32_t w, uint32_t arr);  // compares for the next period
void plant_sample_adc(void);  // fills sim_f3_adc at this instant
void plant_encoder(double delay, double *mech_angle);  // motor angle delay ago [rad, multi-turn]
double plant_get(const char *key, int *ok);
extern double plant_enc_delay;
extern double plant_enc_bits;

// ---- lcnc.c ----
void lcnc_init(void);
int lcnc_cmd(const char *args);  // 0: bad command
void lcnc_tick(double now);      // servo thread: frames due by now are queued
int lcnc_frame(double now, double *pos, double *vel, int *enable, double *scale);  // next frame arrived by now
double lcnc_get(const char *key, int *ok);
extern double lcnc_period;
