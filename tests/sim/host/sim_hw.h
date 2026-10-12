// What the sim components (comps_f4/, comps_f3/, the F3 shim) and the runner
// share. Defined in runner/, so these symbols stay global when each firmware
// side is linked into one object with everything else made local.
#pragma once
#include <stdint.h>
#include "common.h"

// ---- time ----
extern double sim_now;  // [s], the instant of the tick being run

// ---- F4 <-> F3 link (runner/link.c) ----
// F4 side, called by comps_f4/hv.c
void simlink_f4_send(const packet_to_hv_t *p);  // starts a packet now
int simlink_f4_recv(packet_from_hv_t *p);       // 1: a whole reply arrived since the last call
// F3 side, called by comps_f3/ls.c
int simlink_f3_recv(packet_to_hv_t *p, int *partial);  // 1: a whole packet; partial: bytes of one in flight
void simlink_f3_send(const packet_from_hv_t *p);

// ---- F3 hardware (runner/plant.c) ----
typedef struct {
  float iu, iv, iw;  // sampled phase currents [A], with the ADC's offset, noise and LSB
  float ur, vr, wr;  // phase voltages to ground [V]
  float udc;         // link [V]
  float hv_temp;     // [C]
} sim_f3_adc_t;
extern sim_f3_adc_t sim_f3_adc;

typedef struct {
  volatile uint32_t ARR, CCR1, CCR2, CCR3;
  uint32_t moe;  // main output enable, io sets it, the comparator model clears it
  uint32_t brk;  // comparator break flag, set by the plant, cleared by io
  uint32_t cmp;  // comparator outputs, bit 0 u, 1 v, 2 w
  float dac;     // comparator dac word
} sim_tim_t;
extern sim_tim_t sim_tim8;
