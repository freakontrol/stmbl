// The F4 <-> F3 USART link: each packet leaves when its sender's rt reaches
// the send, and is usable once its last byte is in (bytes * bits / baud).
#include <stdlib.h>
#include <string.h>
#include "sim_hw.h"
#include "runner.h"

link_param_t link_param = {
    .tx_delay = 40e-6,
    .f3_delay = 2e-6,
    .baud     = DATABAUD,
    .bits     = 10,
    .jitter   = 0.0,
    .drop     = 0.0,
};

link_stats_t link_stats;

#define QLEN 8

typedef struct {
  double start, done;
  int taken;
  union {
    packet_to_hv_t to;
    packet_from_hv_t from;
  } p;
} slot_t;

static slot_t to_f3[QLEN], to_f4[QLEN];
static int to_f3_n, to_f4_n;

static double wire_time(int bytes) {
  return bytes * link_param.bits / link_param.baud;
}

static double urand(void) {
  return rand() / (double)RAND_MAX;
}

static void push(slot_t *q, int *n, double start, double done, const void *p, size_t size) {
  if(*n == QLEN) {  // the oldest is long gone
    memmove(q, q + 1, sizeof(slot_t) * (QLEN - 1));
    (*n)--;
  }
  slot_t *s = &q[(*n)++];
  s->start  = start;
  s->done   = done;
  s->taken  = 0;
  memcpy(&s->p, p, size);
}

// drop what has been taken or overtaken
static void prune(slot_t *q, int *n, double now) {
  int j = 0;
  for(int i = 0; i < *n; i++) {
    if(!(q[i].taken || (i + 1 < *n && q[i + 1].done <= now))) {
      q[j++] = q[i];
    }
  }
  *n = j;
}

void simlink_reset(void) {
  to_f3_n = to_f4_n = 0;
  memset(&link_stats, 0, sizeof(link_stats));
}

void simlink_f4_send(const packet_to_hv_t *p) {
  link_stats.f4_sent++;
  if(link_param.drop > 0.0 && urand() < link_param.drop) {
    link_stats.f4_dropped++;
    return;
  }
  double start = sim_now + link_param.tx_delay + link_param.jitter * urand();
  push(to_f3, &to_f3_n, start, start + wire_time(sizeof(*p)), p, sizeof(*p));
}

int simlink_f3_recv(packet_to_hv_t *p, int *partial) {
  double now = sim_now;
  int got    = 0;
  *partial   = 0;
  for(int i = 0; i < to_f3_n; i++) {
    slot_t *s = &to_f3[i];
    if(!s->taken && s->done <= now) {
      if(got) {
        link_stats.f3_overrun++;  // two whole packets between ticks: the older is lost
      }
      memcpy(p, &s->p.to, sizeof(*p));
      s->taken = 1;
      got      = 1;
    } else if(s->start <= now && now < s->done) {
      *partial = 1;
    }
  }
  prune(to_f3, &to_f3_n, now);
  link_stats.f3_taken += got;
  return got;
}

void simlink_f3_send(const packet_from_hv_t *p) {
  double start = sim_now + link_param.f3_delay;
  push(to_f4, &to_f4_n, start, start + wire_time(sizeof(*p)), p, sizeof(*p));
  link_stats.f3_sent++;
}

int simlink_f4_recv(packet_from_hv_t *p) {
  // hv0 runs at the end of the F4 rt, where it also sends
  double now = sim_now + link_param.tx_delay;
  int got    = 0;
  for(int i = 0; i < to_f4_n; i++) {
    slot_t *s = &to_f4[i];
    if(!s->taken && s->done <= now) {
      memcpy(p, &s->p.from, sizeof(*p));
      s->taken = 1;
      got      = 1;
    }
  }
  prune(to_f4, &to_f4_n, now);
  link_stats.f4_taken += got;
  return got;
}
