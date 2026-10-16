// LinuxCNC as the drive sees it over sserial: a servo thread every
// lcnc_period computes the joint's commanded position and velocity, and the
// frame reaches the drive latency (+ jitter) later. Moves are planned as
// trapezoids (LinuxCNC's default, no jerk limit) from rest to rest.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "runner.h"

double lcnc_period = 0.001;

static double latency = 100e-6, jitter = 0.0;
static double scale   = 1.0;
static int enable     = 0;

// trajectory
typedef enum { SEG_MOVE, SEG_JOG, SEG_DWELL } seg_kind_t;
typedef struct {
  seg_kind_t kind;
  double dist, vmax, amax, time;
} seg_t;
#define NSEG 64
static seg_t segs[NSEG];
static int nseg, cur;
static double seg_t0, seg_p0, seg_v0;  // start of the running segment
static double pos, vel;                // command at the last servo tick
static double next_servo;
static int started;

// frames in flight
typedef struct {
  double arrive, pos, vel, scale;
  int enable;
} frame_t;
#define NFR 16
static frame_t fr[NFR];
static int nfr;

void lcnc_init(void) {
  nseg = cur = nfr = 0;
  pos = vel = 0.0;
  next_servo = 0.0;
  started    = 0;
}

static void start_seg(double t) {
  seg_t0  = t;
  seg_p0  = pos;
  seg_v0  = vel;
  started = 1;
}

// trapezoid from rest: position and velocity at time t into the move
static int trap(const seg_t *s, double t, double *p, double *v) {
  double d = fabs(s->dist), sg = s->dist < 0 ? -1.0 : 1.0;
  double a = s->amax, vm = s->vmax;
  double ta = vm / a;
  if(a * ta * ta > d) {  // triangle
    ta = sqrt(d / a);
    vm = a * ta;
  }
  double tc = (d - a * ta * ta) / vm;
  double T  = 2 * ta + tc;
  if(t >= T) {
    *p = sg * d;
    *v = 0.0;
    return 1;
  }
  if(t < ta) {
    *p = 0.5 * a * t * t;
    *v = a * t;
  } else if(t < ta + tc) {
    *p = 0.5 * a * ta * ta + vm * (t - ta);
    *v = vm;
  } else {
    double r = T - t;
    *p       = d - 0.5 * a * r * r;
    *v       = a * r;
  }
  *p *= sg;
  *v *= sg;
  return 0;
}

static void servo(double t) {
  while(cur < nseg) {
    seg_t *s = &segs[cur];
    if(!started) {
      start_seg(t);
    }
    double dt = t - seg_t0;
    if(s->kind == SEG_DWELL) {
      vel = 0.0;
      if(dt >= s->time) {
        cur++;
        started = 0;
        continue;
      }
      return;
    }
    if(s->kind == SEG_JOG) {  // ramp from the start speed to vmax, then hold
      double dv = s->vmax - seg_v0;
      double ta = fabs(dv) / s->amax;
      if(dt < ta) {
        double a = dv > 0 ? s->amax : -s->amax;
        vel      = seg_v0 + a * dt;
        pos      = seg_p0 + seg_v0 * dt + 0.5 * a * dt * dt;
      } else {
        vel = s->vmax;
        pos = seg_p0 + seg_v0 * ta + 0.5 * dv * ta + s->vmax * (dt - ta);
      }
      if(cur + 1 < nseg) {  // a jog runs until the next command
        cur++;
        started = 0;
      }
      return;
    }
    double p, v;
    int done = trap(s, dt, &p, &v);
    pos      = seg_p0 + p;
    vel      = v;
    if(done) {
      cur++;
      started = 0;
      continue;
    }
    return;
  }
}

void lcnc_tick(double now) {
  while(next_servo <= now) {
    servo(next_servo);
    if(nfr < NFR) {
      frame_t *f = &fr[nfr++];
      f->arrive  = next_servo + latency + jitter * (rand() / (double)RAND_MAX);
      f->pos     = (float)pos;  // the frame carries floats
      f->vel     = (float)vel;
      f->enable  = enable;
      f->scale   = scale;
    }
    next_servo += lcnc_period;
  }
}

int lcnc_frame(double now, double *p, double *v, int *en, double *sc) {
  int got = 0;
  while(nfr && fr[0].arrive <= now) {
    *p  = fr[0].pos;
    *v  = fr[0].vel;
    *en = fr[0].enable;
    *sc = fr[0].scale;
    memmove(fr, fr + 1, sizeof(frame_t) * (--nfr));
    got = 1;
  }
  return got;
}

static int add(seg_t s) {
  if(nseg == NSEG) {
    return 0;
  }
  // a new command ends a jog in progress where it is: its start speed carries
  segs[nseg++] = s;
  return 1;
}

int lcnc_cmd(const char *args) {
  char w[32];
  double a = 0, b = 0, c = 0;
  int n = sscanf(args, "%31s %lf %lf %lf", w, &a, &b, &c);
  if(n < 1) {
    return 0;
  }
  if(!strcmp(w, "enable") && n >= 2) {
    enable = a > 0;
  } else if(!strcmp(w, "scale") && n >= 2) {
    scale = a;
  } else if(!strcmp(w, "latency") && n >= 2) {
    latency = a;
  } else if(!strcmp(w, "jitter") && n >= 2) {
    jitter = a;
  } else if(!strcmp(w, "period") && n >= 2) {
    lcnc_period = a;
  } else if(!strcmp(w, "move") && n == 4) {  // move <dist> <vel> <acc>
    return add((seg_t){SEG_MOVE, a, fabs(b), fabs(c), 0});
  } else if(!strcmp(w, "jog") && n == 3) {  // jog <vel> <acc>, until the next command
    return add((seg_t){SEG_JOG, 0, a, fabs(b), 0});
  } else if(!strcmp(w, "dwell") && n == 2) {
    return add((seg_t){SEG_DWELL, 0, 0, 0, a});
  } else {
    return 0;
  }
  return 1;
}

double lcnc_get(const char *key, int *ok) {
  *ok = 1;
  if(!strcmp(key, "pos")) return pos;
  if(!strcmp(key, "vel")) return vel;
  if(!strcmp(key, "enable")) return enable;
  if(!strcmp(key, "scale")) return scale;
  *ok = 0;
  return 0.0;
}
