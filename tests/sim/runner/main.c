// stmbl_sim: the drive firmware (F4 and F3 HAL, real components) against a
// plant, driven by a scenario file. See tests/sim/README.md.
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "runner.h"
#include "sim_hw.h"
#include "f3hw.h"

extern const char *const f3_init_lines[];
void plant_print_params(void);

double sim_now;

// time in ticks of 1/60 MHz: the F3, F4 rt and F4 frt periods are whole numbers of it
#define TICK_HZ 60000000LL
static int64_t T(double s) {
  return (int64_t)llround(s * TICK_HZ);
}
static double S(int64_t t) {
  return (double)t / TICK_HZ;
}

// ---------------- scenario ----------------

typedef enum { A_F4, A_F3, A_PLANT, A_LINK, A_LCNC } act_kind_t;
typedef struct {
  int64_t t;
  act_kind_t kind;
  char text[256];
  int line;
  int done;
} action_t;

typedef struct {
  char name[96];
  int side;  // 0 f4, 1 f3, 2 plant, 3 lcnc
  void *pin;
  double *v;  // samples
} probe_t;

typedef struct {
  char stat[16], sig[96], op[4];
  double val, t0, t1;
  int line;
  char text[256];
} expect_t;

static action_t *acts;
static int nacts, cap_acts;
static char configs[16][256];
static int nconfigs;
static probe_t probes[64];
static int nprobes;
static int trace_list[64];
static int ntrace;
static expect_t expects[64];
static int nexpects;
static double t_end = 1.0;
static int sample_f3 = 0;  // sample on F3 ticks instead of F4 rt
static double f3_phase = -1;  // F3 tick phase after the F4 rt [s], <0 = where ls0's lock settles
static char title[256] = "";
static const char *scenario_path;
static int verbose;
static int strict;  // firmware parse errors fail the run
static double *samp_t;
static long nsamp, cap_samp;

static void die(const char *fmt, const char *a, int line) {
  fprintf(stderr, "%s:%d: ", scenario_path, line);
  fprintf(stderr, fmt, a);
  fprintf(stderr, "\n");
  exit(2);
}

static void add_act(int64_t t, act_kind_t k, const char *text, int line) {
  if(nacts == cap_acts) {
    cap_acts = cap_acts ? cap_acts * 2 : 64;
    acts     = realloc(acts, sizeof(action_t) * cap_acts);
  }
  action_t *a = &acts[nacts++];
  a->t        = t;
  a->kind     = k;
  a->line     = line;
  a->done     = 0;
  snprintf(a->text, sizeof(a->text), "%s", text);
}

static int probe(const char *name) {
  for(int i = 0; i < nprobes; i++) {
    if(!strcmp(probes[i].name, name)) {
      return i;
    }
  }
  if(nprobes == 64) {
    fprintf(stderr, "too many signals\n");
    exit(2);
  }
  probe_t *p = &probes[nprobes];
  snprintf(p->name, sizeof(p->name), "%s", name);
  if(!strncmp(name, "f3:", 3)) {
    p->side = 1;
  } else if(!strncmp(name, "plant.", 6)) {
    p->side = 2;
  } else if(!strncmp(name, "lcnc.", 5)) {
    p->side = 3;
  } else {
    p->side = 0;
  }
  return nprobes++;
}

static char *trim(char *s) {
  while(isspace((unsigned char)*s)) {
    s++;
  }
  char *e = s + strlen(s);
  while(e > s && isspace((unsigned char)e[-1])) {
    *--e = 0;
  }
  return s;
}

static void parse_line(char *raw, int line) {
  char *s = trim(raw);
  if(!*s || *s == '#') {
    return;
  }
  int64_t t = 0;
  if(!strncmp(s, "at ", 3)) {
    char *end;
    double at = strtod(s + 3, &end);
    if(end == s + 3) {
      die("bad time: %s", s, line);
    }
    t = T(at);
    s = trim(end);
  }
  char word[32];
  int n = 0;
  if(sscanf(s, "%31s%n", word, &n) != 1) {
    die("empty command: %s", raw, line);
  }
  char *rest = trim(s + n);

  if(!strcmp(word, "f4")) {
    add_act(t, A_F4, rest, line);
  } else if(!strcmp(word, "f3")) {
    add_act(t, A_F3, rest, line);
  } else if(!strcmp(word, "plant")) {
    add_act(t, A_PLANT, rest, line);
  } else if(!strcmp(word, "link")) {
    add_act(t, A_LINK, rest, line);
  } else if(!strcmp(word, "lcnc")) {
    add_act(t, A_LCNC, rest, line);
  } else if(!strcmp(word, "config")) {
    if(t) {
      die("config only at the start: %s", s, line);
    }
    snprintf(configs[nconfigs++], 256, "%s", rest);
  } else if(!strcmp(word, "run")) {
    t_end = atof(rest);
  } else if(!strcmp(word, "title")) {
    snprintf(title, sizeof(title), "%s", rest);
  } else if(!strcmp(word, "strict")) {
    strict = 1;
  } else if(!strcmp(word, "sample")) {
    sample_f3 = !strcmp(rest, "f3");
  } else if(!strcmp(word, "f3_phase")) {
    f3_phase = atof(rest);
  } else if(!strcmp(word, "trace")) {
    char *tok = strtok(rest, " \t");
    while(tok) {
      int i = probe(tok), dup = 0;
      for(int j = 0; j < ntrace; j++) {
        dup |= trace_list[j] == i;
      }
      if(!dup) {
        trace_list[ntrace++] = i;
      }
      tok = strtok(0, " \t");
    }
  } else if(!strcmp(word, "expect")) {
    expect_t *e = &expects[nexpects++];
    memset(e, 0, sizeof(*e));
    e->line = line;
    e->t0   = 0;
    e->t1   = 1e9;
    snprintf(e->text, sizeof(e->text), "%s", rest);
    int m = 0;
    if(sscanf(rest, "%15s %95s %3s %lf%n", e->stat, e->sig, e->op, &e->val, &m) != 4) {
      die("expect <stat> <signal> <op> <value> [from t0] [to t1]: %s", rest, line);
    }
    char *r = rest + m, k[8];
    double v;
    int mm;
    while(sscanf(r, "%7s %lf%n", k, &v, &mm) == 2) {
      if(!strcmp(k, "from")) {
        e->t0 = v;
      } else if(!strcmp(k, "to")) {
        e->t1 = v;
      } else {
        die("expect: from/to: %s", rest, line);
      }
      r += mm;
    }
    probe(e->sig);
  } else {
    die("unknown command: %s", word, line);
  }
}

static void load_scenario(const char *path) {
  FILE *f = fopen(path, "r");
  if(!f) {
    fprintf(stderr, "cannot open %s\n", path);
    exit(2);
  }
  char buf[512];
  int line = 0;
  while(fgets(buf, sizeof(buf), f)) {
    line++;
    parse_line(buf, line);
  }
  fclose(f);
}

static int act_cmp(const void *a, const void *b) {
  const action_t *x = a, *y = b;
  if(x->t != y->t) {
    return x->t < y->t ? -1 : 1;
  }
  return x->line - y->line;  // file order within one instant
}

// ---------------- running ----------------

static void kv_apply(const char *text, int line, int is_link) {
  char buf[256];
  snprintf(buf, sizeof(buf), "%s", text);
  for(char *tok = strtok(buf, " \t"); tok; tok = strtok(0, " \t")) {
    char *eq = strchr(tok, '=');
    if(!eq) {
      die("key=value expected: %s", tok, line);
    }
    *eq      = 0;
    double v = atof(eq + 1);
    if(is_link) {
      if(!strcmp(tok, "tx_delay")) link_param.tx_delay = v;
      else if(!strcmp(tok, "f3_delay")) link_param.f3_delay = v;
      else if(!strcmp(tok, "baud")) link_param.baud = v;
      else if(!strcmp(tok, "bits")) link_param.bits = v;
      else if(!strcmp(tok, "jitter")) link_param.jitter = v;
      else if(!strcmp(tok, "drop")) link_param.drop = v;
      else die("no link parameter %s", tok, line);
    } else if(!plant_set(tok, v)) {
      die("no plant parameter %s (stmbl_sim -p lists them)", tok, line);
    }
  }
}

static void run_act(action_t *a) {
  if(a->done) {
    return;
  }
  a->done = 1;
  switch(a->kind) {
    case A_F4:
      f4_side.parse(a->text);
      break;
    case A_F3:
      f3_side.parse(a->text);
      break;
    case A_PLANT:
      kv_apply(a->text, a->line, 0);
      break;
    case A_LINK:
      kv_apply(a->text, a->line, 1);
      break;
    case A_LCNC:
      if(!lcnc_cmd(a->text)) {
        die("bad lcnc command: %s", a->text, a->line);
      }
      break;
  }
}

// a relative config path is tried from the working directory, then from the
// scenario's directory and each one above it (so conf/x.txt finds the repo's)
static FILE *open_up(const char *path) {
  FILE *f = fopen(path, "r");
  if(f || path[0] == '/') {
    return f;
  }
  char dir[1024], try[8192];
  snprintf(dir, sizeof(dir), "%s", scenario_path);
  char *sl = strrchr(dir, '/');
  if(sl) {
    *sl = 0;
  } else {
    strcpy(dir, ".");
  }
  for(int up = 0; up < 8 && strlen(dir) + 4 < sizeof(dir); up++) {
    snprintf(try, sizeof(try), "%s/%s", dir, path);
    if((f = fopen(try, "r"))) {
      return f;
    }
    strcat(dir, "/..");
  }
  return 0;
}

static void load_config(const char *path) {
  FILE *f = open_up(path);
  if(!f) {
    die("cannot open config %s", path, 0);
  }
  char buf[512];
  while(fgets(buf, sizeof(buf), f)) {
    f4_side.parse(buf);
  }
  fclose(f);
}

static void *need_pin(sim_side_t *s, const char *name) {
  void *p = s->pin(name);
  if(!p) {
    fprintf(stderr, "%s pin %s not loaded\n", s == &f3_side ? "F3" : "F4", name);
    exit(2);
  }
  return p;
}

static float f4_get(const char *name, float dflt) {
  void *p = f4_side.pin(name);
  return p ? f4_side.get(p) : dflt;
}

// plant defaults from the drive's own config, unless the scenario set them
static void plant_from_config(void) {
  struct {
    const char *plant, *pin;
  } m[] = {
      {"r", "conf0.r"}, {"ld", "conf0.l"}, {"psi", "conf0.psi"}, {"pp", "conf0.polecount"},
      {"jm", "conf0.j"}, {"jl", "conf0.j_sys"}, {"f", "conf0.f"}, {"d", "conf0.d"},
  };
  for(unsigned i = 0; i < sizeof(m) / sizeof(m[0]); i++) {
    void *p = f4_side.pin(m[i].pin);
    if(p && !plant_was_set(m[i].plant)) {
      plant_set(m[i].plant, f4_side.get(p));
    }
  }
  // an induction motor config: plant follows acim_flux0's model
  if(f4_side.pin("acim_flux0.lmr") && !plant_was_set("motor")) {
    plant_set("motor", 1);
  }
  if((f4_side.pin("vf0.u_n") || (f4_side.pin("idacim0.r") && !f4_side.pin("idpmsm0.r"))) && !plant_was_set("motor")) {
    plant_set("motor", 1);
  }
  if(plant_get("motor", &(int){0}) > 0.5) {
    struct {
      const char *plant, *pin;
    } a[] = {
        {"lmr", "acim_flux0.lmr"}, {"tr", "acim_flux0.tr"}, {"i_n", "acim_foc0.id_n"},
        {"i_knee", "acim_flux0.i_knee"}, {"lmr_sat", "acim_flux0.lmr_sat"}, {"tr_sat", "acim_flux0.tr_sat"},
        {"i_dip", "acim_flux0.i_dip"}, {"lmr_dip", "acim_flux0.lmr_dip"}, {"tr_dip", "acim_flux0.tr_dip"},
    };
    for(unsigned i = 0; i < sizeof(a) / sizeof(a[0]); i++) {
      void *p = f4_side.pin(a[i].pin);
      if(p && !plant_was_set(a[i].plant) && (f4_side.get(p) != 0.0 || !strcmp(a[i].plant, "tr_dip"))) {
        plant_set(a[i].plant, f4_side.get(p));
      }
    }
  }
  if(!plant_was_set("lq")) {
    float lq = f4_get("conf0.lq", 0.0);
    plant_set("lq", lq > 0.0 ? lq : f4_get("conf0.l", 0.0015));
  }
  // what the firmware calls the electrical angle, see plant.c
  double r_o   = f4_get("hv0.rev", 0.0) > 0.0 ? -1.0 : 1.0;
  double r_f   = f4_get("fb_switch0.mot_rev", 0.0) > 0.0 ? -1.0 : 1.0;
  double off   = f4_get("fb_switch0.mot_offset", 0.0);
  plant_frame(r_o * r_f, off);
}

// encoder shell: encf0 as fanuc_fb0 links it
static void *enc_pos, *enc_abs, *enc_turns, *enc_com, *enc_state, *enc_err;
// or enc_fb0 (sin/cos on the adc) as the spindle links it
static void *efb_pos, *efb_ipos, *efb_abs, *efb_state, *efb_err, *efb_amp;
static void encoder_out(void) {
  if(!enc_pos && !efb_ipos) {
    return;
  }
  double th;
  plant_encoder(plant_enc_delay, &th);
  double bits  = plant_get("enc_bits", &(int){0});
  double n     = ldexp(1.0, (int)bits);
  int64_t cnt  = (int64_t)floor(th / (2.0 * M_PI) * n + 0.5);
  int64_t half = (int64_t)(n / 2);
  int64_t one  = (int64_t)n;
  int64_t turn = (cnt + half) >= 0 ? (cnt + half) / one : -((-(cnt + half) + one - 1) / one);
  double pos   = (double)(cnt - turn * one) * 2.0 * M_PI / n;  // [-pi, pi)
  if(efb_ipos) {
    f4_side.set(efb_ipos, pos);
    if(efb_pos) f4_side.set(efb_pos, pos);
    if(efb_abs) f4_side.set(efb_abs, pos);
    if(efb_state) f4_side.set(efb_state, 3);
    if(efb_err) f4_side.set(efb_err, 0);
    if(efb_amp) f4_side.set(efb_amp, 0.5);
  }
  if(!enc_pos) {
    return;
  }
  f4_side.set(enc_pos, pos);
  if(enc_abs) f4_side.set(enc_abs, pos);
  if(enc_turns) f4_side.set(enc_turns, turn);
  if(enc_com) f4_side.set(enc_com, floor(pos / (2.0 * M_PI) * 1024.0) * 2.0 * M_PI / 1024.0);
  if(enc_state) f4_side.set(enc_state, 3);
  if(enc_err) f4_side.set(enc_err, 0);
}

static void *ss_pos, *ss_vel, *ss_en, *ss_scale, *ss_conn;
static void lcnc_out(double now) {
  lcnc_tick(now);
  double p, v, sc;
  int en;
  if(ss_pos && lcnc_frame(now, &p, &v, &en, &sc)) {
    f4_side.set(ss_pos, p);
    f4_side.set(ss_vel, v);
    f4_side.set(ss_en, en);
    f4_side.set(ss_scale, sc);
    if(ss_conn) f4_side.set(ss_conn, 1);
  }
}

static void sample(double now) {
  if(nsamp == cap_samp) {
    cap_samp = cap_samp ? cap_samp * 2 : 65536;
    samp_t   = realloc(samp_t, sizeof(double) * cap_samp);
    for(int i = 0; i < nprobes; i++) {
      probes[i].v = realloc(probes[i].v, sizeof(double) * cap_samp);
    }
  }
  samp_t[nsamp] = now;
  for(int i = 0; i < nprobes; i++) {
    probe_t *p = &probes[i];
    double v   = 0;
    int ok;
    switch(p->side) {
      case 0: v = f4_side.get(p->pin); break;
      case 1: v = f3_side.get(p->pin); break;
      case 2: v = plant_get(p->name + 6, &ok); break;
      case 3: v = lcnc_get(p->name + 5, &ok); break;
    }
    p->v[nsamp] = v;
  }
  nsamp++;
}

static int check_expects(void) {
  int fails = 0;
  for(int e = 0; e < nexpects; e++) {
    expect_t *x = &expects[e];
    double *v   = probes[probe(x->sig)].v;
    double acc = 0, mx = -INFINITY, mn = INFINITY, sq = 0, first = NAN, last = NAN;
    long n = 0;
    for(long i = 0; i < nsamp; i++) {
      if(samp_t[i] < x->t0 - 1e-12 || samp_t[i] > x->t1 + 1e-12) {
        continue;
      }
      double y = v[i];
      if(!n) first = y;
      last = y;
      acc += y;
      sq += y * y;
      mx = fmax(mx, y);
      mn = fmin(mn, y);
      n++;
    }
    double r;
    if(!n) {
      printf("FAIL line %d: no samples in %s\n", x->line, x->text);
      fails++;
      continue;
    }
    if(!strcmp(x->stat, "max")) r = mx;
    else if(!strcmp(x->stat, "min")) r = mn;
    else if(!strcmp(x->stat, "absmax")) r = fmax(fabs(mx), fabs(mn));
    else if(!strcmp(x->stat, "mean")) r = acc / n;
    else if(!strcmp(x->stat, "rms")) r = sqrt(sq / n);
    else if(!strcmp(x->stat, "std")) r = sqrt(fmax(sq / n - (acc / n) * (acc / n), 0.0));
    else if(!strcmp(x->stat, "p2p")) r = mx - mn;
    else if(!strcmp(x->stat, "first")) r = first;
    else if(!strcmp(x->stat, "final")) r = last;
    else {
      die("unknown stat %s (max min absmax mean rms std p2p first final)", x->stat, x->line);
      r = 0;
    }
    int ok;
    if(!strcmp(x->op, "<")) ok = r < x->val;
    else if(!strcmp(x->op, "<=")) ok = r <= x->val;
    else if(!strcmp(x->op, ">")) ok = r > x->val;
    else if(!strcmp(x->op, ">=")) ok = r >= x->val;
    else {
      die("unknown op %s", x->op, x->line);
      ok = 0;
    }
    printf("%s %s %s = %.6g (want %s %g)\n", ok ? "pass" : "FAIL", x->stat, x->sig, r, x->op, x->val);
    fails += !ok;
  }
  return fails;
}

static void write_csv(const char *path, int decim) {
  FILE *f = fopen(path, "w");
  if(!f) {
    fprintf(stderr, "cannot write %s\n", path);
    exit(2);
  }
  fprintf(f, "t");
  for(int j = 0; j < ntrace; j++) {
    fprintf(f, ",%s", probes[trace_list[j]].name);
  }
  fprintf(f, "\n");
  for(long i = 0; i < nsamp; i += decim) {
    fprintf(f, "%.7f", samp_t[i]);
    for(int j = 0; j < ntrace; j++) {
      fprintf(f, ",%.7g", probes[trace_list[j]].v[i]);
    }
    fprintf(f, "\n");
  }
  fclose(f);
}

static void usage(void) {
  printf(
      "usage: stmbl_sim [-v] [-o out.csv] [-d decimate] [-e 'line']... scenario.txt\n"
      "       stmbl_sim -p   list the plant parameters\n");
}

int main(int argc, char **argv) {
  const char *out = 0;
  int decim       = 1;
  char *extra[32];
  int nextra = 0;
  for(int i = 1; i < argc; i++) {
    if(!strcmp(argv[i], "-v")) {
      verbose = 1;
    } else if(!strcmp(argv[i], "-o") && i + 1 < argc) {
      out = argv[++i];
    } else if(!strcmp(argv[i], "-d") && i + 1 < argc) {
      decim = atoi(argv[++i]);
    } else if(!strcmp(argv[i], "-e") && i + 1 < argc && nextra < 32) {
      extra[nextra++] = argv[++i];
    } else if(!strcmp(argv[i], "-p")) {
      plant_print_params();
      return 0;
    } else if(argv[i][0] == '-') {
      usage();
      return 2;
    } else {
      scenario_path = argv[i];
    }
  }
  if(!scenario_path) {
    usage();
    return 2;
  }
  load_scenario(scenario_path);
  for(int i = 0; i < nextra; i++) {
    parse_line(extra[i], 10000 + i);
  }
  qsort(acts, nacts, sizeof(action_t), act_cmp);

  srand(1);
  plant_init();
  lcnc_init();
  simlink_reset();

  // ---- F4 boot: main.c's order, the scenario's configs in place of loadconf ----
  f4_side.set_verbose(verbose);
  f4_side.init(0.0002, 0.00005);
  f4_side.parse("load term");
  for(int i = 0; i < nconfigs; i++) {
    load_config(configs[i]);
  }
  // the scenario's t = 0 f4 lines count as part of the saved config
  for(int i = 0; i < nacts && acts[i].t == 0; i++) {
    if(acts[i].kind == A_F4) {
      run_act(&acts[i]);
    }
  }
  f4_side.parse("relink");
  f4_side.parse("start");

  // ---- F3 boot: the hal_parse lines of stm32f303/src/main.c ----
  f3_side.set_verbose(verbose);
  f3_side.init(1.0 / PWM_FREQ, 0.0);
  for(int i = 0; f3_init_lines[i]; i++) {
    f3_side.parse(f3_init_lines[i]);
  }
  f3_side.parse("start");

  // the rest of the t = 0 actions (plant, link, lcnc, f3 lines)
  int ai = 0;
  for(; ai < nacts && acts[ai].t == 0; ai++) {
    run_act(&acts[ai]);
  }
  plant_from_config();

  // The F3 ticks where ls0's lock puts them: dma_pos_cmd (4) bytes into each packet.
  double phase = f3_phase >= 0 ? f3_phase : link_param.tx_delay + 4 * link_param.bits / link_param.baud;
  phase        = fmod(phase, 1.0 / PWM_FREQ);
  // Encoder age, unless the scenario sets it: hv0.adv (identified on the
  // board with id_tune) covers the encoder's age plus the packet's way to the
  // F3 tick that takes it; the sim models the second, so the rest is the first.
  if(!plant_was_set("enc_delay")) {
    double done = link_param.tx_delay + sizeof(packet_to_hv_t) * link_param.bits / link_param.baud;
    double take = phase + ceil((done - phase) * PWM_FREQ - 1e-9) / PWM_FREQ;
    double adv  = f4_get("hv0.adv", 0.0);
    plant_set("enc_delay", fmax(adv - take, 0.0));
    if(verbose) {
      printf("plant.enc_delay %.1f us (hv0.adv %.1f us - packet to F3 take %.1f us)\n", plant_get("enc_delay", &(int){0}) * 1e6, adv * 1e6, take * 1e6);
    }
  }
  plant_enc_delay = plant_get("enc_delay", &(int){0});

  enc_pos   = f4_side.pin("encf0.pos");
  enc_abs   = f4_side.pin("encf0.abs_pos");
  enc_turns = f4_side.pin("encf0.turns");
  enc_com   = f4_side.pin("encf0.com_pos");
  enc_state = f4_side.pin("encf0.state");
  enc_err   = f4_side.pin("encf0.error");
  if(f4_side.pin("enc_fb0.ipos") && f4_side.pin("fb_switch0.mot_pos")) {
    efb_ipos  = f4_side.pin("enc_fb0.ipos");
    efb_pos   = f4_side.pin("enc_fb0.pos");
    efb_abs   = f4_side.pin("enc_fb0.abs_pos");
    efb_state = f4_side.pin("enc_fb0.state");
    efb_err   = f4_side.pin("enc_fb0.error");
    efb_amp   = f4_side.pin("enc_fb0.amp");
    if(!plant_was_set("enc_bits")) {  // lines x interpolation
      double n = f4_get("enc_fb0.res", 0.0) * f4_get("enc_fb0.ires", 0.0);
      if(n > 0.0) plant_set("enc_bits", log2(n));
    }
  }
  ss_pos    = f4_side.pin("sserial0.pos_cmd");
  if(ss_pos) {
    ss_vel   = need_pin(&f4_side, "sserial0.pos_cmd_d");
    ss_en    = need_pin(&f4_side, "sserial0.enable");
    ss_scale = need_pin(&f4_side, "sserial0.scale");
    ss_conn  = f4_side.pin("sserial0.connected");
    f4_side.set(ss_scale, 1.0);
  }

  for(int i = 0; i < nprobes; i++) {
    probe_t *p = &probes[i];
    if(p->side == 0) {
      p->pin = need_pin(&f4_side, p->name);
    } else if(p->side == 1) {
      p->pin = need_pin(&f3_side, p->name + 3);
    } else {
      int ok;
      if(p->side == 2) plant_get(p->name + 6, &ok);
      else lcnc_get(p->name + 5, &ok);
      if(!ok) {
        fprintf(stderr, "unknown signal %s\n", p->name);
        return 2;
      }
    }
  }

  // ---- schedule ----
  // F4 rt every 200 us, its frt at 25 + 50 n us (TIM5 preset to half, see
  // the C4 note in the architecture review). The F3 runs its own PWM clock,
  // locked by ls0 so a tick falls dma_pos_cmd (4) bytes into each packet.
  int64_t p_rt = T(0.0002), p_frt = T(0.00005), p_f3 = TICK_HZ / PWM_FREQ;
  int64_t n_rt = 0, n_frt = T(0.000025), n_f3 = T(phase) % p_f3;
  int64_t t_stop = T(t_end), t_plant = 0;
  long f3_ticks = 0;
  int last_state = -1;

  while(1) {
    int64_t t = n_rt;
    if(n_frt < t) t = n_frt;
    if(n_f3 < t) t = n_f3;
    if(ai < nacts && acts[ai].t < t) t = acts[ai].t;
    if(t > t_stop) break;

    plant_advance(S(t - t_plant));
    t_plant = t;
    sim_now = S(t);
    f4_side.set_systime((uint64_t)(sim_now * 1000));
    f3_side.set_systime((uint64_t)(sim_now * 1000));

    while(ai < nacts && acts[ai].t <= t) {
      run_act(&acts[ai++]);
      plant_enc_delay = plant_get("enc_delay", &(int){0});
    }
    if(t == n_f3) {
      plant_sample_adc();
      f3_side.run_rt();
      plant_pwm(sim_tim8.CCR3, sim_tim8.CCR2, sim_tim8.CCR1, sim_tim8.ARR);  // PWM_U CCR3, V CCR2, W CCR1
      if(++f3_ticks % PWM_TICKS_PER_PACKET == 0) {
        f3_side.run_nrt();
      }
      if(sample_f3) {
        sample(sim_now);
      }
      n_f3 += p_f3;
    }
    if(t == n_rt) {
      encoder_out();
      f4_side.run_rt();
      f4_side.run_nrt();
      if(!sample_f3) {
        sample(sim_now);
      }
      void *st = f4_side.pin("fault0.state");
      int s    = st ? (int)f4_side.get(st) : -1;
      if(verbose && s != last_state) {
        printf("%9.5f fault0.state %d fault0.last_fault %d\n", sim_now, s, (int)f4_get("fault0.last_fault", 0));
      }
      last_state = s;
      n_rt += p_rt;
    }
    if(t == n_frt) {
      lcnc_out(sim_now);
      f4_side.run_frt();
      n_frt += p_frt;
    }
  }

  printf("%s%s%s: %.3f s, %ld samples, F4 hal %s, F3 hal %s, link %ld/%ld packets taken%s\n", scenario_path, title[0] ? " - " : "", title, t_end, nsamp,
         f4_side.hal_state() == 7 ? "ok" : "STOPPED", f3_side.hal_state() == 7 ? "ok" : "STOPPED", link_stats.f3_taken, link_stats.f4_sent,
         link_stats.f3_overrun ? " (overruns!)" : "");
  int errs = f4_side.errors() + f3_side.errors();
  if(errs) {
    printf("%s %d firmware parse errors (not found / load_comp), see above\n", strict ? "FAIL" : "warning:", errs);
  }
  if(out) {
    write_csv(out, decim > 0 ? decim : 1);
  }
  int fails = check_expects() + (strict && errs > 0) + (f4_side.hal_state() != 7) + (f3_side.hal_state() != 7);
  printf("%s\n", fails ? "FAILED" : "PASSED");
  return fails ? 1 : 0;
}
