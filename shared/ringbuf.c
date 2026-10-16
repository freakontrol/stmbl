#include "ringbuf.h"

// The byte has to be in buf before the other side sees the moved index.
// Cortex-M4 keeps its own stores in order for an interrupt on the same core,
// so this only stops the compiler from moving the buf access past the index.
#define RB_BARRIER() __asm volatile("" ::: "memory")

static inline unsigned rb_next(const struct ringbuf *rb, unsigned i) {
  return i + 1 < rb->bufsize ? i + 1 : 0;
}

int rb_getc(struct ringbuf *rb, char *data) {
  unsigned r = rb->rd;
  if(r == rb->wr)
    return 0;

  *data = rb->buf[r];
  RB_BARRIER();
  rb->rd = rb_next(rb, r);
  return 1;
}

int rb_putc(struct ringbuf *rb, const char data) {
  unsigned w = rb->wr;
  unsigned n = rb_next(rb, w);
  if(n == rb->rd)
    return 0;

  rb->buf[w] = data;
  RB_BARRIER();
  rb->wr = n;
  return 1;
}

int rb_read(struct ringbuf *rb, void *data, int len) {
  char *d = (char *)data;
  int i   = 0;
  while(i < len && rb_getc(rb, d + i)) {
    i++;
  }
  return i;
}

int rb_write(struct ringbuf *rb, const void *data, int len) {
  const char *d = (const char *)data;
  int i         = 0;
  while(i < len && rb_putc(rb, d[i])) {
    i++;
  }
  return i;
}

// Looks for the '\n' without taking anything, so the writer can keep filling
// the free slots meanwhile; the old version took bytes and gave them back.
int rb_getline(struct ringbuf *rb, char *ptr, int len) {
  unsigned r     = rb->rd;
  unsigned avail = rb_len(rb);
  for(unsigned i = 0; i < avail && (int)i < len; i++) {
    char c = rb->buf[r];
    r      = rb_next(rb, r);
    if(c == '\n') {
      ptr[i] = '\0';
      RB_BARRIER();
      rb->rd = r;
      return i + 1;
    }
    ptr[i] = c;
  }
  // len bytes and still no '\n': this can never become a line that fits.
  // Drop them, or the reader would wait on them forever and every later
  // line behind them too (a dead terminal after one overlong or binary
  // burst from the host).
  if(len > 0 && avail >= (unsigned)len) {
    unsigned d = rb->rd + (unsigned)len;
    rb->rd     = d >= rb->bufsize ? d - rb->bufsize : d;
  }
  return 0;
}
