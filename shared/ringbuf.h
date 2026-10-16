#pragma once

#include <string.h>

/**
 * Ringbuffer structure
 *
 * Single producer, single consumer: one context writes (rb_putc, rb_write)
 * and one reads (rb_getc, rb_read, rb_getline), for example an interrupt and
 * the main loop, without locking. The writer only moves wr and the reader
 * only moves rd, so neither can lose the other's update. One slot stays
 * free to tell full from empty: a buffer holds bufsize - 1 bytes.
 */
struct ringbuf {
  char *buf;             ///< Pointer to buffer memory
  unsigned bufsize;      ///< Size of buffer memory
  volatile unsigned rd;  ///< Next byte to read, moved by the reader only
  volatile unsigned wr;  ///< Next byte to write, moved by the writer only
};

#define RINGBUF(size) \
  { .buf = (char[(size)]){0}, .bufsize = (size) }

/**
 * Number of bytes in the buffer
 */
static inline unsigned rb_len(const struct ringbuf *rb) {
  unsigned w = rb->wr;
  unsigned r = rb->rd;
  return w >= r ? w - r : w + rb->bufsize - r;
}

/**
 * Read a single byte from a buffer
 *
 * \param   rb    pointer to ringbuffer struct
 * \param   data  pointer to data byte
 * \return  number of bytes read (0 if buffer was empty)
 */
int rb_getc(struct ringbuf *rb, char *data);

/**
 * Write a single byte to a buffer
 *
 * \param   rb    pointer to ringbuffer struct
 * \param   data  data byte
 * \return  number of bytes written (0 if buffer was full)
 */
int rb_putc(struct ringbuf *rb, const char data);

/**
 * Read up to len bytes from a buffer
 */
int rb_read(struct ringbuf *rb, void *data, int len);

/**
 * Write multiple bytes to a buffer
 *
 * \param   rb    pointer to ringbuffer struct
 * \param   data  pointer to data
 * \param   len   number of bytes to write
 * \return  number of bytes written
 */
int rb_write(struct ringbuf *rb, const void *data, int len);

/**
 * Read one line, without the '\n', as a string into ptr
 *
 * \return  bytes taken including the '\n', 0 if no complete line of at most
 *          len bytes is in the buffer yet. Nothing is taken then, unless
 *          len bytes without a '\n' are waiting: those are dropped.
 */
int rb_getline(struct ringbuf *rb, char *ptr, int len);
