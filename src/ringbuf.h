#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

struct ringbuf {
    volatile size_t head;
    volatile size_t tail;
    size_t mask;
    size_t mask2;
    char *buf;
};

void ringbuf_init(struct ringbuf *rb, void *buf, uint8_t pow2_size);

static inline size_t ringbuf_mask(const struct ringbuf *rb, size_t val)
{
    return val & rb->mask;
}

static inline size_t ringbuf_mask2(const struct ringbuf *rb, size_t val)
{
    return val & rb->mask2;
}

static inline bool ringbuf_full(const struct ringbuf *rb)
{
    if ((rb->head > rb->mask) != (rb->tail > rb->mask))
        return (rb->mask & rb->tail) == (rb->mask & rb->head);
    return false;
}

static inline bool ringbuf_empty(const struct ringbuf *rb)
{
    if ((rb->head > rb->mask) == (rb->tail > rb->mask))
        return (rb->mask & rb->tail) == (rb->mask & rb->head);
    return false;
}

static inline size_t ringbuf_space_used(const struct ringbuf *rb)
{
    return ringbuf_mask2(rb, rb->head - rb->tail);
}

static inline size_t ringbuf_capacity(const struct ringbuf *rb)
{
    return rb->mask + 1;
}

static inline size_t ringbuf_space_avail(const struct ringbuf *rb)
{
    return ringbuf_capacity(rb) - ringbuf_space_used(rb);
}

size_t ringbuf_write(struct ringbuf *rb, const void *src, size_t len);
size_t ringbuf_read(struct ringbuf *rb, void *dst, size_t len);

#endif
