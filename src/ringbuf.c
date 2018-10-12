#include <string.h>

#include "ringbuf.h"


#define MIN(a, b) ((a) < (b) ? (a) : (b))

void ringbuf_init(struct ringbuf *rb, void *buf, uint8_t order)
{
    *rb = (struct ringbuf) {
        .head = 0,
        .tail = 0,
        .mask = (1UL << order) - 1,
        .mask2 = (2UL << order) - 1,
        .buf = buf,
    };
}

size_t ringbuf_write(struct ringbuf *rb, const void *src, size_t len)
{
    size_t space_avail = ringbuf_space_avail(rb);
    size_t write_len = MIN(space_avail, len);
    size_t masked_head = ringbuf_mask(rb, rb->head);
    size_t new_head = masked_head + write_len;
    size_t n_wrap;
    size_t n;

    if (ringbuf_full(rb))
        return 0;

    if (new_head > rb->mask) {
        n = rb->mask - masked_head + 1;
        n_wrap = new_head & rb->mask;
    } else {
        n = write_len;
        n_wrap = 0;
    }

    memcpy(rb->buf + masked_head, src, n);

    if (n_wrap)
        memcpy (rb->buf, src + n, n_wrap);

    rb->head = (rb->head + write_len) & rb->mask2;

    return write_len;
}

size_t ringbuf_read(struct ringbuf *rb, void *dst, size_t len)
{
    size_t space_used = ringbuf_space_used(rb);
    size_t read_len = MIN(space_used, len);
    size_t masked_tail = ringbuf_mask(rb, rb->tail);
    size_t new_tail = masked_tail + read_len;
    size_t n_wrap;
    size_t n;

    if (!space_used)
        return 0;

    if (new_tail > rb->mask) {
        n = rb->mask - masked_tail + 1;
        n_wrap = new_tail & rb->mask;
    } else {
        n = read_len;
        n_wrap = 0;
    }

    memcpy(dst, rb->buf + masked_tail, n);

    if (n_wrap)
        memcpy(dst + n, rb->buf, n_wrap);

    rb->tail = (rb->tail + read_len) & rb->mask2;

    return read_len;
}
