#include <libopencm3/stm32/usart.h>
#include "retarget.h"
int _write(int fd, char *ptr, int len)
{
    int i;
    (void) fd;
    for (i = 0; i < len; ++i) {
        if (ptr[i] == '\n')
            usart_send_blocking(USART3, '\r');
        usart_send_blocking(USART3, ptr[i]);
    }
    return i;
}

int _read(int fd, char *ptr, int len)
{
    (void) fd;
    if (len > 0) {
        *ptr = usart_recv_blocking(USART3);
        return 1;
    }
    return 0;
}


