#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"


#ifdef PRINTF_USES_UART
int _write(int file, char *ptr, int len)
{

    int i=0;
    uint8_t cr;
    for(i=0 ; i<len ; i++) {
        cr = *ptr++;
        while(app_uart_put(cr) != NRF_SUCCESS);
    }
    return len;
}
#endif

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}
