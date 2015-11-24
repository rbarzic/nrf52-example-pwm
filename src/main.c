// Simple PWM code that will output a 1MHz PWM signal with a
// 50% duty cycle on pin 19
// Note : you must have a PCA10040 board
// This will not run on a PCA10036 board

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf.h"
#include "bsp.h"

#include "app_uart.h"
#include "app_error.h"

#include "nrf_delay.h"

#include "nrf_clock.h"

#include "nrf_gpio.h"

#include "nrf_pwm.h"


// Autocompletion of nrf specific symbols and eldoc work much better when those
// files are included on top level
// This is not heeded if your code is using only SDK code and not accessing
// CMSIS stuff directly
#ifdef NRF51
#include "nrf51.h"
#include "core_cm0.h"
#endif

#ifdef NRF52
#include "nrf52.h"
#include "core_cm4.h"
#endif

#include "uart.h"

#define PIN_LED3 19
#define PIN_LED4 20

void dbg_pwm_registers(NRF_PWM_Type * NRF_PWMx) {
    printf("ENABLE            = 0x%08x\n",(unsigned)NRF_PWMx->ENABLE);
    printf("MODE              = 0x%08x\n",(unsigned)NRF_PWMx->MODE);
    printf("COUNTERTOP        = 0x%08x\n",(unsigned)NRF_PWMx->COUNTERTOP);
    printf("PRESCALER         = 0x%08x\n",(unsigned)NRF_PWMx->PRESCALER);
    printf("DECODER           = 0x%08x\n",(unsigned)NRF_PWMx->DECODER);
    printf("LOOP              = 0x%08x\n",(unsigned)NRF_PWMx->LOOP);
    printf("SEQ[0].PTR        = 0x%08x\n",(unsigned)NRF_PWMx->SEQ[0].PTR);
    printf("SEQ[0].CNT        = 0x%08x\n",(unsigned)NRF_PWMx->SEQ[0].CNT);
    printf("SEQ[0].REFRESH    = 0x%08x\n",(unsigned)NRF_PWMx->SEQ[0].REFRESH);
    printf("SEQ[0].ENDDELAY   = 0x%08x\n",(unsigned)NRF_PWMx->SEQ[0].ENDDELAY);
    printf("SEQ[1].PTR        = 0x%08x\n",(unsigned)NRF_PWMx->SEQ[1].PTR);
    printf("SEQ[1].CNT        = 0x%08x\n",(unsigned)NRF_PWMx->SEQ[1].CNT);
    printf("SEQ[1].REFRESH    = 0x%08x\n",(unsigned)NRF_PWMx->SEQ[1].REFRESH);
    printf("SEQ[1].ENDDELAY   = 0x%08x\n",(unsigned)NRF_PWMx->SEQ[1].ENDDELAY);
    printf("PSEL.OUT[0]       = 0x%08x\n",(unsigned)NRF_PWMx->PSEL.OUT[0]);
    printf("PSEL.OUT[1]       = 0x%08x\n",(unsigned)NRF_PWMx->PSEL.OUT[1]);
    printf("PSEL.OUT[2]       = 0x%08x\n",(unsigned)NRF_PWMx->PSEL.OUT[2]);
    printf("PSEL.OUT[3]       = 0x%08x\n",(unsigned)NRF_PWMx->PSEL.OUT[3]);
    printf("EVENTS_SEQSTARTED[0]       = 0x%08x\n",(unsigned)NRF_PWMx->EVENTS_SEQSTARTED[0]);
    printf("EVENTS_SEQSTARTED[1]       = 0x%08x\n",(unsigned)NRF_PWMx->EVENTS_SEQSTARTED[1]);
    printf("EVENTS_STOPPED             = 0x%08x\n",(unsigned)NRF_PWMx->EVENTS_STOPPED);
    printf("EVENTS_PWMPERIODEND        = 0x%08x\n",(unsigned)NRF_PWMx->EVENTS_PWMPERIODEND);

}

int main(void)
{




    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
        {
            RX_PIN_NUMBER,
            TX_PIN_NUMBER,
            RTS_PIN_NUMBER,
            CTS_PIN_NUMBER,
            APP_UART_FLOW_CONTROL_ENABLED,
            false,
            UART_BAUDRATE_BAUDRATE_Baud38400
        };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);


    printf("Hello world !! (%s)\n",__DATE__);

#define PWM_COUNTER_TOP 16000 // 16MHz divided by 16000-> 1ms
#define PWM_DUTY_CYCLE  50
#define PWM_CH0_DUTY ((uint16_t) ((PWM_COUNTER_TOP*PWM_DUTY_CYCLE)/100))


    if (1){
        uint16_t pwm_seq = PWM_CH0_DUTY;

        nrf_gpio_cfg_output(PIN_LED3);
        nrf_gpio_pin_set(PIN_LED3); // LED is off

        nrf_gpio_cfg_output(PIN_LED4);
        nrf_gpio_pin_set(PIN_LED4); // LED is off

        nrf_pwm_pselout_set(NRF_PWM0, NRF_PWM_CHANNEL0, PIN_LED3);
        nrf_pwm_enable(NRF_PWM0);
        nrf_pwm_mode_set(NRF_PWM0, NRF_PWM_MODE_UP);
        nrf_pwm_prescaler_set(NRF_PWM0, NRF_PWM_PRESCALER_DIV_1);
        nrf_pwm_countertop_set(NRF_PWM0, PWM_COUNTER_TOP);
        nrf_pwm_loop_set(NRF_PWM0, 0); // disabled
        nrf_pwm_decoder_set(NRF_PWM0,
                            NRF_PWM_DECODER_LOAD_COMMON, /* Same value for each channel */
                            NRF_PWM_DECODER_MODE_REFRESHCOUNT);

        nrf_pwm_seq_set(NRF_PWM0, NRF_PWM_SEQUENCE0, (uint32_t)&pwm_seq,
                        1, /* Count */
                        1, /* Refresh */
                        0);
        nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0);

    }



    dbg_pwm_registers(NRF_PWM0);
    while(1);
}
