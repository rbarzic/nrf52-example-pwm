// Simple RTC code that will toggle a pin every second

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf.h"
#include "bsp.h"

#include "app_uart.h"
#include "app_error.h"

#include "nrf_delay.h"

#include "nrf_clock.h"
#include "nrf_rtc.h"
#include "nrf_gpio.h"

#include "nrf_saadc.h"

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

#define PIN_LED 19 // LED1
#define ADC_RESULT_BUFFER_SIZE 1

static volatile unsigned irq_counter = 0;
static volatile unsigned irq_triggered = 0;

#define NRF_SAADC_EVENT_DONE offsetof(NRF_SAADC_Type, EVENTS_DONE)

#define  RTC_PRESCALER_VAL 1

void RTC0_IRQHandler(void) {
    irq_counter++;
    irq_triggered=1;
    nrf_rtc_event_clear(NRF_RTC0,NRF_RTC_EVENT_COMPARE_0);
    nrf_rtc_task_trigger(NRF_RTC0,NRF_RTC_TASK_CLEAR);
    nrf_gpio_pin_toggle(PIN_LED);
}

int main(void)
{

    nrf_saadc_value_t result;;
    nrf_gpio_cfg_output(PIN_LED);
    nrf_gpio_pin_set(PIN_LED); // LED is off

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


    printf("Hello world ! (%s)\n",__DATE__);






    // Irq setup
    NVIC_SetPriority(RTC0_IRQn, 15); // Lowes priority
    NVIC_ClearPendingIRQ(RTC0_IRQn);
    NVIC_EnableIRQ(RTC0_IRQn);

    // Start LFCLK clock
    nrf_clock_lf_src_set(NRF_CLOCK_LF_SRC_RC); // 32KHz RC
    nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART);


    // One hertz RTC event generation

    nrf_rtc_prescaler_set(NRF_RTC0,RTC_PRESCALER_VAL);
    nrf_rtc_cc_set(NRF_RTC0,0 /*ch*/, (32768/(1+RTC_PRESCALER_VAL)));


    nrf_rtc_event_enable(NRF_RTC0, NRF_RTC_EVENT_COMPARE_0);
    nrf_rtc_int_enable(NRF_RTC0,NRF_RTC_INT_COMPARE0_MASK);
    nrf_rtc_task_trigger(NRF_RTC0,NRF_RTC_TASK_START);




    nrf_saadc_channel_config_t config;
    config.acq_time = NRF_SAADC_ACQTIME_20US;
    config.gain = NRF_SAADC_GAIN1_4;
    config.mode = NRF_SAADC_MODE_SINGLE_ENDED;
    config.pin_p = NRF_SAADC_INPUT_AIN0;
    config.pin_n = NRF_SAADC_INPUT_AIN0; // Single ended -> should be ground to zero
    config.reference = NRF_SAADC_REFERENCE_VDD4;
    config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;

    nrf_saadc_enable();
    nrf_saadc_channel_init(NRF_SAADC_INPUT_AIN0, &config);
    nrf_saadc_buffer_init(&result,1); // One sample

    //for (i = 0; i < 1000; i++) {
    //    result = 0;
    //    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
    //
    //    while(!nrf_saadc_event_check(NRF_SAADC_EVENT_STARTED));
    //    nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
    //
    //    nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);
    //
    //
    //    while(!nrf_saadc_event_check(NRF_SAADC_EVENT_END));
    //    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    //
    //    nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
    //    while(!nrf_saadc_event_check(NRF_SAADC_EVENT_STOPPED));
    //    nrf_saadc_event_clear(NRF_SAADC_EVENT_STOPPED);
    //
    //    printf("ADC result = 0x%08x\n",result);
    //    nrf_delay_ms(500);
    //
    //}

    uint32_t local_counter = 0;
    uint32_t iter = 0;
    while(1) {

        if(irq_triggered==1) { // The UART is also generating
                               // interrupt so we don't want extra
                               // conversion
            irq_triggered=0;
            iter++;
            result = 0;
            nrf_saadc_task_trigger(NRF_SAADC_TASK_START);

            while(!nrf_saadc_event_check(NRF_SAADC_EVENT_STARTED));
            nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);

            nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);


            while(!nrf_saadc_event_check(NRF_SAADC_EVENT_END));
            nrf_saadc_event_clear(NRF_SAADC_EVENT_END);

            nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
            while(!nrf_saadc_event_check(NRF_SAADC_EVENT_STOPPED));
            nrf_saadc_event_clear(NRF_SAADC_EVENT_STOPPED);

            printf("%08lu ADC result = 0x%08x (irq=%d)\n",iter,result,irq_counter);


        }

        __WFI();
    };
}
