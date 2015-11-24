// Simple PWM code that will output a 1MHz PWM signal with a
// variable duty cycle on pin 19
// Note : you must have a PCA10040 board
// This will not run on a PCA10036 board

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf.h"
#include "bsp.h"

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


#define PIN_LED3 19

int main(void)
{

#define PWM_COUNTER_TOP 16000 // 16MHz divided by 16000-> 1ms

    volatile uint16_t pwm_seq;

    nrf_gpio_cfg_output(PIN_LED3);
    nrf_gpio_pin_set(PIN_LED3); // LED is off

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



    {
        uint32_t duty_cycle = 0;
        int32_t  duty_cycle_inc = 1;

        while (1) {
            duty_cycle += duty_cycle_inc;
            if(duty_cycle == 100) duty_cycle_inc = -1;
            if(duty_cycle ==  2) duty_cycle_inc = +1;

            pwm_seq = ((uint16_t) ((PWM_COUNTER_TOP*duty_cycle)/100)); // pwm_seq is read by the PWM
            // when task SEQSTARTx is triggered

            nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0);


            nrf_delay_ms(10);

        }
    }

}
