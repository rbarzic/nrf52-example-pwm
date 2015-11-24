// Unfortunatly, there is no pwm-related hal file in the nrf52 SDK yet
// so we make our own

#pragma once


#include <stdint.h>


#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"

/**
 * @enum nrf_pwm_task_t
 * @brief Pwm tasks.
 */
typedef enum
{
    /*lint -save -e30 -esym(628,__INTADDR__)*/
    // NRF_PWM_TASK_START    = offsetof(NRF_PWM_Type, TASKS_START),
    NRF_PWM_TASK_STOP     = offsetof(NRF_PWM_Type, TASKS_STOP),
    NRF_PWM_TASK_SEQSTART0 = offsetof(NRF_PWM_Type, TASKS_SEQSTART[0]),
    NRF_PWM_TASK_SEQSTART1 = offsetof(NRF_PWM_Type, TASKS_SEQSTART[1]),
    NRF_PWM_TASK_NEXTSTEP = offsetof(NRF_PWM_Type, TASKS_NEXTSTEP),
    /*lint -restore*/
} nrf_pwm_task_t;


/**
 * @enum nrf_pwm_event_t
 * @brief Pwm events.
 */
typedef enum
{
    /*lint -save -e30*/
    NRF_PWM_EVENT_STOPPED = offsetof(NRF_PWM_Type, EVENTS_STOPPED),
    NRF_PWM_EVENT_SEQSTARTED0 = offsetof(NRF_PWM_Type, EVENTS_SEQSTARTED[0]),
    NRF_PWM_EVENT_SEQSTARTED1 = offsetof(NRF_PWM_Type, EVENTS_SEQSTARTED[1]),
    NRF_PWM_EVENT_SEQEND0 = offsetof(NRF_PWM_Type, EVENTS_SEQEND[0]),
    NRF_PWM_EVENT_SEQEND1 = offsetof(NRF_PWM_Type, EVENTS_SEQEND[1]),
    NRF_PWM_EVENT_PWNPERIODEND = offsetof(NRF_PWM_Type, EVENTS_PWMPERIODEND),
    NRF_PWM_EVENT_LOOPSDONE = offsetof(NRF_PWM_Type, EVENTS_LOOPSDONE),
    /*lint -restore*/
} nrf_pwm_event_t;

/**
 * @enum nrf_pwm_channel_t
 * @brief Pwm channels.
 */
typedef enum
{
    NRF_PWM_CHANNEL0 = 0, /**< Pwm channel 0. */
    NRF_PWM_CHANNEL1,     /**< Pwm channel 1. */
    NRF_PWM_CHANNEL2,     /**< Pwm channel 2. */
    NRF_PWM_CHANNEL3      /**< Pwm channel 3. */
} nrf_pwm_channel_t;


/**
 * @enum nrf_pwm_channel_t
 * @brief Pwm channels.
 */
typedef enum
{
    NRF_PWM_PRESCALER_DIV_1=0,  // 0 Divide by 1 ( 16MHz)
    NRF_PWM_PRESCALER_DIV_2  ,  // 1 Divide by 2 ( 8MHz)
    NRF_PWM_PRESCALER_DIV_4,    // 2 Divide by 4 ( 4MHz)
    NRF_PWM_PRESCALER_DIV_8,    // 3 Divide by 8 ( 2MHz)
    NRF_PWM_PRESCALER_DIV_16,   // 4 Divide by 16 ( 1MHz)
    NRF_PWM_PRESCALER_DIV_32,   // 5 Divide by 32 ( 500kHz)
    NRF_PWM_PRESCALER_DIV_64,   // 6 Divide by 64 ( 250kHz)
    NRF_PWM_PRESCALER_DIV_128,  // 7 Divide by 128 ( 125kHz)
} nrf_pwm_prescaler_t;




/**
 * @enum nrf_pwm_mode_t
 * @brief Pwm modes.
 */
typedef enum
{
    NRF_PWM_MODE_UP = 0, /**< Pwm mode 0. */
    NRF_PWM_MODE_UPANDDOWN,     /**< Pwm mode 1. */
} nrf_pwm_mode_t;


/**
 * @enum nrf_pwm_decoder_load_t
 * @brief Pwm DECODER load field value.
 */
typedef enum
{
    NRF_PWM_DECODER_LOAD_COMMON = 0, // 0
    NRF_PWM_DECODER_LOAD_GROUPED,    // 1
    NRF_PWM_DECODER_LOAD_INDIVIDUAL, // 2
    NRF_PWM_DECODER_LOAD_WAVEFORM ,  // 3

} nrf_pwm_decoder_load_t;


/**
 * @enum nrf_pwm_decoder_mode_t
 * @brief Pwm DECODER mode field value.
 */
typedef enum
{
    NRF_PWM_DECODER_MODE_REFRESHCOUNT = 0, // 0
    NRF_PWM_DECODER_MODE_NEXTSTEP,        // 1

} nrf_pwm_decoder_mode_t;

/**
 * @enum nrf_pwm_sequence_t
 * @brief Pwm DECODER sequence number.
 */
typedef enum
{
    NRF_PWM_SEQUENCE0 = 0, // 0
    NRF_PWM_SEQUENCE1,     // 1

} nrf_pwm_sequence_t;





// Helper functions


/**
 * @brief Function for activating a specific pwm task.
 *
 * @param NRF_PWMx Pwm instance.
 *
 * @param pwm_task Pwm task.
 */
__STATIC_INLINE void nrf_pwm_task_trigger(NRF_PWM_Type * NRF_PWMx, nrf_pwm_task_t pwm_task)
{
    *((volatile uint32_t *)((uint8_t *)NRF_PWMx + (uint32_t)pwm_task)) = 0x1UL;
}


/**
 * @brief Function for clearing a specific pwm event.
 *
 * @param NRF_PWMx Pwm instance.
 *
 * @param pwm_event Pwm event to clear.
 */
__STATIC_INLINE void nrf_pwm_event_clear(NRF_PWM_Type * NRF_PWMx,
                                           nrf_pwm_event_t pwm_event)
{
    *((volatile uint32_t *)((uint8_t *)NRF_PWMx + (uint32_t)pwm_event)) = 0x0UL;
}

/**
 * @brief Function for returning the state of a specific event.
 *
 * @param NRF_PWMx Pwm instance.
 *
 * @param pwm_event Pwm event to check.
 */
__STATIC_INLINE bool nrf_pwm_event_check(NRF_PWM_Type * NRF_PWMx,
                                           nrf_pwm_event_t pwm_event)
{
    return (bool)*(volatile uint32_t *)((uint8_t *)NRF_PWMx + (uint32_t)pwm_event);
}


/**
 * @brief Function for setting pin for a specific channel
 *
 * @param NRF_PWMx Pwm instance.
 *
 * @param channel Channel number
 *
 * @param pin_number Pin number.
 */
__STATIC_INLINE void nrf_pwm_pselout_set(NRF_PWM_Type * NRF_PWMx, nrf_pwm_channel_t channel,uint32_t pin_number)
{
    NRF_PWMx->PSEL.OUT[channel] = (pin_number << PWM_PSEL_OUT_PIN_Pos) |
        (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
}

/**
 * @brief Function to enable a PWM
 *
 * @param NRF_PWMx Pwm instance.
 *
 */
__STATIC_INLINE void nrf_pwm_enable(NRF_PWM_Type * NRF_PWMx)
{
    NRF_PWMx->ENABLE = 1;
}

/**
 * @brief Function to disable a PWM
 *
 * @param NRF_PWMx Pwm instance.
 *
 */
__STATIC_INLINE void nrf_pwm_disable(NRF_PWM_Type * NRF_PWMx)
{
    NRF_PWMx->ENABLE = 0;
}

/**
 * @brief Function for setting the counter mode
 *
 * @param NRF_PWMx Pwm instance.
 *
 *
 * @param mode Counter mode.
 */
__STATIC_INLINE void nrf_pwm_mode_set(NRF_PWM_Type * NRF_PWMx, nrf_pwm_mode_t mode)
{
    NRF_PWMx->MODE = mode;
}

/**
 * @brief Function for setting the prescaler
 *
 * @param NRF_PWMx Pwm instance.
 *
 *
 * @param mode Prescaler mode.
 */
__STATIC_INLINE void nrf_pwm_prescaler_set(NRF_PWM_Type * NRF_PWMx, nrf_pwm_prescaler_t prescaler)
{
    NRF_PWMx->PRESCALER = prescaler;
}


/**
 * @brief Function for setting the COUNTERTOP register
 *
 * @param NRF_PWMx Pwm instance.
 *
 *
 * @param countertop COUNTERTOP value.
 */
__STATIC_INLINE void nrf_pwm_countertop_set(NRF_PWM_Type * NRF_PWMx, uint32_t countertop)
{
    NRF_PWMx->COUNTERTOP = countertop;
}


/**
 * @brief Function for setting the LOOP register
 *
 * @param NRF_PWMx Pwm instance.
 *
 *
 * @param loop LOOP value.
 */
__STATIC_INLINE void nrf_pwm_loop_set(NRF_PWM_Type * NRF_PWMx, uint32_t loop)
{
    NRF_PWMx->LOOP = loop;
}


/**
 * @brief Function for setting the DECODER register
 *
 * @param NRF_PWMx Pwm instance.
 *
 *
 * @param load LOAD value.

 * @param mode MODE value.
 */
__STATIC_INLINE void nrf_pwm_decoder_set(NRF_PWM_Type * NRF_PWMx,
                                      nrf_pwm_decoder_load_t load,
                                      nrf_pwm_decoder_mode_t mode)
{
    NRF_PWM0->DECODER = (load << PWM_DECODER_LOAD_Pos) |
        (mode << PWM_DECODER_MODE_Pos);

}



/**
 * @brief Function for setting the SEQ register
 *
 * @param NRF_PWMx Pwm instance.
 *
 *
 * @param seq Sequence number.
 *
 * @param ptr Beginning address in Data RAM
 *
 * @param cnt Amount of values (duty cycles)
 *
 * @param refresh Amount of additional PWM periods between samples loaded to compare register (load every CNT+1 PWM periods)
 *
 * @param enddelay Time added after the sequence in PWM periods
 *
 */
__STATIC_INLINE void nrf_pwm_seq_set(NRF_PWM_Type * NRF_PWMx,
                                     nrf_pwm_sequence_t seq,
                                     uint32_t ptr,
                                     uint32_t cnt,
                                     uint32_t refresh,
                                     uint32_t enddelay)
{
    NRF_PWMx->SEQ[seq].PTR = ptr;
    NRF_PWMx->SEQ[seq].CNT = cnt;
    NRF_PWMx->SEQ[seq].REFRESH = refresh;
    NRF_PWMx->SEQ[seq].ENDDELAY = enddelay;
}
