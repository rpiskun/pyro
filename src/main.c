#include <stdbool.h>
#include "system_init.h"
#include "rtc.h"
#include "pyd1588.h"
#include "pyro_fsm.h"
#include "main.h"

#define ADC_MEASURE_BUF_SIZE    (16u)

#define ADC_SLEEP_HIGH_TRESHOLD (70)
#define ADC_SLEEP_LOW_TRESHOLD  (-70)

/* time in ms before go to sleep */
#define GOTO_SLEEP_TIMEOUT      (50)

enum MainFsmState {
    E_MAIN_STATE_INIT = 0,
    E_MAIN_STATE_UPDATE_CONF,
    E_MAIN_STATE_FORCE_READ,
    E_MAIN_STATE_SLEEP,
    E_MAIN_STATE_HANDLE_INTERRUPT,
    E_MAIN_STATE_BLIND_DELAY,
};

struct AdcMeasure {
    int16_t adc_buf[ADC_MEASURE_BUF_SIZE];
    int32_t adc_sum;
    uint8_t window_idx;
};

static enum MainFsmState fsm_state = E_MAIN_STATE_INIT;
static union Pyd1588Config pyro_conf = {
    .fields.threshold = 50,
    .fields.blind_time = 3u,
    .fields.pulse_counter = 2u,
    .fields.window_time = 1u,
    .fields.operating_modes = PYD1588_OPERATION_MODE_WAKEUP,
    .fields.signal_source = PYD1588_SIGNAL_SOURCE_BPF,
    .fields.reserved2 = PYD1588_RESERVED2,
    .fields.hpf_cutoff = PYD1588_HPF_CUTOFF_0_4HZ,
    .fields.reserved1 = PYD1588_RESERVED1,
    .fields.count_mode = PYD1588_COUNT_MODE_WITH_BPF,
};

static struct AdcMeasure adc_data = { 0 };

static void error_handler(void);
static int system_init(void);
static int wakeup_init(void);
static void enter_stop_mode(void);
static void Main_Fsm(void);
static void ForceReadDataHandler(void);
static inline int16_t GetAdcAverage(void);

static inline __attribute__((always_inline)) uint32_t abs(int32_t v)
{
    uint32_t retval = 0;
    int32_t const mask = v >> 31u;

    retval = (v + mask) ^ mask;

    return retval;
}


int main(void)
{
    int status = 0;

    status = system_init();
    if (status < 0) {
        error_handler();
    }

    status = Pyro_Init();
    if (status < 0) {
        error_handler();
    }

    while(1) {
        Pyro_Fsm();
        Main_Fsm();
    }
    return 0;
}

static int system_init(void)
{
    int status = 0;

    do {
        HAL_Init();

        // status = clock_init_msi();
        // status = clock_init_hsi();
        status = clock_init_max();
        if (status < 0) {
            break;
        }

        status = power_init();
        if (status < 0) {
            break;
        }


        status = gpio_init();
        if (status < 0) {
            break;
        }

        status = rtc_init();
        if (status < 0) {
            break;
        }
    } while (0);

    return status;
}

static int wakeup_init(void)
{
    int status = 0;

    do {
        status = clock_init_max();
        if (status < 0) {
            break;
        }
        /* resume systick timer */
        HAL_ResumeTick();
    } while (0);

    return status;
}


static void error_handler(void)
{
    __disable_irq();
    while (1)
    {
        /* put some error handling here */
    }
}

static void enter_stop_mode(void)
{
    /* suspend systick timer */
    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE);
    // HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}

static void Main_Fsm(void)
{
    static uint32_t first_tick = 0;
    static uint32_t blind_timeout = 0;

    bool conf_updated = false;
    bool sensor_ready = false;
    int16_t adc_average = 0;
    uint32_t last_tick = 0;
    uint32_t time_elapsed = 0;

    switch (fsm_state) {
    case E_MAIN_STATE_INIT:
        Pyro_UpdateConf(pyro_conf);
        fsm_state = E_MAIN_STATE_UPDATE_CONF;
        break;

    case E_MAIN_STATE_UPDATE_CONF:
        conf_updated = Pyro_IsConfUpdated();
        if (conf_updated) {
            (void)Pyro_StartAdcRead();
            fsm_state = E_MAIN_STATE_FORCE_READ;
        }
        break;

    case E_MAIN_STATE_FORCE_READ:
        ForceReadDataHandler();
        adc_average = GetAdcAverage();
        if ( (adc_average < ADC_SLEEP_LOW_TRESHOLD) ||
             (adc_average > ADC_SLEEP_LOW_TRESHOLD) ) {
            first_tick = HAL_GetTick();
        }

        last_tick = HAL_GetTick();
        time_elapsed = abs((last_tick - first_tick));
        if (time_elapsed > GOTO_SLEEP_TIMEOUT) {
            fsm_state = E_MAIN_STATE_SLEEP;
        }
        break;

    case E_MAIN_STATE_SLEEP:
        PYD_EnableWakeupEvent();
        enter_stop_mode();
        wakeup_init();
        (void)PYD_HandleIrq();
        fsm_state = E_MAIN_STATE_HANDLE_INTERRUPT;
        break;

    case E_MAIN_STATE_HANDLE_INTERRUPT:
        sensor_ready = PYD_IsReady();
        if (sensor_ready) {
            first_tick = HAL_GetTick();
            blind_timeout = (1 + pyro_conf.fields.blind_time) / 2 + 1;
            fsm_state = E_MAIN_STATE_BLIND_DELAY;
        }
        break;

    case E_MAIN_STATE_BLIND_DELAY:
        last_tick = HAL_GetTick();
        time_elapsed = abs((last_tick - first_tick));
        if (time_elapsed > blind_timeout) {
            fsm_state = E_MAIN_STATE_INIT;
        }
        break;

    }
}

static void ForceReadDataHandler(void)
{
    bool adc_updated = false;
    int16_t adc_val = 0;

    do {
        adc_updated = Pyro_GetAdcValue(&adc_val);
        if (adc_updated) {
            adc_data.adc_sum -= adc_data.adc_buf[adc_data.window_idx];
            adc_data.adc_sum += adc_val;

            adc_data.adc_buf[adc_data.window_idx] = adc_val;

            if (++adc_data.window_idx >= ADC_MEASURE_BUF_SIZE) {
                adc_data.window_idx = 0;
            }
        }
    } while (adc_updated);
}

static inline __attribute__((always_inline)) int16_t GetAdcAverage(void)
{
    return (adc_data.adc_sum / ADC_MEASURE_BUF_SIZE);
}

