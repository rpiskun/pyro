#include <stdbool.h>
#include "system_conf.h"
#include "rtc.h"
#include "pyd1588.h"
#include "pyro_fsm.h"
#include "uart.h"
#include "main.h"

#define ADC_MEASURE_BUF_SIZE    (16u)

#define ADC_SLEEP_HIGH_TRESHOLD (70)
#define ADC_SLEEP_LOW_TRESHOLD  (-70)

/* time in ms before go to sleep */
#define GOTO_SLEEP_TIMEOUT      (2000)

#define UART_BUF_SIZE           (32u)

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
static int16_t adc_average = 0;
static uint8_t uart_buf[UART_BUF_SIZE];

static void error_handler(void);
static int system_init(void);
static int wakeup_init(void);
static void GotoStopMode(void);
static void Main_Fsm(void);
static void ForceReadDataHandler(void);
static inline int16_t GetAdcAverage(void);

static inline __attribute__((always_inline)) bool IsTimeElapsed(uint32_t first_tick, uint32_t timeout)
{
    bool retval = false;

    uint32_t last_tick = HAL_GetTick();
    uint32_t diff = (last_tick > first_tick) ? (last_tick - first_tick) : (first_tick - last_tick + 1);

    if (diff > timeout) {
        retval = true;
    }

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

        // status = Clock_InitMsi();
        // status = Clock_InitHsi();
        status = Clock_InitHsiMax();
        if (status < 0) {
            break;
        }

        status = Power_Init();
        if (status < 0) {
            break;
        }


        status = Gpio_Init();
        if (status < 0) {
            break;
        }

        status = Rtc_Init();
        if (status < 0) {
            break;
        }

        status = Uart_Init();
        if (status < 0) {
            break;
        }
    } while (0);

    return status;
}

static int wakeup_init(void)
{
    int retval = 0;

    (void)Clock_InitHsiMax();
    /* resume systick timer */
    HAL_ResumeTick();
    (void)Uart_Init();

    return retval;
}


static void error_handler(void)
{
    __disable_irq();
    while (1)
    {
        /* put some error handling here */
    }
}

static void GotoStopMode(void)
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
    bool time_elapsed = false;
    // int16_t adc_average = 0;

    switch (fsm_state) {
    case E_MAIN_STATE_INIT:
        Pyro_UpdateConf(pyro_conf);
        fsm_state = E_MAIN_STATE_UPDATE_CONF;
        break;

    case E_MAIN_STATE_UPDATE_CONF:
        conf_updated = Pyro_IsConfUpdated();
        if (conf_updated) {
            (void)Pyro_StartAdcRead();
            first_tick = HAL_GetTick();
            fsm_state = E_MAIN_STATE_FORCE_READ;
        }
        break;

    case E_MAIN_STATE_FORCE_READ:
        ForceReadDataHandler();
        adc_average = GetAdcAverage();
        if (Uart_IsTxReady()) {
            uart_buf[0] = 'h';
            uart_buf[1] = 'e';
            uart_buf[2] = 'l';
            uart_buf[3] = 'l';
            uart_buf[4] = 'o';
            uart_buf[5] = '\r';
            uart_buf[6] = '\n';
            Uart_Send(uart_buf, 7);
        }
        if ( (adc_average < ADC_SLEEP_LOW_TRESHOLD) ||
             (adc_average > ADC_SLEEP_HIGH_TRESHOLD) ) {
            /* don't go to sleep if signal value is too high/low */
            first_tick = HAL_GetTick();
        }

        time_elapsed = IsTimeElapsed(first_tick, GOTO_SLEEP_TIMEOUT);
        if (time_elapsed) {
            fsm_state = E_MAIN_STATE_SLEEP;
        }
        break;

    case E_MAIN_STATE_SLEEP:
        PYD_EnableWakeupEvent();
        GotoStopMode();
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
        time_elapsed = IsTimeElapsed(first_tick, blind_timeout);
        if (time_elapsed) {
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

