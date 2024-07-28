#include <stdbool.h>
#include "system_init.h"
#include "rtc.h"
#include "pyd1588.h"
#include "pyro_fsm.h"
#include "main.h"

static void error_handler(void);
static int system_init(void);
static int wakeup_init(void);
static void enter_stop_mode(void);

int main(void)
{
    int status = 0;
    bool conf_updated = false;
    bool irq_enabled = false;
    union Pyd1588Config pyro_conf = {
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

    status = system_init();
    if (status < 0) {
        error_handler();
    }

    status = Pyro_Init();
    if (status < 0) {
        error_handler();
    }

    Pyro_UpdateConf(pyro_conf);
    // (void)Pyro_StartAdcRead();

    while(1) {
        Pyro_Fsm();

        conf_updated = Pyro_IsConfUpdated();
        if (!irq_enabled && conf_updated) {
            PYD_EnableWakeupEvent();
            enter_stop_mode();
            wakeup_init();
            irq_enabled = true;
        }
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
