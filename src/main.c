#include <stdbool.h>
#include "system_init.h"
#include "rtc.h"
#include "pyd1588.h"
#include "main.h"

#define PYRO_DELAY_TIMEOUT      (160000)

enum PyroReadState {
    E_PYRO_WRITE_IDLE = 0,
    E_PYRO_WAIT_WRITE,
    E_PYRO_WAIT_CONFIG_APPLIED,
    E_PYRO_READ_IDLE,
    E_PYRO_WAIT_READ,
    E_PYRO_DELAY,
};

static enum PyroReadState pyro_state = E_PYRO_READ_IDLE;
static struct Pyd1588RxData pyro_rx_data = { 0 };
static union Pyd1588Config pyro_rx_conf = { 0 };
static int32_t pyro_delay = 0;

static void error_handler(void);
static int system_init(void);

int main(void)
{
    int status = 0;
    bool pyro_ready = false;
    union Pyd1588Config pyro_conf = {
        .fields.threshold = 200u,
        .fields.blind_time = 3u,
        .fields.pulse_counter = 2u,
        .fields.window_time = 1u,
        .fields.operating_modes = PYD1588_OPERATION_MODE_FORCED_READOUT,
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

    (void)Pyro_WriteAsync(Pyd1588DefaultConfig.word);

    while(1) {
        switch (pyro_state)
        {
        case E_PYRO_WRITE_IDLE:
            pyro_ready = Pyro_IsReady();
            if (pyro_ready) {
                Pyro_WriteAsync(pyro_conf.word);
                pyro_state = E_PYRO_WAIT_WRITE;
            }
            break;

        case E_PYRO_WAIT_WRITE:
            pyro_ready = Pyro_IsReady();
            if (pyro_ready) {
                pyro_delay = PYRO_DELAY_TIMEOUT;
                pyro_state = E_PYRO_WAIT_CONFIG_APPLIED;
            }
            break;

        case E_PYRO_WAIT_CONFIG_APPLIED:
            if (--pyro_delay < 0) {
                pyro_state = E_PYRO_READ_IDLE;
            }
            break;

        case E_PYRO_READ_IDLE:
            pyro_ready = Pyro_IsReady();
            if (pyro_ready) {
                Pyro_ReadAsync(E_RX_FRAME_ADC);
                // Pyro_ReadAsync(E_RX_FRAME_FULL);
                pyro_state = E_PYRO_WAIT_READ;
            }
            break;

        case E_PYRO_WAIT_READ:
            pyro_ready = Pyro_IsReady();
            if (pyro_ready) {
                (void)Pyro_ReadRxData(&pyro_rx_data);
                // pyro_rx_conf.word = pyro_rx_data.conf;
                /* ================== */
                pyro_delay = PYRO_DELAY_TIMEOUT;
                /* ================== */
                pyro_state = E_PYRO_DELAY;
            }
            break;

        case E_PYRO_DELAY:
            if (--pyro_delay < 0) {
                pyro_state = E_PYRO_WRITE_IDLE;
            }
            break;
        }
    }
    return 0;
}

int system_init(void)
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

static void error_handler(void)
{
    __disable_irq();
    while (1)
    {
        /* put some error handling here */
    }
}

