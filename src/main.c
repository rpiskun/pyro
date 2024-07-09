#include <stdbool.h>
#include "system_init.h"
#include "rtc.h"
#include "pyd1588.h"
#include "main.h"

enum PyroReadState {
    E_PYRO_READ_IDLE = 0,
    E_PYRO_WAIT_READ,
};

static enum PyroReadState pyro_state = E_PYRO_READ_IDLE;
static union Pyd1588RxData pyro_rx_data = { 0 };
static union Pyd1588Config pyro_rx_conf = { 0 };

static void error_handler(void);
static int system_init(void);

int main(void)
{
    int status = 0;
    bool pyro_ready = false;

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
        case E_PYRO_READ_IDLE:
            pyro_ready = Pyro_IsReady();
            if (pyro_ready) {
                Pyro_ReadAsync();
                pyro_state = E_PYRO_WAIT_READ;
            }
            break;

        case E_PYRO_WAIT_READ:
            pyro_ready = Pyro_IsReady();
            if (pyro_ready) {
                (void)Pyro_ReadRxData(&pyro_rx_data);
                pyro_rx_conf.word = pyro_rx_data.fields.conf;
                pyro_state = E_PYRO_READ_IDLE;
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

        status = clock_init();
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

