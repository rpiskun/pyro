#include "system_init.h"
#include "rtc.h"
#include "main.h"

static void error_handler(void);
static int system_init(void);

int main(void)
{
    int status = 0;

    status = system_init();
    if (status != 0) {
        error_handler();
    }

    while(1) {
    }

    return 0;
}

int system_init(void)
{
    int status = 0;

    do {
        HAL_Init();

        status = clock_init();
        if (status != 0) {
            break;
        }

        status = gpio_init();
        if (status != 0) {
            break;
        }

        status = rtc_init();
        if (status != 0) {
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

