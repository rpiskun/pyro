#include <stdbool.h>
#include <stdint.h>
#include "pyro_fsm.h"

#define PYRO_CONF_APPLY_DELAY   (3u)
#define PYRO_READY_TIMEOUT      (6u)
#define PYRO_READ_RETRIES       (3u)
#define PYRO_CONF_CHECK_RETRIES (3u)

enum PyroFsmState {
    E_PYRO_STATE_CONF_INIT = 0,
    E_PYRO_STATE_CONF_WAIT_FOR_WRITE,
    E_PYRO_STATE_CONF_WRITE,
    E_PYRO_STATE_CONF_WAIT_FOR_APPLYING,
    E_PYRO_STATE_CONF_WAIT_FOR_READ,
    E_PYRO_STATE_CONF_READ,
    E_PYRO_STATE_CONF_WAIT_FOR_CHECK,
    E_PYRO_STATE_CONF_CHECK,
    E_PYRO_STATE_CONF_READY,
};

struct TxConfig {
    union Pyd1588Config conf;
    bool update_requested;
};

struct RxConfig {
    union Pyd1588Config conf;
    bool is_ready;
};

static enum PyroFsmState pyro_fsm_state = E_PYRO_STATE_CONF_INIT;

static struct TxConfig tx_conf;
static struct RxConfig rx_conf;

static inline __attribute__((always_inline)) uint32_t abs(int32_t v)
{
    uint32_t retval = 0;
    int32_t const mask = v >> 31;

    retval = (v + mask) ^ mask;

    return retval;
}

void Pyro_FSM(void)
{
    bool pyro_ready = false;
    struct Pyd1588RxData rx_data = { 0 };
    static uint32_t first_tick = 0;
    static uint32_t read_retries_cnt = 0;
    static uint32_t conf_check_cnt = 0;

    /* config update request has high prio - handle it first */
    if (tx_conf.update_requested) {
        pyro_fsm_state = E_PYRO_STATE_CONF_WAIT_FOR_WRITE;
        tx_conf.update_requested = false;
        first_tick = HAL_GetTick();
    }

    switch (pyro_fsm_state) {
    case E_PYRO_STATE_CONF_WAIT_FOR_WRITE:
        pyro_ready = Pyro_IsReady();
        if (pyro_ready) {
            pyro_fsm_state = E_PYRO_STATE_CONF_WRITE;
        } else {
            uint32_t last_tick = HAL_GetTick();
            uint32_t time_elapsed = abs((last_tick - first_tick));
            if (time_elapsed > PYRO_READY_TIMEOUT) {
                /* pyd driver is not ready for a long time - try to force write */
                pyro_fsm_state = E_PYRO_STATE_CONF_WRITE;
            }
        }
        break;

    case E_PYRO_STATE_CONF_WRITE:
        /* force write */
        Pyro_WriteAsync(tx_conf.conf.word);
        rx_conf.is_ready = false;
        conf_check_cnt = 0;
        /* store start time */
        first_tick = HAL_GetTick();
        pyro_fsm_state = E_PYRO_STATE_CONF_WAIT_FOR_APPLYING;
        break;

    case E_PYRO_STATE_CONF_WAIT_FOR_APPLYING:;
        uint32_t last_tick = HAL_GetTick();
        uint32_t time_elapsed = abs((last_tick - first_tick));
        if (time_elapsed > PYRO_CONF_APPLY_DELAY) {
            first_tick = HAL_GetTick();
            read_retries_cnt = 0;
            pyro_fsm_state = E_PYRO_STATE_CONF_WAIT_FOR_READ;
        }
        break;

    case E_PYRO_STATE_CONF_WAIT_FOR_READ:
        pyro_ready = Pyro_IsReady();
        if (pyro_ready) {
            pyro_fsm_state = E_PYRO_STATE_CONF_READ;
        } else {
            uint32_t last_tick = HAL_GetTick();
            uint32_t time_elapsed = abs((last_tick - first_tick));
            if (time_elapsed > PYRO_READY_TIMEOUT) {
                /* pyd driver is not ready for a long time - try to force read */
                pyro_fsm_state = E_PYRO_STATE_CONF_READ;
            }
        }
        break;

    case E_PYRO_STATE_CONF_READ:
        Pyro_ReadAsync(E_RX_FRAME_FULL);
        /* store start time */
        first_tick = HAL_GetTick();
        pyro_fsm_state = E_PYRO_STATE_CONF_WAIT_FOR_CHECK;
        break;

    case E_PYRO_STATE_CONF_WAIT_FOR_CHECK:
        pyro_ready = Pyro_IsReady();
        if (pyro_ready) {
            pyro_fsm_state = E_PYRO_STATE_CONF_CHECK;
        } else {
            uint32_t last_tick = HAL_GetTick();
            uint32_t time_elapsed = abs((last_tick - first_tick));
            if (time_elapsed > PYRO_READY_TIMEOUT) {
                if (read_retries_cnt < PYRO_READ_RETRIES) {
                    /* pyd driver is not ready for a long time - try to force read */
                    pyro_fsm_state = E_PYRO_STATE_CONF_READ;
                } else {
                    /* lets start from config write */
                    pyro_fsm_state = E_PYRO_STATE_CONF_WRITE;
                }
                read_retries_cnt++;
            }
        }
        break;

    case E_PYRO_STATE_CONF_CHECK:
        (void)Pyro_GetRxData(&rx_data);
        if (rx_data.conf.word == tx_conf.conf.word) {
            /* update cached rx conf */
            rx_conf.conf.word = rx_data.conf.word;
            rx_conf.is_ready = true;

            pyro_fsm_state = E_PYRO_STATE_CONF_READY;
        } else {
            if (conf_check_cnt < PYRO_CONF_CHECK_RETRIES) {
                pyro_fsm_state = E_PYRO_STATE_CONF_READ;
            } else {
                /* number of retries exceeded - lets write config again */
                pyro_fsm_state = E_PYRO_STATE_CONF_WRITE;
            }
            conf_check_cnt++;
        }
        break;

    case E_PYRO_STATE_CONF_READY:
        break;

    default:
        break;
    }
}

int Pyro_UpdateConf(union Pyd1588Config pyro_conf)
{
    int retval = 0;

    tx_conf.conf.word = pyro_conf.word;
    tx_conf.update_requested = true;

    return retval;
}
