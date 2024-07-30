#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "pyro_fsm.h"

#define PYRO_CONF_APPLY_DELAY   (3u)
#define PYRO_READY_TIMEOUT      (6u)
#define PYRO_READ_RETRIES       (2u)
#define PYRO_CONF_CHECK_RETRIES (2u)
#define PYRO_ADC_READ_DELAY     (75u)

#define ADC_BUF_SIZE            (4u)

enum Pyro_ConfUpdateState {
    E_PYRO_CONF_UPD_INIT = 0,
    E_PYRO_CONF_UPD_WAIT_FOR_WRITE,
    E_PYRO_CONF_UPD_WRITE,
    E_PYRO_CONF_UPD_WAIT_FOR_APPLYING,
    E_PYRO_CONF_UPD_WAIT_FOR_READ,
    E_PYRO_CONF_UPD_READ,
    E_PYRO_CONF_UPD_WAIT_FOR_CHECK,
    E_PYRO_CONF_UPD_CHECK,
    E_PYRO_CONF_UPD_READY,
};

enum Pyro_AdcReadState {
    E_PYRO_ADC_READ_INIT = 0,
    E_PYRO_ADC_READ_WAIT_READY,
    E_PYRO_ADC_READ_REQUEST,
    E_PYRO_ADC_READ_WAIT_DATA,
    E_PYRO_ADC_READ_DELAY,
};

enum Pyro_ReadyState {
    E_PYRO_READY_STATE_PENDING = 0,
    E_PYRO_READY_STATE_OK,
    E_PYRO_READY_STATE_TIMEOUT,
};

enum Pyro_BasicState {
    E_PYRO_BASIC_IDLE = 0,
    E_PYRO_BASIC_UPDATE_CONF,
    E_PYRO_BASIC_READ_ADC
};

struct TxConfig {
    union Pyd1588Config conf;
    bool update_requested;
};

struct RxConfig {
    union Pyd1588Config conf;
    bool is_updated;
};

struct AdcData {
    struct AdcInstantValue adc_buf[ADC_BUF_SIZE];
    uint8_t head;
    uint8_t tail;
    bool read_adc;
    bool read_requested;
};

static enum Pyro_ConfUpdateState pyro_conf_update_state = E_PYRO_CONF_UPD_INIT;
static enum Pyro_BasicState pyro_basic_state = E_PYRO_BASIC_IDLE;
static enum Pyro_AdcReadState pyro_adc_state = E_PYRO_ADC_READ_INIT;

static struct TxConfig tx_conf = { 0 };
static struct RxConfig rx_conf = { 0 };
static struct AdcData adc_data = { 0 };

static void Pyro_ConfUpdateFsm(void);
static void Pyro_AdcReadFsm(void);

static inline __attribute__((always_inline)) bool IsTimeElapsed(uint32_t first_tick, uint32_t timeout)
{
    bool retval = false;

    uint32_t last_tick = HAL_GetTick();
    uint32_t diff = (last_tick >= first_tick) ? (last_tick - first_tick) : (first_tick - last_tick + 1);

    if (diff > timeout) {
        retval = true;
    }

    return retval;
}

static inline __attribute__((always_inline)) enum Pyro_ReadyState IsSensorReady(uint32_t first_tick, uint32_t timeout)
{
    enum Pyro_ReadyState retval = E_PYRO_READY_STATE_PENDING;
    bool pyro_ready = false;

    pyro_ready = PYD_IsReady();
    if (pyro_ready) {
        retval = E_PYRO_READY_STATE_OK;
    } else {
        bool time_elapsed = IsTimeElapsed(first_tick, timeout);
        if (time_elapsed) {
            retval = E_PYRO_READY_STATE_TIMEOUT;
        }
    }

    return retval;
}

static void Pyro_ConfUpdateFsm(void)
{
    enum Pyro_ReadyState pyro_ready = E_PYRO_READY_STATE_PENDING;
    struct Pyd1588RxData rx_data = { 0 };
    static uint32_t first_tick = 0;
    static uint32_t read_retries_cnt = 0;
    static uint32_t conf_check_cnt = 0;

    if (tx_conf.update_requested) {
        pyro_conf_update_state = E_PYRO_CONF_UPD_WAIT_FOR_WRITE;
        tx_conf.update_requested = false;
        first_tick = HAL_GetTick();
    }

    switch (pyro_conf_update_state) {
    case E_PYRO_CONF_UPD_WAIT_FOR_WRITE:
        pyro_ready = IsSensorReady(first_tick, PYRO_READY_TIMEOUT);
        if ( (pyro_ready == E_PYRO_READY_STATE_OK) ||
             (pyro_ready == E_PYRO_READY_STATE_TIMEOUT) ) {
            pyro_conf_update_state = E_PYRO_CONF_UPD_WRITE;
        }
        break;

    case E_PYRO_CONF_UPD_WRITE:
        /* force write */
        PYD_WriteAsync(tx_conf.conf.word);
        conf_check_cnt = 0;
        /* store start time */
        first_tick = HAL_GetTick();
        pyro_conf_update_state = E_PYRO_CONF_UPD_WAIT_FOR_APPLYING;
        break;

    case E_PYRO_CONF_UPD_WAIT_FOR_APPLYING:;
        bool time_elapsed = IsTimeElapsed(first_tick, PYRO_CONF_APPLY_DELAY);
        if (time_elapsed) {
            first_tick = HAL_GetTick();
            read_retries_cnt = 0;
            pyro_conf_update_state = E_PYRO_CONF_UPD_WAIT_FOR_READ;
        }
        break;

    case E_PYRO_CONF_UPD_WAIT_FOR_READ:
        pyro_ready = IsSensorReady(first_tick, PYRO_READY_TIMEOUT);
        if ( (pyro_ready == E_PYRO_READY_STATE_OK) ||
             (pyro_ready == E_PYRO_READY_STATE_TIMEOUT) ) {
            pyro_conf_update_state = E_PYRO_CONF_UPD_READ;
        }
        break;

    case E_PYRO_CONF_UPD_READ:
        PYD_ReadAsync(E_RX_FRAME_FULL);
        /* store start time */
        first_tick = HAL_GetTick();
        pyro_conf_update_state = E_PYRO_CONF_UPD_WAIT_FOR_CHECK;
        break;

    case E_PYRO_CONF_UPD_WAIT_FOR_CHECK:
        pyro_ready = IsSensorReady(first_tick, PYRO_READY_TIMEOUT);
        if (pyro_ready == E_PYRO_READY_STATE_OK) {
            pyro_conf_update_state = E_PYRO_CONF_UPD_CHECK;
        } else if (pyro_ready == E_PYRO_READY_STATE_TIMEOUT) {
            if (read_retries_cnt < PYRO_READ_RETRIES) {
                /* pyd driver is not ready for a long time - try to force read */
                pyro_conf_update_state = E_PYRO_CONF_UPD_READ;
            } else {
                /* lets start from config write */
                pyro_conf_update_state = E_PYRO_CONF_UPD_WRITE;
            }
            read_retries_cnt++;
        }
        break;

    case E_PYRO_CONF_UPD_CHECK:
        (void)PYD_GetRxData(&rx_data);
        if (rx_data.conf.word == tx_conf.conf.word) {
            /* update cached rx conf */
            rx_conf.conf.word = rx_data.conf.word;
            rx_conf.is_updated = true;
            pyro_conf_update_state = E_PYRO_CONF_UPD_READY;
        } else {
            if (conf_check_cnt < PYRO_CONF_CHECK_RETRIES) {
                pyro_conf_update_state = E_PYRO_CONF_UPD_READ;
            } else {
                /* number of retries exceeded - lets write config again */
                pyro_conf_update_state = E_PYRO_CONF_UPD_WRITE;
            }
            conf_check_cnt++;
        }
        break;

    case E_PYRO_CONF_UPD_READY:
        break;

    default:
        break;
    }
}

static void Pyro_AdcReadFsm(void)
{
    static uint32_t first_tick = 0;
    enum Pyro_ReadyState pyro_ready = E_PYRO_READY_STATE_PENDING;

    if (adc_data.read_requested) {
        pyro_adc_state = E_PYRO_ADC_READ_WAIT_READY;
        adc_data.read_requested = false;
        first_tick = HAL_GetTick();
    }

    switch (pyro_adc_state) {
    case E_PYRO_ADC_READ_WAIT_READY:
        pyro_ready = IsSensorReady(first_tick, PYRO_READY_TIMEOUT);
        if ( (pyro_ready == E_PYRO_READY_STATE_OK) ||
             (pyro_ready == E_PYRO_READY_STATE_TIMEOUT) ) {
            pyro_adc_state = E_PYRO_ADC_READ_REQUEST;
        }
        break;

    case E_PYRO_ADC_READ_REQUEST:
        PYD_ReadAsync(E_RX_FRAME_ADC);
        /* store start time */
        first_tick = HAL_GetTick();
        pyro_adc_state = E_PYRO_ADC_READ_WAIT_DATA;
        break;

    case E_PYRO_ADC_READ_WAIT_DATA:
        pyro_ready = IsSensorReady(first_tick, PYRO_READY_TIMEOUT);
        if (pyro_ready == E_PYRO_READY_STATE_OK) {
            struct Pyd1588RxData rx_data = { 0 };
            int status = PYD_GetRxData(&rx_data);
            if (status == 0) {
                if (rx_data.out_of_range) {
                    adc_data.adc_buf[adc_data.tail].adc_value = rx_data.adc_val;
                    adc_data.adc_buf[adc_data.tail].timestamp = HAL_GetTick();
                    /* shift tail */
                    if (++adc_data.tail >= ADC_BUF_SIZE) {
                        adc_data.tail = 0;
                    }
                    /* override is allowed - just shift head */
                    if (adc_data.head == adc_data.tail) {
                        if (++adc_data.head >= ADC_BUF_SIZE) {
                            adc_data.head = 0;
                        }
                    }
                }
            }
            first_tick = HAL_GetTick();
            pyro_adc_state = E_PYRO_ADC_READ_DELAY;
        } else if (pyro_ready == E_PYRO_READY_STATE_TIMEOUT) {
            first_tick = HAL_GetTick();
            pyro_adc_state = E_PYRO_ADC_READ_DELAY;
        }
        break;

    case E_PYRO_ADC_READ_DELAY:;
        bool time_elapsed = IsTimeElapsed(first_tick, PYRO_ADC_READ_DELAY);
        if (time_elapsed) {
            pyro_adc_state = E_PYRO_ADC_READ_WAIT_READY;
        }
        break;

    default:
        break;
    }
}

int Pyro_Init(void)
{
    int retval = 0;

    PYD_Init();

    pyro_conf_update_state = E_PYRO_CONF_UPD_INIT;
    pyro_basic_state = E_PYRO_BASIC_IDLE;
    pyro_adc_state = E_PYRO_ADC_READ_INIT;

    memset(&tx_conf, 0, sizeof(struct TxConfig));
    memset(&rx_conf, 0, sizeof(struct RxConfig));
    memset(&adc_data , 0, sizeof(struct AdcData));

    return retval;
}

void Pyro_Fsm(void)
{
    /* config update request has high prio - handle it first */
    if (tx_conf.update_requested) {
        pyro_basic_state = E_PYRO_BASIC_UPDATE_CONF;
        /* if there is ongoing adc read - set read_requested flag to reset
         * ADC read FSM */
        if (adc_data.read_adc) {
            /* adc read will be re-init after config update finished */
            adc_data.read_requested = true;
        }
    } else {
        /* we can read ADC value only after config is written in pyro sensor */
        if (pyro_conf_update_state == E_PYRO_CONF_UPD_READY) {
            if (adc_data.read_adc) {
                pyro_basic_state = E_PYRO_BASIC_READ_ADC;
            } else {
                pyro_basic_state = E_PYRO_BASIC_IDLE;
            }
        }
    }

    switch (pyro_basic_state) {
    case E_PYRO_BASIC_IDLE:
        /* do nothing */
        break;

    case E_PYRO_BASIC_UPDATE_CONF:
        Pyro_ConfUpdateFsm();
        break;

    case E_PYRO_BASIC_READ_ADC:
        Pyro_AdcReadFsm();
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

int Pyro_StartAdcRead(void)
{
    adc_data.head = 0;
    adc_data.tail = 0;
    adc_data.read_adc = true;
    adc_data.read_requested = true;

    return 0;
}

int Pyro_StopAdcRead(void)
{
    adc_data.read_adc = false;
    adc_data.read_requested = false;

    return 0;
}

bool Pyro_GetAdcValue(struct AdcInstantValue *padcval)
{
    bool retval = false;

    if (padcval) {
        if (adc_data.head != adc_data.tail) {
            *padcval = adc_data.adc_buf[adc_data.head];

            /* shift head */
            if (++adc_data.head >= ADC_BUF_SIZE) {
                adc_data.head = 0;
            }

            retval = true;
        }
    }

    return retval;
}

bool Pyro_IsConfUpdated(void)
{
    return rx_conf.is_updated;
}
