#ifndef PYD1588_H
#define PYD1588_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx_hal.h"

/* count mode */
#define PYD1588_COUNT_MODE_WITH_BPF     (0u)
#define PYD1588_COUNT_MODE_WITHOUT_BPF  (1u)

/* HPF Cut-Off */
#define PYD1588_HPF_CUTOFF_0_4HZ        (0u)
#define PYD1588_HPF_CUTOFF_0_2HZ        (1u)

/* Signal source */
#define PYD1588_SIGNAL_SOURCE_BPF           (0u)
#define PYD1588_SIGNAL_SOURCE_LPF           (1u)
#define PYD1588_SIGNAL_SOURCE_RESERVED      (2u)
#define PYD1588_SIGNAL_SOURCE_TEMP_SENSOR   (3u)

/* Operation modes */
#define PYD1588_OPERATION_MODE_FORCED_READOUT       (0u)
#define PYD1588_OPERATION_MODE_INTERRUPT_READOUT    (1u)
#define PYD1588_OPERATION_MODE_WAKEUP               (2u)
#define PYD1588_OPERATION_MODE_RESERVED             (3u)

/* reserved fields */
#define PYD1588_RESERVED1   (0u)
#define PYD1588_RESERVED2   (2u)

#define PYD_SERIAL_IN_PORT      GPIOC
#define PYD_SERIAL_IN_PIN       GPIO_PIN_2
#define PYD_DIRECT_LINK_PORT    GPIOC
#define PYD_DIRECT_LINK_PIN     GPIO_PIN_0

enum PyroRxFrameType {
    E_RX_FRAME_UNKNOWN = 0,
    E_RX_FRAME_FULL,
    E_RX_FRAME_ADC
};

union Pyd1588Config {
    struct __attribute__((packed, aligned(1))) {
        uint8_t count_mode: 1;
        uint8_t reserved1: 1;
        uint8_t hpf_cutoff: 1;
        uint8_t reserved2: 2;
        uint8_t signal_source: 2;
        uint8_t operating_modes: 2;
        uint8_t window_time: 2;
        uint8_t pulse_counter: 2;
        uint8_t blind_time: 4;
        uint8_t threshold: 8;
    } fields;
    uint32_t word;
};

struct Pyd1588RxData {
    union Pyd1588Config conf;
    int16_t adc_val;
    uint8_t out_of_range;
};

extern const union Pyd1588Config Pyd1588DefaultConfig;

int PYD_Init(void);
bool PYD_IsReady(void);
int PYD_WriteAsync(uint32_t data);
int PYD_ReadAsync(enum PyroRxFrameType frame_type);
int PYD_GetRxData(struct Pyd1588RxData *data);
int PYD_EnableWakeupEvent(void);
int PYD_DisableWakeupEvent(void);

#endif /* PYD1588_H */
