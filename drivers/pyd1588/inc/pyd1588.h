#ifndef PYD1588_H
#define PYD1588_H

#include <stdint.h>
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

union pyd1588_config {
    struct {
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

extern const union pyd1588_config pyd_default_config;

#endif /* PYD1588_H */
