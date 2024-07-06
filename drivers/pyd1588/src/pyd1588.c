#include "pyd1588.h"

const union pyd1588_config pyd_default_config = {
        .fields.count_mode = PYD1588_COUNT_MODE_WITH_BPF,
        .fields.reserved1 = PYD1588_RESERVED1,
        .fields.hpf_cutoff = PYD1588_HPF_CUTOFF_0_4HZ,
        .fields.reserved2 = PYD1588_RESERVED2,
        .fields.signal_source = PYD1588_SIGNAL_SOURCE_BPF,
        .fields.operating_modes = PYD1588_OPERATION_MODE_FORCED_READOUT,
        .fields.window_time = 0u,
        .fields.pulse_counter = 0u,
        .fields.blind_time = 0u,
        .fields.threshold = 20u,
};
