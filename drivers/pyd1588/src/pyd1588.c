#include <stdint.h>
#include "pyd1588.h"

#define PYD_SERIAL_IN_PORT      GPIOC
#define PYD_SERIAL_IN_PIN       GPIO_PIN_2
#define PYD_DIRECT_LINK_PORT    GPIOC
#define PYD_DIRECT_LINK_PIN     GPIO_PIN_0

#define PYD_DL_PIN_MODE_IN      (0x00000000u)
#define PYD_DL_PIN_MODE_OUT     (0x00000001u)
#define PYD_DL_PIN_MODE_CLR     (0xFFFFFFFCu);
#define PYD_DL_PIN_PUPD_SET     (0x00000001u)
#define PYD_DL_PIN_PUPD_CLR     (0xFFFFFFFCu);
#define PYD_DL_PIN_IDR          (0x00000001u)

#define PYRO_HW_TIMER           TIM6

#define PYD_CONFIG_BITS_NUM     (25u)

#define PYRO_PRESCALER  (320u)

#if (PYRO_PRESCALER == 320)
#define PYRO_TX_PERIOD_NORMAL           (13u)
#define PYRO_TX_PERIOD_ENDSEQ           (75u)
#define PYRO_RX_PERIOD_START_SEQ        (14u)
#define PYRO_RX_PERIOD_RD_BIT           (1u)
#define PYRO_RX_PERIOD_ENDSEQ           (140u)
#elif (PYRO_PRESCALER == 32)
#define PYRO_TX_PERIOD_NORMAL           (90u)
#define PYRO_TX_PERIOD_ENDSEQ           (670u)
#define PYRO_RX_PERIOD_START_SEQ        (140u)
#define PYRO_RX_PERIOD_RD_BIT           (20u)
#define PYRO_RX_PERIOD_ENDSEQ           (1260u)
#else
#error  "PYRO_PRESCALER value is not supported"
#endif

#define PYD_MSB_BIT     (0x1000000)

#define PYD_READ_BITS_NUM_FULL      (40u)
#define PYD_READ_BITS_NUM_ADC       (15u)

#define PYD_ADC_RAWVALUE_MASK       (0x3FFFu)
#define PYD_ADC_SIGN_BITMASK        (0x2000u)
#define PYD_ADC_VALUE_BITMASK       (0x1FFFu)

#define PYD_RX_CONFIG_MASK          (0x1FFFFFFu)
#define PYD_RX_OUT_OF_RANGE_MASK    ((uint64_t)1 << 39u)

const union Pyd1588Config Pyd1588DefaultConfig = {
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

enum PyroTransactionState {
    E_STATE_IDLE = 0,
    E_TX_STATE_WR_BIT,
    E_TX_STATE_END_SEQ,
    E_RX_STATE_START_SEQ,
    E_RX_STATE_RD_BIT,
    E_RX_STATE_END_SEQ
};

struct PyroTransactionCtl {
    uint64_t rx_frame;
    uint32_t tx_frame;
    int32_t idx;
    enum PyroTransactionState state;
    enum PyroRxFrameType frame_type;
};

static TIM_HandleTypeDef pyro_tim = { 0 };
static volatile struct PyroTransactionCtl transaction_ctl = {
    .rx_frame = 0,
    .tx_frame = 0,
    .idx = 0,
    .state = E_STATE_IDLE,
    .frame_type = E_RX_FRAME_UNKNOWN
};

static int Pyro_GpioInit(void);
static int Pyro_TimerInit(void);
static inline void Pyro_UpdateSiPin(void);
static inline void Pyro_UpdateDlPin(void);
static inline void Pyro_TransactionFSM(void);
static inline void Pyro_SetDlPinOut(void);
static inline void Pyro_SetDlPinIn(void);
static int32_t Pyro_GetRxBitsNum(enum PyroRxFrameType frame_type);
static uint16_t Pyro_GetAdcRawData(uint64_t rx_frame, enum PyroRxFrameType frame_type);

int Pyro_Init(void)
{
    int retval = 0;
    do {
        retval = Pyro_GpioInit();
        if (retval < 0) {
            break;
        }

        retval = Pyro_TimerInit();
        if (retval < 0) {
            break;
        }
    } while(0);

    return retval;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
    if (tim_baseHandle->Instance == PYRO_HW_TIMER)
    {
        /* PYRO_HW_TIMER clock enable */
        __HAL_RCC_TIM6_CLK_ENABLE();

        /* PYRO_HW_TIMER interrupt Init */
        HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
    if (tim_baseHandle->Instance == PYRO_HW_TIMER)
    {
        /* Peripheral clock disable */
        __HAL_RCC_TIM6_CLK_DISABLE();

        /* TIM6 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
    }
}

void TIM6_DAC_IRQHandler(void)
{
    /* the only UPDATE interrup is enabled for TIM6 so no need
     * to check other interrup sources */ 
    __HAL_TIM_CLEAR_IT(&pyro_tim, TIM_IT_UPDATE);
    Pyro_TransactionFSM();
}

bool Pyro_IsReady(void)
{
    return (transaction_ctl.state == E_STATE_IDLE);
}

int Pyro_WriteAsync(uint32_t data)
{
    int retval = 0;

    do {
        if (transaction_ctl.state != E_STATE_IDLE) {
            retval = -1;
            break;
        }

        transaction_ctl.idx = PYD_CONFIG_BITS_NUM;
        
        /* during config update DirectLink should be low */
        /* set BRR first to prevent rising edge after switch pin from in to out */
        PYD_DIRECT_LINK_PORT->BRR = PYD_DIRECT_LINK_PIN;
        Pyro_SetDlPinOut();

        Pyro_UpdateSiPin();

        /* update autoreload reg before start */
        pyro_tim.Instance->ARR = PYRO_TX_PERIOD_NORMAL;
        /* update prescaler reg before start */
        pyro_tim.Instance->PSC = PYRO_PRESCALER;
        __HAL_TIM_CLEAR_IT(&pyro_tim, TIM_IT_UPDATE);
        transaction_ctl.state = E_TX_STATE_WR_BIT;
        HAL_StatusTypeDef status = HAL_TIM_Base_Start_IT(&pyro_tim);
        if (status != HAL_OK) {
            HAL_GPIO_WritePin(PYD_SERIAL_IN_PORT, PYD_SERIAL_IN_PIN, GPIO_PIN_RESET);
            transaction_ctl.state = E_STATE_IDLE;
            retval = -1;
            break;
        }

    } while (0);

    return retval;
}

int Pyro_ReadAsync(enum PyroRxFrameType frame_type)
{
    int retval = -1;

    do {
        if (transaction_ctl.state != E_STATE_IDLE) {
            break;
        }

        int32_t rx_bits = Pyro_GetRxBitsNum(frame_type);
        if (rx_bits <= 0) {
            break;
        }

        transaction_ctl.idx = rx_bits;
        transaction_ctl.frame_type = frame_type;
        /* update autoreload reg before start */
        pyro_tim.Instance->ARR = PYRO_RX_PERIOD_START_SEQ;
        /* update prescaler reg before start */
        pyro_tim.Instance->PSC = PYRO_PRESCALER;
        Pyro_SetDlPinOut();
        HAL_GPIO_WritePin(PYD_DIRECT_LINK_PORT, PYD_DIRECT_LINK_PIN, GPIO_PIN_RESET);

        __HAL_TIM_CLEAR_IT(&pyro_tim, TIM_IT_UPDATE);
        HAL_StatusTypeDef status = HAL_TIM_Base_Start_IT(&pyro_tim);
        if (status != HAL_OK) {
            transaction_ctl.state = E_STATE_IDLE;
            retval = -1;
            break;
        }
        transaction_ctl.state = E_RX_STATE_START_SEQ;
        retval = 0;
    } while(0);

    return retval;
}

int Pyro_ReadRxData(struct Pyd1588RxData *data)
{
    int retval = 0;

    do {
        if (!data) {
            retval = -1;
            break;
        }

        if (transaction_ctl.state != E_STATE_IDLE) {
            retval = -1;
            break;
        }
        // data->out_of_range = transaction_ctl.rx_frame;
        uint16_t adc_raw = Pyro_GetAdcRawData(transaction_ctl.rx_frame, transaction_ctl.frame_type);
        /* check sign of adc value */
        int16_t signed_adc = 0;
        if (adc_raw & PYD_ADC_SIGN_BITMASK) {
            /* 13th bit is 1 - negative value */
            signed_adc = ((~(adc_raw & PYD_ADC_VALUE_BITMASK)) + 1) * (-1);
        } else {
            /* value is positive */
            signed_adc = adc_raw & PYD_ADC_VALUE_BITMASK;
        }

        data->adc_val = signed_adc;
        data->conf.word = transaction_ctl.rx_frame & PYD_RX_CONFIG_MASK;
        data->out_of_range = transaction_ctl.rx_frame & PYD_RX_OUT_OF_RANGE_MASK;
    } while (0);

    return retval;
}

static int Pyro_GpioInit(void)
{
    int retval = 0;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = PYD_DIRECT_LINK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(PYD_DIRECT_LINK_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(PYD_DIRECT_LINK_PORT, PYD_DIRECT_LINK_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = PYD_SERIAL_IN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(PYD_SERIAL_IN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(PYD_SERIAL_IN_PORT, PYD_SERIAL_IN_PIN, GPIO_PIN_RESET);

    return retval;
}

static int Pyro_TimerInit(void)
{
    int retval = 0;
    HAL_StatusTypeDef status = HAL_ERROR;

    pyro_tim.Instance = PYRO_HW_TIMER;
    pyro_tim.Init.Prescaler = PYRO_PRESCALER;
    pyro_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    pyro_tim.Init.Period = PYRO_TX_PERIOD_NORMAL;
    pyro_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    status = HAL_TIM_Base_Init(&pyro_tim);

    if (status != HAL_OK)
    {
        retval = -1;
    }

    return retval;
}

static inline __attribute__((always_inline)) void Pyro_UpdateSiPin(void)
{
    /* bit start condition */
    /* reset SrialIn pin */
    PYD_SERIAL_IN_PORT->BRR = PYD_SERIAL_IN_PIN;
    /* set SrialIn pin */
    PYD_SERIAL_IN_PORT->BSRR = PYD_SERIAL_IN_PIN;

    /* if MSB bit is 0 - reset SerialIn pin;
     * if MSB bit is 1 - SerialIn is already set */
    if (!(transaction_ctl.tx_frame & PYD_MSB_BIT)) {
        PYD_SERIAL_IN_PORT->BRR = PYD_SERIAL_IN_PIN;
    }
}

static inline __attribute__((always_inline)) void Pyro_SetDlPinOut(void)
{
#if 0
    uint32_t tmpval = 0;

    tmpval = PYD_DIRECT_LINK_PORT->MODER;
    tmpval &= PYD_DL_PIN_MODE_CLR;
    tmpval |= PYD_DL_PIN_MODE_OUT;
    /* pin mode = output */
    PYD_DIRECT_LINK_PORT->MODER = tmpval;
#endif 
    PYD_DIRECT_LINK_PORT->MODER |= PYD_DL_PIN_MODE_OUT;
}

static inline __attribute__((always_inline)) void Pyro_SetDlPinIn(void)
{
#if 0
    uint32_t tmpval = 0;

    tmpval = PYD_DIRECT_LINK_PORT->MODER;
    tmpval &= PYD_DL_PIN_MODE_CLR;
    tmpval |= PYD_DL_PIN_MODE_IN;
    /* pin mode = in */
    PYD_DIRECT_LINK_PORT->MODER = tmpval;
#endif
    PYD_DIRECT_LINK_PORT->MODER &= PYD_DL_PIN_MODE_CLR;
}

static inline __attribute__((always_inline)) void Pyro_UpdateDlPin(void)
{
    /* set BRR first to prevent rising edge after switch pin from in to out */
    PYD_DIRECT_LINK_PORT->BRR = PYD_DIRECT_LINK_PIN;
    Pyro_SetDlPinOut();

    // PYD_DIRECT_LINK_PORT->BRR = PYD_DIRECT_LINK_PIN;
    /* set pin high */
    PYD_DIRECT_LINK_PORT->BSRR = PYD_DIRECT_LINK_PIN;

    Pyro_SetDlPinIn();
}

static inline __attribute__((always_inline)) void Pyro_TransactionFSM(void)
{
    switch (transaction_ctl.state) {
    case E_TX_STATE_WR_BIT:
        transaction_ctl.idx--;
        if (transaction_ctl.idx > 0) {
            transaction_ctl.tx_frame <<= 1;
            Pyro_UpdateSiPin();
        } else {
            /* update autoreload reg for end sequence */
            pyro_tim.Instance->ARR = PYRO_TX_PERIOD_ENDSEQ;
            /* set serial in pin to low */
            PYD_SERIAL_IN_PORT->BRR = PYD_SERIAL_IN_PIN;
            /* all bits are sent - start end sequence */
            transaction_ctl.state = E_TX_STATE_END_SEQ;
        }
        break;

    case E_TX_STATE_END_SEQ:
        transaction_ctl.state = E_STATE_IDLE;
        (void)HAL_TIM_Base_Stop_IT(&pyro_tim);
        break;

    case E_RX_STATE_START_SEQ:
        transaction_ctl.rx_frame = 0;
        /* update autoreload reg for rx start sequence */
        pyro_tim.Instance->ARR = PYRO_RX_PERIOD_RD_BIT;
        Pyro_UpdateDlPin();
        transaction_ctl.state = E_RX_STATE_RD_BIT;
        break;

    case E_RX_STATE_RD_BIT:
        if (transaction_ctl.idx > 0) {
            /* if DL is high - set 1, otherwise left it as 0 */
            if (PYD_DIRECT_LINK_PORT->IDR & PYD_DL_PIN_IDR) {
                transaction_ctl.rx_frame |= (1 << (transaction_ctl.idx - 1));
            }
            transaction_ctl.idx--;
            Pyro_UpdateDlPin();
        } else {
            /* update autoreload reg for rx end sequence */
            pyro_tim.Instance->ARR = PYRO_RX_PERIOD_ENDSEQ;
            Pyro_SetDlPinOut();
            /* reset DirectLink pin */
            PYD_DIRECT_LINK_PORT->BRR = PYD_DIRECT_LINK_PIN;
            transaction_ctl.state = E_RX_STATE_END_SEQ;
        }
        break;

    case E_RX_STATE_END_SEQ:
        /* transaction is complete - release DirectLink pin (put it to Hi-Z) */
        Pyro_SetDlPinIn();
        transaction_ctl.state = E_STATE_IDLE;
        (void)HAL_TIM_Base_Stop_IT(&pyro_tim);
        break;

    default:
        break;
    }
}

static int32_t Pyro_GetRxBitsNum(enum PyroRxFrameType frame_type)
{
    int32_t retval = 0;

    switch(frame_type) {
        case E_RX_FRAME_FULL:
            retval = PYD_READ_BITS_NUM_FULL;
            break;

        case E_RX_FRAME_ADC:
            retval = PYD_READ_BITS_NUM_ADC;
            break;

        default:
            break;
    }

    return retval;
}

static uint16_t Pyro_GetAdcRawData(uint64_t rx_frame, enum PyroRxFrameType frame_type)
{
    uint16_t retval = 0;

    uint8_t bitshift = 0;

    if (frame_type == E_RX_FRAME_FULL) {
        bitshift = 25;
    }

    retval = (rx_frame >> bitshift) & PYD_ADC_RAWVALUE_MASK;

    return retval;
}
