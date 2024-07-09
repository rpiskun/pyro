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
#define PYD_READ_BITS_NUM       (40u)

/* MSI = 2.097 MHz => TIM6 freq = 99.857kHz (T = 10 us) */
#define PYRO_PRESCALER_10US     (21u)

/* period = 90 us */
#define PYRO_TX_PERIOD_NORMAL   (9u)

/* period = 670 us */
#define PYRO_TX_PERIOD_ENDSEQ   (67u)

#define PYRO_RX_PERIOD_START_SEQ       (121u)
#define PYRO_RX_PERIOD_RD_BIT          (1u)
#define PYRO_RX_PERIOD_ENDSEQ          (126u)

#define PYD_BITS_SHIFT          (7u)
#define PYD_MSB_BIT_32          (0x80000000)

#define PYD_RX_OUT_OF_RANGE_BIT     (39u)
#define PYD_RX_OUT_OF_RANGE_MASK    (0x01u)
#define PYD_RX_ADC_VALUE_BIT        (25u)
#define PYD_RX_ADC_VALUE_MASK       (0x7FFFu)
#define PYD_RX_ADC_CONFIG_BIT       (0u)
#define PYD_RX_ADC_CONFIG_MASK      (0x1FFFFFFu)

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

enum PyroReadType {
    E_READ_TYPE_ADC = 0,
    E_READ_TYPE_FULL
};

struct PyroTransactionCtl {
    enum PyroTransactionState state;
    uint32_t tx_data;
    uint64_t rx_data;
    int32_t idx;
};

static TIM_HandleTypeDef pyro_tim = { 0 };
static volatile struct PyroTransactionCtl transaction_ctl = {
    .state = E_STATE_IDLE,
    .tx_data = 0,
    .idx = 0
};

static int Pyro_GpioInit(void);
static int Pyro_TimerInit(void);
static inline void Pyro_UpdateSerialInPin(void);
static inline void Pyro_UpdateDirectLinkPin(void);
static inline void Pyro_TransactionFSM(void);
static inline void Pyro_SetDlPinOut(void);
static inline void Pyro_SetDlPinIn(void);

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

        transaction_ctl.tx_data = (data << PYD_BITS_SHIFT);
        transaction_ctl.idx = PYD_CONFIG_BITS_NUM;
        
        /* during config update DirectLink should be low */
        HAL_GPIO_WritePin(PYD_DIRECT_LINK_PORT, PYD_DIRECT_LINK_PIN, GPIO_PIN_RESET);

        Pyro_UpdateSerialInPin();

        /* update autoreload reg before start */
        pyro_tim.Instance->ARR = PYRO_TX_PERIOD_NORMAL;
        /* update prescaler reg before start */
        pyro_tim.Instance->PSC = PYRO_PRESCALER_10US;
        __HAL_TIM_CLEAR_IT(&pyro_tim, TIM_IT_UPDATE);
        HAL_StatusTypeDef status = HAL_TIM_Base_Start_IT(&pyro_tim);
        if (status != HAL_OK) {
            HAL_GPIO_WritePin(PYD_SERIAL_IN_PORT, PYD_SERIAL_IN_PIN, GPIO_PIN_RESET);
            transaction_ctl.state = E_STATE_IDLE;
            retval = -1;
            break;
        }

        transaction_ctl.state = E_TX_STATE_WR_BIT;
    } while (0);

    return retval;
}

int Pyro_ReadAsync(void)
{
    int retval = 0;

    do {
        if (transaction_ctl.state != E_STATE_IDLE) {
            retval = -1;
            break;
        }
        /* update autoreload reg before start */
        pyro_tim.Instance->ARR = PYRO_RX_PERIOD_START_SEQ;
        /* update prescaler reg before start */
        pyro_tim.Instance->PSC = PYRO_PRESCALER_10US;
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
    } while(0);

    return retval;
}

int Pyro_ReadRxData(union Pyd1588RxData *data)
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
#if 0
        uint64_t rx_data = transaction_ctl.rx_data;
        data->fields.out_of_range = ((rx_data >> PYD_RX_OUT_OF_RANGE_BIT) & PYD_RX_OUT_OF_RANGE_MASK);
        data->fields.adc_val = ((rx_data >> PYD_RX_ADC_VALUE_BIT) & PYD_RX_ADC_VALUE_MASK);
        data->fields.conf.word = (rx_data & PYD_RX_ADC_CONFIG_MASK);
#endif 
        data->word = transaction_ctl.rx_data;
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
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PYD_DIRECT_LINK_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(PYD_DIRECT_LINK_PORT, PYD_DIRECT_LINK_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = PYD_SERIAL_IN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PYD_SERIAL_IN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(PYD_SERIAL_IN_PORT, PYD_SERIAL_IN_PIN, GPIO_PIN_RESET);

    return retval;
}

static int Pyro_TimerInit(void)
{
    int retval = 0;
    HAL_StatusTypeDef status = HAL_ERROR;

    pyro_tim.Instance = PYRO_HW_TIMER;
    pyro_tim.Init.Prescaler = PYRO_PRESCALER_10US;
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

static inline __attribute__((always_inline)) void Pyro_UpdateSerialInPin(void)
{
    /* bit start condition */
    HAL_GPIO_WritePin(PYD_SERIAL_IN_PORT, PYD_SERIAL_IN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PYD_SERIAL_IN_PORT, PYD_SERIAL_IN_PIN, GPIO_PIN_SET);

    if (transaction_ctl.tx_data & PYD_MSB_BIT_32) {
        HAL_GPIO_WritePin(PYD_SERIAL_IN_PORT, PYD_SERIAL_IN_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(PYD_SERIAL_IN_PORT, PYD_SERIAL_IN_PIN, GPIO_PIN_RESET);
    }
}

static inline __attribute__((always_inline)) void Pyro_SetDlPinOut(void)
{
    uint32_t tmpval = 0;

    tmpval = PYD_DIRECT_LINK_PORT->MODER;
    tmpval &= PYD_DL_PIN_MODE_CLR;
    tmpval |= PYD_DL_PIN_MODE_OUT;
    /* pin mode = output */
    PYD_DIRECT_LINK_PORT->MODER = tmpval;
}

static inline __attribute__((always_inline)) void Pyro_SetDlPinIn(void)
{
    uint32_t tmpval = 0;

    tmpval = PYD_DIRECT_LINK_PORT->MODER;
    tmpval &= PYD_DL_PIN_MODE_CLR;
    tmpval |= PYD_DL_PIN_MODE_IN;
    /* pin mode = in */
    PYD_DIRECT_LINK_PORT->MODER = tmpval;
}

static inline __attribute__((always_inline)) void Pyro_UpdateDirectLinkPin(void)
{
    Pyro_SetDlPinOut();

    HAL_GPIO_WritePin(PYD_DIRECT_LINK_PORT, PYD_DIRECT_LINK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PYD_DIRECT_LINK_PORT, PYD_DIRECT_LINK_PIN, GPIO_PIN_SET);

    Pyro_SetDlPinIn();
}

static inline __attribute__((always_inline)) void Pyro_TransactionFSM(void)
{
    switch (transaction_ctl.state) {
    case E_TX_STATE_WR_BIT:
        transaction_ctl.idx--;
        if (transaction_ctl.idx > 0) {
            transaction_ctl.tx_data <<= 1;
            Pyro_UpdateSerialInPin();
        } else {
            /* update autoreload reg for end sequence */
            pyro_tim.Instance->ARR = PYRO_TX_PERIOD_ENDSEQ;
            /* set serial in pin to low */
            HAL_GPIO_WritePin(PYD_SERIAL_IN_PORT, PYD_SERIAL_IN_PIN, GPIO_PIN_RESET);
            /* all bits are sent - start end sequence */
            transaction_ctl.state = E_TX_STATE_END_SEQ;
        }
        break;

    case E_TX_STATE_END_SEQ:
        transaction_ctl.state = E_STATE_IDLE;
        (void)HAL_TIM_Base_Stop_IT(&pyro_tim);
        break;

    case E_RX_STATE_START_SEQ:
        transaction_ctl.idx = PYD_READ_BITS_NUM;
        transaction_ctl.rx_data = 0;
        /* update autoreload reg for rx start sequence */
        pyro_tim.Instance->ARR = PYRO_RX_PERIOD_RD_BIT;
        Pyro_UpdateDirectLinkPin();
        transaction_ctl.state = E_RX_STATE_RD_BIT;
        break;

    case E_RX_STATE_RD_BIT:
        if (PYD_DIRECT_LINK_PORT->IDR & PYD_DL_PIN_IDR) {
            transaction_ctl.rx_data |= 1;
        }
        transaction_ctl.idx--;

        if (transaction_ctl.idx > 0) {
            transaction_ctl.rx_data <<= 1;
            Pyro_UpdateDirectLinkPin();
        } else {
            /* update autoreload reg for rx end sequence */
            pyro_tim.Instance->ARR = PYRO_RX_PERIOD_ENDSEQ;
            Pyro_SetDlPinOut();
            HAL_GPIO_WritePin(PYD_DIRECT_LINK_PORT, PYD_DIRECT_LINK_PIN, GPIO_PIN_RESET);
            transaction_ctl.state = E_RX_STATE_END_SEQ;
        }
        break;

    case E_RX_STATE_END_SEQ:
        HAL_GPIO_WritePin(PYD_DIRECT_LINK_PORT, PYD_DIRECT_LINK_PIN, GPIO_PIN_SET);
        transaction_ctl.state = E_STATE_IDLE;
        (void)HAL_TIM_Base_Stop_IT(&pyro_tim);
        break;

    default:
        break;
    }
}

