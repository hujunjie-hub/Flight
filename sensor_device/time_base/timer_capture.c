/* TIM2 monotonic time base and capture device.
 *
 * 32-bit free-running counter extended to 64 bits, CH4 IMU DRDY capture and
 * CH3 PPS capture with a bounded ring. Registered as the native RT-Thread
 * device "timebase"; no project-specific device model or accessor is added.
 */
#include <stdint.h>

static uint64_t timer_extend_capture(uint32_t high, uint8_t overflow_pending,
                                     uint32_t captured, uint32_t counter)
{
    if (overflow_pending && captured <= counter)
    {
        high++;
    }
    return ((uint64_t)high << 32) | captured;
}

#if !defined(FLIGHT_DRIVER_CODEC_TEST)
#include "timer_capture.h"
#include <board.h>
#include <string.h>

static TIM_HandleTypeDef htim2;

static struct rt_device g_timebase_dev;
static volatile uint32_t g_hi;
static volatile uint32_t g_seq[2];
static volatile uint32_t g_lost;
static uint32_t g_epoch = 1u;
static uint32_t g_tick_hz;
static volatile uint8_t g_ready;
static void (*g_imu_capture)(const struct timebase_capture *);
static void (*g_compare[2])(void);
static volatile uint32_t g_head;
static volatile uint32_t g_tail;
static struct timebase_capture g_ring[32];

static void dispatch_compare(unsigned index)
{
    void (*callback)(void) = g_compare[index];
    if (callback)
    {
        TIM2->DIER &= ~(TIM_DIER_CC1IE << index);
        g_compare[index] = NULL;
        callback();
    }
}

static struct timebase_capture make_capture(uint32_t ccr, uint16_t channel,
                                            uint32_t sr, uint32_t cnt)
{
    struct timebase_capture c;
    memset(&c, 0, sizeof(c));
    c.tick64 = timer_extend_capture(g_hi, (sr & TIM_SR_UIF) != 0u, ccr, cnt);
    c.ccr = ccr;
    c.channel = channel;
    c.flags = 0x01u; /* valid */
    if (sr & ((channel == 4u) ? TIM_SR_CC4OF : TIM_SR_CC3OF))
    {
        c.flags |= 0x02u; /* overcapture */
        g_lost++;
    }
    c.sequence = ++g_seq[channel == 4u ? 0u : 1u];
    return c;
}

void TIM2_IRQHandler(void)
{
    uint32_t sr;
    uint32_t cnt;
    uint32_t cc4 = 0u;
    uint32_t cc3 = 0u;
    struct timebase_capture imu;
    struct timebase_capture pps;

    if (!g_ready)
    {
        return;
    }

    sr = TIM2->SR;
    cnt = TIM2->CNT;
    /* Reading a CCR clears its CCIF. Never read a channel not present in the
     * SR snapshot, or clear CCIF again after reading. */
    if (sr & TIM_SR_CC4IF)
    {
        cc4 = TIM2->CCR4;
    }
    if (sr & TIM_SR_CC3IF)
    {
        cc3 = TIM2->CCR3;
    }
    TIM2->SR = ~(sr & (TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF |
                       TIM_SR_CC3OF | TIM_SR_CC4OF));

    if (sr & TIM_SR_CC4IF)
    {
        imu = make_capture(cc4, 4u, sr, cnt);
    }
    if (sr & TIM_SR_CC3IF)
    {
        pps = make_capture(cc3, 3u, sr, cnt);
    }
    if (sr & TIM_SR_UIF)
    {
        g_hi++;
    }

    if ((sr & TIM_SR_CC4IF) && g_imu_capture)
    {
        g_imu_capture(&imu);
    }
    if (sr & TIM_SR_CC3IF)
    {
        uint32_t next = (g_head + 1u) % (sizeof(g_ring) / sizeof(g_ring[0]));
        if (next == g_tail)
        {
            g_lost++;
        }
        else
        {
            g_ring[g_head] = pps;
            __DMB();
            g_head = next;
        }
    }

    /* Dispatch the frame deadline before a simultaneous scheduled start. */
    if (sr & TIM_SR_CC2IF)
    {
        dispatch_compare(1u);
    }
    if (sr & TIM_SR_CC1IF)
    {
        dispatch_compare(0u);
    }
}

static uint32_t timer_clock(uint8_t apb2)
{
    RCC_ClkInitTypeDef cfg;
    uint32_t latency;

    HAL_RCC_GetClockConfig(&cfg, &latency);
    if (RCC->CFGR & RCC_CFGR_TIMPRE)
    {
        return 0u;
    }
    return apb2 ? HAL_RCC_GetPCLK2Freq() * ((cfg.APB2CLKDivider == RCC_APB2_DIV1) ? 1u : 2u)
                : HAL_RCC_GetPCLK1Freq() * ((cfg.APB1CLKDivider == RCC_APB1_DIV1) ? 1u : 2u);
}

static rt_err_t timebase_init(rt_device_t dev)
{
    TIM_IC_InitTypeDef ic;
    uint32_t kernel_hz;
    uint32_t div;
    uint32_t best = 0u;
    uint32_t error = 0xffffffffu;

    (void)dev;
    if (g_ready)
    {
        return RT_EOK;
    }

    kernel_hz = timer_clock(0u);
    if (!kernel_hz)
    {
        return -RT_EIO;
    }

    /* Exact integer ticks near 10 MHz: 275 MHz -> 11 MHz, PSC=24. */
    for (div = 1u; div <= 65536u; div++)
    {
        uint32_t hz;
        uint32_t delta;
        if (kernel_hz % div)
        {
            continue;
        }
        hz = kernel_hz / div;
        delta = hz > 10000000u ? hz - 10000000u : 10000000u - hz;
        if (delta < error)
        {
            error = delta;
            best = div;
        }
    }
    if (!best || kernel_hz / best < 1000000u)
    {
        return -RT_EIO;
    }

    memset(&htim2, 0, sizeof(htim2));
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = best - 1u;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFFu;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        return -RT_EIO;
    }

    memset(&ic, 0, sizeof(ic));
    ic.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    if (HAL_TIM_IC_ConfigChannel(&htim2, &ic, TIM_CHANNEL_4) != HAL_OK ||
        HAL_TIM_IC_ConfigChannel(&htim2, &ic, TIM_CHANNEL_3) != HAL_OK)
    {
        return -RT_EIO;
    }
    TIM2->CCER |= TIM_CCER_CC4E | TIM_CCER_CC3E;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0u;

    g_tick_hz = kernel_hz / best;
    g_hi = 0u;
    g_head = 0u;
    g_tail = 0u;
    g_lost = 0u;
    g_ready = 1u;

    HAL_NVIC_SetPriority(TIM2_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    /* Keep the monotonic extension and PPS capture running once initialized. */
    TIM2->DIER |= TIM_DIER_UIE | TIM_DIER_CC3IE;
    TIM2->CR1 |= TIM_CR1_CEN;
    return RT_EOK;
}

static rt_ssize_t timebase_read(rt_device_t dev, rt_off_t pos, void *buffer,
                                rt_size_t size)
{
    struct timebase_instant *instant = buffer;
    uint32_t primask;
    uint32_t hi;
    uint32_t cnt;
    uint32_t sr1;
    uint32_t sr2;

    (void)dev;
    (void)pos;
    if (!instant || size < sizeof(*instant) || !g_ready)
    {
        return -RT_EINVAL;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    hi = g_hi;
    sr1 = TIM2->SR;
    cnt = TIM2->CNT;
    sr2 = TIM2->SR;
    if ((sr1 ^ sr2) & TIM_SR_UIF)
    {
        cnt = TIM2->CNT;
    }
    if (sr2 & TIM_SR_UIF)
    {
        hi++;
    }
    instant->ticks = ((uint64_t)hi << 32) | cnt;
    instant->epoch = g_epoch;
    __set_PRIMASK(primask);
    return (rt_ssize_t)sizeof(*instant);
}

static rt_err_t timebase_set_compare(const struct timebase_compare *compare)
{
    uint32_t level, enable, flag;
    unsigned index;

    if (!compare || compare->channel < 1u || compare->channel > 2u)
    {
        return -RT_EINVAL;
    }
    index = compare->channel - 1u;
    enable = TIM_DIER_CC1IE << index;
    flag = TIM_SR_CC1IF << index;
    level = __get_PRIMASK();
    __disable_irq();
    TIM2->DIER &= ~enable;
    g_compare[index] = NULL;
    if (compare->callback)
    {
        if (index == 0u)
        {
            TIM2->CCR1 = compare->tick;
        }
        else
        {
            TIM2->CCR2 = compare->tick;
        }
    }
    TIM2->SR = ~flag;
    if (compare->callback)
    {
        /* Reject deadlines already passed during setup or more than half a
         * counter period away. Never leave such a compare armed for a wrap. */
        uint32_t remaining = compare->tick - TIM2->CNT;
        if (!remaining || remaining > INT32_MAX)
        {
            __set_PRIMASK(level);
            return -RT_ETIMEOUT;
        }
        g_compare[index] = compare->callback;
        TIM2->DIER |= enable;
    }
    __set_PRIMASK(level);
    return RT_EOK;
}

static rt_err_t timebase_control(rt_device_t dev, int cmd, void *args)
{
    (void)dev;
    switch (cmd)
    {
    case TIMEBASE_CTRL_GET_TICK_HZ:
        if (!args)
        {
            return -RT_EINVAL;
        }
        *(uint32_t *)args = g_tick_hz;
        return RT_EOK;

    case TIMEBASE_CTRL_READ_CAPTURE:
        if (!args)
        {
            return -RT_EINVAL;
        }
        if (g_tail == g_head)
        {
            return -RT_EEMPTY;
        }
        __DMB();
        *(struct timebase_capture *)args = g_ring[g_tail];
        __DMB();
        g_tail = (g_tail + 1u) % (sizeof(g_ring) / sizeof(g_ring[0]));
        return RT_EOK;

    case TIMEBASE_CTRL_GET_CAPTURE_LOST:
        if (!args)
        {
            return -RT_EINVAL;
        }
        *(uint32_t *)args = g_lost;
        return RT_EOK;

    case TIMEBASE_CTRL_SET_IMU_CAPTURE:
    {
        const struct timebase_imu_capture_config *config = args;
        uint32_t level;
        if (!g_ready || !config)
        {
            return -RT_EINVAL;
        }
        level = __get_PRIMASK();
        __disable_irq();
        TIM2->DIER &= ~TIM_DIER_CC4IE;
        TIM2->SR = ~(TIM_SR_CC4IF | TIM_SR_CC4OF);
        g_imu_capture = config->callback;
        if (g_imu_capture)
        {
            TIM2->DIER |= TIM_DIER_CC4IE;
        }
        __set_PRIMASK(level);
        return RT_EOK;
    }

    case TIMEBASE_CTRL_SET_COMPARE:
        return g_ready ? timebase_set_compare(args) : -RT_EIO;

    case TIMEBASE_CTRL_GET_STATS:
        if (!args)
        {
            return -RT_EINVAL;
        }
        ((struct timebase_stats *)args)->tick_hz = g_tick_hz;
        ((struct timebase_stats *)args)->capture_lost = g_lost;
        ((struct timebase_stats *)args)->imu_capture_enabled = g_imu_capture != NULL;
        return RT_EOK;

    default:
        return -RT_ENOSYS;
    }
}

static const struct rt_device_ops timebase_ops =
{
    .init = timebase_init,
    .read = timebase_read,
    .control = timebase_control,
};

static int timebase_register(void)
{
    g_timebase_dev.type = RT_Device_Class_Miscellaneous;
    g_timebase_dev.ops = &timebase_ops;
    return rt_device_register(&g_timebase_dev, TIMEBASE_DEVICE_NAME,
                              RT_DEVICE_FLAG_RDONLY);
}
INIT_DEVICE_EXPORT(timebase_register);
#endif /* !FLIGHT_DRIVER_CODEC_TEST */
