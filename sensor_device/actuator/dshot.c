/* Four-channel DShot transport on TIM1 with DMA2_Stream2 (TIM1_UP).
 *
 * Native RT-Thread device "dshot". A single writer submits a complete
 * four-channel frame with rt_device_write(); scheduled one-shot frames use
 * DSHOT_CTRL_SCHEDULE. This file contains transport only: no arm authority and
 * no motor policy.
 */
#include <stdint.h>

#define DSHOT_BUF_WORDS 68u

static void dshot_encode_dma(const uint16_t frame[4], uint32_t t0h, uint32_t t1h,
                             uint32_t out[DSHOT_BUF_WORDS])
{
    /* The most significant bit is preloaded. DMA starts with bit 1. */
    for (unsigned bit = 1u; bit < 16u; ++bit)
    {
        for (unsigned channel = 0u; channel < 4u; ++channel)
        {
            out[(bit - 1u) * 4u + channel] =
                (frame[channel] & (0x8000u >> bit)) ? t1h : t0h;
        }
    }
    for (unsigned index = 60u; index < DSHOT_BUF_WORDS; ++index)
    {
        out[index] = 0u;
    }
}

#if !defined(FLIGHT_DRIVER_CODEC_TEST)
#include "dshot.h"
#include "timer_capture.h"
#include "board_device.h"
#include <board.h>
#include <string.h>

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
static TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_up;

static struct rt_device g_dshot_dev;
static rt_device_t g_timebase;
static uint32_t g_tick_hz;

static struct
{
    volatile uint8_t active;
    volatile uint8_t dma_done;
    volatile uint8_t pending;
    volatile uint8_t ready;
    uint32_t arr;
    uint32_t t0h;
    uint32_t t1h;
    uint32_t kernel_hz;
    uint64_t start_raw;
    uint64_t stop_threshold;
    uint64_t next_allowed;
    uint64_t scheduled;
    uint64_t expires;
    volatile uint32_t faults;
    volatile uint32_t frames;
} g_dshot;

static uint32_t g_dshot_buf[DSHOT_BUF_WORDS] DMA_BUFFER;
static uint32_t g_first_duty[4];

static int dshot_launch(uint64_t nominal_start);

static rt_err_t set_compare(uint8_t channel, uint64_t tick, void (*callback)(void))
{
    struct timebase_compare compare = {channel, (uint32_t)tick, callback};
    return rt_device_control(g_timebase, TIMEBASE_CTRL_SET_COMPARE, &compare);
}

static int timebase_now(uint64_t *ticks)
{
    struct timebase_instant instant;
    if (!g_timebase)
    {
        return -1;
    }
    if (rt_device_read(g_timebase, 0, &instant, sizeof(instant)) != (rt_ssize_t)sizeof(instant))
    {
        return -1;
    }
    *ticks = instant.ticks;
    return 0;
}

static void dshot_stop(void)
{
    if (!g_dshot.ready)
    {
        return;
    }
    uint32_t level = __get_PRIMASK();
    __disable_irq();
    (void)set_compare(1u, 0u, NULL);
    (void)set_compare(2u, 0u, NULL);
    g_dshot.pending = 0u;
    TIM1->DIER &= ~(TIM_DIER_UDE | TIM_DIER_UIE);
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
    /* Bounded ISR stop: HAL_DMA_Abort polls on a tick and is unsuitable here. */
    ((DMA_Stream_TypeDef *)hdma_tim1_up.Instance)->CR &= ~DMA_SxCR_EN;
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0u;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0u;
    hdma_tim1_up.State = HAL_DMA_STATE_READY;
    hdma_tim1_up.Lock = HAL_UNLOCKED;
    g_dshot.active = 0u;
    __set_PRIMASK(level);
}

static void dshot_done(DMA_HandleTypeDef *dma)
{
    (void)dma;
    TIM1->DIER &= ~TIM_DIER_UDE;
    g_dshot.dma_done = 1u;
    TIM1->SR = ~TIM_SR_UIF;
    TIM1->DIER |= TIM_DIER_UIE;
}

void DMA2_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_tim1_up);
}

static void dshot_error(DMA_HandleTypeDef *dma)
{
    (void)dma;
    g_dshot.faults++;
    dshot_stop();
}

void TIM1_UP_IRQHandler(void)
{
    uint64_t now;

    TIM1->SR = ~TIM_SR_UIF;
    if (!g_dshot.active)
    {
        return;
    }
    if (timebase_now(&now))
    {
        dshot_error(0);
        return;
    }
    if (g_dshot.dma_done && now >= g_dshot.stop_threshold)
    {
        if (now - g_dshot.start_raw > (uint64_t)g_tick_hz * 80u / 1000000u)
        {
            g_dshot.faults++;
        }
        dshot_stop();
    }
}

static void dshot_timeout(void)
{
    if (g_dshot.active)
    {
        /* Independent deadline even if DMA TC/EOT never arrives. */
        g_dshot.faults++;
        dshot_stop();
    }
}

static void dshot_start_compare(void)
{
    if (g_dshot.pending)
    {
        uint64_t now;
        g_dshot.pending = 0u;
        if (!timebase_now(&now) && now >= g_dshot.scheduled && now <= g_dshot.expires)
        {
            (void)dshot_launch(g_dshot.scheduled);
        }
        else
        {
            g_dshot.faults++;
        }
    }
}

static void dshot_prepare(const uint16_t frame[4])
{
    uint32_t ch;
    dshot_encode_dma(frame, g_dshot.t0h, g_dshot.t1h, g_dshot_buf);
    for (ch = 0u; ch < 4u; ch++)
    {
        g_first_duty[ch] = (frame[ch] & 0x8000u) ? g_dshot.t1h : g_dshot.t0h;
    }
}

static int dshot_launch(uint64_t nominal_start)
{
    uint64_t now;
    uint32_t ch;

    dshot_stop();
    for (ch = 0u; ch < 4u; ch++)
    {
        *(&TIM1->CCR1 + ch) = g_first_duty[ch];
    }
    TIM1->CNT = TIM1->ARR;
    TIM1->SR = 0u;
    TIM1->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_4TRANSFERS;
    hdma_tim1_up.XferCpltCallback = dshot_done;
    hdma_tim1_up.XferHalfCpltCallback = 0;
    hdma_tim1_up.XferErrorCallback = dshot_error;
    __DMB();
    if (HAL_DMA_Start_IT(&hdma_tim1_up, (uint32_t)g_dshot_buf, (uint32_t)&TIM1->DMAR,
                         DSHOT_BUF_WORDS) != HAL_OK)
    {
        g_dshot.faults++;
        return -RT_EIO;
    }
    g_dshot.dma_done = 0u;
    g_dshot.active = 1u;
    TIM1->DIER |= TIM_DIER_UDE;
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM1->BDTR |= TIM_BDTR_MOE; /* active CCRs still zero */
    TIM1->CR1 |= TIM_CR1_CEN;

    if (timebase_now(&now))
    {
        dshot_stop();
        return -RT_EIO;
    }
    g_dshot.start_raw = now;
    g_dshot.stop_threshold = now + (((uint64_t)(1u + 18u * (g_dshot.arr + 1u)) *
                                     g_tick_hz + g_dshot.kernel_hz - 1u) /
                                    g_dshot.kernel_hz) + 2u;
    g_dshot.next_allowed = nominal_start + ((uint64_t)g_tick_hz + 399u) / 400u;
    if (set_compare(2u, now + (uint64_t)g_tick_hz * 80u / 1000000u,
                    dshot_timeout) != RT_EOK)
    {
        dshot_error(NULL);
        return -RT_EIO;
    }
    g_dshot.frames++;
    return RT_EOK;
}

static int dshot_submit(const uint16_t frame[4])
{
    uint64_t now;

    if (!frame || !g_dshot.ready || g_dshot.active || g_dshot.pending || g_dshot.faults)
    {
        return -RT_EBUSY;
    }
    if (timebase_now(&now) || now < g_dshot.next_allowed)
    {
        return -RT_EBUSY;
    }
    dshot_prepare(frame);
    return dshot_launch(now);
}

static int dshot_schedule(const struct dshot_schedule *schedule)
{
    uint64_t now;

    if (!schedule || !g_dshot.ready || g_dshot.active || g_dshot.pending || g_dshot.faults)
    {
        return -RT_EBUSY;
    }
    if (timebase_now(&now) || schedule->start_tick <= now ||
        schedule->start_tick - now > 0x7fffffffu ||
        schedule->start_tick < g_dshot.next_allowed ||
        schedule->expires_tick < schedule->start_tick ||
        schedule->expires_tick - schedule->start_tick > ((uint64_t)g_tick_hz + 399u) / 400u)
    {
        return -RT_EINVAL;
    }
    dshot_prepare(schedule->frame);
    g_dshot.scheduled = schedule->start_tick;
    g_dshot.expires = schedule->expires_tick;
    g_dshot.pending = 1u;
    if (set_compare(1u, schedule->start_tick, dshot_start_compare) != RT_EOK)
    {
        dshot_stop();
        return -RT_EINVAL;
    }
    return RT_EOK;
}

static rt_err_t dshot_init(rt_device_t dev)
{
    TIM_OC_InitTypeDef oc;
    uint32_t n;
    uint32_t kernel_hz;

    (void)dev;
    if (g_dshot.ready)
    {
        return RT_EOK;
    }

    g_timebase = rt_device_find(TIMEBASE_DEVICE_NAME);
    if (!g_timebase || rt_device_open(g_timebase, RT_DEVICE_OFLAG_RDONLY) != RT_EOK)
    {
        return -RT_EIO;
    }
    if (rt_device_control(g_timebase, TIMEBASE_CTRL_GET_TICK_HZ, &g_tick_hz) != RT_EOK ||
        !g_tick_hz)
    {
        goto close_timebase;
    }

    {
        RCC_ClkInitTypeDef clock_cfg;
        uint32_t latency;
        HAL_RCC_GetClockConfig(&clock_cfg, &latency);
        kernel_hz = HAL_RCC_GetPCLK2Freq() *
                    ((clock_cfg.APB2CLKDivider == RCC_APB2_DIV1) ? 1u : 2u);
        if (RCC->CFGR & RCC_CFGR_TIMPRE)
        {
            kernel_hz = HAL_RCC_GetPCLK2Freq() * 4u;
        }
    }
    if (!kernel_hz)
    {
        goto close_timebase;
    }

    n = (kernel_hz + 300000u / 2u) / 300000u;
    if (n < 16u || n > 65535u)
    {
        goto close_timebase;
    }

    memset(&htim1, 0, sizeof(htim1));
    htim1.Instance = TIM1;
    htim1.Init.Period = n - 1u;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        goto close_timebase;
    }
    HAL_TIM_MspPostInit(&htim1);

    memset(&oc, 0, sizeof(oc));
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_4) != HAL_OK)
    {
        (void)HAL_TIM_PWM_DeInit(&htim1);
        goto close_timebase;
    }
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
    TIM1->BDTR |= TIM_BDTR_OSSI | TIM_BDTR_OSSR;

    g_dshot.arr = n - 1u;
    g_dshot.t0h = (3u * n + 4u) / 8u;
    g_dshot.t1h = (3u * n + 2u) / 4u;
    g_dshot.kernel_hz = kernel_hz;

    __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 2u, 0u);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 2u, 0u);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

    g_dshot.ready = 1u;
    dshot_stop();
    return RT_EOK;

close_timebase:
    (void)rt_device_close(g_timebase);
    g_timebase = NULL;
    return -RT_EIO;
}

static rt_err_t dshot_open(rt_device_t dev, rt_uint16_t oflag)
{
    (void)dev;
    (void)oflag;
    return g_dshot.ready ? RT_EOK : -RT_EIO;
}

static rt_err_t dshot_close(rt_device_t dev)
{
    (void)dev;
    dshot_stop();
    return RT_EOK;
}

static rt_ssize_t dshot_write(rt_device_t dev, rt_off_t pos, const void *buffer,
                              rt_size_t size)
{
    uint32_t level;
    int result;
    (void)dev;
    (void)pos;
    if (!buffer || size != 4u * sizeof(uint16_t))
    {
        return -RT_EINVAL;
    }
    level = __get_PRIMASK();
    __disable_irq();
    result = dshot_submit(buffer);
    __set_PRIMASK(level);
    return result == RT_EOK ? (rt_ssize_t)size : result;
}

static rt_err_t dshot_control(rt_device_t dev, int cmd, void *args)
{
    (void)dev;
    switch (cmd)
    {
    case DSHOT_CTRL_SCHEDULE:
    {
        uint32_t level;
        int result;
        if (!args)
        {
            return -RT_EINVAL;
        }
        if (dev->ref_count == 0u)
        {
            return -RT_EIO;
        }
        level = __get_PRIMASK();
        __disable_irq();
        result = dshot_schedule(args);
        __set_PRIMASK(level);
        return result;
    }

    case DSHOT_CTRL_STOP:
        dshot_stop();
        return RT_EOK;

    case DSHOT_CTRL_GET_STATS:
        if (!args)
        {
            return -RT_EINVAL;
        }
        ((struct dshot_stats *)args)->faults = g_dshot.faults;
        ((struct dshot_stats *)args)->frames = g_dshot.frames;
        ((struct dshot_stats *)args)->arr = g_dshot.arr;
        ((struct dshot_stats *)args)->active = g_dshot.active;
        ((struct dshot_stats *)args)->pending = g_dshot.pending;
        ((struct dshot_stats *)args)->ready = g_dshot.ready;
        return RT_EOK;

    case DSHOT_CTRL_GET_ARR:
        if (!args)
        {
            return -RT_EINVAL;
        }
        *(uint32_t *)args = g_dshot.arr;
        return RT_EOK;

    default:
        return -RT_ENOSYS;
    }
}

static const struct rt_device_ops dshot_ops =
{
    .init = dshot_init,
    .open = dshot_open,
    .close = dshot_close,
    .write = dshot_write,
    .control = dshot_control,
};

static int dshot_register(void)
{
    g_dshot_dev.type = RT_Device_Class_Miscellaneous;
    g_dshot_dev.ops = &dshot_ops;
    return rt_device_register(&g_dshot_dev, DSHOT_DEVICE_NAME,
                              RT_DEVICE_FLAG_WRONLY | RT_DEVICE_FLAG_STANDALONE);
}
INIT_DEVICE_EXPORT(dshot_register);
#endif /* !FLIGHT_DRIVER_CODEC_TEST */
