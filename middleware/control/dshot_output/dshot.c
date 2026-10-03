/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 电调输出板级引擎实现 (接口约定见 dshot.h 头注)
 *
 * 实现要点:
 * - PWM 模式: TIM 1MHz 计数 @ DSHOT_PWM_RATE_HZ, CCR=脉宽 us, 写即生效;
 * - DShot 模式: TIM 位率计数 (DShot600@275MHz: ARR=458, bit1=75%/bit0=37.5%
 *   占空), 每输出一帧: 17 组 x 4 电机 CCR 值 (16 bit + 1 复位低) 打包进
 *   DMA 缓冲, HAL_DMA_Start 后由 TIM 更新事件经 DMAR 寄存器逐 bit 突发
 *   装载 CCR1..4 (4 电机严格同步), 帧尾 CCR=0 恒低到下一帧;
 * - 帧缓冲 32B 对齐 + DCache Clean (DMA1 不可达 DTCM, 缓冲放普通 SRAM);
 * - 上一帧未完成时的写请求: 丢弃并 busy_cnt++ (输出周期须 > 帧时长,
 *   DShot600 一帧约 28us)。
 *
 * 硬件资源占位说明与调整指引见 dshot_hw.h 头注。
 * 参考: ref/FMT-Firmware src/hal/actuator (帧编码/命令值域约定)。
 */
#include <math.h>
#include <string.h>

#include "dshot.h"
#include "dshot_enc.h"
#include "dshot_hw.h"
#include "drv_dma.h"

/* ---------------- 内部状态 ---------------- */

static struct
{
    rt_bool_t      inited;
    rt_bool_t      armed;
    int            proto;
    rt_uint32_t    write_cnt;
    rt_uint32_t    bad_cnt;
    rt_uint32_t    busy_cnt;
    /* DShot 位时序 (计数) */
    rt_uint32_t    arr, c1, c0;
    /* PWM: 每 us 的计数值 (PSC=1MHz 时为 1) */
    TIM_HandleTypeDef tim;
    DMA_HandleTypeDef  dma;
    rt_bool_t      dma_ready;
} g_out;

/* DMAR 突发装载缓冲: 17 组 (16 bit + 复位) x 4 电机, 32B 对齐 (DCache 行) */
static rt_uint32_t s_frame[17 * DSHOT_CH_NUM] __attribute__((aligned(32)));

/* 引脚表: m0..m3 -> TIM1_CH1..CH4 (dshot_hw.h, 2026-10-04 硬件定案) */
static const struct
{
    GPIO_TypeDef *port;
    rt_uint16_t   pin;
} s_pins[DSHOT_CH_NUM] =
{
    { GPIOE, GPIO_PIN_9 },  /* m0 FR -> TIM1_CH1 */
    { GPIOE, GPIO_PIN_11 }, /* m1 FL -> TIM1_CH2 */
    { GPIOE, GPIO_PIN_13 }, /* m2 RR -> TIM1_CH3 */
    { GPIOE, GPIO_PIN_14 }, /* m3 RL -> TIM1_CH4 */
};

static const struct stm32_dma_config s_dma_cfg =
{
    .Instance              = DSHOT_DMA_INSTANCE,
    .dma_rcc               = DSHOT_DMA_RCC,
    .dma_irq               = DSHOT_DMA_IRQ,
    .priority              = DMA_PRIORITY_HIGH,
    .preempt_priority      = DSHOT_DMA_PREEMPT,
    .sub_priority          = 0,
    .request               = DSHOT_DMA_REQUEST,
    .direction             = DMA_MEMORY_TO_PERIPH,
    .periph_inc            = DMA_PINC_DISABLE,
    .mem_inc               = DMA_MINC_ENABLE,
    .periph_data_alignment = DMA_PDATAALIGN_WORD,
    .mem_data_alignment    = DMA_MDATAALIGN_WORD,
    .mode                  = DMA_NORMAL,
    .fifo_mode             = DMA_FIFOMODE_DISABLE,
    .fifo_threshold        = DMA_FIFO_THRESHOLD_FULL,
    .mem_burst             = DMA_MBURST_SINGLE,
    .periph_burst          = DMA_PBURST_SINGLE,
};

/* ---------------- 内部工具 ---------------- */

static void out_idle_low(void)
{
    for (int i = 0; i < DSHOT_CH_NUM; i++)
        __HAL_TIM_SET_COMPARE(&g_out.tim, TIM_CHANNEL_1 + (i * 4), 0);
}

static int proto_is_dshot(int proto)
{
    return proto != DSHOT_OUT_PWM;
}

static rt_uint32_t dshot_bit_rate_hz(int proto)
{
    switch (proto)
    {
    case DSHOT_OUT_DSHOT600: return 600000;
    case DSHOT_OUT_DSHOT300: return 300000;
    default:                 return 150000;
    }
}

/* ---------------- 对外接口 ---------------- */

int dshot_out_init(enum dshot_out_proto proto)
{
    GPIO_InitTypeDef gpio;
    TIM_OC_InitTypeDef oc;

    if (g_out.inited)
        return -RT_EBUSY;
    if ((unsigned)proto > DSHOT_OUT_DSHOT600)
        return -RT_ERROR;

    memset(&g_out, 0, sizeof(g_out));
    memset(&g_out.tim, 0, sizeof(g_out.tim));
    memset(&g_out.dma, 0, sizeof(g_out.dma));
    g_out.proto = (int)proto;

    __HAL_RCC_GPIOE_CLK_ENABLE();
    DSHOT_TIM_CLK_ENABLE();

    /* 引脚: AF 推挽, DShot 沿速率要求 very high */
    gpio.Pin = 0;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = DSHOT_PIN_AF;
    for (int i = 0; i < DSHOT_CH_NUM; i++)
    {
        gpio.Pin = s_pins[i].pin;
        HAL_GPIO_Init(s_pins[i].port, &gpio);
    }

    g_out.tim.Instance = DSHOT_TIM;
    g_out.tim.Init.RepetitionCounter = 0;
    g_out.tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_out.tim.Init.CounterMode = TIM_COUNTERMODE_UP;

    if (proto_is_dshot(proto))
    {
        rt_uint32_t rate = dshot_bit_rate_hz(proto);
        g_out.tim.Init.Prescaler = 0;
        g_out.arr = (DSHOT_TIM_CLK_HZ + rate / 2) / rate - 1;
        g_out.tim.Init.Period = g_out.arr;
        g_out.c1 = (g_out.arr * 3 + 2) / 4;   /* bit1 ~75% 占空 */
        g_out.c0 = (g_out.arr * 3 + 4) / 8;   /* bit0 ~37.5% 占空 */
    }
    else
    {
        g_out.tim.Init.Prescaler = DSHOT_TIM_CLK_HZ / 1000000UL - 1;  /* 1MHz */
        g_out.tim.Init.Period = 1000000UL / DSHOT_PWM_RATE_HZ - 1;
    }

    if (HAL_TIM_PWM_Init(&g_out.tim) != HAL_OK)
        return -RT_ERROR;

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    for (int i = 0; i < DSHOT_CH_NUM; i++)
    {
        if (HAL_TIM_PWM_ConfigChannel(&g_out.tim, &oc,
                                      TIM_CHANNEL_1 + (i * 4)) != HAL_OK)
            return -RT_ERROR;
        HAL_TIM_PWM_Start(&g_out.tim, TIM_CHANNEL_1 + (i * 4));
    }

    if (proto_is_dshot(proto))
    {
        rt_err_t rc = stm32_dma_setup(&g_out.dma, &g_out.tim,
                                      &g_out.tim.hdma[TIM_DMA_ID_UPDATE],
                                      &s_dma_cfg);
        if (rc != RT_EOK)
            return (int)rc;
        g_out.dma_ready = RT_TRUE;

        /* DMAR 突发: 基址 CCR1, 长度 4 寄存器 (CCR1..CCR4) */
        DSHOT_TIM->DCR = 0;
        DSHOT_TIM->DCR = ((((rt_uint32_t)&DSHOT_TIM->CCR1 -
                            (rt_uint32_t)DSHOT_TIM) / 4U) | (3U << 8));
        __HAL_TIM_ENABLE_DMA(&g_out.tim, TIM_DMA_UPDATE);
    }

    g_out.inited = RT_TRUE;      /* 初始 disarm: 输出恒低 */
    return 0;
}

void dshot_out_deinit(void)
{
    if (!g_out.inited)
        return;
    dshot_out_disarm();
    for (int i = 0; i < DSHOT_CH_NUM; i++)
        HAL_TIM_PWM_Stop(&g_out.tim, TIM_CHANNEL_1 + (i * 4));
    if (g_out.dma_ready)
    {
        HAL_DMA_Abort(&g_out.dma);
        HAL_DMA_DeInit(&g_out.dma);
        g_out.dma_ready = RT_FALSE;
    }
    HAL_TIM_PWM_DeInit(&g_out.tim);
    g_out.inited = RT_FALSE;
}

int dshot_out_arm(void)
{
    if (!g_out.inited)
        return -RT_ERROR;
    g_out.armed = RT_TRUE;
    return 0;
}

void dshot_out_disarm(void)
{
    if (!g_out.inited)
        return;
    g_out.armed = RT_FALSE;
    if (g_out.dma_ready)
        HAL_DMA_Abort(&g_out.dma);      /* 丢弃在途帧 */
    out_idle_low();                     /* CCR=0 -> 恒低 */
}

static int frame_dma_send(const unsigned short fr[DSHOT_CH_NUM])
{
    /* 打包: bit k (MSB 先发) x 4 电机; 第 17 组为复位低电平 */
    for (int k = 0; k < 17; k++)
        for (int m = 0; m < DSHOT_CH_NUM; m++)
        {
            rt_uint32_t c = 0;
            if (k < 16 && ((fr[m] >> (15 - k)) & 1U))
                c = g_out.c1;
            else if (k < 16)
                c = g_out.c0;
            s_frame[k * DSHOT_CH_NUM + m] = c;
        }

    SCB_CleanDCache_by_Addr((rt_uint32_t *)s_frame, sizeof(s_frame));

    /* 上一帧未完成: 丢弃旧帧重发 (输出周期须大于帧时长) */
    if (HAL_DMA_GetState(&g_out.dma) == HAL_DMA_STATE_BUSY)
    {
        HAL_DMA_Abort(&g_out.dma);
        g_out.busy_cnt++;
    }

    if (HAL_DMA_Start(&g_out.dma, (rt_uint32_t)s_frame,
                      (rt_uint32_t)&DSHOT_TIM->DMAR,
                      17 * DSHOT_CH_NUM) != HAL_OK)
        return -RT_ERROR;
    return 0;
}

int dshot_out_write(const double u[4])
{
    unsigned short fr[DSHOT_CH_NUM];

    if (!g_out.inited || !g_out.armed)
        return -RT_ERROR;

    if (proto_is_dshot(g_out.proto))
    {
        for (int m = 0; m < DSHOT_CH_NUM; m++)
        {
            double v = u[m];
            if (!isfinite(v) || v <= 0.0)
            {
                if (!isfinite(v))
                    g_out.bad_cnt++;
                fr[m] = dshot_enc_frame(DSHOT_CMD_MOTOR_STOP, 0);
                continue;
            }
            if (v > 1.0)
                v = 1.0;
            fr[m] = dshot_enc_frame_norm(v, 0);
        }
        g_out.write_cnt++;
        return frame_dma_send(fr);
    }

    /* PWM: 1000~2000us */
    for (int m = 0; m < DSHOT_CH_NUM; m++)
    {
        double v = u[m];
        if (!isfinite(v))
        {
            g_out.bad_cnt++;
            v = 0.0;
        }
        if (v < 0.0)
            v = 0.0;
        if (v > 1.0)
            v = 1.0;
        double us = DSHOT_PWM_US_MIN + v * (DSHOT_PWM_US_MAX - DSHOT_PWM_US_MIN);
        __HAL_TIM_SET_COMPARE(&g_out.tim, TIM_CHANNEL_1 + (m * 4),
                              (rt_uint32_t)(us + 0.5));  /* 1MHz 计数: 1 count = 1us */
    }
    g_out.write_cnt++;
    return 0;
}

int dshot_out_write_raw(unsigned value)
{
    unsigned short fr[DSHOT_CH_NUM];

    if (!g_out.inited || !g_out.armed)
        return -RT_ERROR;

    if (proto_is_dshot(g_out.proto))
    {
        for (int m = 0; m < DSHOT_CH_NUM; m++)
            fr[m] = dshot_enc_frame(value, 0);
        g_out.write_cnt++;
        return frame_dma_send(fr);
    }

    /* PWM 协议下命令值 (0..47) 映射为最小脉宽静默 */
    __HAL_TIM_SET_COMPARE(&g_out.tim, TIM_CHANNEL_1, (rt_uint32_t)DSHOT_PWM_US_MIN);
    __HAL_TIM_SET_COMPARE(&g_out.tim, TIM_CHANNEL_1 + 4, (rt_uint32_t)DSHOT_PWM_US_MIN);
    __HAL_TIM_SET_COMPARE(&g_out.tim, TIM_CHANNEL_1 + 8, (rt_uint32_t)DSHOT_PWM_US_MIN);
    __HAL_TIM_SET_COMPARE(&g_out.tim, TIM_CHANNEL_1 + 12, (rt_uint32_t)DSHOT_PWM_US_MIN);
    g_out.write_cnt++;
    return 0;
}

rt_bool_t dshot_out_armed(void)
{
    return g_out.armed;
}

rt_bool_t dshot_out_inited(void)
{
    return g_out.inited;
}

void dshot_out_get_status(struct dshot_out_status *st)
{
    st->inited = g_out.inited;
    st->armed = g_out.armed;
    st->proto = g_out.proto;
    st->write_cnt = g_out.write_cnt;
    st->bad_cnt = g_out.bad_cnt;
    st->busy_cnt = g_out.busy_cnt;
}

/* ------------------------- FinSH 调试命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>
#include "param_calib.h"      /* calib_parse_num (项目风格: 不引 libc atof) */

#define LOG_TAG "dshot"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

static const char *proto_name(int p)
{
    switch (p)
    {
    case DSHOT_OUT_PWM: return "pwm400";
    case DSHOT_OUT_DSHOT600: return "dshot600";
    case DSHOT_OUT_DSHOT300: return "dshot300";
    default: return "dshot150";
    }
}

static void dshot(int argc, char **argv)
{
    if (argc < 2)
    {
        struct dshot_out_status st;
        dshot_out_get_status(&st);
        LOG_I("=== dshot output (TIM1 PE9/PE11/PE13/PE14, dshot_hw.h) ===");
        LOG_I("state : inited=%d armed=%d proto=%s", st.inited, st.armed,
              proto_name(st.proto));
        if (st.inited && proto_is_dshot(st.proto))
            LOG_I("dshot : arr=%lu c1=%lu c0=%lu (%.0f kbit/s)",
                  (unsigned long)g_out.arr, (unsigned long)g_out.c1,
                  (unsigned long)g_out.c0,
                  (double)DSHOT_TIM_CLK_HZ / (g_out.arr + 1) / 1000.0);
        LOG_I("stat  : write=%u bad=%u busy=%u", st.write_cnt, st.bad_cnt,
              st.busy_cnt);
        LOG_W("usage: dshot init [pwm|dshot600|dshot300|dshot150] | arm | "
              "disarm | w <u0..u3> | raw <v> | deinit");
        return;
    }

    if (!rt_strcmp(argv[1], "init") && argc >= 3)
    {
        int p = -1;
        for (int i = 0; i <= DSHOT_OUT_DSHOT600; i++)
            if (!rt_strcmp(argv[2], proto_name(i)))
                p = i;
        if (p < 0)
        {
            LOG_W("proto: pwm|dshot600|dshot300|dshot150");
            return;
        }
        int rc = dshot_out_init((enum dshot_out_proto)p);
        if (rc == 0)
            LOG_I("inited, proto=%s, disarmed (恒低)", proto_name(p));
        else
            LOG_W("init failed: %d", rc);
        return;
    }

    if (!rt_strcmp(argv[1], "arm"))
    {
        if (dshot_out_arm() == 0)
            LOG_I("armed");
        else
            LOG_W("not inited");
        return;
    }
    if (!rt_strcmp(argv[1], "disarm"))
    {
        dshot_out_disarm();
        LOG_I("disarmed (输出恒低)");
        return;
    }
    if (!rt_strcmp(argv[1], "deinit"))
    {
        dshot_out_deinit();
        LOG_I("deinited");
        return;
    }

    if (!rt_strcmp(argv[1], "w") && argc >= 6)
    {
        double u[4];
        for (int i = 0; i < 4; i++)
            u[i] = calib_parse_num(argv[2 + i]);
        int rc = dshot_out_write(u);
        if (rc == 0)
            LOG_I("u = (%.3f %.3f %.3f %.3f)", u[0], u[1], u[2], u[3]);
        else
            LOG_W("write rejected (%d): inited+armed?", rc);
        return;
    }

    if (!rt_strcmp(argv[1], "raw") && argc >= 3)
    {
        int v = (int)calib_parse_num(argv[2]);
        if (dshot_out_write_raw((unsigned)v) == 0)
            LOG_I("raw value %d -> %s", v,
                  dshot_enc_is_cmd((unsigned)v) ? "命令" : "节流");
        else
            LOG_W("rejected: inited+armed?");
        return;
    }

    LOG_W("usage: dshot [init <proto>|arm|disarm|w u0 u1 u2 u3|raw <v>|deinit]");
}
MSH_CMD_EXPORT(dshot, ESC output PWM/DShot: dshot [init <proto>|arm|disarm|w ...|raw <v>]);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */
