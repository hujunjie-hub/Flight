/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 时间同步基座实现 (设计说明与接口见 timebase.h 与根 README "时间同步框架")
 */

#include "timebase.h"
#include "board.h"

#define LOG_TAG "timebase"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#include <math.h>
#include <string.h>

/* ------------------------- 配置 ------------------------- */

/* TIM2 时钟: APB1 定时器时钟 275MHz, PSC=274 -> 275MHz/275 = 1MHz (1us/LSB) */
#define TB_TIM_PSC              274u
#define TB_TIM_PERIOD           0xFFFFFFFFUL      /* 32bit 满量程, ~71.6min 回绕 */

/* 门槛常量 (README "PPS 配对与滑窗校准") */
#define TB_GATE_ARRIVAL_US      900000ULL         /* 门槛2: 语句到达距 PPS 沿 < 0.9s */
#define TB_GATE_INTVL_US        1000LL            /* 门槛3: 间隔容忍 1ms @1s 间隔 */
#define TB_GATE_SLOPE_PPM       100e-6            /* 门槛4: |scale-1| <= 100ppm */
#define TB_FIT_RESID_US         10.0              /* 拟合残差剔除门槛, 10us */
/* UTC 间隔 > 10s 才判失锁重启: 整秒句解析率 ~50% 时 2-4s 间隙常见,
 * 1.5s 门限会反复清滑窗 -> 映射间歇失效 -> GNSS 观测断供;
 * 间隔容差按 du 比例缩放 (tol = 1ms x du), 10s 间隙下晶振误差余量 >500x */
#define TB_RESTART_UTC_GAP_US   10000000ULL        /* 失锁后再捕获门限 */

/* 半量程判别阈值: 双读溢出计数不一致时, CNT/CCR1 小于半量程视为已回绕 */
#define TB_HALF_PERIOD          0x80000000UL

/* ------------------------- TIM2 句柄与运行状态 ------------------------- */

TIM_HandleTypeDef htim2;                 /* 本模块独占 (drv_tim 的 hwtimer 未启用) */

static volatile rt_uint32_t s_ovf;       /* TIM2 溢出累计 (高位), 更新中断内递增 */

/* 最近一次 PPS 硬件捕获 (捕获中断写, 任意线程关中断读) */
static struct
{
    volatile rt_uint32_t seq;            /* 捕获序号, 0 = 上电以来无 PPS */
    rt_uint64_t          t_mcu;          /* T_MCU,PPS, us */
} s_pps;

/* pps_ref 滑窗 (旧移新进, 解析线程独占写) + 映射缓存 */
static struct
{
    struct timebase_pair win[TIMEBASE_PPS_WINDOW];
    rt_uint8_t           valid_cnt;      /* 滑窗内有效配对数 (0~3) */
    struct clock_map     map;            /* 基准点 = 最新配对, 斜率 = 窗口最小二乘 */
    rt_uint64_t          last_pair_mcu;  /* 最近一次成功配对的 T_MCU (失锁计时) */

    struct
    {
        rt_uint32_t pair_ok;
        rt_uint32_t rej_no_pps;
        rt_uint32_t rej_arrival;
        rt_uint32_t rej_interval;
        rt_uint32_t rej_slope;
        rt_uint32_t rej_residual;
        rt_uint32_t restart_cnt;
        rt_uint32_t erase_repair;
    } st;

    /* SWD 取证: 最近一次 restart 的冻结现场 (原因码 1=陈旧单调 2=UTC 跳变) */
    struct
    {
        rt_uint32_t reason;
        rt_uint32_t call_cnt;            /* pps_pair 总调用次数 */
        rt_uint64_t t_utc_pps;           /* 本次配对参数 */
        rt_uint64_t t_arrival;
        rt_uint64_t cap;                 /* 本次读到的 PPS 捕获 */
        rt_uint64_t prev_t_utc;          /* restart 判定时的 prev */
        rt_uint64_t prev_t_mcu;
    } dbg;
} g_map;

/* ------------------------- 64 位合成 (回绕竞态防护) ------------------------- */

/*
 * 64 位合成的公共判别: 给定 (hi1, cnt/ccr, hi2) 与 UIF 状态解析高位。
 * hi1/hi2 是跨夹计数读取的前后两次溢出累计 (竞态防护, 见下);
 * 一致时再查 UIF: 已置位且计数值已回绕, 说明溢出中断尚未运行, 手动补 1。
 */
static rt_uint32_t tb_resolve_hi(rt_uint32_t hi1, rt_uint32_t cnt,
                                 rt_uint32_t hi2)
{
    if (hi1 == hi2)
    {
        if ((TIM2->SR & TIM_SR_UIF) && (cnt < TB_HALF_PERIOD))
            return hi1 + 1u;             /* 回绕已发生, 溢出中断未及计数 */
        return hi1;
    }
    return (cnt < TB_HALF_PERIOD) ? hi2 : hi1;
}

/*
 * 当前 T_MCU: 溢出累计前后双读跨夹 CNT 读取 (README "CNT 回绕"竞态修正)。
 * 期间 TIM2 更新中断 (抢占优先级 1) 可能抢占本函数, 双读 + 半量程判别
 * 保证任一抢占位置下高位与 32bit 计数值正确配对; 高位 32bit 对齐读取。
 * 任意 ISR/线程上下文均安全。
 */
rt_uint64_t timebase_now_us(void)
{
    rt_uint32_t hi1 = s_ovf;
    rt_uint32_t cnt = (rt_uint32_t)TIM2->CNT;
    rt_uint32_t hi2 = s_ovf;

    return ((rt_uint64_t)tb_resolve_hi(hi1, cnt, hi2) << 32) | cnt;
}

/* ------------------------- 映射有效性 (两态模型) ------------------------- */

/* 就绪 = 滑窗满 且 距最近配对未超失锁保持阈值; 超时后映射作废直至滑窗重建 */
static rt_bool_t tb_map_ready(rt_uint64_t now)
{
    if (!g_map.map.valid || g_map.valid_cnt != TIMEBASE_PPS_WINDOW)
        return RT_FALSE;
    return (rt_bool_t)(now - g_map.last_pair_mcu <=
                       (rt_uint64_t)TIMEBASE_PPS_HOLD_S * 1000000ULL);
}

rt_bool_t timebase_map_valid(void)
{
    return tb_map_ready(timebase_now_us());
}

/* ------------------------- UTC <-> T_MCU 换算 ------------------------- */

rt_bool_t timebase_utc_to_mcu(rt_uint64_t t_utc_us, rt_uint64_t *t_mcu_us)
{
    struct clock_map m;
    rt_base_t level;
    rt_bool_t ok;

    if (t_mcu_us == RT_NULL)
        return RT_FALSE;

    level = rt_hw_interrupt_disable();
    m = g_map.map;
    ok = tb_map_ready(timebase_now_us());
    rt_hw_interrupt_enable(level);

    if (!ok || !m.valid)
    {
        *t_mcu_us = 0;                    /* 映射未就绪/作废: T_event 置 0 */
        return RT_FALSE;
    }

    /* T_MCU = base_m + (T_UTC - base_u) * scale; 待换算时标不早于基准点,
     * 属 <=1 个 PPS 周期的外推 (误差 ~ 斜率误差 x 1s = us 量级) */
    {
        double d = (double)t_utc_us - (double)m.t_utc_base;

        *t_mcu_us = m.t_mcu_base + (rt_uint64_t)(d * m.scale + 0.5);
    }
    return RT_TRUE;
}

rt_bool_t timebase_mcu_to_utc(rt_uint64_t t_mcu_us, rt_uint64_t *t_utc_us)
{
    struct clock_map m;
    rt_base_t level;
    rt_bool_t ok;

    if (t_utc_us == RT_NULL)
        return RT_FALSE;

    level = rt_hw_interrupt_disable();
    m = g_map.map;
    ok = tb_map_ready(timebase_now_us());
    rt_hw_interrupt_enable(level);

    if (!ok || !m.valid)
    {
        *t_utc_us = 0;
        return RT_FALSE;
    }

    {
        double d = (double)t_mcu_us - (double)m.t_mcu_base;

        *t_utc_us = m.t_utc_base + (rt_uint64_t)(d / m.scale + 0.5);
    }
    return RT_TRUE;
}

/* ------------------------- 滑窗拟合 ------------------------- */

/*
 * 最小二乘斜率 (x = T_UTC, y = T_MCU, 单位 us)。相对首元素中心化,
 * 避免 Unix 纪元 µs (1.7e15, 接近 double 53bit 整数精度) 直接参与运算。
 * 返回 0 表示无跨度 (n<2 或全部同点)。
 */
static double tb_fit_slope(const struct timebase_pair *p, int n)
{
    double x0, y0;
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    int i;

    if (n < 2)
        return 0.0;

    x0 = (double)p[0].t_utc_pps;
    y0 = (double)p[0].t_mcu_pps;

    for (i = 0; i < n; i++)
    {
        double dx = (double)p[i].t_utc_pps - x0;
        double dy = (double)p[i].t_mcu_pps - y0;

        sx += dx;
        sy += dy;
        sxx += dx * dx;
        sxy += dx * dy;
    }

    {
        double xb = sx / n;
        double yb = sy / n;
        double den = sxx - n * xb * xb;   /* = sum((dx - xbar)^2) */

        if (den < 1.0)
            return 0.0;                   /* UTC 无跨度 */
        return (sxy - n * xb * yb) / den;
    }
}

/* 用最新窗口刷新映射缓存 (基准点 = 最新配对 PPS2, 斜率 = 窗口最小二乘) */
static void tb_refresh_map(void)
{
    struct timebase_pair *latest;

    if (g_map.valid_cnt == 0u)
    {
        /* 空窗: 无基准点可取。valid_cnt 为 rt_uint8_t, 0-1 下溢成 255 会使
         * win[255] 远越界读 (与旧 win[3] 越界同族 UB, 须索引层面拦住)。 */
        g_map.map.valid = 0;
        g_map.map.scale = 1.0;
        return;
    }

    latest = &g_map.win[g_map.valid_cnt - 1];
    g_map.map.t_mcu_base = latest->t_mcu_pps;
    g_map.map.t_utc_base = latest->t_utc_pps;
    g_map.map.scale = (g_map.valid_cnt >= 2)
                      ? tb_fit_slope(g_map.win, g_map.valid_cnt) : 1.0;
    g_map.map.valid = (g_map.valid_cnt == TIMEBASE_PPS_WINDOW);
}

/* ------------------------- PPS 配对 (解析线程调用) ------------------------- */

rt_err_t timebase_pps_pair(rt_uint64_t t_utc_pps, rt_uint64_t t_arrival)
{
    rt_uint64_t cap;
    rt_uint32_t seq;
    rt_base_t level;

    /* 最近一次硬件捕获 (解析线程读, 捕获中断写, 关中断拷贝) */
    level = rt_hw_interrupt_disable();
    cap = s_pps.t_mcu;
    seq = s_pps.seq;
    rt_hw_interrupt_enable(level);

    g_map.dbg.call_cnt++;
    g_map.dbg.t_utc_pps = t_utc_pps;
    g_map.dbg.t_arrival = t_arrival;
    g_map.dbg.cap = cap;

    if (seq == 0u)
    {
        g_map.st.rej_no_pps++;            /* PPS 未接/无捕获 */
        return -RT_EEMPTY;
    }

    /* 门槛2: 语句到达距该 PPS 沿 < 0.9s (捕获晚于到达 -> 沿属下一秒, 违例) */
    if (t_arrival <= cap || t_arrival - cap > TB_GATE_ARRIVAL_US)
    {
        g_map.st.rej_arrival++;
        return -RT_ETIMEOUT;
    }

    if (g_map.valid_cnt > 0)
    {
        const struct timebase_pair *prev = &g_map.win[g_map.valid_cnt - 1];

        if (t_utc_pps <= prev->t_utc_pps)
        {
            /*
             * 滑窗条目陈旧检测: 正常滑窗内 prev 距当前 PPS 捕获 <=1s。
             * 解析链停摆 (打印反压/调度饿死等) 恢复后, prev 的 UTC 落后
             * 当前语句, 单调性检查在 restart 判定之前, 永远走不到重建
             * 路径 -> 配对楔死 (实测停摆 2min 后 rej_interval 每秒 +1,
             * T_event 恒 0, GNSS 观测断供)。陈旧 (>2s) 则重开滑窗重建。
             */
            if (cap > prev->t_mcu_pps &&
                cap - prev->t_mcu_pps > 2u * 1000000u)
            {
                g_map.dbg.reason = 1;
                g_map.dbg.prev_t_utc = prev->t_utc_pps;
                g_map.dbg.prev_t_mcu = prev->t_mcu_pps;
                g_map.valid_cnt = 0;
                g_map.st.restart_cnt++;
            }
            else
            {
                g_map.st.rej_interval++;  /* 时标非单调 (同秒重复/乱序) */
                return -RT_ERROR;
            }
        }
        else
        {
            /* 仅时标单调 (t_utc_pps > prev) 才做间隔一致性检查。陈旧重启
             * 分支已清空滑窗, prev 不再是有效参照: 落入此处会在
             * t_utc_pps - prev->t_utc_pps 上无符号下溢 (~2^64 > 失锁门限),
             * 把同一次事件重复计成 restart 并把 reason 覆盖为 2, 污染取证。 */
            rt_uint64_t du = t_utc_pps - prev->t_utc_pps;
            rt_uint64_t dm = cap - prev->t_mcu_pps;
            double tol = TB_GATE_INTVL_US * ((double)du / 1000000.0);

            if (tol < (double)TB_GATE_INTVL_US)
                tol = (double)TB_GATE_INTVL_US;

            if (du > TB_RESTART_UTC_GAP_US)
            {
                /* 失锁后再捕获: 旧窗口已失效, 重开滑窗从此对重建 */
                g_map.dbg.reason = 2;
                g_map.dbg.prev_t_utc = prev->t_utc_pps;
                g_map.dbg.prev_t_mcu = prev->t_mcu_pps;
                g_map.valid_cnt = 0;
                g_map.st.restart_cnt++;
            }
            else if (fabs((double)dm - (double)du) > tol)
            {
                g_map.st.rej_interval++;  /* 门槛3: 间隔与 UTC 间隔不一致 */
                return -RT_ERROR;
            }
        }
    }

    /* 候选集 = 滑窗 + 新配对 (最新), 拟合 -> 门槛4 -> 残差剔除 -> 提交 */
    {
        struct timebase_pair cand[TIMEBASE_PPS_WINDOW + 1];
        int n = g_map.valid_cnt + 1;
        int kept[TIMEBASE_PPS_WINDOW + 1];
        int n_kept = 0, i, cand_kept = 0;

        memcpy(cand, g_map.win, sizeof(struct timebase_pair) * g_map.valid_cnt);
        cand[n - 1].t_mcu_pps = cap;
        cand[n - 1].t_utc_pps = t_utc_pps;

        if (n >= 2)
        {
            double scale = tb_fit_slope(cand, n);

            if (fabs(scale - 1.0) > TB_GATE_SLOPE_PPM)
            {
                g_map.st.rej_slope++;     /* 门槛4: 拦截残余错配 */
                return -RT_ERROR;
            }

            /* 残差剔除: 以最新配对为基准点, >10us 的配对剔出窗口 */
            {
                double base_m = (double)cand[n - 1].t_mcu_pps;
                double base_u = (double)cand[n - 1].t_utc_pps;

                for (i = 0; i < n; i++)
                {
                    double r = fabs((double)cand[i].t_mcu_pps -
                                    (base_m + ((double)cand[i].t_utc_pps - base_u) * scale));

                    if (r <= TB_FIT_RESID_US)
                    {
                        kept[n_kept++] = i;
                        if (i == n - 1)
                            cand_kept = 1;
                    }
                }
            }

            if (n_kept < n)
                g_map.st.rej_residual += (rt_uint32_t)(n - n_kept);

            if (n_kept < 2)
            {
                /* 剩余不足 2 对: 映射回退等待新配对 (pps_valid_cnt 递减) */
                for (i = 0; i < n_kept; i++)
                    g_map.win[i] = cand[kept[i]];
                g_map.valid_cnt = (rt_uint8_t)n_kept;
                g_map.map.valid = 0;
                tb_refresh_map();          /* 基准/斜率刷新, valid 已置 0 */
                if (!cand_kept)
                    return -RT_ERROR;     /* 候选自身为野值: 本对违例 */
                g_map.st.pair_ok++;
                g_map.last_pair_mcu = cap;
                return RT_EOK;
            }

            if (!cand_kept)
            {
                /* 候选是野值: 丢弃本对, 窗口保留幸存配对 */
                for (i = 0; i < n_kept; i++)
                    g_map.win[i] = cand[kept[i]];
                g_map.valid_cnt = (rt_uint8_t)n_kept;
                tb_refresh_map();
                return -RT_ERROR;
            }

            /* 幸存配对 (含候选) 提交: 先在 kept 下标层面裁剪到窗口大小再写
             * win, 全程不产生 win[WINDOW] 越界写。
             * (原实现先提交再 memmove 裁剪, n_kept==WINDOW+1 时循环体写
             *  win[3] 越界属 UB —— GCC 据此删除了裁剪分支, 运行时
             *  valid_cnt=4 且踩 valid_cnt/map 字段: 滑窗每 4s 被伪 UTC
             *  跳变 (>10s) 推倒重建, 映射仅 ~1s/4s 有效, 样本大面积
             *  ts_zero/stale, 引擎 GNSS 观测断供) */
            {
                int first = n_kept > TIMEBASE_PPS_WINDOW
                                ? n_kept - TIMEBASE_PPS_WINDOW : 0;

                for (i = 0; i < n_kept - first; i++)
                    g_map.win[i] = cand[kept[first + i]];
                n_kept -= first;
            }
            g_map.valid_cnt = (rt_uint8_t)n_kept;
        }
        else
        {
            /* 窗口空 (首配对 / 再捕获重开): 单对入窗, 斜率无信息置 1 */
            g_map.win[0] = cand[0];
            g_map.valid_cnt = 1;
        }
    }

    g_map.st.pair_ok++;
    g_map.last_pair_mcu = cap;
    tb_refresh_map();                     /* 满窗 (3 对) 时映射就绪 */
    return RT_EOK;
}

/* ------------------------- 擦写窗口回绕核对 ------------------------- */
/* 现状核注 (2026-10-03): 标定参数迁移 W25Q64 后擦写不再关中断,
 * 本组接口当前无调用方, 保留备用 (见 timebase.h 头注)。 */

rt_uint64_t timebase_irq_window_begin(void)
{
    return timebase_now_us();
}

void timebase_irq_window_end(rt_uint64_t snap_us)
{
    rt_base_t level = rt_hw_interrupt_disable();

    RT_UNUSED(snap_us);

    /* 全局关中断窗口跨回绕点: UIF 置位但溢出中断无法运行 —— 补计并清标志
     * (防恢复后挂起的中断再计一次); 擦写窗口远短于 71.6min, 至多一次回绕。
     * 若恢复后中断已先行运行 (UIF 已清、高位已计), 此处检查自然通过。 */
    if (TIM2->SR & TIM_SR_UIF)
    {
        s_ovf++;
        g_map.st.erase_repair++;
        TIM2->SR = ~TIM_SR_UIF;           /* rc_w0: 仅清 UIF, 其余位写 1 不变 */
    }

    rt_hw_interrupt_enable(level);
}

/* ------------------------- TIM2 中断 ------------------------- */

/* 溢出累计: 32bit @1MHz 约 71.6min 一次, HAL 已在回调前清 UIF */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
        s_ovf++;
}

/* PPS 捕获: 脉冲沿时刻已由硬件锁存进 CCR1, 此处只合成 64 位 T_MCU,PPS */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2)
        return;

    {
        rt_uint32_t hi1 = s_ovf;
        rt_uint32_t ccr = (rt_uint32_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        rt_uint32_t hi2 = s_ovf;

        /* CCR1 在沿时刻硬件锁存 (早于本中断执行), 更新与捕获同属 TIM2
         * 中断无嵌套, 双读为冗余防护; UIF 修正覆盖"沿后回绕、更新分支
         * 尚未执行"的窗口 */
        s_pps.t_mcu = ((rt_uint64_t)tb_resolve_hi(hi1, ccr, hi2) << 32) | ccr;
        s_pps.seq++;
    }
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

/* ------------------------- 初始化 ------------------------- */

/*
 * TIM2: 1MHz 自由计数 + PA0 (TIM2_CH1) 输入捕获。
 * GPIO/NVIC/时钟由 CubeMX 生成的 HAL_TIM_Base_MspInit (stm32h7xx_hal_msp.c)
 * 配置 (PA0 AF1, TIM2_IRQn 抢占优先级 1 与 board.c 表一致);
 * 本模块 INIT_BOARD 级启动, 早于所有传感器驱动 (INIT_DEVICE) 打戳。
 */
static int timebase_init(void)
{
    TIM_IC_InitTypeDef ic = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = TB_TIM_PSC;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = TB_TIM_PERIOD;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.RepetitionCounter = 0;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
        return -RT_ERROR;

    /* PPS 上升沿直接捕获: 沿时刻硬件锁存 CCR1, 与中断响应延迟解耦 */
    ic.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter    = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim2, &ic, TIM_CHANNEL_1) != HAL_OK)
        return -RT_ERROR;

    __HAL_TIM_URS_ENABLE(&htim2);         /* 仅计数器溢出产生更新中断 */
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE | TIM_FLAG_CC1 | TIM_FLAG_CC1OF);

    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)      /* 自由计数 + 溢出中断 */
        return -RT_ERROR;
    if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)   /* PPS 捕获 */
        return -RT_ERROR;

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);            /* 与 board.c NVIC 表一致 */

    rt_kprintf("[timebase] TIM2 @1MHz free-run + PPS capture (PA0) ready\n");
    return RT_EOK;
}
INIT_BOARD_EXPORT(timebase_init);

/* ------------------------- 状态 ------------------------- */

void timebase_get_status(struct timebase_status *st)
{
    rt_base_t level;

    if (st == RT_NULL)
        return;

    memset(st, 0, sizeof(*st));

    st->now_us  = timebase_now_us();
    st->tim_cnt = (rt_uint32_t)TIM2->CNT;

    level = rt_hw_interrupt_disable();
    st->ovf_count    = s_ovf;
    st->erase_repair = g_map.st.erase_repair;
    if (s_pps.seq != 0u)
    {
        st->pps_caps = s_pps.seq;
        st->pps_last_age_us = (rt_uint32_t)(st->now_us - s_pps.t_mcu);
    }
    rt_hw_interrupt_enable(level);

    level = rt_hw_interrupt_disable();
    st->pair_ok      = g_map.st.pair_ok;
    st->rej_no_pps   = g_map.st.rej_no_pps;
    st->rej_arrival  = g_map.st.rej_arrival;
    st->rej_interval = g_map.st.rej_interval;
    st->rej_slope    = g_map.st.rej_slope;
    st->rej_residual = g_map.st.rej_residual;
    st->restart_cnt  = g_map.st.restart_cnt;
    st->pps_valid_cnt = g_map.valid_cnt;
    memcpy(st->win, g_map.win, sizeof(st->win));
    st->map = g_map.map;
    rt_hw_interrupt_enable(level);

    st->map_valid = (rt_uint8_t)tb_map_ready(st->now_us);
    if (g_map.valid_cnt > 0)
        st->map_age_ms = (rt_uint32_t)((st->now_us - g_map.last_pair_mcu) / 1000u);
}

/* ------------------------- FinSH 命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void timebase(void)
{
    struct timebase_status st;

    timebase_get_status(&st);

    LOG_I("=== timebase (TIM2 1MHz + PPS) ===");
    LOG_I("t_mcu   : %u.%06u s (CNT=0x%08X ovf=%u)",
          (rt_uint32_t)(st.now_us / 1000000u), (rt_uint32_t)(st.now_us % 1000000u),
          st.tim_cnt, st.ovf_count);
    LOG_I("pps     : caps=%u last_age=%u.%03u ms",
          st.pps_caps, st.pps_last_age_us / 1000u, st.pps_last_age_us % 1000u);
    LOG_I("pair    : ok=%u rej[pps=%u arriv=%u intvl=%u slope=%u resid=%u] "
          "restart=%u",
          st.pair_ok, st.rej_no_pps, st.rej_arrival, st.rej_interval,
          st.rej_slope, st.rej_residual, st.restart_cnt);
    LOG_I("window  : %u/%u pairs", st.pps_valid_cnt, TIMEBASE_PPS_WINDOW);
    for (int i = 0; i < st.pps_valid_cnt; i++)
        LOG_I("  [%d] utc=%u s  mcu=%u.%06u s", i,
              (rt_uint32_t)(st.win[i].t_utc_pps / 1000000u),
              (rt_uint32_t)(st.win[i].t_mcu_pps / 1000000u),
              (rt_uint32_t)(st.win[i].t_mcu_pps % 1000000u));
    LOG_I("map     : %s, scale=%+.1f ppm, age=%u ms (hold %ds)",
          st.map_valid ? "VALID" : (st.pps_valid_cnt ? "BUILDING" : "NONE"),
          (st.map.scale - 1.0) * 1e6, st.map_age_ms, TIMEBASE_PPS_HOLD_S);
    LOG_I("erase   : wrap_repair=%u", st.erase_repair);
    LOG_I("hint    : 映射未就绪时 gnss 样本 T_event=0 (gins 跳过); "
          "PPS 断接看 pps last_age");
}
MSH_CMD_EXPORT(timebase, time sync framework status);
#endif /* RT_USING_FINSH && FINSH_USING_MSH */
