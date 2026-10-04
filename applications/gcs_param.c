/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * QGC 参数协议装配 (MAVLink v2, 阶段 2, FMT_README §13.4-13.5)
 *
 * ---------------------------------------------------------------------------
 * 定位与数据流
 * ---------------------------------------------------------------------------
 * 纯装配层: 把 W25Q64 参数域 (middleware/Sensor_Preprocessing/param_calib: sys/nav/calib) 映射
 * 成一张静态 MAVLink 参数表, QGC 参数页按标准 PARAM 协议读写:
 *
 *   QGC PARAM_REQUEST_LIST/READ ──> handler (mavgcs 线程上下文)
 *                                     └─ 查表 -> PARAM_VALUE 逐帧回发
 *   QGC PARAM_SET ──> handler: 校验 (类型/值域) -> param_nav_set() 或
 *                      calib_store_ram() 改镜像 -> 回发 PARAM_VALUE
 *                     └─ 3s 防抖 (gcs_param_poll 驱动) -> W25Q64 落盘
 *
 * 与 FMT (ref/FMT-Firmware) 的有意差异 (§13.4 "与 FMT 的有意差异"):
 *   不做 ~150 个 PX4 虚拟参数表 (机架识别不可用不影响地图/曲线), 参数
 *   全部为本工程真实参数; 不做带宽自适应节流 (条目少, @115200 全表
 *   回放 ~1.5KB ≈ 0.13s, 串口 poll 发送天然按线速节流)。
 *
 * ---------------------------------------------------------------------------
 * 参数表 (44 项 = SYS 2 只读 + NAV 5 可写/12 只读 + CALIB 9 可写/16 只读)
 * ---------------------------------------------------------------------------
 * - NAV_NOGNSS_LAT7/LON7: 1e-7 deg 整型口径 (同 GLOBAL_POSITION_INT),
 *   避免 float 参数丢失经纬度精度 (float32 在 ±120° 处量化 ~1m);
 * - NAV 轴映射只读展示: 三分量需一次一致地改, 逐参 SET 会出现非法中间
 *   状态 (重复轴), 修改走 FinSH `nav set iaxis|maxis` (整体校验);
 * - CALIB 只读项为椭球拟合产物与质量指标 (半径/残差/样本数/软磁矩阵),
 *   重新标定由 `magcal`/`barocal` 流程产出, 不从 QGC 手改;
 * - SYS 域只读遥测 (上电计数/固件标识)。
 *
 * 类型语义: 遵循 mavlink_param_union_t —— PARAM_VALUE/PARAM_SET 的
 * float 载荷按条目类型重解释 (INT32/UINT32 整型直通, REAL32 浮点)。
 * PARAM_SET 值为 NaN 按 PX4 惯例 = 请求重发现值。
 *
 * 落盘策略: PARAM_SET 只改 RAM 镜像并标记, 最后一次修改 3s 后
 * gcs_param_poll() 统一保存 (nav 域 payload_sane 兜底, calib 域直接
 * 追加记录) —— QGC 批量修改合并为一次 Flash 写, 掉电窗口 3s。
 *
 * 线程模型: 全部 handler 与 poll 均在 mavgcs 线程上下文执行, 无锁;
 * 参数镜像与 FinSH/nav 命令的并发写与 param_nav 既有语义一致
 * (读侧任意线程, 写侧操作者串行)。
 */

#include <rtthread.h>
#include <string.h>
#include <math.h>

#include "mavlink_link.h"
#include "gcs_param.h"
#include "param_calib.h"            /* calib_store_ram(): calib 域镜像 */
#include "param_nav.h"              /* param_nav()/param_nav_set(): nav 域 */
#include "param_sys.h"              /* param_sys_info(): sys 域只读遥测 */

/* ulog 日志: LOG_E/LOG_W/LOG_I/LOG_D, 行尾自动补 \r\n */
#define LOG_TAG "gcs"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define GCS_PARAM_SAVE_DELAY_MS 3000    /* 最后一次 PARAM_SET 后落盘延时 */

/* ---------------------------- 参数表 ----------------------------
 * 条目: 名称 (≤15 字符, 第 16 字节留 '\0') + MAVLink 类型 + 读写属性
 * + get/set 回调 (set 为 NULL 即只读) + 数组下标。get/set 统一经
 * double 传值 (int32/uint32 在 double 53 位尾数内无损)。 */

typedef struct
{
    const char *name;
    uint8_t     type;               /* MAV_PARAM_TYPE_* */
    rt_bool_t   ro;
    double    (*get)(rt_uint8_t idx);
    rt_err_t  (*set)(rt_uint8_t idx, double v);
    rt_uint8_t  idx;
} param_ent_t;

static rt_bool_t s_calib_dirty;     /* calib 域镜像已改未落盘 */

/* nav 域落盘由 param_nav 自身 dirty 标志驱动; calib 域无脏标志, 此处记 */

/* ---- SYS 域 (只读遥测) ---- */

static double get_sys_bootcnt(rt_uint8_t idx)
{
    RT_UNUSED(idx);
    return (double)param_sys_info()->boot_count;
}

static double get_sys_fwid(rt_uint8_t idx)
{
    RT_UNUSED(idx);
    return (double)param_sys_info()->fw_id;
}

/* ---- NAV 域 (param_nav 镜像) ---- */

static double get_nav_decl(rt_uint8_t idx)
{
    RT_UNUSED(idx);
    return param_nav()->mag_decl_deg;
}

static rt_err_t set_nav_decl(rt_uint8_t idx, double v)
{
    RT_UNUSED(idx);
    return param_nav_set(NAVP_DECL, v);
}

/* 经纬度 1e-7 deg 整型口径: 读侧就近取整 (float 参数会丢精度, 见头注) */
static double get_nav_lla7(rt_uint8_t idx)
{
    double v = (idx == 0) ? param_nav()->nognss_lat : param_nav()->nognss_lon;

    return (double)(rt_int32_t)(v >= 0 ? v * 1.0e7 + 0.5 : v * 1.0e7 - 0.5);
}

static rt_err_t set_nav_lat7(rt_uint8_t idx, double v)
{
    RT_UNUSED(idx);
    return param_nav_set(NAVP_NOGNSS_LAT, v / 1.0e7);
}

static rt_err_t set_nav_lon7(rt_uint8_t idx, double v)
{
    RT_UNUSED(idx);
    return param_nav_set(NAVP_NOGNSS_LON, v / 1.0e7);
}

static double get_nav_alt(rt_uint8_t idx)
{
    RT_UNUSED(idx);
    return param_nav()->nognss_alt;
}

static rt_err_t set_nav_alt(rt_uint8_t idx, double v)
{
    RT_UNUSED(idx);
    return param_nav_set(NAVP_NOGNSS_ALT, v);
}

static double get_nav_magen(rt_uint8_t idx)
{
    RT_UNUSED(idx);
    return (double)param_nav()->mag_enable;
}

static rt_err_t set_nav_magen(rt_uint8_t idx, double v)
{
    RT_UNUSED(idx);
    return param_nav_set(NAVP_MAG_ENABLE, v);
}

/* NAV 轴映射 (只读展示): 修改走 `nav set iaxis|maxis` (三分量整体校验) */
static double get_nav_isrc(rt_uint8_t i)
{
    return (double)param_nav()->imu_axis_src[i];
}

static double get_nav_isign(rt_uint8_t i)
{
    return (double)param_nav()->imu_axis_sign[i];
}

static double get_nav_msrc(rt_uint8_t i)
{
    return (double)param_nav()->mag_axis_src[i];
}

static double get_nav_msign(rt_uint8_t i)
{
    return (double)param_nav()->mag_axis_sign[i];
}

/* ---- CALIB 域 (calib_store RAM 镜像) ---- */

/* 值域护栏: NaN 比较恒假即拒绝, inf/巨值按界拒 (专家旋钮, 防手误量级) */
static rt_bool_t val_sane(double v, double lim)
{
    return (rt_bool_t)(v > -lim && v < lim);
}

static double get_cal_magen(rt_uint8_t idx)
{
    RT_UNUSED(idx);
    return calib_store_ram()->mag_valid ? 1.0 : 0.0;
}

static rt_err_t set_cal_magen(rt_uint8_t idx, double v)
{
    RT_UNUSED(idx);
    if (v != 0.0 && v != 1.0)
        return -RT_EINVAL;
    calib_store_ram()->mag_valid = (v != 0.0);
    s_calib_dirty = RT_TRUE;
    return RT_EOK;
}

static double get_cal_magb(rt_uint8_t i)
{
    return calib_store_ram()->mag_bias_ut[i];
}

static rt_err_t set_cal_magb(rt_uint8_t i, double v)
{
    if (!val_sane(v, 1.0e4))            /* µT 量级, 台架硬磁 <1e2 */
        return -RT_EINVAL;
    calib_store_ram()->mag_bias_ut[i] = (float)v;
    s_calib_dirty = RT_TRUE;
    return RT_EOK;
}

static double get_cal_barooff(rt_uint8_t idx)
{
    RT_UNUSED(idx);
    return calib_store_ram()->baro_offset_pa;
}

static rt_err_t set_cal_barooff(rt_uint8_t idx, double v)
{
    RT_UNUSED(idx);
    if (!val_sane(v, 1.0e4))            /* Pa, 标定偏移 <1e2 */
        return -RT_EINVAL;
    calib_store_ram()->baro_offset_pa = (float)v;
    s_calib_dirty = RT_TRUE;
    return RT_EOK;
}

static double get_cal_acen(rt_uint8_t idx)
{
    RT_UNUSED(idx);
    return calib_store_ram()->acc_valid ? 1.0 : 0.0;
}

static rt_err_t set_cal_acen(rt_uint8_t idx, double v)
{
    RT_UNUSED(idx);
    if (v != 0.0 && v != 1.0)
        return -RT_EINVAL;
    calib_store_ram()->acc_valid = (v != 0.0);
    s_calib_dirty = RT_TRUE;
    return RT_EOK;
}

static double get_cal_accb(rt_uint8_t i)
{
    return calib_store_ram()->acc_bias_mgal[i];
}

static rt_err_t set_cal_accb(rt_uint8_t i, double v)
{
    if (!val_sane(v, 1.0e6))            /* mGal, EKF 估值 <1e4 */
        return -RT_EINVAL;
    calib_store_ram()->acc_bias_mgal[i] = (float)v;
    s_calib_dirty = RT_TRUE;
    return RT_EOK;
}

/* CALIB 只读: 拟合产物与质量指标 (重标定走 `magcal`/`barocal` 流程) */
static double get_cal_magrad(rt_uint8_t idx)
{
    RT_UNUSED(idx);
    return calib_store_ram()->mag_radius_ut;
}

static double get_cal_magresid(rt_uint8_t idx)
{
    RT_UNUSED(idx);
    return calib_store_ram()->mag_resid;
}

static double get_cal_magratio(rt_uint8_t idx)
{
    RT_UNUSED(idx);
    return calib_store_ram()->mag_maxratio;
}

static double get_cal_magsamp(rt_uint8_t idx)
{
    RT_UNUSED(idx);
    return (double)calib_store_ram()->mag_samples;
}

static double get_cal_si(rt_uint8_t k)      /* k = 行*3+列 */
{
    return calib_store_ram()->mag_softiron[k / 3][k % 3];
}

static double get_cal_barot(rt_uint8_t idx)
{
    RT_UNUSED(idx);
    return calib_store_ram()->baro_cal_temp;
}

static double get_cal_baromean(rt_uint8_t idx)
{
    RT_UNUSED(idx);
    return calib_store_ram()->baro_mean_pa;
}

static double get_cal_acct(rt_uint8_t idx)
{
    RT_UNUSED(idx);
    return calib_store_ram()->acc_cal_temp;
}

/* ---- 参数表 (顺序即 param_index; QGC 参数页按此展示) ---- */

static const param_ent_t s_params[] =
{
    /* SYS 域 (只读遥测) */
    {"SYS_BOOTCNT", MAV_PARAM_TYPE_UINT32, RT_TRUE, get_sys_bootcnt, RT_NULL, 0},
    {"SYS_FWID",    MAV_PARAM_TYPE_UINT32, RT_TRUE, get_sys_fwid,    RT_NULL, 0},

    /* NAV 域可写 */
    {"NAV_DECL",        MAV_PARAM_TYPE_REAL32, RT_FALSE, get_nav_decl,  set_nav_decl,  0},
    {"NAV_NOGNSS_LAT7", MAV_PARAM_TYPE_INT32,  RT_FALSE, get_nav_lla7,  set_nav_lat7,  0},
    {"NAV_NOGNSS_LON7", MAV_PARAM_TYPE_INT32,  RT_FALSE, get_nav_lla7,  set_nav_lon7,  1},
    {"NAV_NOGNSS_ALT",  MAV_PARAM_TYPE_REAL32, RT_FALSE, get_nav_alt,   set_nav_alt,   0},
    {"NAV_MAG_ENABLE",  MAV_PARAM_TYPE_INT32,  RT_FALSE, get_nav_magen, set_nav_magen, 0},

    /* NAV 域轴映射 (只读展示, 修改走 `nav set iaxis|maxis`) */
    {"NAV_IMU_SRC0", MAV_PARAM_TYPE_INT32,  RT_TRUE, get_nav_isrc,  RT_NULL, 0},
    {"NAV_IMU_SRC1", MAV_PARAM_TYPE_INT32,  RT_TRUE, get_nav_isrc,  RT_NULL, 1},
    {"NAV_IMU_SRC2", MAV_PARAM_TYPE_INT32,  RT_TRUE, get_nav_isrc,  RT_NULL, 2},
    {"NAV_IMU_SIGN0", MAV_PARAM_TYPE_REAL32, RT_TRUE, get_nav_isign, RT_NULL, 0},
    {"NAV_IMU_SIGN1", MAV_PARAM_TYPE_REAL32, RT_TRUE, get_nav_isign, RT_NULL, 1},
    {"NAV_IMU_SIGN2", MAV_PARAM_TYPE_REAL32, RT_TRUE, get_nav_isign, RT_NULL, 2},
    {"NAV_MAG_SRC0", MAV_PARAM_TYPE_INT32,  RT_TRUE, get_nav_msrc,  RT_NULL, 0},
    {"NAV_MAG_SRC1", MAV_PARAM_TYPE_INT32,  RT_TRUE, get_nav_msrc,  RT_NULL, 1},
    {"NAV_MAG_SRC2", MAV_PARAM_TYPE_INT32,  RT_TRUE, get_nav_msrc,  RT_NULL, 2},
    {"NAV_MAG_SIGN0", MAV_PARAM_TYPE_REAL32, RT_TRUE, get_nav_msign, RT_NULL, 0},
    {"NAV_MAG_SIGN1", MAV_PARAM_TYPE_REAL32, RT_TRUE, get_nav_msign, RT_NULL, 1},
    {"NAV_MAG_SIGN2", MAV_PARAM_TYPE_REAL32, RT_TRUE, get_nav_msign, RT_NULL, 2},

    /* CALIB 域可写 */
    {"CAL_MAG_ENABLE",  MAV_PARAM_TYPE_INT32,  RT_FALSE, get_cal_magen,    set_cal_magen,    0},
    {"CAL_MAG_BIAS0",   MAV_PARAM_TYPE_REAL32, RT_FALSE, get_cal_magb,     set_cal_magb,     0},
    {"CAL_MAG_BIAS1",   MAV_PARAM_TYPE_REAL32, RT_FALSE, get_cal_magb,     set_cal_magb,     1},
    {"CAL_MAG_BIAS2",   MAV_PARAM_TYPE_REAL32, RT_FALSE, get_cal_magb,     set_cal_magb,     2},
    {"CAL_BARO_OFFSET", MAV_PARAM_TYPE_REAL32, RT_FALSE, get_cal_barooff,  set_cal_barooff,  0},
    {"CAL_ACC_ENABLE",  MAV_PARAM_TYPE_INT32,  RT_FALSE, get_cal_acen,     set_cal_acen,     0},
    {"CAL_ACC_BIAS0",   MAV_PARAM_TYPE_REAL32, RT_FALSE, get_cal_accb,     set_cal_accb,     0},
    {"CAL_ACC_BIAS1",   MAV_PARAM_TYPE_REAL32, RT_FALSE, get_cal_accb,     set_cal_accb,     1},
    {"CAL_ACC_BIAS2",   MAV_PARAM_TYPE_REAL32, RT_FALSE, get_cal_accb,     set_cal_accb,     2},

    /* CALIB 域只读 (椭球拟合产物与质量指标) */
    {"CAL_MAG_RADIUS", MAV_PARAM_TYPE_REAL32, RT_TRUE, get_cal_magrad,   RT_NULL, 0},
    {"CAL_MAG_RESID",  MAV_PARAM_TYPE_REAL32, RT_TRUE, get_cal_magresid, RT_NULL, 0},
    {"CAL_MAG_RATIO",  MAV_PARAM_TYPE_REAL32, RT_TRUE, get_cal_magratio, RT_NULL, 0},
    {"CAL_MAG_SAMP",   MAV_PARAM_TYPE_UINT32, RT_TRUE, get_cal_magsamp,  RT_NULL, 0},
    {"CAL_MAG_SI00",   MAV_PARAM_TYPE_REAL32, RT_TRUE, get_cal_si,       RT_NULL, 0},
    {"CAL_MAG_SI01",   MAV_PARAM_TYPE_REAL32, RT_TRUE, get_cal_si,       RT_NULL, 1},
    {"CAL_MAG_SI02",   MAV_PARAM_TYPE_REAL32, RT_TRUE, get_cal_si,       RT_NULL, 2},
    {"CAL_MAG_SI10",   MAV_PARAM_TYPE_REAL32, RT_TRUE, get_cal_si,       RT_NULL, 3},
    {"CAL_MAG_SI11",   MAV_PARAM_TYPE_REAL32, RT_TRUE, get_cal_si,       RT_NULL, 4},
    {"CAL_MAG_SI12",   MAV_PARAM_TYPE_REAL32, RT_TRUE, get_cal_si,       RT_NULL, 5},
    {"CAL_MAG_SI20",   MAV_PARAM_TYPE_REAL32, RT_TRUE, get_cal_si,       RT_NULL, 6},
    {"CAL_MAG_SI21",   MAV_PARAM_TYPE_REAL32, RT_TRUE, get_cal_si,       RT_NULL, 7},
    {"CAL_MAG_SI22",   MAV_PARAM_TYPE_REAL32, RT_TRUE, get_cal_si,       RT_NULL, 8},
    {"CAL_BARO_TEMP",  MAV_PARAM_TYPE_REAL32, RT_TRUE, get_cal_barot,    RT_NULL, 0},
    {"CAL_BARO_MEAN",  MAV_PARAM_TYPE_REAL32, RT_TRUE, get_cal_baromean, RT_NULL, 0},
    {"CAL_ACC_TEMP",   MAV_PARAM_TYPE_REAL32, RT_TRUE, get_cal_acct,     RT_NULL, 0},
};

#define GCS_PARAM_COUNT     (sizeof(s_params) / sizeof(s_params[0]))
#define GCS_PARAM_NAME_MAX  15u     /* param_id[16] 须容纳 '\0' */

/* ---------------------------- 收发 ---------------------------- */

static rt_bool_t target_ok(uint8_t sys, uint8_t comp)
{
    return (rt_bool_t)((sys == 0 || sys == mavlink_link_sysid()) &&
                       (comp == 0 || comp == mavlink_link_compid()));
}

/* 查表: param_id[16] 可能不满长无 '\0', 拷到 17 字节再比较 */
static int find_param(const char id16[16])
{
    char name[GCS_PARAM_NAME_MAX + 2];
    rt_uint32_t n = 0;

    while (n < 16u && id16[n] != '\0')
    {
        name[n] = id16[n];
        n++;
        if (n > GCS_PARAM_NAME_MAX)
            return -1;                  /* 超长名不可能命中本表 */
    }
    name[n] = '\0';

    for (rt_uint32_t i = 0; i < GCS_PARAM_COUNT; i++)
    {
        if (!rt_strcmp(name, s_params[i].name))
            return (int)i;
    }
    return -1;
}

/* double -> union (按条目类型重解释, 返回 float 载荷) */
static float union_put(const param_ent_t *e, double v,
                       mavlink_param_union_t *u)
{
    rt_memset(u, 0, sizeof(*u));

    switch (e->type)
    {
    case MAV_PARAM_TYPE_INT32:
        u->param_int32 = (int32_t)v;
        break;
    case MAV_PARAM_TYPE_UINT32:
        u->param_uint32 = (rt_uint32_t)v;
        break;
    default:                            /* REAL32 */
        u->param_float = (float)v;
        break;
    }
    return u->param_float;
}

/* union (按条目类型重解释) -> double; 类型不符返回 NaN 由调用方拒绝 */
static double union_get(const param_ent_t *e, float raw, uint8_t type)
{
    mavlink_param_union_t u;

    if (type != e->type)
        return (double)NAN;

    rt_memset(&u, 0, sizeof(u));
    u.param_float = raw;

    switch (e->type)
    {
    case MAV_PARAM_TYPE_INT32:
        return (double)u.param_int32;
    case MAV_PARAM_TYPE_UINT32:
        return (double)u.param_uint32;
    default:
        return (double)u.param_float;
    }
}

static void send_param_value(rt_uint16_t idx)
{
    const param_ent_t *e = &s_params[idx];
    mavlink_message_t msg;
    mavlink_param_union_t u;
    char id[16];
    rt_uint32_t n;

    for (n = 0; n < 16u; n++)
        id[n] = '\0';
    for (n = 0; e->name[n] != '\0' && n < 16u; n++)
        id[n] = e->name[n];

    mavlink_msg_param_value_pack(mavlink_link_sysid(), mavlink_link_compid(),
                                 &msg, id, union_put(e, e->get(e->idx), &u),
                                 e->type, (uint16_t)GCS_PARAM_COUNT, idx);
    mavlink_link_send_msg(&msg);
}

/* STATUSTEXT 事件 (落盘结果; 与 mavgcs.c 同款组包) */
static void send_statustext(uint8_t severity, const char *text)
{
    mavlink_message_t msg;

    mavlink_msg_statustext_pack(mavlink_link_sysid(), mavlink_link_compid(),
                                &msg, severity, text, 0, 0);
    mavlink_link_send_msg(&msg);
}

/* ---------------------------- 落盘 (防抖) ---------------------------- */

static rt_tick_t s_save_at;             /* 到点时刻 (0 = 无待存) */

static void save_schedule(void)
{
    s_save_at = rt_tick_get() +
                rt_tick_from_millisecond(GCS_PARAM_SAVE_DELAY_MS);
}

static void save_flush(void)
{
    rt_err_t err_nav = RT_EOK, err_cal = RT_EOK;

    if (param_nav_dirty())
    {
        err_nav = param_nav_save();
        if (err_nav != RT_EOK)
            LOG_W("nav save failed: %d", (int)err_nav);
    }
    if (s_calib_dirty)
    {
        err_cal = calib_store_save();
        if (err_cal == RT_EOK)
            s_calib_dirty = RT_FALSE;
        else
            LOG_W("calib save failed: %d (keep dirty, next PARAM_SET retries)",
                  (int)err_cal);
    }

    if (err_nav == RT_EOK && err_cal == RT_EOK)
    {
        LOG_I("param save OK (W25Q64)");
        send_statustext(MAV_SEVERITY_INFO, "params saved");
    }
    else
    {
        send_statustext(MAV_SEVERITY_ERROR, "param save failed (flash)");
    }
}

void gcs_param_poll(void)
{
    if (s_save_at == 0)
        return;
    if ((rt_int32_t)(rt_tick_get() - s_save_at) < 0)
        return;                         /* 防抖窗口未到 (回绕安全) */

    s_save_at = 0;
    save_flush();
}

/* ---------------------------- 上行 handler ----------------------------
 * 全部在 mavgcs 线程上下文 (feed 调用方) 执行, 无锁。 */

static void gcs_param_on_list(const mavlink_message_t *msg)
{
    mavlink_param_request_list_t rq;

    mavlink_msg_param_request_list_decode(msg, &rq);
    if (!target_ok(rq.target_system, rq.target_component))
        return;

    LOG_I("param list request -> %u entries", (unsigned)GCS_PARAM_COUNT);
    for (rt_uint16_t i = 0; i < GCS_PARAM_COUNT; i++)
        send_param_value(i);
}

static void gcs_param_on_read(const mavlink_message_t *msg)
{
    mavlink_param_request_read_t rq;
    int idx;

    mavlink_msg_param_request_read_decode(msg, &rq);
    if (!target_ok(rq.target_system, rq.target_component))
        return;

    if (rq.param_index >= 0 &&
        (rt_uint16_t)rq.param_index < GCS_PARAM_COUNT)
    {
        send_param_value((rt_uint16_t)rq.param_index);
        return;
    }

    idx = find_param(rq.param_id);
    if (idx >= 0)
        send_param_value((rt_uint16_t)idx);
    else
        LOG_W("param read: unknown id");
}

static void gcs_param_on_set(const mavlink_message_t *msg)
{
    mavlink_param_set_t ps;
    char name[GCS_PARAM_NAME_MAX + 2];
    const param_ent_t *e;
    double v;
    int idx;

    mavlink_msg_param_set_decode(msg, &ps);
    if (!target_ok(ps.target_system, ps.target_component))
        return;

    idx = find_param(ps.param_id);
    if (idx < 0)
    {
        memcpy(name, ps.param_id, 16);
        name[16] = '\0';
        LOG_W("param set: unknown \"%s\"", name);
        return;                         /* 未知参数不应答 (QGC 标记失败) */
    }

    e = &s_params[idx];

    if (isnan(ps.param_value))
    {
        send_param_value((rt_uint16_t)idx);   /* PX4 惯例: NaN = 重发现值 */
        return;
    }

    v = union_get(e, ps.param_value, ps.param_type);
    if (isnan(v))
    {
        LOG_W("param %s: type mismatch (want %u)", e->name,
              (unsigned)e->type);
        send_param_value((rt_uint16_t)idx);
        return;
    }

    if (e->ro || e->set == RT_NULL)
    {
        LOG_I("param %s readonly", e->name);
        send_param_value((rt_uint16_t)idx);   /* 回显现值, QGC 显示未生效 */
        return;
    }

    if (e->set(e->idx, v) != RT_EOK)
    {
        LOG_W("param %s: value out of range", e->name);
    }
    else
    {
        LOG_I("param %s set (save in %ds)",
              e->name, GCS_PARAM_SAVE_DELAY_MS / 1000);
        save_schedule();
    }
    send_param_value((rt_uint16_t)idx);       /* 成败均回发 (QGC 校对) */
}

/* ---------------------------- 初始化 ---------------------------- */

rt_err_t gcs_param_init(void)
{
    static rt_bool_t inited = RT_FALSE;
    rt_uint32_t i;

    if (inited)
        return RT_EOK;

    /* 名称长度护栏: param_id[16] 末字节必须留给 '\0' (编译期表, 人工维护) */
    for (i = 0; i < GCS_PARAM_COUNT; i++)
    {
        if (rt_strlen(s_params[i].name) > GCS_PARAM_NAME_MAX)
        {
            LOG_E("param %s name too long (> %u)", s_params[i].name,
                  (unsigned)GCS_PARAM_NAME_MAX);
            return -RT_EINVAL;
        }
    }

    if (mavlink_link_attach(gcs_param_on_list, MAVLINK_MSG_ID_PARAM_REQUEST_LIST) != RT_EOK ||
        mavlink_link_attach(gcs_param_on_read, MAVLINK_MSG_ID_PARAM_REQUEST_READ) != RT_EOK ||
        mavlink_link_attach(gcs_param_on_set, MAVLINK_MSG_ID_PARAM_SET) != RT_EOK)
    {
        LOG_E("param: mavlink handler attach failed");
        return -RT_EFULL;
    }

    inited = RT_TRUE;
    LOG_I("gcs param: %u entries (sys/nav/calib), autosave %ds",
          (unsigned)GCS_PARAM_COUNT, GCS_PARAM_SAVE_DELAY_MS / 1000);
    return RT_EOK;
}

void gcs_param_status(struct gcs_param_status *out)
{
    if (out == RT_NULL)
        return;
    out->count = (rt_uint16_t)GCS_PARAM_COUNT;
    out->save_pending = (rt_bool_t)(s_save_at != 0);
}
