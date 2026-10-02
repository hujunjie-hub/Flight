/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * C++17 对齐 new/delete -> RT-Thread 堆适配
 *
 * 背景: KF-GINS/GIEngine 的固定尺寸 Eigen 成员带 alignas(16/32/64),
 * 构造时编译器生成 operator new(size, align_val_t) 调用。工具链默认实现
 * 落到 newlib 的 _memalign_r, 它用自己的块簿记去切分/归还 _malloc_r 的
 * 内存, 与 RT-Thread 小内存分配器的块头完全对不上: 引擎首次构造时
 * rt_smem_free 内 RT_ASSERT(MEM_ISUSED) 断言失败, gins 线程 (优先级 9)
 * 卡死在断言循环, 全系统低优先级线程饿死 (实测表现为控制台静默)。
 *
 * 修法: 接管 4 个对齐 new/delete (普通 new 走 _malloc_r -> rt_malloc,
 * 不受影响)。策略为 rt_malloc 超配 + 返回对齐地址, 对齐地址前一个指针
 * 字保存原始块指针, delete 时原样归还 rt_free —— 与分配器簿记完全兼容。
 *
 * 注意: -fno-exceptions, OOM 时不抛 bad_alloc, 打印后返回 RT_NULL
 * (调用方空指针访问会以可调试的 HardFault 暴露, 好过静默死循环)。
 */

#include <new>
#include <stddef.h>
#include <rtthread.h>

#define LOG_TAG "gins.new"
#define LOG_LVL LOG_LVL_ERROR
#include <ulog.h>

static void *gins_aligned_alloc(size_t size, size_t align)
{
    void *raw, *aligned;

    if (align < sizeof(void *))
        align = sizeof(void *);

    raw = rt_malloc(size + align);
    if (raw == RT_NULL)
    {
        LOG_E("aligned new: OOM (%u + %u bytes)",
              (rt_uint32_t)size, (rt_uint32_t)align);
        return RT_NULL;
    }

    aligned = (void *)((((rt_uintptr_t)raw) + sizeof(void *) + align - 1u)
                       & ~((rt_uintptr_t)align - 1u));
    ((void **)aligned)[-1] = raw;
    return aligned;
}

static void gins_aligned_free(void *p) noexcept
{
    if (p != RT_NULL)
        rt_free(((void **)p)[-1]);
}

void *operator new(std::size_t size, std::align_val_t al)
{
    return gins_aligned_alloc(size, (size_t)al);
}

void *operator new[](std::size_t size, std::align_val_t al)
{
    return gins_aligned_alloc(size, (size_t)al);
}

void operator delete(void *p, std::align_val_t al) noexcept
{
    (void)al;
    gins_aligned_free(p);
}

void operator delete[](void *p, std::align_val_t al) noexcept
{
    (void)al;
    gins_aligned_free(p);
}

/* 带尺寸的对齐 delete (编译器可能优先匹配带 size 重载) */
void operator delete(void *p, std::size_t size, std::align_val_t al) noexcept
{
    (void)size;
    (void)al;
    gins_aligned_free(p);
}

void operator delete[](void *p, std::size_t size, std::align_val_t al) noexcept
{
    (void)size;
    (void)al;
    gins_aligned_free(p);
}
