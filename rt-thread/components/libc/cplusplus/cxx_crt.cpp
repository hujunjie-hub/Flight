/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-03-07     Bernard      Add copyright header.
 */

#include <rtthread.h>
#include "cxx_crt.h"

void *operator new(size_t size)
{
    return rt_malloc(size);
}

void *operator new[](size_t size)
{
    return rt_malloc(size);
}

void operator delete(void *ptr)
{
    rt_free(ptr);
}

/* C++14 带尺寸释放: 与上面等价 (rt_free 无尺寸变体), 消除 -Wsized-deallocation */
void operator delete(void *ptr, size_t size)
{
    (void)size;
    rt_free(ptr);
}

void operator delete[](void *ptr)
{
    return rt_free(ptr);
}

void operator delete[](void *ptr, size_t size)
{
    (void)size;
    rt_free(ptr);
}

void __cxa_pure_virtual(void)
{
    rt_kprintf("Illegal to call a pure virtual function.\n");
}
