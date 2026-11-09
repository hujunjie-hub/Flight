#ifndef TEST_MTD_NOR_H
#define TEST_MTD_NOR_H

#include <rtthread.h>

#define RT_Device_Class_MTD 23
#define RT_MTD_NOR_DEVICE(device) ((struct rt_mtd_nor_device *)(device))

struct rt_device { int type; };
struct rt_mtd_nor_device {
    struct rt_device parent;
    uint32_t block_size, block_start, block_end;
};
rt_ssize_t rt_mtd_nor_read(struct rt_mtd_nor_device *, rt_off_t, uint8_t *, rt_size_t);
rt_ssize_t rt_mtd_nor_write(struct rt_mtd_nor_device *, rt_off_t, const uint8_t *, rt_size_t);
rt_err_t rt_mtd_nor_erase_block(struct rt_mtd_nor_device *, rt_off_t, rt_size_t);
#endif
