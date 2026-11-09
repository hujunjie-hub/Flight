#ifndef TEST_BOARD_H
#define TEST_BOARD_H
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#define __DMB() __atomic_thread_fence(__ATOMIC_SEQ_CST)
typedef intptr_t rt_base_t;
typedef uint32_t rt_tick_t;
typedef uint32_t rt_uint32_t;
typedef int32_t rt_int32_t;
typedef uint8_t rt_uint8_t;
typedef uint16_t rt_uint16_t;
typedef uint32_t rt_size_t;
typedef int32_t rt_off_t;
typedef int rt_err_t;
typedef intptr_t rt_ssize_t;
typedef struct rt_device *rt_device_t;
typedef struct { int unused; } USART_TypeDef;
#define USART2 ((USART_TypeDef *)0)
#define USART3 ((USART_TypeDef *)1)
#define UART4  ((USART_TypeDef *)2)
struct rt_mutex { int unused; };
struct rt_semaphore { unsigned value, max_value; };
struct rt_thread { int unused; };
#define RT_EOK 0
#define RT_ERROR 1
#define RT_EBUSY 16
#define RT_EINVAL 10
#define RT_ENOSYS 11
#define RT_IPC_FLAG_PRIO 1
#define RT_WAITING_FOREVER -1
#define RT_NULL ((void *)0)
#define RT_DEVICE_OFLAG_RDONLY 0x001
#define RT_DEVICE_OFLAG_WRONLY 0x002
#define RT_DEVICE_OFLAG_RDWR 0x003
#define RT_DEVICE_CTRL_CONFIG 0x03
#define RT_SERIAL_RX_NON_BLOCKING 0x2000
#define RT_SERIAL_TX_BLOCKING 0x4000
#define RT_SERIAL_TX_NON_BLOCKING 0x8000
#define RT_SERIAL_CTRL_GET_CONFIG 0x48
struct serial_configure { uint32_t baud_rate; };
#define RT_Device_Class_Miscellaneous 20
#define RT_Device_Class_Sensor 21
#define RT_DEVICE_CTRL_BASE(type) ((RT_Device_Class_##type + 1) * 0x100)
#define RT_DEVICE_CTRL_WDT_SET_TIMEOUT 1
#define RT_DEVICE_CTRL_WDT_START 2
#define RT_DEVICE_CTRL_WDT_KEEPALIVE 3
#define MSH_CMD_EXPORT(fn,desc)
#define rt_kprintf(...) printf(__VA_ARGS__)
rt_base_t rt_hw_interrupt_disable(void);
void rt_hw_interrupt_enable(rt_base_t level);
rt_base_t rt_enter_critical(void);
void rt_exit_critical(void);
int rt_mutex_init(struct rt_mutex *,const char *,int);
int rt_mutex_detach(struct rt_mutex *);
int rt_mutex_take(struct rt_mutex *,rt_int32_t);
int rt_mutex_release(struct rt_mutex *);
int rt_sem_init(struct rt_semaphore *,const char *,int,int);
int rt_sem_detach(struct rt_semaphore *);
int rt_sem_release(struct rt_semaphore *);
int rt_sem_take(struct rt_semaphore *,rt_tick_t);
rt_tick_t rt_tick_get(void);
rt_tick_t rt_tick_from_millisecond(unsigned);
void rt_thread_delay_until(rt_tick_t *,rt_tick_t);
rt_err_t rt_thread_mdelay(rt_int32_t);
int rt_thread_init(struct rt_thread *,const char *,void (*)(void *),void *,void *,unsigned,rt_uint8_t,unsigned);
int rt_thread_startup(struct rt_thread *);
int rt_thread_detach(struct rt_thread *);
rt_device_t rt_device_find(const char *name);
rt_err_t rt_device_open(rt_device_t,rt_uint16_t);
rt_err_t rt_device_close(rt_device_t);
rt_err_t rt_device_control(rt_device_t,int,void *);
rt_ssize_t rt_device_read(rt_device_t,int,void *,size_t);
rt_ssize_t rt_device_write(rt_device_t,int,const void *,size_t);
rt_err_t rt_device_set_tx_complete(rt_device_t,rt_err_t (*)(rt_device_t,void *));
rt_err_t rt_device_set_rx_indicate(rt_device_t,rt_err_t (*)(rt_device_t,rt_size_t));
#endif
