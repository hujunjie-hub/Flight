#ifndef TEST_RTTHREAD_H
#define TEST_RTTHREAD_H
#include "board.h"
#include <string.h>

/* Non-blocking IPC model for the deterministic single-threaded Host fixture. */
#define RT_ALIGN_SIZE 8u
#define rt_align(bytes) __attribute__((aligned(bytes)))
#define RT_IPC_FLAG_FIFO 0
#define RT_IPC_CMD_RESET 1
#define RT_IPC_CMD_SET_VLIMIT 3
#define RT_EFULL 2
#define RT_ETIMEOUT 3
#define HOST_MQ_STRIDE(size) ((((size) + RT_ALIGN_SIZE - 1u) & ~(RT_ALIGN_SIZE - 1u)) + sizeof(size_t))
#define RT_MQ_BUF_SIZE(size, count) (HOST_MQ_STRIDE(size) * (count))

static inline rt_err_t rt_sem_control(struct rt_semaphore *sem, int command, void *argument)
{
    if (command != RT_IPC_CMD_SET_VLIMIT)
        return -RT_EINVAL;
    sem->max_value = (unsigned)(uintptr_t)argument;
    return RT_EOK;
}

struct rt_messagequeue {
    uint8_t *pool;
    size_t message_size, stride, capacity, head, count;
};

static inline rt_err_t rt_mq_init(struct rt_messagequeue *queue, const char *name,
                                  void *pool, rt_size_t size, rt_size_t pool_size, rt_uint8_t flag)
{
    (void)name;
    (void)flag;
    const size_t stride = HOST_MQ_STRIDE(size);
    *queue = (struct rt_messagequeue){
        .pool = (uint8_t *)pool, .message_size = size, .stride = stride,
        .capacity = pool_size / stride, .head = 0u, .count = 0u,
    };
    return queue->capacity ? RT_EOK : -RT_EINVAL;
}

static inline rt_err_t rt_mq_detach(struct rt_messagequeue *queue)
{
    queue->capacity = 0u;
    queue->count = 0u;
    return RT_EOK;
}

static inline rt_err_t rt_mq_send(struct rt_messagequeue *queue, const void *data, rt_size_t size)
{
    if (size > queue->message_size)
        return -RT_EINVAL;
    if (queue->count == queue->capacity)
        return -RT_EFULL;
    const size_t index = (queue->head + queue->count) % queue->capacity;
    uint8_t *slot = queue->pool + index * queue->stride;
    const size_t length = size;
    memcpy(slot, &length, sizeof(length));
    memcpy(slot + sizeof(length), data, size);
    ++queue->count;
    return RT_EOK;
}

static inline rt_ssize_t rt_mq_recv(struct rt_messagequeue *queue, void *data,
                                    rt_size_t capacity, rt_int32_t timeout)
{
    (void)timeout;
    if (!queue->count)
        return -RT_ETIMEOUT;
    const uint8_t *slot = queue->pool + queue->head * queue->stride;
    size_t length;
    memcpy(&length, slot, sizeof(length));
    if (length > capacity)
        length = capacity;
    memcpy(data, slot + sizeof(length), length);
    queue->head = (queue->head + 1u) % queue->capacity;
    --queue->count;
    return (rt_ssize_t)length;
}

static inline rt_err_t rt_mq_control(struct rt_messagequeue *queue, int command, void *argument)
{
    (void)argument;
    if (command != RT_IPC_CMD_RESET)
        return -RT_EINVAL;
    queue->head = queue->count = 0u;
    return RT_EOK;
}

#endif
