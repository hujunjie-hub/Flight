#include <utest.h>

#include "mavlink/mavlink_frame.h"
#include "nmea/nmea.h"

static void nmea_checksum(void)
{
    const char good[] = "$GPGLL,4916.45,N,12311.12,W,225444,A,*1D\r\n";
    const char bad[] = "$GPGLL,4916.45,N,12311.12,W,225444,A,*00\r\n";
    NmeaSentence sentence = {0};

    uassert_true(nmea_parse(good, sizeof(good) - 1u, &sentence));
    uassert_true(sentence.body == good + 1);
    uassert_false(nmea_parse(bad, sizeof(bad) - 1u, &sentence));
}

static void mavlink_frame_encoding(void)
{
    const uint8_t payload[] = {42u, 0u, 0u};
    uint8_t frame[16] = {0};

    uassert_int_equal(wire_encode(0x010203u, 50u, payload, sizeof(payload), 9u,
                                  frame, sizeof(frame)), 13u);
    uassert_int_equal(frame[0], 0xfdu);
    uassert_int_equal(frame[1], 1u);
    uassert_int_equal(frame[4], 9u);
    uassert_int_equal(frame[7], 3u);
    uassert_int_equal(frame[10], 42u);
}

static void static_queue_fifo(void)
{
    struct rt_messagequeue queue = {0};
    rt_align(RT_ALIGN_SIZE) uint8_t pool[RT_MQ_BUF_SIZE(sizeof(uint32_t), 2u)];
    const uint32_t first = 1u, second = 2u;
    uint32_t received = 0u;

    rt_err_t status = rt_mq_init(&queue, "fltutmq", pool, sizeof(first), sizeof(pool), RT_IPC_FLAG_FIFO);
    uassert_int_equal(status, RT_EOK);
    if (status != RT_EOK)
        return;

    uassert_int_equal(rt_mq_send(&queue, &first, sizeof(first)), RT_EOK);
    uassert_int_equal(rt_mq_send(&queue, &second, sizeof(second)), RT_EOK);
    uassert_int_equal(rt_mq_send(&queue, &first, sizeof(first)), -RT_EFULL);
    uassert_int_equal(rt_mq_recv(&queue, &received, sizeof(received), 0), (rt_ssize_t)sizeof(received));
    uassert_int_equal(received, first);
    uassert_int_equal(rt_mq_recv(&queue, &received, sizeof(received), 0), (rt_ssize_t)sizeof(received));
    uassert_int_equal(received, second);
    uassert_int_equal(rt_mq_recv(&queue, &received, sizeof(received), 0), -RT_ETIMEOUT);
    uassert_int_equal(rt_mq_detach(&queue), RT_EOK);
}

static void testcase(void)
{
    UTEST_UNIT_RUN(nmea_checksum);
    UTEST_UNIT_RUN(mavlink_frame_encoding);
    UTEST_UNIT_RUN(static_queue_fifo);
}

UTEST_TC_EXPORT(testcase, "flight.protocol_ipc", RT_NULL, RT_NULL, 5);
