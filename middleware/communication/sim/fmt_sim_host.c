#include "sim/fmt_sim_host.h"
#include <stdint.h>
#include <stddef.h>

#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
typedef SOCKET fmt_socket_t;
#define FMT_INVALID_SOCKET INVALID_SOCKET
#define FMT_CLOSE_SOCKET closesocket
static LARGE_INTEGER clock_frequency;
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
typedef int fmt_socket_t;
#define FMT_INVALID_SOCKET (-1)
#define FMT_CLOSE_SOCKET close
#endif

static fmt_socket_t peer = FMT_INVALID_SOCKET;
static uint8_t sequence;
static uint64_t next_heartbeat_us;

int fmt_sim_host_open(uint16_t port)
{
    struct sockaddr_in address = {0};
#ifdef _WIN32
    WSADATA data;
    u_long nonblocking = 1;
    if (peer != FMT_INVALID_SOCKET || port == 0 ||
        !QueryPerformanceFrequency(&clock_frequency) ||
        WSAStartup(MAKEWORD(2, 2), &data) != 0)
        return 0;
    peer = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (peer == INVALID_SOCKET || ioctlsocket(peer, FIONBIO, &nonblocking) != 0 ||
        (address.sin_family = AF_INET, address.sin_port = htons(port),
         address.sin_addr.s_addr = htonl(INADDR_LOOPBACK),
         connect(peer, (const struct sockaddr *)&address, sizeof(address)) != 0)) {
        if (peer != INVALID_SOCKET) FMT_CLOSE_SOCKET(peer);
        peer = FMT_INVALID_SOCKET;
        WSACleanup();
        return 0;
    }
#else
    int flags;
    if (peer != FMT_INVALID_SOCKET || port == 0)
        return 0;
    peer = socket(AF_INET, SOCK_DGRAM, 0);
    if (peer < 0) return 0;
    flags = fcntl(peer, F_GETFL, 0);
    if (flags < 0 || fcntl(peer, F_SETFL, flags | O_NONBLOCK) != 0) {
        FMT_CLOSE_SOCKET(peer);
        peer = FMT_INVALID_SOCKET;
        return 0;
    }
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(peer, (const struct sockaddr *)&address, sizeof(address)) != 0) {
        FMT_CLOSE_SOCKET(peer);
        peer = FMT_INVALID_SOCKET;
        return 0;
    }
#endif
    next_heartbeat_us = 0;
    sequence = 0;
    return 1;
}

void fmt_sim_host_close(void)
{
    if (peer != FMT_INVALID_SOCKET) {
        FMT_CLOSE_SOCKET(peer);
        peer = FMT_INVALID_SOCKET;
#ifdef _WIN32
        WSACleanup();
#endif
    }
}

unsigned fmt_sim_host_receive(mavlink_fmt_state_t *state)
{
    uint8_t packet[1024];
    unsigned received = 0;
    const uint64_t wall_us = fmt_sim_host_now_us();
    if (peer == FMT_INVALID_SOCKET || !state) return 0u;
    /* Simulator liveness is measured on the PC clock. The plant clock can
     * pause while waiting for a frame, including during slow rendering. */
    if (wall_us >= next_heartbeat_us) {
        size_t length = mavlink_hil_build_heartbeat(sequence++, packet, sizeof(packet));
        if (send(peer, (const char *)packet, (int)length, 0) == (int)length)
            next_heartbeat_us = wall_us + 1000000u;
    }
    /* Bound the receive work per scheduler turn. */
    for (unsigned i = 0; i < 16; ++i) {
        int bytes = (int)recv(peer, (char *)packet, sizeof(packet), 0);
        if (bytes <= 0)
            break;
        for (unsigned offset = 0; offset + 12u <= (unsigned)bytes;) {
            const uint8_t *frame = packet + offset;
            if (frame[0] != MAVLINK_V2_MAGIC) {
                ++offset;
                continue;
            }
            unsigned size = 12u + frame[1] + ((frame[2] & 1u) ? 13u : 0u);
            if (size > (unsigned)bytes - offset)
                break;
            received += (unsigned)mavlink_hil_decode_fmt_state(frame, size, state);
            offset += size;
        }
    }
    return received;
}

int fmt_sim_host_send(uint64_t now_us, const float controls[4], int armed)
{
    static const float stopped[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
    uint8_t frame[96];
    size_t length;
    if (peer == FMT_INVALID_SOCKET || !controls) return 0;
    length = mavlink_hil_build_actuator(now_us, armed ? controls : stopped,
                                        armed ? 128u : 0u, sequence++, frame, sizeof(frame));
    return length != 0 && send(peer, (const char *)frame, (int)length, 0) == (int)length;
}

uint64_t fmt_sim_host_now_us(void)
{
#ifdef _WIN32
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (uint64_t)(now.QuadPart / clock_frequency.QuadPart) * 1000000u +
           (uint64_t)(now.QuadPart % clock_frequency.QuadPart) * 1000000u /
               (uint64_t)clock_frequency.QuadPart;
#else
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return 0u;
    return (uint64_t)now.tv_sec * 1000000u + (uint64_t)now.tv_nsec / 1000u;
#endif
}

void fmt_sim_host_sleep(void)
{
#ifdef _WIN32
    Sleep(1);
#else
    const struct timespec delay = {0, 1000000L};
    (void)nanosleep(&delay, NULL);
#endif
}
