#include "dshot_packet.h"
#define FLIGHT_DRIVER_CODEC_TEST
#include "../../sensor_device/imu/adis16505.c"
#include "../../sensor_device/time_base/timer_capture.c"
#include "../../sensor_device/actuator/dshot.c"
#undef FLIGHT_DRIVER_CODEC_TEST
#include <stdio.h>
#include <stdint.h>
static unsigned checks,failures;
#define CHECK(value) do { checks++; if (!(value)) { failures++; printf("FAIL line %u\n",(unsigned)__LINE__); } } while(0)
int main(void)
{
    uint16_t words[17], frame[4]={0x8000u,0x0001u,0xaaaau,0x5555u};
    uint8_t wire[34]; uint32_t out[68]; unsigned i;
    CHECK(dshot_encode_packet(0,0)==0x0000u);
    CHECK(dshot_encode_packet(48,0)==0x0606u);
    CHECK(dshot_encode_packet(2047,1)==0xffffu);
    CHECK(dshot_encode_packet(2048,0)==0u);
    CHECK(dshot_encode_packet(48,2)==0u);
    dshot_encode_dma(frame,344,688,out);
    /* Golden serialization: preloaded MSB is excluded from DMA; channel1's
     * only one is already sent, channel2's LSB must be the final data bit. */
    for(i=0;i<15;i++) CHECK(out[i*4u]==344u);
    for(i=0;i<14;i++) CHECK(out[i*4u+1u]==344u);
    CHECK(out[57]==688u);
    CHECK(out[2]==344u && out[6]==688u && out[58]==344u);
    CHECK(out[3]==688u && out[7]==344u && out[59]==688u);
    for(i=60;i<68;i++) CHECK(out[i]==0u);
    words[0]=0xdeadu;
    for(i=1;i<16;i++) words[i]=(uint16_t)(0x0100u+i);
    words[16]=0x0087u; /* sum of 15 high bytes (1) and low bytes 1..15 */
    imu_burst_to_bytes(words,wire);
    CHECK(wire[0]==0xdeu && wire[1]==0xadu && wire[2]==1u && wire[3]==1u);
    CHECK(wire[30]==1u && wire[31]==15u && wire[32]==0u && wire[33]==0x87u);
    CHECK(imu_burst_checksum_valid(wire));
    wire[0]^=0xffu; CHECK(imu_burst_checksum_valid(wire));
    wire[3]^=1u; CHECK(!imu_burst_checksum_valid(wire)); wire[3]^=1u;
    wire[33]^=1u; CHECK(!imu_burst_checksum_valid(wire));
    CHECK(timer_extend_capture(7,0,100,110)==UINT64_C(0x700000064));
    CHECK(timer_extend_capture(7,1,0xfffffffeu,3)==UINT64_C(0x7fffffffe));
    CHECK(timer_extend_capture(7,1,2,3)==UINT64_C(0x800000002));
    CHECK(timer_extend_capture(0xffffffffu,1,2,3)==2u);
    printf("driver codec: %u checks, %u failures\n",checks,failures);
    return failures ? 1 : 0;
}
