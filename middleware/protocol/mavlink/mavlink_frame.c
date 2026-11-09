#include "mavlink/mavlink_frame.h"
#include <math.h>
#include <string.h>
void wire_u16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
void wire_u32(uint8_t *p,uint32_t v) { unsigned i; for(i=0;i<4;++i) p[i]=(uint8_t)(v>>(8*i)); }
void wire_u64(uint8_t *p,uint64_t v) { unsigned i; for(i=0;i<8;++i) p[i]=(uint8_t)(v>>(8*i)); }
void wire_float(uint8_t *p,float v) { uint32_t bits; memcpy(&bits,&v,4); wire_u32(p,bits); }
float wire_read_float(const uint8_t *p)
{
  const uint32_t bits = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                        ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}
void wire_altitude(uint8_t p[MAVLINK_ALTITUDE_LEN], uint64_t sample_us,
                   float amsl_m, float local_m, float relative_m)
{
  wire_u64(p, sample_us);
  wire_float(p + 8, NAN); /* No boot-relative barometric altitude. */
  wire_float(p + 12, amsl_m);
  wire_float(p + 16, local_m);
  wire_float(p + 20, relative_m);
  wire_float(p + 24, -1001.0f); /* Terrain altitude unknown. */
  wire_float(p + 28, -1.0f); /* No bottom clearance measurement. */
}
size_t wire_encode(uint32_t id,uint8_t extra,const uint8_t *payload,size_t n,
                       uint8_t seq,uint8_t *out,size_t cap) {
  size_t i; uint16_t c=0xffff;
  if(!out || !payload || !n || n>255 || id>0xffffff) return 0;
  while(n>1 && payload[n-1]==0) --n;
  const size_t data_size=MAVLINK_V2_HEADER_SIZE+n;
  if(cap<data_size+MAVLINK_CRC_SIZE) return 0;
  out[0]=MAVLINK_V2_MAGIC; out[1]=(uint8_t)n; out[2]=out[3]=0; out[4]=seq;
  out[5]=1; out[6]=1; out[7]=(uint8_t)id; out[8]=(uint8_t)(id>>8); out[9]=(uint8_t)(id>>16);
  memcpy(out+MAVLINK_V2_HEADER_SIZE,payload,n);
  for(i=1;i<data_size;++i) c=mavlink_crc_accumulate(c,out[i]);
  c=mavlink_crc_accumulate(c,extra);
  wire_u16(out+data_size,c);
  return data_size+MAVLINK_CRC_SIZE;
}
