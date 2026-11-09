/* Unicore N4 Reference Commands V2 EN R1.13, tables 7-49/7-70/7-149.
 * ASCII BESTNAVA and RECTIMEA. CRC covers bytes after '#' through before '*'.
 * Receiver output configuration is volatile; never issue SAVECONFIG here. */
#include "unicore_ascii/unicore_ascii.h"
#include "base_types/common_base.h"
#include <math.h>
#include <string.h>
static uint32_t crc32(const char *p,size_t n) {
  uint32_t c=0; unsigned b;
  while(n--) { c^=(uint8_t)*p++; for(b=0;b<8;++b) c=(c>>1)^((c&1)?0xedb88320u:0); }
  return c;
}
static int number(const char *s,double *x) {
  double value=0; int sign=1,exponent=0,exponent_sign=1,fraction=0,digits=0;
  unsigned length=0;
  if(!s || !*s) return 0;
  /* newlib strtod may allocate Bigints. This bounded decimal grammar is
   * sufficient for native logs and has no locale/heap dependencies. */
  if(*s=='-' || *s=='+') { if(*s=='-') sign=-1; ++s; }
  while(*s>='0' && *s<='9') {
    if(++length>20) return 0;
    value=value*10+(*s++-'0'); ++digits;
  }
  if(*s=='.') {
    ++s;
    while(*s>='0' && *s<='9') {
      if(++length>20) return 0;
      value=value*10+(*s++-'0'); ++digits; ++fraction;
    }
  }
  if(!digits) return 0;
  if(*s=='e' || *s=='E') {
    unsigned count=0; ++s;
    if(*s=='-' || *s=='+') { if(*s=='-') exponent_sign=-1; ++s; }
    while(*s>='0' && *s<='9') {
      if(++count>3) return 0;
      exponent=exponent*10+(*s++-'0');
    }
    if(!count || exponent>308) return 0;
  }
  if(*s) return 0;
  exponent=exponent*exponent_sign-fraction;
  *x=sign*value*pow(10.0,exponent); return isfinite(*x);
}
static int integer(const char *s,uint32_t limit,uint32_t *out) {
  double x;
  if(!number(s,&x) || x<0 || x>limit || floor(x)!=x) return 0;
  *out=(uint32_t)x; return 1;
}
static unsigned split(char *s,char **fields,unsigned cap) {
  unsigned n=0;
  do { if(n==cap) return 0; fields[n++]=s; s=strchr(s,','); if(s) *s++=0; } while(s);
  return n;
}
static int fix(const char *type) {
  if(!strcmp(type,"SINGLE")) return GNSS_FIX_SINGLE;
  if(!strcmp(type,"PSRDIFF") || !strcmp(type,"SBAS")) return GNSS_FIX_DGPS;
  if(!strcmp(type,"L1_FLOAT") || !strcmp(type,"IONOFREE_FLOAT") || !strcmp(type,"NARROW_FLOAT")) return GNSS_FIX_RTK_FLOAT;
  if(!strcmp(type,"L1_INT") || !strcmp(type,"WIDE_INT") || !strcmp(type,"NARROW_INT")) return GNSS_FIX_RTK_FIXED;
  return GNSS_FIX_NONE;
}
static int utc_date(char **f,uint64_t *ns) {
  uint32_t y,m,d,hh,mm,ms; unsigned days_in_month; int64_t era,yoe,doy,doe,days; int year;
  static const unsigned mdays[]={31,28,31,30,31,30,31,31,30,31,30,31};
  if(!integer(f[4],2200,&y) || y<1980 || !integer(f[5],12,&m) || !m ||
     !integer(f[6],31,&d) || !d || !integer(f[7],23,&hh) ||
     !integer(f[8],59,&mm) || !integer(f[9],59999,&ms)) return 0;
  /* POSIX cannot represent a leap second. Reject that epoch explicitly. */
  days_in_month=mdays[m-1]+(m==2 && y%4==0 && (y%100!=0 || y%400==0));
  if(d>days_in_month) return 0;
  year=(int)y-(m<=2); era=year/400; yoe=year-era*400;
  doy=(153*((int)m+(m>2?-3:9))+2)/5+d-1; doe=yoe*365+yoe/4-yoe/100+doy;
  days=era*146097+doe-719468;
  if(days<0) return 0;
  *ns=((uint64_t)days*86400000u+hh*3600000u+mm*60000u+ms)*1000000u; return 1;
}
static int decode(char *line,UnicoreAsciiMessage *out) {
  char *star=strchr(line,'*'),*semi; char *h[12],*f[40];
  unsigned hn,fn; uint32_t week,ms,delay,expected=0;
  double x[7]; unsigned i;
  if(!star || strlen(star+1)!=8) return -1;
  for(i=0;i<8;++i) if(!((star[1+i]>='0' && star[1+i]<='9') ||
      (star[1+i]>='a' && star[1+i]<='f') || (star[1+i]>='A' && star[1+i]<='F'))) return -1;
  for(i=0;i<8;++i) {
    char digit=star[1+i];
    expected=(expected<<4)|(uint32_t)(digit<='9' ? digit-'0' : ((digit|32)-'a'+10));
  }
  if(crc32(line+1,(size_t)(star-line-1))!=expected) return -2;
  *star=0; semi=strchr(line,';'); if(!semi) return -1; *semi++=0;
  hn=split(line+1,h,12); fn=split(semi,f,40);
  if(hn!=10 || !fn || strcmp(h[2],"GPS") || strcmp(h[3],"FINE") ||
     !integer(h[4],8191,&week) || !integer(h[5],604799999,&ms) ||
     !integer(h[9],65535,&delay)) return -1;
  memset(out,0,sizeof(*out));
  out->gps_time_ns=((uint64_t)week*604800000u+ms)*1000000u;
  out->output_delay_ms=(uint16_t)delay;
  if(!strcmp(h[0],"BESTNAVA")) {
    UnicorePvt *p=&out->pvt; uint32_t sat; double undulation,latency,hs,track,vs,vstd,hstd;
    if(fn!=30 || !number(f[2],&p->latitude_deg) || !number(f[3],&p->longitude_deg) ||
       !number(f[4],&p->altitude_m) || !number(f[5],&undulation) || strcmp(f[6],"WGS84") ||
       !number(f[7],&x[0]) || !number(f[8],&x[1]) || !number(f[9],&x[2]) ||
       !integer(f[14],255,&sat) || !number(f[23],&latency) || !number(f[25],&hs) ||
       !number(f[26],&track) || !number(f[27],&vs) || !number(f[28],&vstd) || !number(f[29],&hstd)) return -1;
    p->altitude_msl_m=p->altitude_m;
    p->altitude_m+=undulation; /* MSL + geoid undulation = ellipsoid height. */
    p->fix_type=(uint8_t)fix(f[1]); p->satellites=(uint8_t)sat;
    if(!strcmp(f[0],"SOL_COMPUTED") && p->fix_type!=GNSS_FIX_NONE) {
      p->valid_flags|=UNICORE_PVT_POSITION_VALID;
      if(x[0]>0 && x[1]>0 && x[2]>0) {
        p->valid_flags|=UNICORE_PVT_POSITION_COV_VALID;
        for(i=0;i<3;++i) p->position_variance_m2[i]=(float)(x[i]*x[i]);
      }
    }
    /* The public PVT has one epoch. Nonzero velocity latency requires a
     * separate observation, so refuse its velocity instead of misdating it. */
    if(!strcmp(f[21],"SOL_COMPUTED") && !strcmp(f[22],"DOPPLER_VELOCITY") &&
       fabs(latency)<1e-9 && hs>=0 && track>=0 && track<360) {
      p->valid_flags|=UNICORE_PVT_VELOCITY_VALID;
      p->velocity_ned_mps[0]=(float)(hs*cos(track*0.017453292519943295));
      p->velocity_ned_mps[1]=(float)(hs*sin(track*0.017453292519943295)); p->velocity_ned_mps[2]=(float)-vs;
      if(vstd>0 && hstd>0) {
        p->valid_flags|=UNICORE_PVT_VELOCITY_COV_VALID;
        p->velocity_variance_m2ps2[0]=p->velocity_variance_m2ps2[1]=(float)(hstd*hstd);
        p->velocity_variance_m2ps2[2]=(float)(vstd*vstd);
      }
    }
    out->kind=UNICORE_ASCII_BESTNAV; return 1;
  }
  if(!strcmp(h[0],"RECTIMEA")) {
    double offset,std,utc_offset;
    if(fn!=11 || strcmp(f[0],"VALID") || strcmp(f[10],"VALID") ||
       !number(f[1],&offset) || !number(f[2],&std) || std<0 ||
       !number(f[3],&utc_offset) || fabs(utc_offset)>100 ||
       !utc_date(f,&out->utc_time_ns) || ms%1000 || out->utc_time_ns%1000000000u) return -1;
    {
      const uint64_t gps_epoch_posix_ns=315964800000000000ULL;
      const int64_t separation=(int64_t)out->utc_time_ns-
          (int64_t)(out->gps_time_ns+gps_epoch_posix_ns);
      /* Cross-check calendar and week/TOW before publishing a mapping. The
       * calendar has millisecond resolution; never accept unrelated epochs
       * or an offset large enough to overflow subsequent signed arithmetic. */
      if(fabs((double)separation-(utc_offset+offset)*1e9)>1000000.0) return -1;
    }
    /* Conservative receiver model bound, not a zero-uncertainty UTC claim. */
    x[0]=(fabs(offset)+3*std)*1e9+1000;
    if(x[0]>1000000000) return -1;
    out->receiver_accuracy_ns=(uint32_t)ceil(x[0]); out->kind=UNICORE_ASCII_RECTIME; return 1;
  }
  return 0;
}
int unicore_ascii_feed(UnicoreAsciiParser *p,uint8_t b,UnicoreAsciiMessage *out) {
  int r;
  if(!p || !out) return -1;
  if(b=='#') { p->collecting=1; p->length=0; }
  if(!p->collecting) return 0;
  if(b=='\r') return 0;
  if(b=='\n') {
    p->line[p->length]=0; p->collecting=0;
    r=decode(p->line,out);
    if(r==1) p->valid_frames++; else if(r==-2) p->crc_errors++; else if(r<0) p->malformed_frames++;
    return r;
  }
  if(p->length>=UNICORE_ASCII_LINE_CAPACITY-1) { p->collecting=0; p->overflows++; return -1; }
  p->line[p->length++]=(char)b; return 0;
}
