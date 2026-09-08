#include "ds3231.h"
#include <stdio.h>
#include <string.h>
static unsigned days(unsigned y, unsigned m) {
    static const unsigned d[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    return d[m-1] + (m == 2 && y % 4 == 0);
}
static int valid(const RTC_DateTime *d) {
    return d && d->year >= 2000 && d->year <= 2099 && d->month >= 1 &&
        d->month <= 12 && d->day >= 1 && d->day <= days(d->year,d->month) &&
        d->hour < 24 && d->minute < 60 && d->second < 60;
}
static unsigned bcd(unsigned n) { return (n/10)*16+n%10; }
static unsigned unbcd(unsigned n) { return (n%16 > 9 || n/16 > 9) ? 999 : n/16*10+n%16; }
uint32_t RTC_ToUnix(const RTC_DateTime *d) {
    if (!valid(d)) return 0;
    uint32_t n = 10957; /* Days 1970-01-01 -> 2000-01-01. */
    for (unsigned y=2000; y<d->year; ++y) n += 365+(y%4==0);
    for (unsigned m=1; m<d->month; ++m) n += days(d->year,m);
    return ((n+d->day-1)*24+d->hour)*3600+d->minute*60+d->second;
}
int RTC_Encode(const RTC_DateTime *d, uint8_t r[7]) {
    if (!valid(d) || !r) return -1;
    r[0]=bcd(d->second); r[1]=bcd(d->minute); r[2]=bcd(d->hour);
    r[3]=(RTC_ToUnix(d)/86400+4)%7+1; /* Sunday=1. */
    r[4]=bcd(d->day); r[5]=bcd(d->month); r[6]=bcd(d->year-2000);
    return 0;
}
int RTC_Decode(const uint8_t r[7], RTC_DateTime *d) {
    if (!r || !d || (r[0]&0x80) || (r[1]&0x80) || (r[2]&0x80) ||
        r[3]<1 || r[3]>7 || (r[4]&0xc0) || (r[5]&0xe0)) return -1;
    d->second=unbcd(r[0]); d->minute=unbcd(r[1]);
    d->hour=unbcd(r[2]&0x3f);
    if (r[2]&0x40) {
        unsigned h=unbcd(r[2]&0x1f);
        if (h<1 || h>12) return -1;
        d->hour=h%12+((r[2]&0x20)?12:0);
    }
    d->day=unbcd(r[4]); d->month=unbcd(r[5]); d->year=2000+unbcd(r[6]);
    return valid(d)?0:-1;
}
int RTC_ParseUTC(const char *s, RTC_DateTime *d) {
    if (!s || !d || strlen(s)!=20 || s[4]!='-' || s[7]!='-' || s[10]!='T' ||
        s[13]!=':' || s[16]!=':' || s[19]!='Z') return -1;
    for (unsigned i=0;i<19;++i)
        if (i!=4 && i!=7 && i!=10 && i!=13 && i!=16 && (s[i]<'0'||s[i]>'9')) return -1;
    if (sscanf(s,"%u-%u-%uT%u:%u:%uZ",&d->year,&d->month,&d->day,
        &d->hour,&d->minute,&d->second)!=6) return -1;
    return valid(d)?0:-1;
}
void RTC_FormatUTC(const RTC_DateTime *d, char *s, size_t n) {
    if (!valid(d)) { if (n) s[0]=0; return; }
    snprintf(s,n,"%04u-%02u-%02uT%02u:%02u:%02uZ",d->year,d->month,d->day,d->hour,d->minute,d->second);
}
