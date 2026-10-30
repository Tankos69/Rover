#include <assert.h>
#include <stdio.h>
#include "../RoverController/display_format.h"
int main() {
 uint8_t d[4];
 rover::degreeDigits(0,true,d);
 assert(d[0]==0 && d[1]==0 && d[2]==0 && d[3]==0x3f);
 rover::degreeDigits(29.6f,true,d);
 assert(d[0]==0 && d[1]==0 && d[2]==0x4f && d[3]==0x3f);
 rover::degreeDigits(-12,true,d);
 assert(d[0]==0 && d[1]==0x40 && d[2]==0x06 && d[3]==0x5b);
 rover::degreeDigits(180,true,d);
 assert(d[0]==0 && d[1]==0x06 && d[2]==0x7f && d[3]==0x3f);
 rover::degreeDigits(9999,true,d);
 for(int i=0;i<4;++i) assert(d[i]==0x6f);
 rover::degreeDigits(10000,true,d);
 for(int i=0;i<4;++i) assert(d[i]==0x40);
 rover::degreeDigits(90,false,d);
 for(int i=0;i<4;++i) assert(d[i]==0x40);
 puts("PASS: degree formatting, rounding, sign, leading blanks, range and fault display");
}
