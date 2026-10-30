#ifndef ROVER_DISPLAY_FORMAT_H
#define ROVER_DISPLAY_FORMAT_H
#include <stdint.h>
#include <math.h>
namespace rover {
// Bits 0..6 are a..g, HIGH lights a segment on a common-cathode display.
inline void degreeDigits(float degrees, bool valid, uint8_t out[4]) {
  static const uint8_t numerals[10]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};
  for (uint8_t i=0;i<4;++i) out[i]=0;
  if (!valid || !isfinite(degrees) || degrees>9999.49f || degrees< -999.49f) {
    for (uint8_t i=0;i<4;++i) out[i]=0x40; // ----: unavailable/overflow
    return;
  }
  bool negative=degrees<0;
  uint16_t value=uint16_t(fabsf(degrees)+.5f);
  uint8_t position=4;
  bool nonzero=value!=0;
  do { out[--position]=numerals[value%10]; value/=10; } while (value && position);
  if (negative && nonzero && position) out[--position]=0x40;
}
}
#endif
