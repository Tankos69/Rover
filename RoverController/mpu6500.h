#ifndef ROVER_MPU6500_H
#define ROVER_MPU6500_H
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
// Mount flat: X forward, Y left, Z up. Heading is positive RIGHT.
class Mpu6500 {
 public:
  bool ready=false, calibrating=false, fault=false;
  float yaw=0, speed=0, pitch=0, temperature=0;
  float accel[3]={}, gyro[3]={};
  void begin() {
    ready=false; calibrating=false; fault=false;
    Wire.begin(); Wire.setClock(100000);
    Wire.setWireTimeout(3000,true);
    uint8_t id;
    for (address_=0x68; address_<=0x69; ++address_)
      if (read(0x75,&id,1) && id==0x70) break;
    if (address_>0x69) { fault=true; return; }
    if (!write(0x6B,0x80)) { fault=true; return; }
    delay(100);
    if (!write(0x6B,1) || !write(0x6C,0) || !write(0x1A,3) ||
        !write(0x19,9) || !write(0x1B,0) || !write(0x1C,0) || !write(0x1D,3)) {
      fault=true; return;
    }
    delay(100); calibrate();
  }
  void calibrate() {
    ready=false; calibrating=true; fault=false; count_=0;
    for (uint8_t i=0;i<3;++i) { sum_[i]=0; squares_[i]=0; bias_[i]=0; }
    accelSum_=accelSquares_=0;
    yaw=speed=0; last_=lastSample_=millis();
  }
  bool fresh(uint32_t now) const { return ready && !fault && uint32_t(now-lastSample_)<100; }
  void update(uint32_t now) {
    if (fault || uint32_t(now-last_)<10) return;
    float dt=uint32_t(now-last_)*.001f; last_=now;
    uint8_t status;
    if (!read(0x3A,&status,1)) { fail(); return; }
    if (!(status&1)) {
      if (uint32_t(now-lastSample_)>=100) fail();
      return;
    }
    // Integrate using time between actual fresh samples, not polling time.
    dt=uint32_t(now-lastSample_)*.001f; lastSample_=now;
    uint8_t b[14];
    if (!read(0x3B,b,14) || (ready && dt>.05f)) { fail(); return; }
    for (uint8_t i=0;i<3;++i) {
      int16_t a=be(b+2*i), g=be(b+8+2*i);
      if (a>32000 || a< -32000 || g>32000 || g< -32000) { fail(); return; }
      accel[i]=a/16384.0f; gyro[i]=g/131.0f;
    }
    temperature=be(b+6)/333.87f+21;
    float accPitch=atan2f(-accel[0],sqrtf(accel[1]*accel[1]+accel[2]*accel[2]));
    if (calibrating) {
      float norm=sqrtf(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
      if (fabsf(norm-1)>.15f || fabsf(gyro[0])>5 || fabsf(gyro[1])>5 || fabsf(gyro[2])>5) { fail(); return; }
      for (uint8_t i=0;i<3;++i) { sum_[i]+=gyro[i]; squares_[i]+=gyro[i]*gyro[i]; }
      accelSum_+=norm; accelSquares_+=norm*norm;
      if (++count_==200) {
        for (uint8_t i=0;i<3;++i) {
          bias_[i]=sum_[i]/200;
          if (squares_[i]/200-bias_[i]*bias_[i]>.1f) { fail(); return; }
        }
        if (accelSquares_/200-powf(accelSum_/200,2)>.0025f) { fail(); return; }
        pitch=accPitch; ready=true; calibrating=false; yaw=speed=0;
      }
      return;
    }
    if (!ready) return;
    yaw-=(gyro[2]-bias_[2])*dt;
    // Complementary pitch estimate for approximate gravity removal. This is
    // NOT an independent speed measurement and drifts under real driving.
    pitch=.98f*(pitch+(gyro[1]-bias_[1])*.017453293f*dt)+.02f*accPitch;
    float forward=(accel[0]+sinf(pitch))*9.80665f;
    speed+=forward*dt;
  }
 private:
  uint8_t address_=0x68;
  uint16_t count_=0;
  uint32_t last_=0,lastSample_=0;
  float sum_[3]={},squares_[3]={},bias_[3]={},accelSum_=0,accelSquares_=0;
  void fail() { ready=false; calibrating=false; fault=true; }
  static int16_t be(const uint8_t* p) { return int16_t(uint16_t(p[0])<<8|p[1]); }
  bool write(uint8_t reg,uint8_t value) {
    Wire.beginTransmission(address_); Wire.write(reg); Wire.write(value);
    return Wire.endTransmission()==0;
  }
  bool read(uint8_t reg,uint8_t* data,uint8_t n) {
    Wire.beginTransmission(address_); Wire.write(reg);
    if (Wire.endTransmission(false)!=0 || Wire.requestFrom(address_,n,uint8_t(true))!=n) return false;
    for (uint8_t i=0;i<n;++i) data[i]=Wire.read();
    return true;
  }
};
#endif
