#include "global.h"

const uint8_t ALARM2_SETTING[5] {0x0b, 0x28, 0x06, 0x80,0b00000110};
byte rtcReadings[7];
byte previousMinute;

void readDateTimeFromRTC(){
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();

  Wire.requestFrom(CLOCK_ADDRESS, 7);
  uint8_t itr = 0;
  for(;itr<7;itr++){
    rtcReadings[itr] = Wire.read();
  }
}

void updateRTCSettings(){
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(tempRtcValues[7]);
  Wire.write(tempRtcValues[6]);
  Wire.write(tempRtcValues[5]);
  Wire.write(tempRtcValues[3]);
  Wire.write(tempRtcValues[2]);
  Wire.write(tempRtcValues[1]);
  Wire.endTransmission();

  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write(ALARM2_SETTING,5);
  Wire.endTransmission();
}

void clearAlarm2Flag(){
    Wire.beginTransmission(CLOCK_ADDRESS);
    Wire.write(0x0F);
    Wire.write(0x08);
    Wire.endTransmission();
}
