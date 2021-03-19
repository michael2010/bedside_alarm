#ifndef _BEDSIDEALARM_HEADER_
#define _BEDSIDEALARM_HEADER_

#include <Arduino.h>
#include "Wire.h"

#define TIMEOUT_S 5
#define ALARM_BEEP_PERIOD 20
#define RTC_SETTING_TIMEOUT 60*2

/* Pin Defintions*/
//Bitbang I2C
#define SCL_PIN PIN_PB2
#define SDA_PIN PIN_PB1

//Human pins
#define DISABLE_ALARM_PIN PIN_PD1
#define BUTTON_PIN PIN_PD2
#define SUPPRESS_NEXT_ALARM_PIN PIN_PD5
#define UPD_RTC_PIN PIN_PD6
#define UPD_RTC_INC_PIN PIN_PD7
#define UPD_RTC_DEC_PIN PIN_PB0

//Interface Pin
#define DS3231_INT PIN_PD3
#define SPEAKER_PIN PIN_PB7
#define FD650_CLK_GATE PIN_PC1

//Power Pin
#define PERI_PWR PIN_PB3
#define DS3231_VCC PIN_PC3
#define DS3231_GND PIN_PC2

#define CLOCK_ADDRESS 0x68

enum rtcSetupPhase:uint8_t{
  year=0,
  month_date=1,
  weekDay=2,
  hour_minute=3,
  reset=4
};

enum ClockState :uint8_t{
  Initing,
  Sleeping,
  AlarmTriggered,
  Alarming,
  ButtonTriggered,
  RTCUpdating,
  Idle
};

//#define ST7920_CS_PIN 10
//#define ST7920_PWR_PIN 9

static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val>>4); }
static uint8_t dec2bin (uint8_t val) { return val + 6 * (val/10); }

void turnOnOffPeri(bool on);
void readDateTimeFromRTC();
void updateRTCSettings();
void refreshScreen();

extern ClockState centralClkState;
extern rtcSetupPhase rtcPhase;
extern byte rtcReadings[7];
extern byte tempRtcValues[8];

#endif
