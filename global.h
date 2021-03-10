#ifndef _BEDSIDEALARM_HEADER_
#define _BEDSIDEALARM_HEADER_

#include <Arduino.h>

#define TIMEOUT_S 5
#define ALARM_BEEP_PERIOD 20
#define RTC_SETTING_TIMEOUT 60*2

/* Pin Defintions*/
//Bitbang I2C
#define SDA_PIN PIN_PC0
#define SCL_PIN PIN_PC1

//Human pins
#define DISABLE_ALARM_PIN PIN_PD1
#define BUTTON_PIN PIN_PD2
#define SUPPRESS_NEXT_ALARM_PIN PIN_PD4
#define UPD_RTC_PIN PIN_PD5
#define UPD_RTC_INC_PIN PIN_PD6
#define UPD_RTC_DEC_PIN PIN_PD7

//Interface Pin
#define DS3231_INT PIN_PD3
#define PERI_PWR PIN_PC3
#define SPEAKER_PIN PIN_PD0

#define CLOCK_ADDRESS 0x68

enum rtcSetupPhase:uint8_t{
  year=0,
  month_date=1,
  weekDay=2,
  hour_minute=3,
  reset=4
};

//#define ST7920_CS_PIN 10
//#define ST7920_PWR_PIN 9

static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val>>4); }
static uint8_t dec2bin (uint8_t val) { return val + 6 * (val/10); }

void turnOnOffPeri(bool on);

extern rtcSetupPhase rtcPhase;

#endif
