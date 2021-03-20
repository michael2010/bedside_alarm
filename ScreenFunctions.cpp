#include "global.h"
#include <SimpleSleep.h>

#define TM1650_DISPLAY_BASE 0x34 // Address of the left-most digit 
#define TM1650_DCTRL_BASE   0x24 // Address of the control register of the left-most digit
#define TM1650_MSK_BRIGHT  0b10001111
#define TM1650_MSK_ONOFF  0b11111110
#define TM1650_BRIGHT_SHIFT  4
#define INITIAL_DISPLAY_CTRL 0b00010001;

const byte digitMap[11] {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7f, 0x6f, 0x40};
uint8_t SUPP_CODES[4] {0x6D, 0x3E, 0x73, 0x73};
uint8_t FrEE_CODES[4] {0x71, 0x50, 0x79, 0x79};

byte iCtrl=INITIAL_DISPLAY_CTRL;
uint8_t displayString[4];
bool fullPageRefreshNeeded = false;
rtcSetupPhase rtcPhase = reset;
byte tempRtcValues[8];
uint8_t tempRtcValue;
volatile bool displayStatus = false;
volatile byte timeout = TIMEOUT_S;

extern byte rtcReadings[7];
extern bool isNextAlarmSuppressed;
extern SimpleSleep Sleep;
extern bool buttonInterruptDetachmentAllowed;

//function declarations
void buttonPressHandler();
void DS3231InterruptHandler();

void turnOnOffPeri(bool on){

  digitalWrite(PERI_PWR, on?LOW:HIGH);
  digitalWrite(DS3231_VCC, on?HIGH:LOW);
//  digitalWrite(FD650_CLK_GATE, on?HIGH:LOW);

  delay(50);
  
  if(!on){
    pinMode(SDA, OUTPUT);
    digitalWrite(SDA, LOW);
  }else{
    // I2C init
    Wire.begin();
  
    // Turn on/off display
//    iCtrl = (iCtrl & TM1650_MSK_ONOFF) | 0x01;
    digitalWrite(FD650_CLK_GATE, HIGH);
    Wire.beginTransmission(TM1650_DCTRL_BASE);
    Wire.write((iCtrl & TM1650_MSK_ONOFF) | 0x01);
    Wire.endTransmission();
    digitalWrite(FD650_CLK_GATE, LOW);
  }
}

void changeDisplayPowerStatus(){  
  // attempts to perform a weak debounce
  if(!displayStatus){
    // Shutdown Display and Power down MCU
    rtcPhase = reset;
    turnOnOffPeri(false);
    noTone(SPEAKER_PIN);
    pinMode(SPEAKER_PIN, INPUT);
    buttonInterruptDetachmentAllowed = false;
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressHandler, LOW);
    attachInterrupt(digitalPinToInterrupt(DS3231_INT), DS3231InterruptHandler, LOW);
    Sleep.deeply();
    buttonInterruptDetachmentAllowed = true;
  }else{
    turnOnOffPeri(true);
    previousMinute = 0xFF;
    refreshScreen();
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressHandler, LOW);
  }
}

bool updateSecondFieldAndIcons(){
  if(rtcPhase==reset){
    if(displayString[1] & 0x80){
      displayString[1] &= ~0x80;
    }else{
      displayString[1] |= 0x80;
    }
    digitalWrite(FD650_CLK_GATE, HIGH);
    Wire.beginTransmission(TM1650_DISPLAY_BASE+1);
    Wire.write(displayString+1, 1);
    Wire.endTransmission();
    digitalWrite(FD650_CLK_GATE, LOW);
    return true;
  }
  return false;
}

void refreshDisplayString(uint8_t* displayStr){
  digitalWrite(FD650_CLK_GATE, HIGH);
  Wire.beginTransmission(TM1650_DISPLAY_BASE);
  Wire.write(displayStr,4);
  Wire.endTransmission();
  digitalWrite(FD650_CLK_GATE, LOW);
}

void refreshAll4Digits(byte highByte, byte lowByte){
  displayString[0] = digitMap[highByte>>4];
  displayString[1] = digitMap[highByte&0x0F] | 0x80;
  displayString[2] = digitMap[lowByte>>4];
  displayString[3] = digitMap[lowByte&0x0F];

  refreshDisplayString(displayString);
}

void displayAlarmState(){
  refreshDisplayString(isNextAlarmSuppressed?SUPP_CODES:FrEE_CODES);
//  Sleep.deeplyFor(1000);
}

void refreshScreen(){
  readDateTimeFromRTC();
  // determine if update the second field is enough
  if(previousMinute != 0xFF && (previousMinute == rtcReadings[1] || rtcPhase !=reset)){
    if(updateSecondFieldAndIcons()){
      return;
    }
  }
  refreshAll4Digits(rtcReadings[2],rtcReadings[1]);
  previousMinute = rtcReadings[1];
}
