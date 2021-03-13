#include "DS3231.h"
#include "global.h"
#include <SimpleSleep.h>
#include <BitBang_I2C.h>

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
BBI2C bbi2c;
//extern DateTime now;
//extern DateTime previous;
extern byte rtcReadings[7];
extern byte previousMinute;
extern RTClib RTC;
extern bool isNextAlarmSuppressed;
extern bool displayStatus;
extern bool alarm2Set;
extern SimpleSleep Sleep;
extern bool buttonInterruptDetachmentAllowed;
extern uint8_t timeout;

//function declarations
void buttonPressHandler();
void refreshWholeScreen();
byte readButtonState(byte pinNo);
void readDateTimeFromRTC();
void DS3231InterruptHandler();
void refreshAll4Digits(byte highByte, byte lowByte);

void bbi2cInit(){
  memset(&bbi2c, 0, sizeof(bbi2c));
  bbi2c.bWire = 0; // use bit bang, not wire library
  bbi2c.iSDA = SDA_PIN;
  bbi2c.iSCL = SCL_PIN;
  I2CInit(&bbi2c, 100000L);
}

void turnOnOffPeri(bool on){
  if(on){
    digitalWrite(PERI_PWR, LOW);
//    digitalWrite(DS3231_VCC, HIGH);
    pinMode(SUPPRESS_NEXT_ALARM_PIN, INPUT_PULLUP);
    pinMode(DISABLE_ALARM_PIN, INPUT_PULLUP);
    pinMode(UPD_RTC_PIN, INPUT_PULLUP);
    pinMode(UPD_RTC_INC_PIN, INPUT_PULLUP);
    pinMode(UPD_RTC_DEC_PIN, INPUT_PULLUP);
  }else{
    digitalWrite(PERI_PWR, HIGH);
//    digitalWrite(DS3231_VCC, LOW);
//    pinMode(SUPPRESS_NEXT_ALARM_PIN, INPUT);
//    pinMode(DISABLE_ALARM_PIN, INPUT);
//    pinMode(UPD_RTC_PIN, INPUT);
//    pinMode(UPD_RTC_INC_PIN, INPUT);
//    pinMode(UPD_RTC_DEC_PIN, INPUT);
  }

  iCtrl = (iCtrl & TM1650_MSK_ONOFF) | on;
//  I2CWrite(&bbi2c, TM1650_DCTRL_BASE, &iCtrl, 1); 
  Wire.beginTransmission(TM1650_DCTRL_BASE);
  Wire.write(iCtrl);
  Wire.endTransmission();
}

void changeDisplayPowerStatus(){  
  // attempts to perform a weak debounce
  if(!displayStatus){
    // Shutdown Display and Power down MCU
    rtcPhase = reset;
    turnOnOffPeri(false);
    alarm2Set = false;
    pinMode(SPEAKER_PIN, INPUT);
    buttonInterruptDetachmentAllowed = false;
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressHandler, LOW);
    attachInterrupt(digitalPinToInterrupt(DS3231_INT), DS3231InterruptHandler, LOW);
    Sleep.deeply();
    buttonInterruptDetachmentAllowed = true;
    
  }else{
    turnOnOffPeri(true);
    readDateTimeFromRTC();
    refreshWholeScreen();
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressHandler, LOW);
  }
}

bool updateAlarmIcons(){
    // Samples supress-pin value; if LOW, then toggle the supression state
  if(digitalRead(SUPPRESS_NEXT_ALARM_PIN) == LOW){
    isNextAlarmSuppressed = !isNextAlarmSuppressed;
//    I2CWrite(&bbi2c, TM1650_DISPLAY_BASE, isNextAlarmSuppressed?SUPP_CODES:FrEE_CODES, 4); 
    Wire.beginTransmission(TM1650_DISPLAY_BASE);
    Wire.write(isNextAlarmSuppressed?SUPP_CODES:FrEE_CODES,4);
    Wire.endTransmission();
    fullPageRefreshNeeded = true;
    timeout = TIMEOUT_S;
    return true;
  }else if(digitalRead(UPD_RTC_PIN) == LOW){
    rtcPhase=rtcSetupPhase((((uint8_t)rtcPhase)+1)%5);

    if(rtcPhase==reset){
      // Update RTC
      digitalWrite(FD650_CLK_GATE, LOW);
      timeout = TIMEOUT_S;
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
      Wire.write(0x0b);  // A1 starts at 0bh
      Wire.write(0x28);// | 0x80);
      Wire.write(0x06);// | 0x80);
      Wire.write(0x80);
      Wire.write(0b00000110);
      Wire.endTransmission();
      digitalWrite(FD650_CLK_GATE, HIGH);
    }else{
      if(rtcPhase==year){
        tempRtcValues[0]=0x20;
        tempRtcValues[1]=rtcReadings[6];
        tempRtcValues[2]=rtcReadings[5];
        tempRtcValues[3]=rtcReadings[4];
        tempRtcValues[4]=0xAA;
        tempRtcValues[5]=rtcReadings[3];
        tempRtcValues[6]=rtcReadings[2];
        tempRtcValues[7]=rtcReadings[1];
      } 
      refreshAll4Digits(tempRtcValues[2*(uint8_t)rtcPhase],tempRtcValues[2*(uint8_t)rtcPhase+1]);
      timeout = RTC_SETTING_TIMEOUT;
    }
    return true;

  }else if(digitalRead(UPD_RTC_INC_PIN) == LOW){
    timeout = RTC_SETTING_TIMEOUT;
    switch(rtcPhase){
      case year:
      {
        if(((++tempRtcValues[1])&0x0F) == 0x0A){
          tempRtcValues[1]=tempRtcValues[1]+6;
          if(tempRtcValues[1]==0xA0){
            tempRtcValues[1]=0x00;
          }
        }
        break;
      }
      case month_date:
      {
        /*
         * 1. Check leap year
         * 2. Check month's max date
         * 3. Check date falls within the range
         */
        bool leapYear = tempRtcValues[1]%4 == 0;
        tempRtcValues[3]++;

        /*
         * Month is Feb and (leapyear and new date is 30 or non-leapyear and new date is 29)
         */
        if(tempRtcValues[2]==2 && ((leapYear && tempRtcValues[3]==0x2A) || tempRtcValues[3]==0x29)){
            tempRtcValues[2]++;
            tempRtcValues[3] = 1;
        }else if((tempRtcValues[3]&0x0F) == 0x0A){
          tempRtcValues[3]=tempRtcValues[3]+6;
        }else if(tempRtcValues[3]>0x30){
          /*
           * Handle date overflow to 31 or 32
           */
          if(tempRtcValues[3]==0x32 ||
             tempRtcValues[2]==4 || 
             tempRtcValues[2]==6 || 
             tempRtcValues[2]==9 || 
             tempRtcValues[2]==11){
              tempRtcValues[2]++;
              tempRtcValues[3] = 1;
           }
        }

        /*
         * check and adjust month, set bcd for 0x0A and reset to 1 for 0x13
         */
        if(tempRtcValues[2] == 0x0A){
          tempRtcValues[2] = 0x10;
        }
        if(tempRtcValues[2] == 0x13){
          tempRtcValues[2]=1;
        }
        break;
      }
      case weekDay:
      {
        ++tempRtcValues[5];
        if(tempRtcValues[5]==8)
          tempRtcValues[5]=1;
        break;
      }
      case hour_minute:
      {
        if(((++tempRtcValues[7])&0x0F) == 0x0A){
          tempRtcValues[7]=tempRtcValues[7]+6;
          if(tempRtcValues[7]==0x60){
            tempRtcValues[7]=0;
            if(tempRtcValues[6]==0x23){
              tempRtcValues[6]==0x00;
            }else{
              if(((++tempRtcValues[6])&0x0F) == 0x0A){
                tempRtcValues[6]=tempRtcValues[6]+6;
              }
            }
          }
        }
        break;
      }
    }
    refreshAll4Digits(tempRtcValues[2*(uint8_t)rtcPhase],tempRtcValues[2*(uint8_t)rtcPhase+1]);
    return true;
  }else if(digitalRead(UPD_RTC_DEC_PIN) == LOW){
    timeout = RTC_SETTING_TIMEOUT;
    switch(rtcPhase){
      case year:
      {
        if((--tempRtcValues[1] & 0x0F)==0x0F){
          tempRtcValues[1]-=0x06;
          if(tempRtcValues[1]==0xF9){
            tempRtcValues[1]=0x99;
          }
        }
        break;
      }
      case month_date:
      {
        bool leapYear = tempRtcValues[1]%4 == 0;
        tempRtcValues[3]--;
        
        if((tempRtcValues[3] & 0x0F) == 0x0F){
          tempRtcValues[3]-=0x06;
        }
        if(tempRtcValues[3] == 0x00){
          if(--tempRtcValues[2]==2){
            tempRtcValues[3] = leapYear?0x29:0x28;
          }else{
            if(tempRtcValues[2]==4 || 
             tempRtcValues[2]==6 || 
             tempRtcValues[2]==9 || 
             tempRtcValues[2]==11){
              tempRtcValues[3] = 0x30;
            }else{
              tempRtcValues[3] = 0x31;
            }
          }
        }

        /*
         * check and adjust month to Dec
         */
        if(tempRtcValues[2] == 0xFF){
          tempRtcValues[2]=12;
        }
        break;
      }
      case weekDay:
      {
        --tempRtcValues[5];
        if(tempRtcValues[5]==0)
          tempRtcValues[5]=7;
        break;
      }
      case hour_minute:
      {
        if((--tempRtcValues[7]&0x0F) == 0x0F){
          tempRtcValues[7]-=0x06;
          if(tempRtcValues[7]==0xF9){
            tempRtcValues[7]=0x59; 
            if((--tempRtcValues[6]&0x0F) == 0x0F){
              tempRtcValues[6] -= 0x06;
              if(tempRtcValues[6]==0xF9){
                tempRtcValues[6] = 0x23;
              }
            }
          }
        }
        break;
      }
    }
    refreshAll4Digits(tempRtcValues[2*(uint8_t)rtcPhase],tempRtcValues[2*(uint8_t)rtcPhase+1]);
    return true;
  }
  return false;
}

void updateSecondFieldAndIcons(){
  if(!updateAlarmIcons()){
    if(fullPageRefreshNeeded){
      fullPageRefreshNeeded = false;
      refreshWholeScreen();
    }else if(rtcPhase==reset){
      if(displayString[1] & 0x80){
        displayString[1] &= ~0x80;
      }else{
        displayString[1] |= 0x80;
      }
//      I2CWrite(&bbi2c, TM1650_DISPLAY_BASE+1, displayString+1, 1); 
      Wire.beginTransmission(TM1650_DISPLAY_BASE+1);
      Wire.write(displayString+1, 1);
      Wire.endTransmission();
    }
  }
}

void refreshAll4Digits(byte highByte, byte lowByte){
  displayString[0] = digitMap[highByte>>4];
  displayString[1] = digitMap[highByte&0x0F] | 0x80;
  displayString[2] = digitMap[lowByte>>4];
  displayString[3] = digitMap[lowByte&0x0F];

//  I2CWrite(&bbi2c, TM1650_DISPLAY_BASE, displayString, 4); 
  Wire.beginTransmission(TM1650_DISPLAY_BASE);
  Wire.write(displayString,4);
  Wire.endTransmission();
}

void refreshWholeScreen(){
  refreshAll4Digits(rtcReadings[2],rtcReadings[1]);
  previousMinute = rtcReadings[1];
}
