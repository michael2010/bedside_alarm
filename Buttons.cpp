#include "global.h"

volatile bool buttonPressed = true;
bool isNextAlarmSuppressed = false;

extern uint8_t timeout;

void displayAlarmState();
void refreshAll4Digits(byte highByte, byte lowByte);

void waitUntilReleased(uint8_t button){
  while(digitalRead(button) == LOW){}
}

void SuppressButtonHandler(){
    isNextAlarmSuppressed = !isNextAlarmSuppressed;
    displayAlarmState();
    timeout = TIMEOUT_S;
}

bool CheckButton(uint8_t pinNumb, void (*handler)()){
  if(digitalRead(pinNumb) == LOW){
    handler();
    waitUntilReleased(pinNumb);
    previousMinute = 0xFF;
    return true;
  }
  return false;
}

bool CheckAndDisplayMenu(uint8_t pinNumb, void (*handler)()){
  if(CheckButton(pinNumb, handler)){
    
  }
}

void SettingButtonHandler(){
    rtcPhase=rtcSetupPhase((((uint8_t)rtcPhase)+1)%5);

    if(rtcPhase==reset){
      // Update RTC
      timeout = TIMEOUT_S;
      updateRTCSettings();
      centralClkState = Idle;
    }else{
      if(rtcPhase==year){
        centralClkState = RTCUpdating;
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
}

void UpButtonHandler(){
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
}

void DownButtonHandler(){
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
}

void advanceMenu(){
  currentMenu=Menu((((uint8_t)currentMenu)+1)%4);
}

void revertMenu(){
  currentMenu=Menu((((uint8_t)currentMenu)-1)%4);
}
