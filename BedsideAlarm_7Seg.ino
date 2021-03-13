#include "global.h"
#include "DS3231.h"
#include <SimpleSleep.h>


DS3231 clockController;
//DateTime previous;
//DateTime now;

SimpleSleep Sleep;

//Alarm Flags
volatile bool alarm2Set = false;
volatile bool rtcInterrupted = false;
volatile bool inBeepingMode = false;
bool isNextAlarmSuppressed = false;
//bool isAlarmDisabled = false;

//Screen Flags
volatile bool buttonPressed = true;
volatile bool displayStatus = true;
volatile byte timeout = TIMEOUT_S;

// Time data from RTC
byte rtcReadings[7];
byte previousMinute;

// Function declarations
void refreshWholeScreen();
void buttonPressHandler();
void DS3231InterruptHandler();
void changeDisplayPowerStatus();
void updateSecondFieldAndIcons();
byte readButtonState(byte pinNo);
void bbi2cInit();

void setup() {
  // DS3231 init
  Wire.begin();
  // Alarm config pins init
  // Power Init
  pinMode(PERI_PWR, OUTPUT);
  pinMode(DS3231_GND, OUTPUT);
  digitalWrite(DS3231_GND, LOW);
  turnOnOffPeri(true);

  Sleep.deeplyFor(200);

  // External interrupts init
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(DS3231_INT, INPUT_PULLUP);

//  clockController.turnOnAlarm(2);
  readDateTimeFromRTC();
  previousMinute = rtcReadings[1];

  // Display Init
  bbi2cInit();

  // Debug via serial
//  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressHandler, LOW);
  attachInterrupt(digitalPinToInterrupt(DS3231_INT), DS3231InterruptHandler, LOW);
}

void loop() {

  if(rtcInterrupted){
//    Serial.println("RTC interrupted");
    // Must call checkIfAlarm to clear the DS3231 interrupt flag
    alarm2Set = clockController.checkIfAlarm(2);
    

    if(!isNextAlarmSuppressed && digitalRead(DISABLE_ALARM_PIN) /*&& (clockController.getDoW()<6)*/){
      if(alarm2Set){
        if(!displayStatus){
          buttonPressHandler();
        }
        timeout = ALARM_BEEP_PERIOD;
        alarm2Set = true;
      }
    }else{
      alarm2Set = false;
    }
    isNextAlarmSuppressed = false; // reset the suppression state to false anyways
    
    //re-attach interrupt handler
    attachInterrupt(digitalPinToInterrupt(DS3231_INT), DS3231InterruptHandler, LOW);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressHandler, LOW);
    rtcInterrupted = false;
  }

  if(alarm2Set){
//    Serial.println("tone");
    tone(SPEAKER_PIN, 1000,700);
  }

  if(buttonPressed){
    buttonPressed = false;
//    clockController.enableOscillator(true,false,0);
    changeDisplayPowerStatus();
    
  }else{
    // reload the current time from RTC
    readDateTimeFromRTC();

    // determine if update the second field is enough
    if(previousMinute == rtcReadings[1] || rtcPhase !=reset){
      updateSecondFieldAndIcons();
    }else{
      refreshWholeScreen();
    }

    if(alarm2Set){
      delay(1000);
    }else{
      if(rtcPhase !=reset){
        Sleep.deeplyFor(200);
      }else{
        Sleep.deeplyFor(1000);
      }
    }

    // Signal to shutdown display if timeout is reached
    if(digitalRead(BUTTON_PIN) == LOW){
      timeout = TIMEOUT_S;
    }
    else if(--timeout == 0){
      clockController.turnOnAlarm(2);
      buttonPressed = true;
      displayStatus = false;
    }
  }
}

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
