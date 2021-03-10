#include "global.h"
#include "DS3231.h"
#include <SimpleSleep.h>

RTClib RTC;
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
  // Alarm config pins init
//  pinMode(SUPPRESS_NEXT_ALARM_PIN, INPUT_PULLUP);
//  pinMode(DISABLE_ALARM_PIN, INPUT_PULLUP);

  Sleep.deeplyFor(200);

  // External interrupts init
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(DS3231_INT, INPUT_PULLUP);

  // DS3231 init
  Wire.begin();
  clockController.turnOnAlarm(2);
  readDateTimeFromRTC();
  previousMinute = rtcReadings[1];

  // Display Init
  bbi2cInit();
  
//  now = previous = RTC.now();

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
    
//    Serial.println(isNextAlarmSuppressed+0x30);
//    Serial.println("^supp");
    if(!isNextAlarmSuppressed && digitalRead(DISABLE_ALARM_PIN) /*&& (clockController.getDoW()<6)*/){
//      Serial.println("1");
      if(alarm2Set){
//        Serial.println("2");
        if(!displayStatus){
//          Serial.println("3");
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
//    now = RTC.now();

    // determine if update the second field is enough
    if(previousMinute == rtcReadings[1]){
      updateSecondFieldAndIcons();
    }else{
      refreshWholeScreen();
    }

    // Call delay instead of power-down if alarm is active

//    detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));

//    // wait for DS3231 pin to go high
//    while(!digitalRead(DS3231_INT)){
//      Sleep.deeplyFor(10);
//    };
    
//    inBeepingMode = true;
//    attachInterrupt(digitalPinToInterrupt(DS3231_INT), DS3231InterruptHandler, LOW);
//    Sleep.deeply();
//    detachInterrupt(digitalPinToInterrupt(DS3231_INT));
//    inBeepingMode = false;

    if(alarm2Set){
      delay(1000);
    }else{
      Sleep.deeplyFor(1000);
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
