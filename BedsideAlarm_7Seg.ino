#include "global.h"
#include "DS3231.h"
#include <SimpleSleep.h>


DS3231 clockController;

SimpleSleep Sleep;

ClockState centralClkState = Initing;
Menu currentMenu = Suppression;

//Alarm Flags
volatile bool rtcInterrupted = false;
extern bool isNextAlarmSuppressed;
//bool isAlarmDisabled = false;

//Screen Flags
extern bool buttonPressed;
extern bool displayStatus;
extern uint8_t timeout;

// Function declarations
void refreshScreen();
void buttonPressHandler();
void DS3231InterruptHandler();
void changeDisplayPowerStatus();

/*
 * Turn on power pins for other peripherals
 * Re-init I2C and display
 * Enable button pin pullups
 * Enable interrupt pins
 * Enable interrupt handlers
 */
void setup() {
  // Alarm config pins init
  // Power Init
  pinMode(DS3231_GND, OUTPUT);
  pinMode(PERI_PWR, OUTPUT);
  pinMode(FD650_CLK_GATE, OUTPUT);
//  turnOnOffPeri(true);

//  pinMode(SUPPRESS_NEXT_ALARM_PIN, INPUT_PULLUP);
  pinMode(DISABLE_ALARM_PIN, INPUT_PULLUP);
  pinMode(UPD_RTC_PIN, INPUT_PULLUP);
  pinMode(UPD_RTC_INC_PIN, INPUT_PULLUP);
  pinMode(UPD_RTC_DEC_PIN, INPUT_PULLUP);

//  Sleep.deeplyFor(200);

  // External interrupts init
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(DS3231_INT, INPUT_PULLUP);

  // Debug via serial
//  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressHandler, LOW);
  attachInterrupt(digitalPinToInterrupt(DS3231_INT), DS3231InterruptHandler, LOW);

//  centralClkState = Idle;
}

void loop(){

  // Interrupt handling
  if(buttonPressed){
    buttonPressed = false;
    noTone(SPEAKER_PIN);
    timeout = TIMEOUT_S;
    rtcPhase = reset;
    centralClkState = Idle;
    if(!displayStatus){
      displayStatus = true;
      changeDisplayPowerStatus();
    }
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressHandler, LOW);
  }else if(rtcInterrupted){
    turnOnOffPeri(true);
    clockController.checkIfAlarm(2); // Must call checkIfAlarm to clear the DS3231 interrupt flag
    rtcInterrupted = false;
    readDateTimeFromRTC();
    if(!isNextAlarmSuppressed && (alarmMode!=Disabled) && (!alarmMode==Weekdays || rtcReadings[3]<6)){
      timeout = ALARM_BEEP_PERIOD;
      centralClkState = Alarming;
      //re-attach interrupt handler
      attachInterrupt(digitalPinToInterrupt(DS3231_INT), DS3231InterruptHandler, LOW);
      attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressHandler, LOW);
    }else{
      isNextAlarmSuppressed = false; // reset the suppression state to false anyways
      centralClkState = Sleeping;
    }
  }
  
  // Clock State handling
  switch(centralClkState){
    case Sleeping:
      displayStatus = false;
      changeDisplayPowerStatus();
      break;
    case RTCUpdating:
    {
      CheckButton(UPD_RTC_INC_PIN, UpButtonHandler);
      CheckButton(UPD_RTC_DEC_PIN, DownButtonHandler);
      CheckButton(UPD_RTC_PIN, SettingButtonHandler);
      Sleep.deeplyFor(200);
      break;
    }
    case FunctionMenu:
    {
      CheckButton(UPD_RTC_INC_PIN, advanceMenu);
      CheckButton(UPD_RTC_DEC_PIN, previousMenu);
      CheckButton(UPD_RTC_PIN, useFunction);
      Sleep.deeplyFor(500);
      break;
    }
    case Alarming:
      tone(SPEAKER_PIN, 1000,700);
    case Idle:
    default:
    {
      if(CheckButton(UPD_RTC_PIN, enterFunctionMenu)){
        break;
      }else{
        refreshScreen(); 
      }
      Sleep.deeplyFor(1000);
    
      // Signal to shutdown display if timeout is reached
      if(digitalRead(BUTTON_PIN) == LOW){
        timeout = TIMEOUT_S;
      }
      else if(--timeout == 0){
        centralClkState = Sleeping;
      }
    }
  }
}
