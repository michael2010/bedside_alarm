#include "global.h"
#include <Arduino.h>
#include <Wire.h>

extern bool buttonPressed;
extern bool displayStatus;
extern uint8_t timeout;
extern bool alarm2Set;
extern bool inBeepingMode;
extern bool rtcInterrupted;

bool buttonInterruptDetachmentAllowed = true;

void buttonPressHandler(){
  if(!buttonPressed){
    if(!displayStatus){
      buttonPressed = true;
      rtcPhase = reset;
    }
    displayStatus = true;
    timeout = TIMEOUT_S;
    noTone(SPEAKER_PIN);
    alarm2Set = false;
  }

  if(buttonInterruptDetachmentAllowed)
    detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
}

void DS3231InterruptHandler(){
  if(!inBeepingMode){
    detachInterrupt(digitalPinToInterrupt(DS3231_INT));
    rtcInterrupted = true;
  }
}

byte readButtonState(byte pinNo){
  pinMode(pinNo, INPUT_PULLUP);
  delay(100);
  byte value = digitalRead(pinNo);
  pinMode(pinNo, INPUT);
  return value;
}
