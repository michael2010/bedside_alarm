#include "global.h"
#include <Arduino.h>
#include <Wire.h>

extern bool buttonPressed;
extern bool rtcInterrupted;

bool buttonInterruptDetachmentAllowed = true;

void buttonPressHandler(){
  buttonPressed = true;
  if(buttonInterruptDetachmentAllowed)
    detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
}

void DS3231InterruptHandler(){
  detachInterrupt(digitalPinToInterrupt(DS3231_INT));
  rtcInterrupted = true;
}

byte readButtonState(byte pinNo){
  pinMode(pinNo, INPUT_PULLUP);
  delay(100);
  byte value = digitalRead(pinNo);
  pinMode(pinNo, INPUT);
  return value;
}
