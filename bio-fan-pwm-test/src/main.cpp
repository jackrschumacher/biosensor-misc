/**
 * @file main.cpp
 * @author Jack Schumacher (js0342@uah.edu)
 * @brief ASTRA Biosensor Citadel fan PWM test script
 *
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include "AstraMisc.h"

#define BLINK
#define COMMS_UART Serial


void setup() {
  Serial.begin(SERIAL_BAUD);
}

void loop() {
  
}
