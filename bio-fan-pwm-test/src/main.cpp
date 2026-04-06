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

bool ledState = false;

Servo fanMotor;

void setup()
{
  Serial.begin(SERIAL_BAUD); // Start Serial at Baud rate

  // Attach the fan motor
  fanMotor.attach(13);
}

void loop()
{
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    Serial.println(input);

    input.trim();                  // Remove preceding and trailing whitespace
    std::vector<String> args = {}; // Initialize empty vector to hold separated arguments
    parseInput(input, args);       // Separate `input` by commas and place into args vector
    args[0].toLowerCase();         // Make command case-insensitive
    String command = args[0];      // To make processing code more readable

    if (command == "ping")
    {
      Serial.println("pong");
    }

    else if (command == "time")
    {
      Serial.println(millis());
    }

    else if (command == "led") // This command will not work when using boards that do not have a inbuilt
                               // LED (i.e. the ESP32 Dev Module 1)
    {
      digitalWrite(LED_BUILTIN, !ledState);
      ledState = !ledState;
    }
  }
}
