/**
 * @file main.cpp
 * @author Jack Schumacher (jackrschumacher@gmail.com)
 *
 */
#include <Arduino.h>
#include <ESP32Servo.h>
// #include <ASTRARevCAN.h>

// Remove to disable the boards inbuilt LED blinking
#define BLINK



void setup() {
  // put your setup code here, to run once:
  // Defined servos (3 for valves, 3 for distributors, 3 for chemicals)
  Servo valve1, valve2, valve3, distributor1, distrutor2, distributor3, chemical1, chemical2, chemical3;
  Serial.begin(9600); // May need to change this later
  pinMode(LED_BUILTIN, OUTPUT); // Enable the bult-in LED so that we can use it for status

}

void loop() {
  // put your main code here, to run repeatedly:
  readSerialCommand();

}

void readSerialCommand(){
  const int BUFFER_SIZE = 32;
  static char command_buffer[BUFFER_SIZE+1];
  static int length = 0;

  if(Serial.available()){
    char current_value = Serial.read();
    if((current_value == '\r') || (current_value == '\n')){

      if(length > 0){
        processSerialCommand();
      }
      length = 0;
    }
    else{
      if(length < BUFFER_SIZE){
        command_buffer[length++];
        command_buffer[length] = 0;
      }
      else{
        Serial.println("Command buffer overflow");

      }
    }
  }
}

void processSerialCommand(){

}
  



