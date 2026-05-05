/**
 * @file main.cpp
 * @author Jack Schumacher (js0342@uah.edu)
 * @author David Sharpe (ds0196@uah.edu)
 * @brief ASTRA Biosensor Citadel embedded code
 *
 */
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#include <cmath>
#include <typeinfo>

#include "AstraMisc.h"
#include "AstraMotors.h"
#include "AstraREVCAN.h"
#include "AstraVicCAN.h"

// Uses default address of 0x40 according to Adafruit documentation
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Remove to disable the boards inbuilt LED blinking
#define BLINK

#define CAN_TX 12
#define CAN_RX 13

// Manually define the I2C SDA and SLC pins for communication with the PWM linear actuator driver
#define I2C_SDA 37
#define I2C_SCL 36

#define FAN_PWM_MIN 1000 // us  -1.0 duty
#define FAN_PWM_MAX 2000 // us  1.0 duty

#define FAN_PWM 9

#define COMMS_UART Serial // To/from USB for debugging
#define PWM_FREQUENCY 50

struct linearActuators
{
    int chem_increase;
    int chem_decrease;
};
linearActuators chemicalActuators[3] = {
    {10, 11},
    {9, 8},
    {7, 6}};


int chemicalMove;

// Value of movement to ignore
double chemicalDeadzone = 0.05;
// Constraining the speed values for Chemical linear actuators
float chemActuatorsSpeed = constrain(chemActuatorsSpeed,-1.0,1.0);

// Defined servos (3 for valves, 3 for distributors, 3 for chemicals)
Servo valve0, valve1, valve2, distributor0, distributor1, distributor2;

// Track the current distributor positions and the requested distributor positions
int distributorPos[3] = {0, 0, 0};
int distributorReq[3] = {0, 0, 0};
Servo *valves[3] = {&valve0, &valve1, &valve2};

long lastWiggle = 0; // For Distributor servos- count last time moved

unsigned long lastCtrlCmd = 0;
unsigned long lastMotorStatus = 0;

// Variables for CAN commands- since all servos in a group should be writing the same
int valveID;
int chemicalID;

int distributorID;

bool ledState = false;
// Control the NEO550 functioning as the fan motor
Servo fanMotor;

void setup()
{
   
    Serial.begin(SERIAL_BAUD);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000); // Internal oscillator frequency
    pwm.setPWMFreq(PWM_FREQUENCY);        // External PWM frequency
    Wire.begin(I2C_SDA, I2C_SCL);         // Use the defined SDA and SCL pins

    // Valves are on top of the unit
    valve0.attach(1);
    valve1.attach(2);
    valve2.attach(3);

    // Distributors are on each of the pieces that hang down
    distributor0.attach(4);
    distributor1.attach(5);
    distributor2.attach(6);

    fanMotor.attach(FAN_PWM, FAN_PWM_MIN, FAN_PWM_MAX);

    // Set all valve, distributor, and chemical servos to their 0 positions
    valve0.write(0);
    valve1.write(0);
    valve2.write(0);

    distributor0.write(0);
    distributor1.write(0);
    distributor2.write(0);

    fanMotor.writeMicroseconds((FAN_PWM_MIN + FAN_PWM_MAX) / 2);

    if (ESP32Can.begin(TWAI_SPEED_1000KBPS, CAN_TX, CAN_RX))
        Serial.println("CAN bus started!");
    else
        Serial.println("CAN bus failed!");
}

void loop()
{

    // Motor control safety timeout- if no command is received in 1 second, shut off the NEO
    if (millis() - lastCtrlCmd > 1000)
    {
        lastCtrlCmd = millis();
        fanMotor.writeMicroseconds((FAN_PWM_MIN + FAN_PWM_MAX) / 2);
        Serial.println("CITADEL Fan Motor - Safety Timeout.");
    }

    // Serial commands
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

        else if (args[0] == "can_relay_tovic")
        {
            vicCAN.relayFromSerial(args);
        }

        else if (args[0] == "can_relay_mode")
        {
            if (args[1] == "on")
            {
                vicCAN.relayOn();
            }
            else if (args[1] == "off")
            {
                vicCAN.relayOff();
            }
        }
    }

    // CAN
    if (vicCAN.readCan())
    {
        const uint8_t commandID = vicCAN.getCmdId();
        static std::vector<double> canData;
        vicCAN.parseData(canData);

        Serial.print("VicCAN: ");
        Serial.print(commandID);
        Serial.print("; ");
        if (canData.size() > 0)
        {
            for (const double &data : canData)
            {
                Serial.print(data);
                Serial.print(", ");
            }
        }
        Serial.println();

        // Misc CAN commands

        if (commandID == CMD_PING)
        {
            vicCAN.respond(1); // "pong"
            Serial.println("Received ping over CAN");
        }

        if (commandID == 40)
        {
            if (canData.size() == 1)
            {
                valveID = canData[0];
            }
            
            if (canData.size() == 4)
            {
                for (int i = 0; i < 3; i++)
                {
                    distributorReq[i] = canData[i];
                }
            }
            // If -1 is passed in, close all valves
            // Valve movement
            if (valveID >= 0 && valveID <= 2)
            {
                valves[valveID]->write(180);
            }
            // If the valve IDs are not valid or -1 is passed in, loop through and close all valves
            else
            {
                for (int i = 0; i < 3; i++)
                {
                    valves[i]->write(0);
                }
            }
        }

        if (commandID == 24)
        {
            chemicalID = canData[0];
            chemicalMove = canData[1];
            if((chemicalID >= 0) && (chemicalID <=2)){
                
            }
        }
    }
    // Wiggle every 500ms
    if ((millis() - lastWiggle > 500) && (millis() - lastWiggle < 2000))
    {
        // Max movement for these servos is 100 degrees due to hardware mounting limit
        lastWiggle = millis();

        distributorPos[0] = distributorReq[0] && !distributorPos[0];
        distributor0.write(distributorPos[0] ? 100 : 0);

        distributorPos[1] = distributorReq[1] && !distributorPos[1];
        distributor1.write(distributorPos[1] ? 100 : 0);

        distributorPos[2] = distributorReq[2] && !distributorPos[2];
        distributor2.write(distributorPos[2] ? 100 : 0);
    }
}
