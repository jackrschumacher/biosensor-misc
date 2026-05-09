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

// Fan PWM pin (connected directly to ESP, unlike the others connected through the multiplexer)
#define FAN_PWM 9

// Other servos PWM pins (connected through the multiplexer)
#define VALVE_0 1
#define VALVE_1 2
#define VALVE_2 3

#define DISTRIBUTOR_0 4
#define DISTRIBUTOR_1 5
#define DISTRIBUTOR_2 6

// Typical PWM servo movement
#define SERVOMOVEMIN 150
#define SERVOMOVEMAX 600

#define COMMS_UART Serial // To/from USB for debugging
#define PWM_FREQUENCY 50

// Linear actuator structure to easily allow for extension/retraction of the chemical linear actuators (increase = extend, decrease = retract)
struct linearActuators
{
    int chem_increase;
    int chem_decrease;
};
// Linear actuator pins
linearActuators chemicalActuators[3] = {
    {10, 11},
    {9, 8},
    {7, 6}};

int chemicalMove;

// Value of movement to ignore
double chemicalDeadzone = 0.05;
// Constraining the speed values for Chemical linear actuators
uint16_t chemActuatorsSpeed = constrain(chemActuatorsSpeed, -1.0, 1.0);

// Create valve and distributor servo arrays with Pin ID
uint16_t distributorServos[3] = {DISTRIBUTOR_0, DISTRIBUTOR_1, DISTRIBUTOR_2};
uint16_t valveServos[3] = {VALVE_0, VALVE_1, VALVE_2};

// Track the current distributor positions and the requested distributor positions
int distributorPos[3] = {0, 0, 0};
int distributorReq[3] = {0, 0, 0};

long lastWiggle = 0; // For Distributor servos- count last time moved

unsigned long lastCtrlCmd = 0;
unsigned long lastMotorStatus = 0;
int servoAngle;
int pwmPulse;
// Variables for CAN commands- since all servos in a group should be writing the same
int valveID;
int chemicalID;
int distributorID;

bool ledState = false;
// Control the NEO550 functioning as the fan motor
Servo fanMotor;

// Since there is no function in this library for natively writing servos (0-180), take raw angle and write to pwm value for that pin
void writeServo(uint16_t pin, int angle)
{
    servoAngle = constrain(angle, 0, 180);
    pwmPulse = map(angle, 0, 180, SERVOMOVEMIN, SERVOMOVEMAX);
    pwm.setPWM(pin, 0, angle);
}

void setup()
{

    Serial.begin(SERIAL_BAUD);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000); // Internal oscillator frequency
    pwm.setPWMFreq(PWM_FREQUENCY);        // External PWM frequency
    Wire.begin(I2C_SDA, I2C_SCL);         // Use the defined SDA and SCL pins

    // Loop through all of the servos and close them
    for (int i = 0; i <= 2; i++)
    {
        writeServo(distributorServos[i], 0);
        writeServo(valveServos[i], 0);
    }

    // Set all Linear actuators to 0 (IDs 0-2)
    pwm.setPWM(chemicalActuators[0].chem_decrease, 0, 0);
    pwm.setPWM(chemicalActuators[0].chem_increase, 0, 0);
    pwm.setPWM(chemicalActuators[1].chem_decrease, 0, 0);
    pwm.setPWM(chemicalActuators[1].chem_increase, 0, 0);
    pwm.setPWM(chemicalActuators[2].chem_decrease, 0, 0);
    pwm.setPWM(chemicalActuators[2].chem_increase, 0, 0);

    fanMotor.attach(FAN_PWM, FAN_PWM_MIN, FAN_PWM_MAX);

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

        // Misc CAN commands - ASTRA Embedded Lib

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
            // Valve movement - valveID must be between 0-2
            // Open all valves
            if (valveID >= 0 && valveID <= 2)
            {
                writeServo(valveID, 180);
            }
            // If the valve IDs are not valid or -1 is passed in, loop through and close all valves
            else
            {
                for (int valveID = 0; valveID <= 2; valveID++)
                {
                    writeServo(valveID, 0);
                }
            }
        }

        if (commandID == 24)
        {
            chemicalID = canData[0];
            chemicalMove = canData[1];
            chemActuatorsSpeed = (uint16_t)(abs(chemicalMove) * 4095); // Map -1,0,1 values into scale that the actuator can understand
            if ((chemicalID >= 0) && (chemicalID <= 2))
            { // Ensure that chemical value is between
                if (chemActuatorsSpeed > 0)
                { // Extend
                    pwm.setPWM(chemicalActuators[chemicalID].chem_increase, 0, chemActuatorsSpeed);
                    pwm.setPWM(chemicalActuators[chemicalID].chem_decrease, 0, 0);
                }
                if (chemActuatorsSpeed == 0)
                { // Do not move linear actuator
                    pwm.setPWM(chemicalActuators[chemicalID].chem_increase, 0, 0);
                    pwm.setPWM(chemicalActuators[chemicalID].chem_decrease, 0, 0);
                }
                else
                { // Retract linear actuator
                    pwm.setPWM(chemicalActuators[chemicalID].chem_decrease, 0, chemActuatorsSpeed);
                    pwm.setPWM(chemicalActuators[chemicalID].chem_increase, 0, 0);
                }
            }
        }
    }
    // Wiggle every 500ms
    if ((millis() - lastWiggle > 500) && (millis() - lastWiggle < 2000))
    {
        // Max movement for these servos is 100 degrees due to hardware mounting limit
        lastWiggle = millis();

        distributorPos[0] = distributorReq[0] && !distributorPos[0];
        writeServo(distributorServos[0], distributorPos[0] ? 100 : 0);

        distributorPos[1] = distributorReq[1] && !distributorPos[1];
        writeServo(distributorServos[1], distributorPos[1] ? 100 : 0);

        distributorPos[2] = distributorReq[2] && !distributorPos[2];
        writeServo(distributorServos[2], distributorPos[2] ? 100 : 0);
    }
}
