/**
 * @file AstraREVCAN.cpp
 * @author David Sharpe (ds0196@uah.edu)
 * @brief ASTRA's utilities for interfacing with the REV SparkMax over CAN
 *
 */

#if (defined(ESP32) && __has_include("ESP32-TWAI-CAN.hpp")) || \
     (defined(CORE_TEENSY) && __has_include("FlexCAN_T4.h"))

#    include "AstraREVCAN.h"


//--------------------------------------------------------------------------//
//   Specific REV Commands                                                  //
//--------------------------------------------------------------------------//

void CAN_enumerate() {
    uint8_t frame[8] = {0};
    CAN_sendPacket(0, 0x99, frame, 0);
    delay(80);  // Let devices finish enumeration; they will wait ID * 1ms before responding
}

void CAN_sendControl(uint8_t deviceId, sparkMax_ctrlType ctrlType, float value) {
    // DO NOT USE THE NEW CAN FUNCTIONS FOR THIS (specifically duty cycle). SEE COMMENT BELOW.

    CanFrame msg = {0};
    msg.identifier = 0x2050000 + deviceId;
    msg.identifier |= static_cast<uint32_t>(ctrlType) << 6;
    msg.extd = 1;
    msg.data_length_code = 8;

    Float2LEDec(value, msg.data);

    ESP32Can.writeFrame(msg);

    // WARNING: DO NOT USE THIS CODE. THE MOTORS WILL HAVE A SEIZURE.
    //          Ask David, Tristan, or Maddy. We don't fucking know why.
    // uint8_t frame[8] = {0};
    // Float2LEDec(value, frame);
    // CAN_sendPacket(deviceId, static_cast<uint32_t>(ctrlType), frame, 8);
}

void CAN_sendHeartbeat(uint8_t deviceId) {
    uint8_t frame[8] = {0};
    frame[0] = pow(2, deviceId);
    CAN_sendPacket(0, 0xB2, frame, 8);  // Heartbeat is weird...
}

void CAN_identifySparkMax(uint8_t deviceId) {
    uint8_t frame[8] = {0};
    CAN_sendPacket(deviceId, 0x76, frame, 0);
}


void CAN_setParameter(uint8_t deviceId, sparkMax_ConfigParameter parameterID,
                      sparkMax_ParameterType type, uint32_t value) {
    uint8_t frame[8] = {0};  // First 32 bits of frame are value, next 8 bits are type
    frame[4] = static_cast<uint8_t>(type);

    // Load value into frame based on type
    if (type == sparkMax_ParameterType::kUint32 || type == sparkMax_ParameterType::kInt32) {
        frame[0] = value & 0xFF;
        frame[1] = (value >> 8) & 0xFF;
        frame[2] = (value >> 16) & 0xFF;
        frame[3] = (value >> 24) & 0xFF;
    } else if (type == sparkMax_ParameterType::kFloat32) {
        Float2LEDec(*reinterpret_cast<float*>(&value), frame);
    } else if (type == sparkMax_ParameterType::kBool) {
        frame[0] = value ? 1 : 0;
    }

    CAN_sendPacket(deviceId, static_cast<uint8_t>(parameterID) | 0x300, frame, 5);
}

void CAN_reqParameter(uint8_t deviceId, sparkMax_ConfigParameter parameterID) {
    uint8_t frame[8] = {0};
    CAN_sendPacket(deviceId, static_cast<uint8_t>(parameterID) | 0x300, frame, 0);
}


//--------------------------------------------------------------------------//
//   Basic Helper functions                                                 //
//--------------------------------------------------------------------------//

void Float2LEDec(float x, uint8_t (&buffer_data)[8]) {
    unsigned char b[8] = {0};
    memcpy(b, &x, 4);
    for (int i = 0; i < 4; i++) {
        buffer_data[i] = b[i];
    }
    for (int i = 4; i < 8; i++) {
        buffer_data[i] = 0;
    }
}


void CAN_sendPacket(uint8_t deviceId, int32_t apiId, uint8_t data[], uint8_t dataLen) {
    // uint32_t createdId = 0;
    // createdId |= (static_cast<int32_t>(storage->deviceType) & 0x1F) << 24;
    // createdId |= (static_cast<int32_t>(storage->manufacturer) & 0xFF) << 16;
    uint32_t createdId = 0x2050000;
    createdId |= (apiId & 0x3FF) << 6;
    createdId |= (deviceId & 0x3F);

#ifdef DEBUG
    if (apiId != 0xB2) {  // Don't spam the serial monitor with heartbeats
        Serial.print("Sending to ");
        Serial.print(createdId, HEX);
        Serial.print(" - ");
        for (int i = 0; i < dataLen; i++) {
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
        Serial.print(" at ");
        Serial.print(millis());
        Serial.println();
    }
#endif

    CAN_sendPacket(createdId, data, dataLen);
}

void printREVFrame(CanFrame frame) {
    uint8_t deviceId = frame.identifier & 0x3F;
    uint32_t apiId = (frame.identifier >> 6) & 0x3FF;

    Serial.print(apiId, HEX);
    Serial.print(" from ");
    Serial.print(deviceId);
    // Message data:
    if (frame.data_length_code == 0)
        Serial.print(" - no data.");
    else {
        Serial.print(" - data: (");
        Serial.print(frame.data_length_code);
        Serial.print(" B) ");
        for (int i = 0; i < frame.data_length_code; i++) {
            Serial.print(frame.data[i], HEX);
            Serial.print(" ");
        }
    }
    Serial.println();
}


void printREVParameter(CanFrame rxFrame) {
    uint8_t deviceId = rxFrame.identifier & 0x3F;
    uint32_t apiId = (rxFrame.identifier >> 6) & 0x3FF;

    Serial.print("Parameter 0x");
    Serial.print(apiId & 0xFF, HEX);
    Serial.print(" for: ");
    Serial.print(deviceId);
    Serial.print(" (type ");
    Serial.print(rxFrame.data[4]);
    Serial.print("): ");

    //  uint32_t
    if (rxFrame.data[4] == static_cast<uint8_t>(sparkMax_ParameterType::kUint32)) {
        uint32_t val = (rxFrame.data[3] << 24) | (rxFrame.data[2] << 16) | (rxFrame.data[1] << 8) | rxFrame.data[0];
        Serial.print(val);
    //  int32_t
    } else if (rxFrame.data[4] == static_cast<uint8_t>(sparkMax_ParameterType::kInt32)) {
        uint32_t val = (rxFrame.data[3] << 24) | (rxFrame.data[2] << 16) | (rxFrame.data[1] << 8) | rxFrame.data[0];
        Serial.print(static_cast<int32_t>(val));
    // float
    } else if (rxFrame.data[4] == static_cast<uint8_t>(sparkMax_ParameterType::kFloat32)) {
        uint32_t val = (rxFrame.data[3] << 24) | (rxFrame.data[2] << 16) | (rxFrame.data[1] << 8) | rxFrame.data[0];
        Serial.print(*reinterpret_cast<float*>(&val));
    // bool
    } else if (rxFrame.data[4] == static_cast<uint8_t>(sparkMax_ParameterType::kBool)) {
        Serial.print(rxFrame.data[0] ? "True" : "False");
    }

    // Error check
    if (rxFrame.data[5] != static_cast<uint8_t>(sparkMax_paramStatus::kOK)) {
        Serial.print(" - Error: ");
        switch (static_cast<sparkMax_paramStatus>(rxFrame.data[5])) {
        case sparkMax_paramStatus::kInvalidID:
            Serial.print("Invalid ID");
            break;
        
        case sparkMax_paramStatus::kMismatchType:
            Serial.print("Mismatched Type");
            break;
        
        case sparkMax_paramStatus::kAccessMode:
            Serial.print("Access Mode");
            break;
        
        case sparkMax_paramStatus::kInvalid:
            Serial.print("Invalid");
            break;
        
        case sparkMax_paramStatus::kNotImplementedDeprecated:
            Serial.print("Deprecated or Not Implemented");
            break;
        
        default:
            Serial.print("Unknown");
            break;
        }
    }

    Serial.println();
}


//--------------------------------------------------------------------------//
//   Microcontroller-specific                                               //
//--------------------------------------------------------------------------//

void CAN_sendPacket(uint32_t messageID, uint8_t data[], uint8_t dataLen) {
#if defined(ESP32) && __has_include("ESP32-TWAI-CAN.hpp")  // ESP32
    CanFrame outMsg;
    outMsg.extd = 1;  // All REV CAN messages are extended
    outMsg.data_length_code = dataLen;
    outMsg.identifier = messageID;
    for (uint8_t i = 0; i < dataLen; i++)
        outMsg.data[i] = data[i];
    ESP32Can.writeFrame(outMsg);
#elif defined(CORE_TEENSY) && __has_include("FlexCAN_T4.h")  // Teensy
    CAN_message_t outMsg;
    outMsg.flags.extended = 1;  // All REV CAN messages are extended
    outMsg.len = dataLen;
    outMsg.id = messageID;
    for (uint8_t i = 0; i < dataLen; i++)
        outMsg.buf[i] = data[i];
    Can0.write(outMsg);
#endif  // End MCU check
}


#endif  // End library check