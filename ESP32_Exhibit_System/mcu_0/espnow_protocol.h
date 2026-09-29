#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// ==========================================
// 1. MAC ADDRESS REGISTRY
// ==========================================
const uint8_t MAC_CONTROLLER[6] = {0xB4, 0xBF, 0xE9, 0x60, 0x59, 0x50}; // Node 0
const uint8_t MAC_STATION_1[6]  = {0x70, 0x4B, 0xCA, 0x90, 0x4B, 0x80}; // Node 1 (Wind)
const uint8_t MAC_STATION_2[6]  = {0xB4, 0xBF, 0xE9, 0x60, 0xB7, 0x58}; // Node 2
const uint8_t MAC_STATION_3[6]  = {0x70, 0x4B, 0xCA, 0x90, 0xCB, 0x80}; // Node 3

// Lookup array indexed by Station ID (0 = Main Ctrl, 1..3 = Stations)
const uint8_t* const STATION_MACS[] = {
    MAC_CONTROLLER,
    MAC_STATION_1,
    MAC_STATION_2,
    MAC_STATION_3
};

// ==========================================
// 2. COMMAND CODES (Controller -> Stations)
// ==========================================
enum CommandType : uint8_t {
    CMD_NONE         = 0,
    CMD_RUN_TEST     = 1, // Triggers self-test
    CMD_POWER_SAVE   = 2, // Toggles low power
    CMD_SET_PWM_VAL  = 3, // Saves new PWM speed
    CMD_PING         = 4  // Connection check
};

// ==========================================
// 3. STRUCT PAYLOAD DEFINITIONS
// ==========================================

// Sent BY Stations TO Main Controller
typedef struct __attribute__((packed)) {
    uint8_t  station_id;       // 1, 2, or 3
    char     station_name[16]; // e.g., "Wind Turbine"
    uint8_t  status;           // 0 = Error/Boot, 1 = OK, 2 = Testing, 3 = PowerSave
    uint8_t  battery_pct;      // 0 to 100%
    uint8_t  car_status;       // 0 = No car, 1 = Car detected
} StationTelemetryPayload;

// Sent BY Main Controller TO Stations
typedef struct __attribute__((packed)) {
    uint8_t  target_station_id; // 0 = Broadcast to all, 1..3 = Specific Station
    uint8_t  command;           // Uses CommandType enum
    uint16_t parameter;         // Optional value (e.g. PWM speed 0-255)
} ControllerCommandPayload;

#endif